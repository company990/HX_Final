`timescale 1ns/1ps

// ============================================================================
// D5-D1
// Fixed DISPLAY_BUF0 / DISPLAY_BUF1 sequential read + atomic commit controller.
//
// Purpose:
//   The Bilinear frame builder has already produced a complete 640x480 image
//   in one DISPLAY buffer.  The HDMI side therefore no longer performs any
//   scaling, random address mapping, dummy black reads, or return-mask work.
//
// Per display frame, this controller issues exactly:
//
//     active_base + 0
//     active_base + 1
//     ...
//     active_base + 307199
//
// A newly built buffer is captured with commit_valid, but it is NOT made
// active immediately.  It becomes active only at display_frame_start, i.e.
// at a safe display-frame boundary.  This is the double-buffer anti-tearing
// rule.
//
// Important:
//   App_rd_en is the ONLY event that advances the sequential address.
//   Controller stalls and read_request gaps therefore cannot disturb alignment.
//
// This block controls READ COMMANDS only.  D5-D2 will connect the delayed
// SDRAM returns to the display FIFO / HDMI consumer.
//
// Verilog-2001 compatible.
// ============================================================================

module display_buffer_read_ctrl #(
    parameter TRANSITIONS = 0,
    parameter ADDR_BITS    = 21,
    parameter FRAME_PIXELS = 307200,
    parameter DISP0_BASE   = 1048576,
    parameter DISP1_BASE   = 1355776,
    parameter BURST_SIZE   = 128
)(
    input                       clk,
    input                       rst,
    input transition_mix_enable,transition_old_buffer_index,
    input [2:0] transition_mode,
    input [9:0] transition_pos,
    input [5:0] transition_counter,

    // ------------------------------------------------------------------------
    // Completed-frame commit from bilinear_frame_builder.
    // ------------------------------------------------------------------------
    input                       commit_valid,
    input                       commit_buffer_index,

    // Safe display-frame boundary pulse.
    input                       display_frame_start,

    // Downstream aligner confirms that one complete V1-sized command run has
    // reserved storage.
    input                       read_request,

    // Existing SDRAM availability signals.
    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,
    input                       App_wr_busy,

    // SDRAM read command.
    output                      App_rd_en,
    output     [ADDR_BITS-1:0]  App_rd_addr,

    // ------------------------------------------------------------------------
    // State / debug.
    // ------------------------------------------------------------------------
    output reg                  active_buffer_index,

    output reg                  commit_pending,
    output reg                  pending_buffer_index,

    output reg                  buffer_switch_pulse,

    output reg                  display_active,
    output reg                  frame_read_complete,

    output     [18:0]           word_index,
    output                      frame_last,

    output reg [18:0]           commands_this_frame,

    output reg                  commit_overwrite_sticky
);

    reg [18:0] word_index_r;
    reg [2:0] pixel_block;
    wire [9:0] pixel_x={pixel_block,word_index_r[6:0]};
    reg [8:0] pixel_y;
    reg        burst_active;
    reg [7:0]  burst_count;
    reg [7:0]  burst_last_count;

    wire [ADDR_BITS-1:0] active_base;
    wire [19:0] commands_remaining;
    wire [7:0]  next_burst_len;

    wire select_new;
    generate if(TRANSITIONS) begin : gen_transition_read
        transition_compositor #(.POSITION_FROM_COUNTER(1)) mask(
            .old_rgb(24'd0),.new_rgb(24'd0),
            .pixel_x(pixel_x),.pixel_y({1'b0,pixel_y}),
            .transition_mode(transition_mode),
            .transition_pos(transition_pos),.transition_counter(transition_counter),
            .transition_busy(transition_mix_enable),
            .transition_rgb(),.select_new(select_new));
    end else begin : gen_plain_read
        assign select_new=1'b1;
    end endgenerate
    wire selected_buffer_index=transition_mix_enable && !select_new ?
                                transition_old_buffer_index : active_buffer_index;
    assign active_base =
        selected_buffer_index ?
        DISP1_BASE[ADDR_BITS-1:0] :
        DISP0_BASE[ADDR_BITS-1:0];

    assign commands_remaining =
        FRAME_PIXELS - commands_this_frame;

    assign next_burst_len =
        (commands_remaining < BURST_SIZE) ?
        commands_remaining[7:0] :
        BURST_SIZE[7:0];

    // Never issue a command in the same cycle as display_frame_start.
    // The active buffer may change on that edge; the first command from the
    // new buffer is intentionally issued on a later cycle.
    assign App_rd_en =
        display_active &&
        burst_active &&
        !display_frame_start &&
        !frame_read_complete;

    assign App_rd_addr =
        active_base +
        {{(ADDR_BITS-19){1'b0}}, word_index_r};

    assign word_index =
        word_index_r;

    assign frame_last =
        (word_index_r == FRAME_PIXELS - 1);


    always @(posedge clk or posedge rst) begin
        if (rst) begin

            active_buffer_index <= 1'b0;

            commit_pending       <= 1'b0;
            pending_buffer_index <= 1'b0;

            buffer_switch_pulse <= 1'b0;

            display_active      <= 1'b0;
            frame_read_complete <= 1'b0;

            word_index_r        <= 19'd0;
            pixel_block<=0;pixel_y<=0;
            commands_this_frame <= 19'd0;

            burst_active     <= 1'b0;
            burst_count      <= 8'd0;
            burst_last_count <= 8'd0;

            commit_overwrite_sticky <= 1'b0;

        end
        else begin

            buffer_switch_pulse <= 1'b0;

            // ---------------------------------------------------------------
            // Safe frame boundary has highest priority for display address
            // restart and pending-buffer activation.
            //
            // If commit_valid arrives on exactly the same cycle, use that
            // fresh commit directly instead of delaying it by one frame.
            // ---------------------------------------------------------------
            if (display_frame_start) begin

                display_active      <= 1'b1;
                frame_read_complete <= 1'b0;

                word_index_r        <= 19'd0;
                pixel_block<=0;pixel_y<=0;
                commands_this_frame <= 19'd0;

                burst_active     <= 1'b0;
                burst_count      <= 8'd0;
                burst_last_count <= 8'd0;

                if (commit_valid) begin

                    if (active_buffer_index != commit_buffer_index)
                        buffer_switch_pulse <= 1'b1;

                    active_buffer_index <= commit_buffer_index;

                    commit_pending <= 1'b0;

                end
                else if (commit_pending) begin

                    if (active_buffer_index != pending_buffer_index)
                        buffer_switch_pulse <= 1'b1;

                    active_buffer_index <= pending_buffer_index;

                    commit_pending <= 1'b0;
                end
            end
            else begin

                // -----------------------------------------------------------
                // A mid-frame commit is only queued.
                // The current active buffer never changes here.
                // -----------------------------------------------------------
                if (commit_valid) begin

                    if (
                        commit_pending &&
                        (pending_buffer_index != commit_buffer_index)
                    )
                        commit_overwrite_sticky <= 1'b1;

                    pending_buffer_index <= commit_buffer_index;
                    commit_pending       <= 1'b1;
                end


                // Start only after a complete 128-command run has reserved
                // return-path capacity AND idle SDRAM, as in the supplied
                // read-admission fix. Do not gate individual words in a run.
                if (
                    display_active &&
                    !frame_read_complete &&
                    !burst_active &&
                    read_request &&
                    Sdr_init_done &&
                    !Sdr_busy &&
                    !Sdr_init_ref_vld &&
                    !App_wr_busy &&
                    (commands_remaining != 0)
                ) begin
                    burst_active <= 1'b1;
                    burst_count <= 8'd0;
                    burst_last_count <= next_burst_len - 1'b1;
                end

                // App_rd_en remains high for the complete reserved run.  It
                // is still the unique address/count advance event.
                if (App_rd_en) begin

                    if (burst_count == burst_last_count) begin
                        burst_active <= 1'b0;
                        burst_count <= 8'd0;
                    end
                    else begin
                        burst_count <= burst_count + 1'b1;
                    end

                    if (word_index_r == FRAME_PIXELS - 1) begin

                        commands_this_frame <= FRAME_PIXELS[18:0];
                        frame_read_complete <= 1'b1;
                        burst_active <= 1'b0;

                        // Keep the last valid index visible until the next
                        // display_frame_start.
                        word_index_r <= word_index_r;

                    end
                    else begin

                        word_index_r <= word_index_r + 1'b1;
                        if(word_index_r[6:0]==7'd127) begin
                            if(pixel_block==3'd4) begin
                                pixel_block<=0;pixel_y<=pixel_y+1'b1;
                            end else pixel_block<=pixel_block+1'b1;
                        end

                        commands_this_frame <=
                            commands_this_frame + 1'b1;
                    end
                end
            end
        end
    end

endmodule
