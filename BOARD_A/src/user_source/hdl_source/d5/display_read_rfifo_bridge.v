`timescale 1ns/1ps

// ============================================================================
// D5-D2-B
// Fixed DISPLAY buffer read path -> delayed-return aligner -> existing
// rfifo_32_32_512 -> video/read clock domain.
//
// This wrapper is intentionally shaped around the OLD frame_read_write video
// interface so the existing:
//
//      d5_video_timing_data
//      video_delay
//
// side can later be reconnected with minimal change.
//
// Existing project FIFO interface preserved exactly:
//   rfifo_32_32_512
//      clkr       = read_clk
//      clkw       = mem_clk
//      rst        = fifo_rst
//      we         = aligned returned-word valid
//      re         = read_en
//      di         = aligned returned SDRAM word
//      empty_flag = read_fifo_empty
//      full_flag  = read_fifo_full
//      dout       = read_data
//
// New display path:
//   DISPLAY_BUF0/1
//        -> display_buffer_read_ctrl (D5-D1)
//        -> SDRAM
//        -> display_sdram_return_fifo (D5-D2-A)
//        -> rfifo_32_32_512
//        -> read_data / video_delay
//
// read_req/read_req_ack compatibility:
//   read_req is a READ_CLK-domain level request held high until ack.
//   It is level-synchronized into MEM_CLK.
//   A new frame is accepted only when:
//      - all delayed SDRAM returns are drained from D2-A
//      - the asynchronous video FIFO is empty
//   That accepted request generates the MEM_CLK display_frame_start pulse.
//
// This means old-frame data can never leak across a new-frame boundary.
//
// Verilog-2001 compatible.
// ============================================================================

module display_read_rfifo_bridge #(
    parameter TRANSITIONS = 0,
    parameter ADDR_BITS             = 21,
    parameter FRAME_PIXELS          = 307200,
    parameter DISP0_BASE            = 1048576,
    parameter DISP1_BASE            = 1355776,
    parameter ALIGN_FIFO_ADDR_BITS  = 8,
    parameter TRANSPORT_BURST_SIZE  = 128,
    parameter FRAME_LOCKED = 0
)(
    // ------------------------------------------------------------------------
    // Memory clock domain
    // ------------------------------------------------------------------------
    input                       mem_clk,
    input                       mem_rst,
    input transition_mix_enable,transition_old_buffer_index,
    input [2:0] transition_mode,
    input [9:0] transition_pos,
    input [5:0] transition_counter,

    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,
    input                       App_wr_busy,

    output                      App_rd_en,
    output     [ADDR_BITS-1:0]  App_rd_addr,

    input                       Sdr_rd_en,
    input      [31:0]           Sdr_rd_dout,

    // Completed DISPLAY buffer from the Bilinear frame builder.
    // These are MEM_CLK-domain signals.
    input                       commit_valid,
    input                       commit_buffer_index,

    // ------------------------------------------------------------------------
    // Existing video/read clock interface
    // ------------------------------------------------------------------------
    input                       read_clk,
    input                       read_rst,

    // Common asynchronous clear for the existing dual-clock FIFO.
    // In the final top this can be tied to the common system reset source.
    input                       fifo_rst,

    // Same request/ack contract used by the existing d5_video_timing_data path.
    input                       read_req,
    // D9 real VIDEO_CLK frame boundary, independent of a held read_req.
    input                       video_frame_tick,
    // MEM_CLK admission; never gate an asynchronous request with local state.
    input                       frame_admit_mem,
    output                      read_req_ack,

    // Same FIFO read interface used by video_delay.
    input                       read_en,
    output     [31:0]           read_data,
    output reg                  read_finish,

    // ------------------------------------------------------------------------
    // FIFO observability
    // ------------------------------------------------------------------------
    output                      read_fifo_empty,
    output                      read_fifo_full,
    output     [9:0]            read_fifo_wrusedw,
    output     [9:0]            read_fifo_rdusedw,

    // ------------------------------------------------------------------------
    // D5 debug / verification
    // ------------------------------------------------------------------------
    output                      active_buffer_index,
    output reg                  frame_activation_mem,

    output                      aligner_idle,

    // Strict MEM_CLK-domain idle for SDRAM ownership handoff.
    output                      reader_idle_mem,

    output     [ALIGN_FIFO_ADDR_BITS:0] align_tag_count,
    output     [ALIGN_FIFO_ADDR_BITS:0] align_out_count,

    output                      return_without_command_sticky,
    output                      credit_overflow_sticky,
    output                      frame_start_with_pending_sticky,
    output                      command_order_error_sticky,

    output reg                  read_fifo_underflow_sticky,
    output reg                  rfifo_write_overflow_sticky,

    // D7 diagnostic counters.  They observe existing events only and do not
    // participate in command, FIFO, or video control.
    output reg [31:0]           display_app_rd_cmd_count, // Unused diagnostic outputs may be trimmed by synthesis.
    output reg [31:0]           display_sdr_return_count, // Unused diagnostic outputs may be trimmed by synthesis.
    output reg [31:0]           display_fifo_write_count, // Unused diagnostic outputs may be trimmed by synthesis.
    output reg [31:0]           display_video_pop_count,  // Unused diagnostic outputs may be trimmed by synthesis.
    output reg [31:0]           video_pop_while_empty_count, // Unused diagnostic outputs may be trimmed by synthesis.
    output reg [9:0]            min_fifo_level, // Unused diagnostic outputs may be trimmed by synthesis.
    output reg [9:0]            max_fifo_level  // Unused diagnostic outputs may be trimmed by synthesis.
);

    // ========================================================================
    // READ_CLK request -> MEM_CLK level synchronizer
    // ========================================================================

    reg read_req_meta_mem;
    reg read_req_sync_mem;

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            read_req_meta_mem <= 1'b0;
            read_req_sync_mem <= 1'b0;
        end
        else begin
            read_req_meta_mem <= read_req;
            read_req_sync_mem <= read_req_meta_mem;
        end
    end


    // ========================================================================
    // MEM_CLK request accept / ACK level
    //
    // Do NOT clear the async FIFO at every frame.  Instead, accept a new
    // frame only after both the D2-A return path and rfifo are empty.
    // This is safer than asynchronous mid-stream clearing and guarantees that
    // no stale old-frame word crosses the boundary.
    // ========================================================================

    reg read_req_ack_mem;
    reg display_frame_start_mem;
    reg boundary_toggle_video;
    reg boundary_meta_mem, boundary_sync_mem, boundary_seen_mem;
    reg start_toggle_mem;
    reg start_meta_video, start_sync_video, start_seen_video;
    reg frame_available_mem;
    wire boundary_mem = boundary_sync_mem != boundary_seen_mem;

    always @(posedge read_clk or posedge read_rst) begin
        if (read_rst) boundary_toggle_video <= 0;
        else if (video_frame_tick) boundary_toggle_video <= ~boundary_toggle_video;
    end
    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            boundary_meta_mem<=0; boundary_sync_mem<=0; boundary_seen_mem<=0;
            start_toggle_mem<=0; frame_available_mem<=0;
            frame_activation_mem<=0;
        end else begin
            boundary_meta_mem<=boundary_toggle_video;
            boundary_sync_mem<=boundary_meta_mem;
            boundary_seen_mem<=boundary_sync_mem;
            frame_activation_mem<=display_frame_start_mem;
            if (commit_valid) frame_available_mem<=1;
            if (display_frame_start_mem) start_toggle_mem<=~start_toggle_mem;
        end
    end

    reg fifo_empty_meta_mem;
    reg fifo_empty_sync_mem;

    wire d1_display_active;
    wire d1_frame_read_complete;
    wire d1_command_frame_idle;
    wire aligner_storage_idle;
    reg returned_word_valid;
    reg [31:0] returned_word_r;
    // Include the elastic output register in ownership/drain accounting.
    assign aligner_idle = aligner_storage_idle && !returned_word_valid;
    reg transaction_meta_mem, transaction_sync_mem;
    reg transaction_video;
    reg transport_idle_meta_video, transport_idle_sync_video;
    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin transaction_meta_mem<=0; transaction_sync_mem<=0; end
        else begin
            transaction_meta_mem<=transaction_video;
            transaction_sync_mem<=transaction_meta_mem;
        end
    end
    always @(posedge read_clk or posedge read_rst) begin
        if (read_rst) begin transport_idle_meta_video<=1; transport_idle_sync_video<=1; end
        else begin
            transport_idle_meta_video<=d1_command_frame_idle && aligner_idle;
            transport_idle_sync_video<=transport_idle_meta_video;
        end
    end

    wire frame_start_safe;

    // The command engine is idle only before any frame has started, or after
    // all FRAME_PIXELS read commands of the current frame have been issued.
    assign d1_command_frame_idle =
        !d1_display_active ||
        d1_frame_read_complete;

    // Strict MEM_CLK-domain ownership handoff condition:
    //   1) D1 is no longer issuing the current frame;
    //   2) all delayed SDRAM returns are drained;
    //   3) the asynchronous video FIFO is empty, observed through a 2FF sync.
    assign reader_idle_mem =
        (!FRAME_LOCKED || !transaction_sync_mem) &&
        !display_frame_start_mem &&
        d1_command_frame_idle &&
        aligner_idle &&
        fifo_empty_sync_mem;

    // Never accept a new frame request while the previous command frame is
    // still active, even if the return/FIFO path is momentarily empty.
    assign frame_start_safe =
        Sdr_init_done &&
        reader_idle_mem;

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin

            fifo_empty_meta_mem <= 1'b1;
            fifo_empty_sync_mem <= 1'b1;

            read_req_ack_mem        <= 1'b0;
            display_frame_start_mem <= 1'b0;

        end
        else begin

            fifo_empty_meta_mem <= read_fifo_empty;
            fifo_empty_sync_mem <= fifo_empty_meta_mem;

            display_frame_start_mem <= 1'b0;

            if (FRAME_LOCKED) begin
                // A missed boundary is skipped, never accepted later in DE.
                if (boundary_mem && frame_admit_mem && frame_start_safe &&
                    (frame_available_mem || commit_valid)) begin
                    read_req_ack_mem <= 1;
                    display_frame_start_mem <= 1;
                end else if (!read_req_sync_mem) read_req_ack_mem <= 0;
            end else if (!read_req_ack_mem) begin

                if (read_req_sync_mem && frame_start_safe) begin

                    read_req_ack_mem        <= 1'b1;
                    display_frame_start_mem <= 1'b1;
                end

            end
            else if (!read_req_sync_mem) begin

                read_req_ack_mem <= 1'b0;
            end
        end
    end


    // ========================================================================
    // MEM_CLK ACK -> READ_CLK level synchronizer
    // ========================================================================

    reg read_ack_meta_read;
    reg read_ack_sync_read;
    reg read_ack_sync_read_d;

    always @(posedge read_clk or posedge read_rst) begin
        if (read_rst) begin

            read_ack_meta_read   <= 1'b0;
            read_ack_sync_read   <= 1'b0;
            read_ack_sync_read_d <= 1'b0;

        end
        else begin

            read_ack_meta_read   <= read_req_ack_mem;
            read_ack_sync_read   <= read_ack_meta_read;
            read_ack_sync_read_d <= read_ack_sync_read;
        end
    end

    assign read_req_ack =
        read_ack_sync_read;

    wire read_ack_rise;

    assign read_ack_rise =
        read_ack_sync_read &&
        !read_ack_sync_read_d;


    // ========================================================================
    // D5-D1: fixed sequential DISPLAY buffer read commands
    // ========================================================================

    wire [18:0] d1_word_index;

    wire align_cmd_ready;
    wire align_burst_ready;

    display_buffer_read_ctrl #(
        .TRANSITIONS(TRANSITIONS),
        .ADDR_BITS    (ADDR_BITS),
        .FRAME_PIXELS (FRAME_PIXELS),
        .DISP0_BASE   (DISP0_BASE),
        .DISP1_BASE   (DISP1_BASE),
        .BURST_SIZE   (TRANSPORT_BURST_SIZE)
    ) u_display_buffer_read_ctrl (
        .clk                     (mem_clk),
        .rst                     (mem_rst),
        .transition_mix_enable(transition_mix_enable),
        .transition_old_buffer_index(transition_old_buffer_index),
        .transition_mode(transition_mode),.transition_pos(transition_pos),
        .transition_counter(transition_counter),

        .commit_valid            (commit_valid),
        .commit_buffer_index     (commit_buffer_index),

        .display_frame_start     (display_frame_start_mem),

        .read_request            (align_burst_ready),

        .Sdr_init_done           (Sdr_init_done),
        .Sdr_init_ref_vld        (Sdr_init_ref_vld),
        .Sdr_busy                (Sdr_busy),
        .App_wr_busy             (App_wr_busy),

        .App_rd_en               (App_rd_en),
        .App_rd_addr             (App_rd_addr),

        .active_buffer_index     (active_buffer_index),

        .commit_pending          (),
        .pending_buffer_index    (),

        .buffer_switch_pulse     (),

        .display_active          (d1_display_active),
        .frame_read_complete     (d1_frame_read_complete),

        .word_index              (d1_word_index),
        .frame_last              (),

        .commands_this_frame     (),

        .commit_overwrite_sticky ()
    );


    // ========================================================================
    // D5-D2-A: command tag / delayed SDRAM return alignment
    // ========================================================================

    wire        aligned_pixel_valid;
    wire        aligned_pixel_ready;

    wire [18:0] aligned_pixel_word_index_unused;
    wire [31:0] aligned_pixel_word;

    display_sdram_return_fifo #(
        .FIFO_ADDR_BITS (ALIGN_FIFO_ADDR_BITS),
        .FRAME_PIXELS   (FRAME_PIXELS),
        .BURST_SIZE     (TRANSPORT_BURST_SIZE)
    ) u_display_sdram_return_fifo (
        .clk                              (mem_clk),
        .rst                              (mem_rst),

        .frame_start                      (display_frame_start_mem),

        .cmd_fire                         (App_rd_en),
        .cmd_word_index                   (d1_word_index),
        .cmd_ready                        (align_cmd_ready),
        .burst_ready                      (align_burst_ready),

        .Sdr_rd_en                        (Sdr_rd_en),
        .Sdr_rd_dout                      (Sdr_rd_dout),

        .pixel_valid                      (aligned_pixel_valid),
        .pixel_ready                      (aligned_pixel_ready),

        .pixel_word_index                 (aligned_pixel_word_index_unused),
        .pixel_word                       (aligned_pixel_word),
        .pixel_frame_last                 (),

        .tag_count                        (align_tag_count),
        .out_count                        (align_out_count),

        .aligner_idle                     (aligner_storage_idle),

        .return_without_command_sticky    (return_without_command_sticky),
        .credit_overflow_sticky           (credit_overflow_sticky),
        .frame_start_with_pending_sticky  (frame_start_with_pending_sticky),
        .command_order_error_sticky       (command_order_error_sticky)
    );


    // ========================================================================
    // Existing rfifo_32_32_512
    //
    // D2-A is backpressured whenever the real async FIFO is full.
    // Therefore a returned word is never dropped at this boundary.
    // ========================================================================

    wire rfifo_we;
    wire [8:0] fifo_wrusedw_native, fifo_rdusedw_native;
    assign read_fifo_wrusedw = {1'b0, fifo_wrusedw_native};
    assign read_fifo_rdusedw = {1'b0, fifo_rdusedw_native};
    wire [31:0] fifo_read_data;
    reg video_armed, raster_started, discard_frame;
    reg video_pixel_valid_d;
    reg [18:0] video_words;
    wire video_take = read_en && video_armed && !discard_frame &&
                      !read_fifo_empty && (video_words < FRAME_PIXELS);
    wire fifo_re = FRAME_LOCKED ?
                   ((!read_fifo_empty && discard_frame) || video_take) : read_en;
    assign read_data = (!FRAME_LOCKED || video_pixel_valid_d) ? fifo_read_data : 32'd0;

    // An HDMI raster cannot stall. After a fault discard the rest of the old
    // command frame in the background; re-arm only at a later real boundary.
    // No asynchronous per-frame FIFO reset and no late pixel-zero insertion.
    always @(posedge read_clk or posedge read_rst) begin
        if (read_rst) begin
            start_meta_video<=0; start_sync_video<=0; start_seen_video<=0;
            video_armed<=0; raster_started<=0; discard_frame<=0;
            transaction_video<=0; video_pixel_valid_d<=0; video_words<=0;
        end else begin
            start_meta_video<=start_toggle_mem;
            start_sync_video<=start_meta_video;
            start_seen_video<=start_sync_video;
            video_pixel_valid_d<=video_take;
            if (discard_frame && transport_idle_sync_video && read_fifo_empty) begin
                discard_frame<=0; transaction_video<=0;
            end
            if (read_en) raster_started<=1;
            if (video_frame_tick) begin
                video_armed<=0; raster_started<=0;
                if (transaction_video && video_words!=FRAME_PIXELS)
                    discard_frame<=1;
            end
            if (start_sync_video!=start_seen_video) begin
                video_words<=0; transaction_video<=1;
                if (!raster_started && !read_en) begin
                    video_armed<=1; discard_frame<=0;
                end else begin
                    video_armed<=0; discard_frame<=1;
                end
            end
            if (video_take) begin
                video_words<=video_words+1'b1;
                if (video_words==FRAME_PIXELS-1) transaction_video<=0;
            end
            if (read_en && video_armed && read_fifo_empty) begin
                video_armed<=0; discard_frame<=1;
            end
        end
    end

    // D9 timing cut: asynchronous aligner RAM/mux terminates at a register,
    // not at the physically distant asynchronous video FIFO RAM input.
    // One elastic slot: replace on downstream acceptance, freeze when full.
    assign aligned_pixel_ready = !returned_word_valid || !read_fifo_full;
    assign rfifo_we = returned_word_valid && !read_fifo_full;
    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            returned_word_valid <= 1'b0;
        end else if (aligned_pixel_ready) begin
            returned_word_valid <= aligned_pixel_valid;
        end
    end
    always @(posedge mem_clk) begin
        if (aligned_pixel_valid && aligned_pixel_ready)
            returned_word_r <= aligned_pixel_word;
    end

    rfifo_32_32_512 read_buf (
        .clkr       (read_clk),
        .clkw       (mem_clk),
        .rst        (fifo_rst),

        .we         (rfifo_we),
        .re         (fifo_re),

        .di         (returned_word_r),

        .empty_flag (read_fifo_empty),
        .full_flag  (read_fifo_full),

        .wrusedw    (fifo_wrusedw_native),
        .rdusedw    (fifo_rdusedw_native),

        .dout       (fifo_read_data)
    );


    // ========================================================================
    // Sticky interface fault monitors
    // ========================================================================

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst)
            rfifo_write_overflow_sticky <= 1'b0;
        else if (aligned_pixel_valid && read_fifo_full)
            // This is backpressure, not a lost write.  Record only if an
            // illegal rfifo WE somehow also occurs while full.
            if (rfifo_we)
                rfifo_write_overflow_sticky <= 1'b1;
    end

    always @(posedge read_clk or posedge read_rst) begin
        if (read_rst)
            read_fifo_underflow_sticky <= 1'b0;
        else if (read_en && (!FRAME_LOCKED || video_armed) && read_fifo_empty)
            read_fifo_underflow_sticky <= 1'b1;
    end


    // ========================================================================
    // Successful read counter / compatibility read_finish pulse
    // ========================================================================

    reg [18:0] read_word_count;

    wire read_pop;

    assign read_pop =
        (FRAME_LOCKED ? video_take : read_en) &&
        !read_fifo_empty;

    function [31:0] sat_inc32;
        input [31:0] value;
        begin
            sat_inc32 = (&value) ? value : value + 1'b1;
        end
    endfunction

    // MEM_CLK event accounting.
    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            display_app_rd_cmd_count  <= 32'd0;
            display_sdr_return_count  <= 32'd0;
            display_fifo_write_count  <= 32'd0;
        end
        else begin
            if (App_rd_en)
                display_app_rd_cmd_count <=
                    sat_inc32(display_app_rd_cmd_count);
            if (Sdr_rd_en)
                display_sdr_return_count <=
                    sat_inc32(display_sdr_return_count);
            if (rfifo_we)
                display_fifo_write_count <=
                    sat_inc32(display_fifo_write_count);
        end
    end

    // VIDEO/READ_CLK event accounting and observed FIFO-water marks.
    always @(posedge read_clk or posedge read_rst) begin
        if (read_rst) begin
            display_video_pop_count      <= 32'd0;
            video_pop_while_empty_count  <= 32'd0;
            min_fifo_level               <= 10'h3ff;
            max_fifo_level               <= 10'd0;
        end
        else begin
            if (read_pop)
                display_video_pop_count <=
                    sat_inc32(display_video_pop_count);
            if (read_en && read_fifo_empty)
                video_pop_while_empty_count <=
                    sat_inc32(video_pop_while_empty_count);
            if (read_en) begin
                if (read_fifo_rdusedw < min_fifo_level)
                    min_fifo_level <= read_fifo_rdusedw;
                if (read_fifo_rdusedw > max_fifo_level)
                    max_fifo_level <= read_fifo_rdusedw;
            end
        end
    end

    always @(posedge read_clk or posedge read_rst) begin
        if (read_rst) begin

            read_word_count <= 19'd0;
            read_finish     <= 1'b0;

        end
        else begin

            read_finish <= 1'b0;

            if (read_ack_rise) begin

                read_word_count <= 19'd0;

            end
            else if (read_pop) begin

                if (read_word_count == FRAME_PIXELS - 1) begin

                    read_finish <= 1'b1;
                    read_word_count <= read_word_count;

                end
                else begin

                    read_word_count <= read_word_count + 1'b1;
                end
            end
        end
    end

endmodule
