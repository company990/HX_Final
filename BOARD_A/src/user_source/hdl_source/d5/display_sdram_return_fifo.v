`timescale 1ns/1ps

// ============================================================================
// D5-D2-A
// SDRAM delayed-return aligner + small elastic read FIFO.
//
// This block sits between:
//   display_buffer_read_ctrl  -> SDRAM read commands
// and
//   delayed Sdr_rd_en/Sdr_rd_dout returns -> HDMI/read-FIFO consumer.
//
// Why it exists:
//   SDRAM returns are delayed relative to App_rd_en.  The command-side pixel
//   index must therefore be carried through the delay and paired with the
//   correct returned RGB word.
//
// Architecture:
//   1) Every accepted display read command pushes its 0..307199 word_index
//      into a tag FIFO.
//   2) Every Sdr_rd_en return pops the oldest tag and pushes
//      {tag, Sdr_rd_dout} into an elastic output FIFO.
//   3) pixel_valid/pixel_ready drains the output FIFO toward the HDMI/read path.
//
// Credit rule:
//   tag_count + out_count < FIFO_DEPTH
// is required before another command may be issued.  Therefore even if the
// downstream consumer stalls, every outstanding SDRAM command always has a
// reserved output slot when it returns.
//
// SDRAM is assumed to return reads in command order, matching the existing
// controller interface.
//
// Verilog-2001 compatible.
// ============================================================================

module display_sdram_return_fifo #(
    parameter FIFO_ADDR_BITS = 8,      // room for two 128-command runs
    parameter FRAME_PIXELS   = 307200,
    parameter BURST_SIZE     = 128
)(
    input                   clk,
    input                   rst,

    // Safe display-frame start. Used only for sequence checking.
    input                   frame_start,

    // ------------------------------------------------------------------------
    // Accepted command tracking.
    // cmd_fire must pulse exactly when App_rd_en is accepted.
    // ------------------------------------------------------------------------
    input                   cmd_fire,
    input      [18:0]       cmd_word_index,
    output                  cmd_ready,
    output                  burst_ready,

    // ------------------------------------------------------------------------
    // Delayed SDRAM return.
    // ------------------------------------------------------------------------
    input                   Sdr_rd_en,
    input      [31:0]       Sdr_rd_dout,

    // ------------------------------------------------------------------------
    // Elastic output toward HDMI/read FIFO.
    // ------------------------------------------------------------------------
    output                  pixel_valid,
    input                   pixel_ready,

    output     [18:0]       pixel_word_index,
    output     [31:0]       pixel_word,
    output                  pixel_frame_last,

    // ------------------------------------------------------------------------
    // Debug / verification.
    // ------------------------------------------------------------------------
    output     [FIFO_ADDR_BITS:0] tag_count,
    output     [FIFO_ADDR_BITS:0] out_count,

    output                  aligner_idle,

    output reg              return_without_command_sticky,
    output reg              credit_overflow_sticky,
    output reg              frame_start_with_pending_sticky,
    output reg              command_order_error_sticky
);

    localparam FIFO_DEPTH = (1 << FIFO_ADDR_BITS);

    // ------------------------------------------------------------------------
    // Command-tag FIFO
    // ------------------------------------------------------------------------
    reg [18:0] tag_mem [0:FIFO_DEPTH-1];

    reg [FIFO_ADDR_BITS-1:0] tag_wr_ptr;
    reg [FIFO_ADDR_BITS-1:0] tag_rd_ptr;

    reg [FIFO_ADDR_BITS:0] tag_count_r;

    wire tag_push;
    wire tag_pop;

    wire [18:0] tag_head;

    assign tag_head = tag_mem[tag_rd_ptr];


    // ------------------------------------------------------------------------
    // Elastic returned-data FIFO
    // ------------------------------------------------------------------------
    reg [18:0] out_index_mem [0:FIFO_DEPTH-1];
    reg [31:0] out_data_mem  [0:FIFO_DEPTH-1];

    reg [FIFO_ADDR_BITS-1:0] out_wr_ptr;
    reg [FIFO_ADDR_BITS-1:0] out_rd_ptr;

    reg [FIFO_ADDR_BITS:0] out_count_r;

    wire out_push;
    wire out_pop;


    // ------------------------------------------------------------------------
    // Credit accounting
    //
    // A command consumes one credit immediately.  When its SDRAM return
    // arrives, the credit simply moves from tag FIFO -> output FIFO.
    // The credit is released only when the downstream consumer pops the pixel.
    // ------------------------------------------------------------------------
    wire [FIFO_ADDR_BITS+1:0] total_reserved;

    assign total_reserved =
        tag_count_r +
        out_count_r;

    assign cmd_ready =
        (total_reserved < FIFO_DEPTH);

    // D8-A reserves a complete V1-sized command run before App_rd_en starts,
    // so the command valid cannot be fragmented by downstream backpressure.
    assign burst_ready =
        (FIFO_DEPTH >= BURST_SIZE) &&
        (total_reserved <= (FIFO_DEPTH - BURST_SIZE));

    assign tag_push =
        cmd_fire &&
        cmd_ready;

    assign tag_pop =
        Sdr_rd_en &&
        (tag_count_r != 0);

    assign out_push =
        tag_pop;

    assign pixel_valid =
        (out_count_r != 0);

    assign out_pop =
        pixel_valid &&
        pixel_ready;

    assign pixel_word_index =
        out_index_mem[out_rd_ptr];

    assign pixel_word =
        out_data_mem[out_rd_ptr];

    assign pixel_frame_last =
        pixel_valid &&
        (pixel_word_index == FRAME_PIXELS - 1);

    assign tag_count = tag_count_r;
    assign out_count = out_count_r;

    assign aligner_idle =
        (tag_count_r == 0) &&
        (out_count_r == 0);


    // ------------------------------------------------------------------------
    // Expected command index, reset at each safe frame start.
    // ------------------------------------------------------------------------
    reg [18:0] expected_cmd_index;


    always @(posedge clk or posedge rst) begin
        if (rst) begin

            tag_wr_ptr  <= {FIFO_ADDR_BITS{1'b0}};
            tag_rd_ptr  <= {FIFO_ADDR_BITS{1'b0}};
            tag_count_r <= {(FIFO_ADDR_BITS+1){1'b0}};

            out_wr_ptr  <= {FIFO_ADDR_BITS{1'b0}};
            out_rd_ptr  <= {FIFO_ADDR_BITS{1'b0}};
            out_count_r <= {(FIFO_ADDR_BITS+1){1'b0}};

            expected_cmd_index <= 19'd0;

            return_without_command_sticky   <= 1'b0;
            credit_overflow_sticky          <= 1'b0;
            frame_start_with_pending_sticky <= 1'b0;
            command_order_error_sticky      <= 1'b0;

        end
        else begin

            // ---------------------------------------------------------------
            // Frame-boundary sequence checker.
            // No old-frame command/return may remain when a new display frame
            // starts in this isolated D5-D2 contract.
            // ---------------------------------------------------------------
            if (frame_start) begin

                expected_cmd_index <= 19'd0;

                if (!aligner_idle)
                    frame_start_with_pending_sticky <= 1'b1;
            end


            // ---------------------------------------------------------------
            // Illegal command presented without credit.
            // The producer must gate App_rd_en with cmd_ready.
            // ---------------------------------------------------------------
            if (cmd_fire && !cmd_ready)
                credit_overflow_sticky <= 1'b1;


            // ---------------------------------------------------------------
            // Tag FIFO write + command sequence check.
            // ---------------------------------------------------------------
            if (tag_push) begin

                tag_mem[tag_wr_ptr] <= cmd_word_index;
                tag_wr_ptr <= tag_wr_ptr + 1'b1;

                if (cmd_word_index != expected_cmd_index)
                    command_order_error_sticky <= 1'b1;

                if (cmd_word_index == FRAME_PIXELS - 1)
                    expected_cmd_index <= expected_cmd_index;
                else
                    expected_cmd_index <= expected_cmd_index + 1'b1;
            end


            // ---------------------------------------------------------------
            // SDRAM return without a matching outstanding command.
            // ---------------------------------------------------------------
            if (Sdr_rd_en && (tag_count_r == 0))
                return_without_command_sticky <= 1'b1;


            // ---------------------------------------------------------------
            // A valid SDRAM return transfers the oldest tag into the output
            // FIFO together with the returned word.
            // ---------------------------------------------------------------
            if (out_push) begin

                out_index_mem[out_wr_ptr] <= tag_head;
                out_data_mem[out_wr_ptr]  <= Sdr_rd_dout;

                out_wr_ptr <= out_wr_ptr + 1'b1;

                tag_rd_ptr <= tag_rd_ptr + 1'b1;
            end


            // ---------------------------------------------------------------
            // Downstream pop.
            // ---------------------------------------------------------------
            if (out_pop)
                out_rd_ptr <= out_rd_ptr + 1'b1;


            // ---------------------------------------------------------------
            // Tag count update.
            // ---------------------------------------------------------------
            case ({tag_push, tag_pop})

                2'b10:
                    tag_count_r <= tag_count_r + 1'b1;

                2'b01:
                    tag_count_r <= tag_count_r - 1'b1;

                default:
                    tag_count_r <= tag_count_r;

            endcase


            // ---------------------------------------------------------------
            // Output FIFO count update.
            // ---------------------------------------------------------------
            case ({out_push, out_pop})

                2'b10:
                    out_count_r <= out_count_r + 1'b1;

                2'b01:
                    out_count_r <= out_count_r - 1'b1;

                default:
                    out_count_r <= out_count_r;

            endcase
        end
    end

endmodule
