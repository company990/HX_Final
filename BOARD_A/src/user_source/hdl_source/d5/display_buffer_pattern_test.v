`timescale 1ns/1ps

// ============================================================================
// D6 hardware-only framebuffer diagnostic writer.
//
// The active D5 display contract is one RGB888 pixel per 32-bit SDRAM
// application word, with the HDMI-visible RGB bytes in [31:8].  This module
// fills one complete DISPLAY buffer through the real SDRAM application port,
// pulses commit only after the final write has been accepted, and then stops.
//
// PATTERN_MODE:
//   1: adjacent white/black words -> one-pixel vertical stripes at width 640
//   2: solid green
//   3: optional address signature (not built in the first D6 diagnostic)
//
// Verilog-2001 compatible.
// ============================================================================

module display_buffer_pattern_test #(
    parameter ADDR_BITS           = 21,
    parameter FRAME_WORDS         = 307200,
    parameter TARGET_BASE         = 1355776,
    parameter TARGET_BUFFER_INDEX = 1,
    parameter PATTERN_MODE        = 1
)(
    input                       clk,
    input                       rst,

    input                       enable,
    input                       port_grant,

    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,
    input                       App_rd_busy,

    output                      App_wr_en,
    output     [ADDR_BITS-1:0]  App_wr_addr,
    output     [31:0]           App_wr_din,
    output     [3:0]            App_wr_dm,

    output reg                  write_busy,
    output reg                  fill_done,
    output reg                  commit_valid,
    output                      commit_buffer_index,
    output reg                  completed,

    output     [ADDR_BITS-1:0]  word_index,
    output reg [ADDR_BITS:0]    words_written,

    output reg                  range_error_sticky
);

    localparam ST_WAIT_INIT = 2'd0;
    localparam ST_WRITE     = 2'd1;
    localparam ST_COMMIT    = 2'd2;
    localparam ST_DONE      = 2'd3;

    reg [1:0] state;
    reg [ADDR_BITS-1:0] word_index_r;

    wire controller_can_write;

    function [31:0] diagnostic_word;
        input [ADDR_BITS-1:0] index;
        begin
            if (PATTERN_MODE == 1)
                diagnostic_word = index[0] ? 32'h0000_0000
                                            : 32'hFFFF_FF00;
            else if (PATTERN_MODE == 2)
                diagnostic_word = 32'h00FF_0000;
            else
                diagnostic_word = 32'hA500_0000 |
                    {{(32-ADDR_BITS){1'b0}}, index};
        end
    endfunction

    assign controller_can_write =
        enable &&
        port_grant &&
        Sdr_init_done &&
        !Sdr_init_ref_vld &&
        !Sdr_busy &&
        !App_rd_busy;

    assign App_wr_en =
        (state == ST_WRITE) &&
        controller_can_write;

    assign App_wr_addr =
        TARGET_BASE[ADDR_BITS-1:0] + word_index_r;

    assign App_wr_din =
        diagnostic_word(word_index_r);

    assign App_wr_dm =
        4'b0000;

    assign commit_buffer_index =
        TARGET_BUFFER_INDEX[0];

    assign word_index =
        word_index_r;


    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state              <= ST_WAIT_INIT;
            word_index_r       <= {ADDR_BITS{1'b0}};
            words_written      <= {(ADDR_BITS+1){1'b0}};

            write_busy         <= 1'b0;
            fill_done          <= 1'b0;
            commit_valid       <= 1'b0;
            completed          <= 1'b0;

            range_error_sticky <= 1'b0;
        end
        else begin
            fill_done    <= 1'b0;
            commit_valid <= 1'b0;

            if (
                App_wr_en &&
                (
                    (App_wr_addr < TARGET_BASE[ADDR_BITS-1:0]) ||
                    (App_wr_addr >=
                        TARGET_BASE[ADDR_BITS-1:0] +
                        FRAME_WORDS[ADDR_BITS-1:0])
                )
            )
                range_error_sticky <= 1'b1;

            case (state)
                ST_WAIT_INIT: begin
                    write_busy <= 1'b0;

                    if (enable && Sdr_init_done) begin
                        word_index_r  <= {ADDR_BITS{1'b0}};
                        words_written <= {(ADDR_BITS+1){1'b0}};
                        completed     <= 1'b0;
                        write_busy    <= 1'b1;
                        state         <= ST_WRITE;
                    end
                end

                ST_WRITE: begin
                    write_busy <= 1'b1;

                    if (App_wr_en) begin
                        if (word_index_r == FRAME_WORDS - 1) begin
                            words_written <= FRAME_WORDS;
                            write_busy    <= 1'b0;
                            state         <= ST_COMMIT;
                        end
                        else begin
                            word_index_r  <= word_index_r + 1'b1;
                            words_written <= words_written + 1'b1;
                        end
                    end
                end

                ST_COMMIT: begin
                    write_busy   <= 1'b0;
                    fill_done    <= 1'b1;
                    commit_valid <= 1'b1;
                    completed    <= 1'b1;
                    state        <= ST_DONE;
                end

                default: begin
                    // ST_DONE: the writer owns no cycles and never overwrites.
                    write_busy <= 1'b0;
                    completed  <= 1'b1;
                    state      <= ST_DONE;
                end
            endcase
        end
    end

endmodule
