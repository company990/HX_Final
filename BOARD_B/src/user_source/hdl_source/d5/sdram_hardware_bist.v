`timescale 1ns/1ps

// D7 physical-SDRAM write/read/compare BIST.
// All command generation, return capture, and comparison are in mem_clk.
// The read side deliberately permits exactly one outstanding command.
module sdram_hardware_bist #(
    parameter ADDR_BITS       = 21,
    parameter FRAME_WORDS     = 307200,
    parameter TARGET_BASE     = 1355776,
    parameter RUN_ALL_TESTS   = 1,
    parameter SETTLE_CYCLES   = 32,
    parameter TIMEOUT_CYCLES  = 12500000
)(
    input                       clk,
    input                       rst,
    input                       enable,
    input                       port_grant,

    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,

    output                      App_wr_en,
    output     [ADDR_BITS-1:0]  App_wr_addr,
    output     [31:0]           App_wr_din,
    output     [3:0]            App_wr_dm,

    output                      App_rd_en,
    output     [ADDR_BITS-1:0]  App_rd_addr,
    input                       Sdr_rd_en,
    input      [31:0]           Sdr_rd_dout,

    output reg                  running,
    output reg                  completed,
    output reg                  all_tests_pass,
    output reg                  data_mismatch,
    output reg                  timeout_error,
    output reg                  sequence_error,
    output reg                  release_port,

    output reg [2:0]            test_id,
    output reg [4:0]            state_code,

    output reg [31:0]           write_attempt_count, // synthesis keep
    output reg [31:0]           write_accept_count,  // synthesis keep
    output reg [31:0]           read_command_count,  // synthesis keep
    output reg [31:0]           read_accept_count,   // synthesis keep
    output reg [31:0]           read_return_count,   // synthesis keep
    output reg [31:0]           compare_count,       // synthesis keep
    output reg [31:0]           mismatch_count,      // synthesis keep
    output reg [31:0]           timeout_count,       // synthesis keep

    output reg                  first_error_valid,
    output reg [2:0]            first_error_test,
    output reg [ADDR_BITS-1:0]  first_error_addr,
    output reg [31:0]           first_error_expected,
    output reg [31:0]           first_error_actual,
    output reg [4:0]            first_timeout_state,
    output reg [31:0]           error_xor_or,        // synthesis keep

    output reg [31:0]           even_addr_error_count, // synthesis keep
    output reg [31:0]           odd_addr_error_count,  // synthesis keep
    output reg [31:0]           addr_bit0_error_count, // synthesis keep
    output reg [31:0]           addr_bit1_error_count, // synthesis keep
    output reg [31:0]           addr_bit2_error_count, // synthesis keep
    output reg [31:0]           addr_bit3_error_count  // synthesis keep
);

    localparam ST_WAIT_INIT  = 5'd0;
    localparam ST_PREP_TEST  = 5'd1;
    localparam ST_WRITE      = 5'd2;
    localparam ST_SETTLE     = 5'd3;
    localparam ST_READ_ISSUE = 5'd4;
    localparam ST_READ_WAIT  = 5'd5;
    localparam ST_PASS       = 5'd8;
    localparam ST_FAIL_DATA  = 5'd9;
    localparam ST_FAIL_TO    = 5'd10;
    localparam ST_FAIL_SEQ   = 5'd11;
    localparam [31:0] TOTAL_WORDS =
        RUN_ALL_TESTS ? (FRAME_WORDS * 5) : FRAME_WORDS;

    reg [ADDR_BITS-1:0] word_index_r;
    reg [ADDR_BITS-1:0] pending_index_r;
    reg [31:0] pending_expected_r;
    reg [31:0] wait_count_r;
    reg [31:0] settle_count_r;

    wire command_eligible;
    wire mismatch_now;
    wire unexpected_return;
    wire [2:0] last_test_id;

    assign command_eligible =
        enable &&
        port_grant &&
        Sdr_init_done &&
        !Sdr_init_ref_vld &&
        !Sdr_busy;

    function [31:0] expected_word;
        input [2:0] test_sel;
        input [ADDR_BITS-1:0] index;
        begin
            case (test_sel)
                3'd0: expected_word = 32'h00FF0000;
                3'd1: expected_word = 32'h00000000;
                3'd2: expected_word = 32'hFFFFFFFF;
                3'd3: expected_word = index[0] ? 32'h55555555 :
                                                  32'hAAAAAAAA;
                default: expected_word =
                    32'hA5000000 ^ {{(32-ADDR_BITS){1'b0}}, index};
            endcase
        end
    endfunction

    function [31:0] sat_inc;
        input [31:0] value;
        begin
            sat_inc = (&value) ? value : value + 1'b1;
        end
    endfunction

    assign last_test_id = RUN_ALL_TESTS ? 3'd4 : 3'd0;

    assign App_wr_en =
        (state_code == ST_WRITE) && command_eligible;
    assign App_wr_addr =
        TARGET_BASE[ADDR_BITS-1:0] + word_index_r;
    assign App_wr_din = expected_word(test_id, word_index_r);
    assign App_wr_dm = 4'b0000;

    assign App_rd_en =
        (state_code == ST_READ_ISSUE) && command_eligible;
    assign App_rd_addr =
        TARGET_BASE[ADDR_BITS-1:0] + word_index_r;

    assign mismatch_now =
        Sdr_rd_en && (Sdr_rd_dout != pending_expected_r);
    assign unexpected_return =
        Sdr_rd_en && (state_code != ST_READ_WAIT);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            running                  <= 1'b0;
            completed                <= 1'b0;
            all_tests_pass           <= 1'b0;
            data_mismatch            <= 1'b0;
            timeout_error            <= 1'b0;
            sequence_error           <= 1'b0;
            release_port             <= 1'b0;
            test_id                  <= 3'd0;
            state_code               <= ST_WAIT_INIT;
            word_index_r             <= {ADDR_BITS{1'b0}};
            pending_index_r          <= {ADDR_BITS{1'b0}};
            pending_expected_r       <= 32'd0;
            wait_count_r             <= 32'd0;
            settle_count_r           <= 32'd0;
            write_attempt_count      <= 32'd0;
            write_accept_count       <= 32'd0;
            read_command_count       <= 32'd0;
            read_accept_count        <= 32'd0;
            read_return_count        <= 32'd0;
            compare_count            <= 32'd0;
            mismatch_count           <= 32'd0;
            timeout_count            <= 32'd0;
            first_error_valid        <= 1'b0;
            first_error_test         <= 3'd0;
            first_error_addr         <= {ADDR_BITS{1'b0}};
            first_error_expected     <= 32'd0;
            first_error_actual       <= 32'd0;
            first_timeout_state      <= 5'd0;
            error_xor_or             <= 32'd0;
            even_addr_error_count    <= 32'd0;
            odd_addr_error_count     <= 32'd0;
            addr_bit0_error_count    <= 32'd0;
            addr_bit1_error_count    <= 32'd0;
            addr_bit2_error_count    <= 32'd0;
            addr_bit3_error_count    <= 32'd0;
        end
        else if (!enable) begin
            running      <= 1'b0;
            completed    <= 1'b0;
            release_port <= 1'b0;
            state_code   <= ST_WAIT_INIT;
        end
        else if (unexpected_return && running) begin
            // A return without the single outstanding command is decisive.
            sequence_error <= 1'b1;
            completed      <= 1'b1;
            running        <= 1'b0;
            release_port   <= 1'b0;
            state_code     <= ST_FAIL_SEQ;
            if (!first_error_valid) begin
                first_error_valid    <= 1'b1;
                first_error_test     <= test_id;
                first_error_addr     <= App_rd_addr;
                first_error_expected <= 32'd0;
                first_error_actual   <= Sdr_rd_dout;
            end
        end
        else begin
            case (state_code)
                ST_WAIT_INIT: begin
                    if (Sdr_init_done && port_grant) begin
                        running       <= 1'b1;
                        completed     <= 1'b0;
                        all_tests_pass<= 1'b0;
                        release_port  <= 1'b0;
                        test_id       <= 3'd0;
                        state_code    <= ST_PREP_TEST;
                    end
                end

                ST_PREP_TEST: begin
                    word_index_r   <= {ADDR_BITS{1'b0}};
                    wait_count_r   <= 32'd0;
                    settle_count_r <= 32'd0;
                    state_code     <= ST_WRITE;
                end

                ST_WRITE: begin
                    write_attempt_count <= sat_inc(write_attempt_count);
                    if (App_wr_en) begin
                        write_accept_count <= sat_inc(write_accept_count);
                        wait_count_r <= 32'd0;
                        if (word_index_r == FRAME_WORDS - 1) begin
                            settle_count_r <= 32'd0;
                            state_code <= ST_SETTLE;
                        end
                        else begin
                            word_index_r <= word_index_r + 1'b1;
                        end
                    end
                    else if (wait_count_r >= TIMEOUT_CYCLES - 1) begin
                        timeout_count       <= sat_inc(timeout_count);
                        timeout_error       <= 1'b1;
                        first_timeout_state <= ST_WRITE;
                        completed           <= 1'b1;
                        running             <= 1'b0;
                        state_code          <= ST_FAIL_TO;
                    end
                    else begin
                        wait_count_r <= wait_count_r + 1'b1;
                    end
                end

                ST_SETTLE: begin
                    // Require a contiguous controller-idle window after the
                    // final accepted application write strobe.
                    if (!Sdr_init_done || Sdr_init_ref_vld || Sdr_busy) begin
                        settle_count_r <= 32'd0;
                        if (wait_count_r >= TIMEOUT_CYCLES - 1) begin
                            timeout_count       <= sat_inc(timeout_count);
                            timeout_error       <= 1'b1;
                            first_timeout_state <= ST_SETTLE;
                            completed           <= 1'b1;
                            running             <= 1'b0;
                            state_code          <= ST_FAIL_TO;
                        end
                        else begin
                            wait_count_r <= wait_count_r + 1'b1;
                        end
                    end
                    else if (settle_count_r >= SETTLE_CYCLES - 1) begin
                        word_index_r <= {ADDR_BITS{1'b0}};
                        wait_count_r <= 32'd0;
                        state_code   <= ST_READ_ISSUE;
                    end
                    else begin
                        settle_count_r <= settle_count_r + 1'b1;
                        wait_count_r   <= 32'd0;
                    end
                end

                ST_READ_ISSUE: begin
                    if (App_rd_en) begin
                        // There is no separate controller ready/ack on this
                        // interface; the eligible command strobe is the only
                        // observable accept event.
                        read_command_count  <= sat_inc(read_command_count);
                        read_accept_count   <= sat_inc(read_accept_count);
                        pending_index_r     <= word_index_r;
                        pending_expected_r  <= expected_word(test_id,
                                                             word_index_r);
                        wait_count_r        <= 32'd0;
                        state_code          <= ST_READ_WAIT;
                    end
                    else if (wait_count_r >= TIMEOUT_CYCLES - 1) begin
                        timeout_count       <= sat_inc(timeout_count);
                        timeout_error       <= 1'b1;
                        first_timeout_state <= ST_READ_ISSUE;
                        completed           <= 1'b1;
                        running             <= 1'b0;
                        state_code          <= ST_FAIL_TO;
                    end
                    else begin
                        wait_count_r <= wait_count_r + 1'b1;
                    end
                end

                ST_READ_WAIT: begin
                    if (Sdr_rd_en) begin
                        read_return_count <= sat_inc(read_return_count);
                        compare_count     <= sat_inc(compare_count);
                        wait_count_r      <= 32'd0;

                        if (mismatch_now) begin
                            data_mismatch <= 1'b1;
                            mismatch_count <= sat_inc(mismatch_count);
                            error_xor_or <= error_xor_or |
                                            (pending_expected_r ^
                                             Sdr_rd_dout);
                            if (pending_index_r[0])
                                odd_addr_error_count <=
                                    sat_inc(odd_addr_error_count);
                            else
                                even_addr_error_count <=
                                    sat_inc(even_addr_error_count);
                            if (pending_index_r[0])
                                addr_bit0_error_count <=
                                    sat_inc(addr_bit0_error_count);
                            if (pending_index_r[1])
                                addr_bit1_error_count <=
                                    sat_inc(addr_bit1_error_count);
                            if (pending_index_r[2])
                                addr_bit2_error_count <=
                                    sat_inc(addr_bit2_error_count);
                            if (pending_index_r[3])
                                addr_bit3_error_count <=
                                    sat_inc(addr_bit3_error_count);
                            if (!first_error_valid) begin
                                first_error_valid    <= 1'b1;
                                first_error_test     <= test_id;
                                first_error_addr     <=
                                    TARGET_BASE[ADDR_BITS-1:0] +
                                    pending_index_r;
                                first_error_expected <= pending_expected_r;
                                first_error_actual   <= Sdr_rd_dout;
                            end
                        end

                        if (pending_index_r == FRAME_WORDS - 1) begin
                            if (test_id == last_test_id) begin
                                completed <= 1'b1;
                                running   <= 1'b0;
                                if (
                                    (write_accept_count != TOTAL_WORDS) ||
                                    (read_command_count != TOTAL_WORDS) ||
                                    (read_accept_count != TOTAL_WORDS) ||
                                    (read_return_count != TOTAL_WORDS - 1) ||
                                    (compare_count != TOTAL_WORDS - 1)
                                ) begin
                                    sequence_error <= 1'b1;
                                    all_tests_pass <= 1'b0;
                                    state_code <= ST_FAIL_SEQ;
                                end
                                else if (data_mismatch || mismatch_now) begin
                                    all_tests_pass <= 1'b0;
                                    state_code <= ST_FAIL_DATA;
                                end
                                else begin
                                    all_tests_pass <= 1'b1;
                                    release_port <= !RUN_ALL_TESTS;
                                    state_code <= ST_PASS;
                                end
                            end
                            else begin
                                test_id <= test_id + 1'b1;
                                state_code <= ST_PREP_TEST;
                            end
                        end
                        else begin
                            word_index_r <= pending_index_r + 1'b1;
                            state_code <= ST_READ_ISSUE;
                        end
                    end
                    else if (wait_count_r >= TIMEOUT_CYCLES - 1) begin
                        timeout_count       <= sat_inc(timeout_count);
                        timeout_error       <= 1'b1;
                        first_timeout_state <= ST_READ_WAIT;
                        completed           <= 1'b1;
                        running             <= 1'b0;
                        state_code          <= ST_FAIL_TO;
                    end
                    else begin
                        wait_count_r <= wait_count_r + 1'b1;
                    end
                end

                default: begin
                    // PASS/FAIL states are terminal and keep all evidence.
                    running <= 1'b0;
                end
            endcase
        end
    end

endmodule
