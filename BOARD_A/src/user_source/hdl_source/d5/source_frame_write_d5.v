`timescale 1ns/1ps

module source_frame_write_d5 #(
    parameter MEM_DATA_BITS = 32,
    parameter ADDR_BITS     = 21,
    parameter BURST_BITS    = 9,
    parameter BURST_SIZE    = 128,
    parameter SOURCE_BASE   = 0,
    parameter SOURCE_WORDS  = 1048576
)(
    output wire transport_busy,
    input mem_clk,
    input mem_rst,
    input Sdr_init_done,
    input Sdr_init_ref_vld,
    input Sdr_busy,
    input read_port_busy,

    output App_wr_en,
    output [ADDR_BITS-1:0] App_wr_addr,
    output [MEM_DATA_BITS-1:0] App_wr_din,
    output [3:0] App_wr_dm,

    input write_clk,
    input write_rst,
    input write_fifo_rst,

    input write_req,
    output write_req_ack,
    input [ADDR_BITS-1:0] write_len,
    input write_en,
    input [MEM_DATA_BITS-1:0] write_data,

    output write_finish,
    output reg source_write_active,

    output write_fifo_full,
    output write_fifo_empty,
    output reg write_fifo_overflow_sticky,
    output reg write_fifo_underflow_sticky,
    output reg source_range_error_sticky
);

    wire [9:0] rdusedw;
    wire [8:0] rdusedw_native;
    // The generated FIFO exports ADDR_WIDTH_R=9 occupancy bits. Explicitly
    // extend at the adapter; never leave the comparator's high bit undriven.
    assign rdusedw = {1'b0, rdusedw_native};
    wire frame_fifo_aclr;
    wire write_req_ack_mem;
    wire frame_writer_busy;
    assign transport_busy = frame_writer_busy;
    wire fifo_rst_i;

    reg ack_meta, ack_sync;
    reg req_meta_mem, req_sync_mem;

    assign App_wr_dm = 4'b0000;
    assign fifo_rst_i = frame_fifo_aclr | write_fifo_rst;
    assign write_req_ack = ack_sync;

    always @(posedge write_clk or posedge write_rst) begin
        if (write_rst) begin
            ack_meta <= 1'b0;
            ack_sync <= 1'b0;
            write_fifo_overflow_sticky <= 1'b0;
        end else begin
            ack_meta <= write_req_ack_mem;
            ack_sync <= ack_meta;
            if (write_en && write_fifo_full)
                write_fifo_overflow_sticky <= 1'b1;
        end
    end

    wfifo_32_32_512 write_buf (
        .clkr(mem_clk),
        .clkw(write_clk),
        .rst(fifo_rst_i),
        .we(write_en),
        .re(App_wr_en),
        .di(write_data),
        .empty_flag(write_fifo_empty),
        .full_flag(write_fifo_full),
        .wrusedw(),
        .rdusedw(rdusedw_native),
        .dout(App_wr_din)
    );

    d5_frame_fifo_write #(
        .MEM_DATA_BITS(MEM_DATA_BITS),
        .ADDR_BITS(ADDR_BITS),
        .BURST_BITS(BURST_BITS),
        .BURST_SIZE(BURST_SIZE),
        .WRITE_V_FLIP(0),
        .FRAME_WIDTH(640),
        .FRAME_HEIGHT(480)
    ) u_frame_fifo_write (
        .rst(mem_rst),
        .mem_clk(mem_clk),
        .Sdr_init_done(Sdr_init_done),
        .Sdr_init_ref_vld(Sdr_init_ref_vld),
        .Sdr_busy(Sdr_busy),
        .App_rd_busy(read_port_busy),
        .O_wr_busy(frame_writer_busy),
        .App_wr_en(App_wr_en),
        .App_wr_addr(App_wr_addr),
        .write_req(write_req),
        .write_req_ack(write_req_ack_mem),
        .write_finish(write_finish),
        .write_addr_0(SOURCE_BASE[ADDR_BITS-1:0]),
        .write_addr_1(SOURCE_BASE[ADDR_BITS-1:0]),
        .write_addr_2(SOURCE_BASE[ADDR_BITS-1:0]),
        .write_addr_3(SOURCE_BASE[ADDR_BITS-1:0]),
        .write_addr_index(2'b00),
        .write_len(write_len),
        .fifo_aclr(frame_fifo_aclr),
        .rdusedw(rdusedw)
    );

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            req_meta_mem <= 1'b0;
            req_sync_mem <= 1'b0;
            source_write_active <= 1'b0;
            write_fifo_underflow_sticky <= 1'b0;
            source_range_error_sticky <= 1'b0;
        end else begin
            req_meta_mem <= write_req;
            req_sync_mem <= req_meta_mem;

            if (req_sync_mem)
                source_write_active <= 1'b1;
            if (write_finish)
                source_write_active <= 1'b0;

            if (App_wr_en && write_fifo_empty)
                write_fifo_underflow_sticky <= 1'b1;

            if (App_wr_en &&
                ((App_wr_addr < SOURCE_BASE) ||
                 (App_wr_addr >= SOURCE_BASE + SOURCE_WORDS)))
                source_range_error_sticky <= 1'b1;
        end
    end

endmodule
