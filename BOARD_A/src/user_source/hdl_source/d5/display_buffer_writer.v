`timescale 1ns/1ps

// ============================================================================
// D5-C3-A
// Fixed 640x480 DISPLAY buffer writer.
//
// Purpose:
//   Accept the final 24-bit destination stream produced by the Bilinear scaler
//   and write it into one of two fixed 640x480 SDRAM display frame buffers.
//
// SDRAM layout used by D5:
//   SOURCE_BUFFER : 0       .. 1048575
//   DISPLAY_BUF0  : 1048576 .. 1355775   (307200 words)
//   DISPLAY_BUF1  : 1355776 .. 1662975   (307200 words)
//
// Important architectural property:
//   The scaler already outputs dst_addr = 0..307199 in raster order.
//   Therefore this writer never performs adaptive/random display addressing.
//   It writes one complete, already-scaled 640x480 frame sequentially.
//
// D8-A transport policy:
//   - Scaler pixels enter the same wfifo_32_32_512 + d5_frame_fifo_write transport
//     used by the hardware-working V1 project.
//   - SDRAM writes are emitted in uninterrupted 128-word application runs;
//     final partial-burst support remains inside d5_frame_fifo_write.
//   - Sdr_busy/Sdr_init_ref_vld are not interpreted as per-word ready.
//   - The CURRENT low-24-bit display word convention is intentionally retained
//     for the transport-only D8-A hardware experiment.
//
// Verilog-2001 compatible.
// ============================================================================

module display_buffer_writer #(
    parameter ADDR_BITS      = 21,
    parameter FRAME_PIXELS   = 307200,
    parameter DISP0_BASE     = 1048576,
    parameter DISP1_BASE     = 1355776
)(
    input                       clk,
    input                       rst,

    // ------------------------------------------------------------------------
    // Start one complete destination-frame write.
    // buffer_index is latched on frame_start.
    // ------------------------------------------------------------------------
    input                       frame_start,
    input                       buffer_index,

    output reg                  write_busy,
    output reg                  write_done,
    output reg                  completed_buffer_index,

    // V1 local read/write exclusion: asserted for an active or immediately
    // pending SDRAM write burst, not for the whole frame transaction.
    output                      transport_busy,

    // ------------------------------------------------------------------------
    // Scaler destination stream.
    // pixel_addr is expected to be exactly 0..307199.
    // ------------------------------------------------------------------------
    input                       pixel_valid,
    output                      pixel_ready,

    input      [18:0]           pixel_addr,
    input      [23:0]           pixel_data,

    // ------------------------------------------------------------------------
    // Existing SDRAM user interface.
    // ------------------------------------------------------------------------
    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,
    input                       App_rd_busy,

    output                      App_wr_en,
    output     [ADDR_BITS-1:0]  App_wr_addr,
    output     [31:0]           App_wr_din,
    output     [3:0]            App_wr_dm,

    // ------------------------------------------------------------------------
    // Debug / verification.
    // ------------------------------------------------------------------------
    output reg [18:0]           words_written,
    output reg                  address_order_error_sticky,
    output reg                  start_while_busy_sticky
);

    localparam WR_IDLE       = 3'd0;
    localparam WR_REQUEST    = 3'd1;
    localparam WR_WAIT_CLEAR = 3'd2;
    localparam WR_STREAM     = 3'd3;
    localparam WR_DRAIN      = 3'd4;

    reg [2:0] state;

    reg        active_buffer_index;
    reg [18:0] expected_pixel_addr;
    reg        write_req_r;
    reg [2:0]  fifo_release_count;

    wire       pixel_fire;
    wire       fifo_aclr;
    wire       fifo_full;
    wire       fifo_empty;
    wire [8:0] fifo_rdusedw;
    wire [8:0] fifo_wrusedw;
    reg        fifo_room_r;
    wire       write_req_ack_i;
    wire       write_finish_i;
    wire       transport_busy_i;

    wire [31:0] fifo_write_word;

    // D9 canonical application word: [31:8]=RGB888, [7:0]=zero.
    assign fifo_write_word = {pixel_data, 8'h00};

    assign pixel_ready =
        write_busy &&
        (state == WR_STREAM) &&
        fifo_room_r;

    // Break the vendor FIFO pointer/subtract/full -> scaler CE chain.
    // Native FIFO full threshold is 511. Reserve two slots for this one-cycle
    // delayed credit decision; at most one word is accepted per cycle.
    // Read-pointer synchronization is conservative (overstates occupancy).
    always @(posedge clk or posedge rst) begin
        if (rst) fifo_room_r <= 1'b0;
        else if (fifo_aclr) fifo_room_r <= 1'b0;
        else fifo_room_r <= (fifo_wrusedw < 9'd509);
    end

    assign pixel_fire = pixel_valid && pixel_ready;
    assign transport_busy = transport_busy_i;

    wfifo_32_32_512 u_d8_display_write_fifo (
        .clkr       (clk),
        .clkw       (clk),
        .rst        (fifo_aclr),
        .we         (pixel_fire),
        .re         (App_wr_en),
        .di         (fifo_write_word),
        .empty_flag (fifo_empty),
        .full_flag  (fifo_full),
        .valid      (),
        .afull      (),
        .aempty     (),
        .wrusedw    (fifo_wrusedw),
        .rdusedw    (fifo_rdusedw),
        .dout       (App_wr_din)
    );

    // Reuse the V1-proven application-port write transport verbatim.  The
    // timing-optimized CURRENT revision preserves its external burst protocol.
    d5_frame_fifo_write #(
        .MEM_DATA_BITS (32),
        .ADDR_BITS     (ADDR_BITS),
        .BURST_BITS    (9),
        .BURST_SIZE    (128),
        .WRITE_V_FLIP  (0),
        .FRAME_WIDTH   (640),
        .FRAME_HEIGHT  (480)
    ) u_d8_v1_frame_fifo_write (
        .rst              (rst),
        .mem_clk          (clk),
        .Sdr_init_done    (Sdr_init_done),
        .Sdr_init_ref_vld (Sdr_init_ref_vld),
        .Sdr_busy         (Sdr_busy),
        .App_rd_busy      (App_rd_busy),
        .O_wr_busy        (transport_busy_i),
        .App_wr_en        (App_wr_en),
        .App_wr_addr      (App_wr_addr),
        .write_req        (write_req_r),
        .write_req_ack    (write_req_ack_i),
        .write_finish     (write_finish_i),
        .write_addr_0     (DISP0_BASE[ADDR_BITS-1:0]),
        .write_addr_1     (DISP1_BASE[ADDR_BITS-1:0]),
        .write_addr_2     ({ADDR_BITS{1'b0}}),
        .write_addr_3     ({ADDR_BITS{1'b0}}),
        .write_addr_index ({1'b0, active_buffer_index}),
        .write_len        (FRAME_PIXELS[ADDR_BITS-1:0]),
        .fifo_aclr        (fifo_aclr),
        .rdusedw          ({1'b0, fifo_rdusedw})
    );

    assign App_wr_dm = 4'b0000;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= WR_IDLE;

            write_busy <= 1'b0;
            write_done <= 1'b0;
            write_req_r <= 1'b0;
            fifo_release_count <= 3'd0;

            active_buffer_index    <= 1'b0;
            completed_buffer_index <= 1'b0;

            expected_pixel_addr <= 19'd0;
            words_written       <= 19'd0;

            address_order_error_sticky <= 1'b0;
            start_while_busy_sticky    <= 1'b0;
        end
        else begin
            write_done <= 1'b0;

            if (App_wr_en)
                words_written <= words_written + 1'b1;

            if (frame_start && write_busy)
                start_while_busy_sticky <= 1'b1;

            case (state)
                WR_IDLE: begin
                    write_req_r <= 1'b0;

                    if (frame_start && !write_busy) begin
                        active_buffer_index <= buffer_index;
                        expected_pixel_addr <= 19'd0;
                        words_written       <= 19'd0;
                        write_busy          <= 1'b1;
                        write_req_r         <= 1'b1;
                        fifo_release_count  <= 3'd0;
                        state               <= WR_REQUEST;
                    end
                end

                WR_REQUEST: begin
                    if (write_req_ack_i) begin
                        write_req_r <= 1'b0;
                        state <= WR_WAIT_CLEAR;
                    end
                end

                WR_WAIT_CLEAR: begin
                    // The vendor FIFO releases each clock-domain reset
                    // through two flops. Do not acknowledge source pixels
                    // until that internal reset has cleared.
                    if (write_req_ack_i || fifo_aclr)
                        fifo_release_count <= 3'd0;
                    else if (fifo_release_count == 3'd3)
                        state <= WR_STREAM;
                    else
                        fifo_release_count <= fifo_release_count + 1'b1;
                end

                WR_STREAM: begin
                    if (pixel_fire) begin
                        if (pixel_addr != expected_pixel_addr)
                            address_order_error_sticky <= 1'b1;

                        if (expected_pixel_addr == FRAME_PIXELS - 1) begin
                            state <= WR_DRAIN;
                        end
                        else begin
                            expected_pixel_addr <= expected_pixel_addr + 1'b1;
                        end
                    end
                end

                WR_DRAIN: begin
                    if (write_finish_i) begin
                        completed_buffer_index <= active_buffer_index;
                        write_busy <= 1'b0;
                        write_done <= 1'b1;
                        state <= WR_IDLE;
                    end
                end

                default: begin
                    write_req_r <= 1'b0;
                    write_busy  <= 1'b0;
                    state       <= WR_IDLE;
                end
            endcase
        end
    end

endmodule
