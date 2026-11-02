`timescale 1ns/1ps

// ============================================================================
// D5-D3-A
// Final-project display-read adapter.
//
// This is the replacement boundary for the OLD frame_read_write READ side.
// It deliberately keeps the top-level video-facing signals:
//
//      read_req / read_req_ack
//      read_en
//      read_data
//      read_finish
//
// while internally using the verified D5 path:
//
//      DISPLAY_BUF0/1
//          -> display_buffer_read_ctrl
//          -> SDRAM delayed returns
//          -> display_sdram_return_fifo
//          -> existing rfifo_32_32_512
//          -> video_clk domain
//
// The old adaptive read-time blocks are NOT instantiated here:
//      adaptive_sdram_read_addr_gen
//      adaptive_sdram_return_mask
//      dummy black-bar reads
//
// Black borders already exist physically inside the fixed 640x480 DISPLAY
// buffers, so the HDMI side only reads sequential addresses.
//
// Verilog-2001 compatible.
// ============================================================================

module frame_display_read_d5 #(
    parameter ADDR_BITS            = 21,
    parameter FRAME_PIXELS         = 307200,
    parameter DISP0_BASE           = 1048576,
    parameter DISP1_BASE           = 1355776,
    parameter ALIGN_FIFO_ADDR_BITS = 8,
    parameter TRANSPORT_BURST_SIZE = 128,
    parameter FRAME_LOCKED = 0
)(
    // Memory domain
    input                       mem_clk,
    input                       mem_rst,

    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,
    input                       App_wr_busy,

    output                      App_rd_en,
    output     [ADDR_BITS-1:0]  App_rd_addr,

    input                       Sdr_rd_en,
    input      [31:0]           Sdr_rd_dout,

    // Completed target-frame commit from bilinear_frame_builder (mem_clk).
    input                       commit_valid,
    input                       commit_buffer_index,

    // Existing video/read side
    input                       read_clk,
    input                       read_rst,

    // Asynchronous reset for the existing dual-clock rfifo IP.
    input                       read_fifo_rst,

    input                       read_req,
    input                       video_frame_tick,
    input                       frame_admit_mem,
    output                      read_req_ack,

    input                       read_en,
    output     [31:0]           read_data,
    output                      read_finish,

    // Useful status for D5-D4 system arbitration/control.
    output                      reader_idle,
    output                      active_buffer_index,
    output                      frame_activation_mem,

    // Debug/fault visibility
    output                      read_fifo_empty,
    output                      read_fifo_full,
    output     [9:0]            read_fifo_wrusedw,
    output     [9:0]            read_fifo_rdusedw,

    output                      return_without_command_sticky,
    output                      credit_overflow_sticky,
    output                      frame_start_with_pending_sticky,
    output                      command_order_error_sticky,
    output                      read_fifo_underflow_sticky,
    output                      rfifo_write_overflow_sticky,

    output     [31:0]           display_app_rd_cmd_count,
    output     [31:0]           display_sdr_return_count,
    output     [31:0]           display_fifo_write_count,
    output     [31:0]           display_video_pop_count,
    output     [31:0]           video_pop_while_empty_count,
    output     [9:0]            min_fifo_level,
    output     [9:0]            max_fifo_level
);

    wire aligner_idle;
    wire reader_idle_mem;

    wire [ALIGN_FIFO_ADDR_BITS:0] align_tag_count_unused;
    wire [ALIGN_FIFO_ADDR_BITS:0] align_out_count_unused;

    display_read_rfifo_bridge #(
        .ADDR_BITS            (ADDR_BITS),
        .FRAME_PIXELS         (FRAME_PIXELS),
        .DISP0_BASE           (DISP0_BASE),
        .DISP1_BASE           (DISP1_BASE),
        .ALIGN_FIFO_ADDR_BITS (ALIGN_FIFO_ADDR_BITS),
        .TRANSPORT_BURST_SIZE (TRANSPORT_BURST_SIZE),
        .FRAME_LOCKED         (FRAME_LOCKED)
    ) u_display_read_rfifo_bridge (
        .mem_clk                         (mem_clk),
        .mem_rst                         (mem_rst),

        .Sdr_init_done                   (Sdr_init_done),
        .Sdr_init_ref_vld                (Sdr_init_ref_vld),
        .Sdr_busy                        (Sdr_busy),
        .App_wr_busy                     (App_wr_busy),

        .App_rd_en                       (App_rd_en),
        .App_rd_addr                     (App_rd_addr),

        .Sdr_rd_en                       (Sdr_rd_en),
        .Sdr_rd_dout                     (Sdr_rd_dout),

        .commit_valid                    (commit_valid),
        .commit_buffer_index             (commit_buffer_index),

        .read_clk                        (read_clk),
        .read_rst                        (read_rst),
        .fifo_rst                        (read_fifo_rst),

        .read_req                        (read_req),
        .video_frame_tick                (video_frame_tick),
        .frame_admit_mem                 (frame_admit_mem),
        .read_req_ack                    (read_req_ack),

        .read_en                         (read_en),
        .read_data                       (read_data),
        .read_finish                     (read_finish),

        .read_fifo_empty                 (read_fifo_empty),
        .read_fifo_full                  (read_fifo_full),
        .read_fifo_wrusedw               (read_fifo_wrusedw),
        .read_fifo_rdusedw               (read_fifo_rdusedw),

        .active_buffer_index             (active_buffer_index),
        .frame_activation_mem            (frame_activation_mem),

        .aligner_idle                    (aligner_idle),
        .reader_idle_mem                 (reader_idle_mem),
        .align_tag_count                 (align_tag_count_unused),
        .align_out_count                 (align_out_count_unused),

        .return_without_command_sticky   (return_without_command_sticky),
        .credit_overflow_sticky          (credit_overflow_sticky),
        .frame_start_with_pending_sticky (frame_start_with_pending_sticky),
        .command_order_error_sticky      (command_order_error_sticky),

        .read_fifo_underflow_sticky      (read_fifo_underflow_sticky),
        .rfifo_write_overflow_sticky     (rfifo_write_overflow_sticky),

        .display_app_rd_cmd_count        (display_app_rd_cmd_count),
        .display_sdr_return_count        (display_sdr_return_count),
        .display_fifo_write_count        (display_fifo_write_count),
        .display_video_pop_count         (display_video_pop_count),
        .video_pop_while_empty_count     (video_pop_while_empty_count),
        .min_fifo_level                  (min_fifo_level),
        .max_fifo_level                  (max_fifo_level)
    );

    // Strict MEM_CLK-domain ownership handoff status.  Do not derive this
    // directly from asynchronous read_fifo_empty in this wrapper.
    assign reader_idle =
        reader_idle_mem;

endmodule
