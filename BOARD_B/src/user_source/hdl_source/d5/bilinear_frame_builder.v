`timescale 1ns/1ps

// ============================================================================
// D5-C3-B
// Complete Bilinear frame-builder:
//   SOURCE_BUFFER -> Bilinear scaler -> DISPLAY_BUF0 / DISPLAY_BUF1
//
// Integrates:
//   scaler_sdram_pipeline  (D5-A1/B1/B2/C1/C2)
//   display_buffer_writer  (D5-C3-A)
//
// One SDRAM user port is shared by the source-reader and destination-writer.
// D8-A retains the scaler mathematics but batches the completed destination
// stream through the V1 FIFO writer.  Four source reads form one uninterrupted
// command run; target writes form uninterrupted 128-word runs.
//
// Black-border pixels skip all source reads and are written as real 0x000000
// pixels into the display buffer.
//
// A frame commit pulse is generated only after BOTH:
//   - scaler has consumed destination pixel 307199
//   - display writer has accepted/written destination pixel 307199
//
// Verilog-2001 compatible.
// ============================================================================

module bilinear_frame_builder #(
    parameter ENABLE_AUDIO_SHARE = 0,
    parameter ADDR_BITS    = 21,
    parameter DISP0_BASE   = 1048576,
    parameter DISP1_BASE   = 1355776,
    parameter FRAME_PIXELS = 307200
)(
    input wire audio_pause,
    output wire transport_busy,
    input                       clk,
    input                       rst,

    // ------------------------------------------------------------------------
    // Geometry configuration
    // ------------------------------------------------------------------------
    input                       cfg_start,

    input      [ADDR_BITS-1:0]  src_base_addr,
    input      [15:0]           src_width,
    input      [15:0]           src_height,

    input      [15:0]           dst_width,
    input      [15:0]           dst_height,
    input      [15:0]           x_offset,
    input      [15:0]           y_offset,

    input                       geometry_valid,

    output                      cfg_busy,
    output                      cfg_done,
    output                      cfg_valid,

    // ------------------------------------------------------------------------
    // Build one complete 640x480 target frame
    // ------------------------------------------------------------------------
    input                       build_start,
    input                       target_buffer_index,

    output reg                  build_busy,
    output reg                  build_done,

    output reg                  commit_valid,
    output reg                  commit_buffer_index,

    // ------------------------------------------------------------------------
    // Shared SDRAM user interface
    // ------------------------------------------------------------------------
    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,

    output                      App_rd_en,
    output     [ADDR_BITS-1:0]  App_rd_addr,

    input                       Sdr_rd_en,
    input      [31:0]           Sdr_rd_dout,

    output                      App_wr_en,
    output     [ADDR_BITS-1:0]  App_wr_addr,
    output     [31:0]           App_wr_din,
    output     [3:0]            App_wr_dm,

    // ------------------------------------------------------------------------
    // Debug / verification
    // ------------------------------------------------------------------------
    output                      scaler_frame_busy,
    output                      display_write_busy,
    output                      fetch_busy,

    output                      fetch_unexpected_return_sticky,
    output                      writer_address_order_error_sticky,
    output                      writer_start_while_busy_sticky
);
    wire pause_new = ENABLE_AUDIO_SHARE ? audio_pause : 1'b0;


    // ========================================================================
    // Start / completion control
    // ========================================================================

    wire run_start;

    reg active_target_buffer_index;

    reg scaler_done_seen;
    reg writer_done_seen;

    wire scaler_frame_done;
    wire writer_write_done;

    wire both_done_now;

    assign run_start =
        build_start &&
        cfg_valid &&
        !build_busy;

    assign both_done_now =
        (scaler_done_seen || scaler_frame_done) &&
        (writer_done_seen || writer_write_done);


    always @(posedge clk or posedge rst) begin
        if (rst) begin

            build_busy  <= 1'b0;
            build_done  <= 1'b0;

            commit_valid        <= 1'b0;
            commit_buffer_index <= 1'b0;

            active_target_buffer_index <= 1'b0;

            scaler_done_seen <= 1'b0;
            writer_done_seen <= 1'b0;

        end
        else begin

            build_done   <= 1'b0;
            commit_valid <= 1'b0;

            if (run_start) begin

                build_busy <= 1'b1;

                active_target_buffer_index <= target_buffer_index;

                scaler_done_seen <= 1'b0;
                writer_done_seen <= 1'b0;

            end
            else if (build_busy) begin

                if (scaler_frame_done)
                    scaler_done_seen <= 1'b1;

                if (writer_write_done)
                    writer_done_seen <= 1'b1;

                if (both_done_now) begin

                    build_busy <= 1'b0;
                    build_done <= 1'b1;

                    commit_valid        <= 1'b1;
                    commit_buffer_index <= active_target_buffer_index;
                end
            end
        end
    end


    // ========================================================================
    // Scaler side
    // ========================================================================

    wire        scaler_dst_valid;
    wire        scaler_dst_ready;

    wire [18:0] scaler_dst_addr;
    wire [23:0] scaler_dst_pixel;

    wire [9:0]  scaler_dst_x_unused;
    wire [8:0]  scaler_dst_y_unused;

    wire        scaler_App_rd_en;
    wire [ADDR_BITS-1:0] scaler_App_rd_addr;

    wire [2:0] fetch_issued_count_unused;
    wire [2:0] fetch_returned_count_unused;


    // ========================================================================
    // Writer side
    // ========================================================================

    wire        writer_pixel_ready;

    wire        writer_App_wr_en;
    wire [ADDR_BITS-1:0] writer_App_wr_addr;
    wire [31:0] writer_App_wr_din;
    wire [3:0]  writer_App_wr_dm;
    wire        writer_transport_busy;

    wire        writer_completed_buffer_index_unused;
    wire [18:0] writer_words_written_unused;


    // ========================================================================
    // Read/write arbitration
    //
    // fetch_busy is registered state from D5-C2-A.  A display write is blocked
    // for the whole four-read transaction.
    //
    // A source request is blocked during the whole pending/active V1 write
    // burst. Writer launch waits for fetch_busy to clear after ordered returns.
    // ========================================================================

    scaler_sdram_pipeline #(
        .ADDR_BITS (ADDR_BITS)
    ) u_scaler_sdram (
        .clk                            (clk),
        .rst                            (rst),

        .cfg_start                      (cfg_start),

        .src_base_addr                  (src_base_addr),
        .src_width                      (src_width),
        .src_height                     (src_height),

        .dst_width                      (dst_width),
        .dst_height                     (dst_height),
        .x_offset                       (x_offset),
        .y_offset                       (y_offset),

        .geometry_valid                 (geometry_valid),

        .cfg_busy                       (cfg_busy),
        .cfg_done                       (cfg_done),
        .cfg_valid                      (cfg_valid),

        .frame_start                    (run_start),
        .frame_busy                     (scaler_frame_busy),
        .frame_done                     (scaler_frame_done),

        .Sdr_init_done                  (Sdr_init_done),
        .Sdr_init_ref_vld               (Sdr_init_ref_vld),
        .Sdr_busy                       (Sdr_busy),

        .App_wr_busy                    (writer_transport_busy | pause_new),

        .App_rd_en                      (scaler_App_rd_en),
        .App_rd_addr                    (scaler_App_rd_addr),

        .Sdr_rd_en                      (Sdr_rd_en),
        .Sdr_rd_dout                    (Sdr_rd_dout),

        .dst_valid                      (scaler_dst_valid),
        .dst_ready                      (scaler_dst_ready),

        .dst_addr                       (scaler_dst_addr),
        .dst_pixel                      (scaler_dst_pixel),

        .dst_x                          (scaler_dst_x_unused),
        .dst_y                          (scaler_dst_y_unused),

        .fetch_busy                     (fetch_busy),
        .fetch_issued_count             (fetch_issued_count_unused),
        .fetch_returned_count           (fetch_returned_count_unused),
        .fetch_unexpected_return_sticky (fetch_unexpected_return_sticky)
    );


    display_buffer_writer #(
        .ADDR_BITS    (ADDR_BITS),
        .FRAME_PIXELS (FRAME_PIXELS),
        .DISP0_BASE   (DISP0_BASE),
        .DISP1_BASE   (DISP1_BASE)
    ) u_display_writer (
        .clk                        (clk),
        .rst                        (rst),

        .frame_start                (run_start),
        .buffer_index               (target_buffer_index),

        .write_busy                 (display_write_busy),
        .write_done                 (writer_write_done),
        .completed_buffer_index     (writer_completed_buffer_index_unused),
        .transport_busy             (writer_transport_busy),

        .pixel_valid                (scaler_dst_valid),
        .pixel_ready                (writer_pixel_ready),

        .pixel_addr                 (scaler_dst_addr),
        .pixel_data                 (scaler_dst_pixel),

        .Sdr_init_done              (Sdr_init_done),
        .Sdr_init_ref_vld           (Sdr_init_ref_vld),
        .Sdr_busy                   (Sdr_busy),

        .App_rd_busy                (fetch_busy | pause_new),

        .App_wr_en                  (writer_App_wr_en),
        .App_wr_addr                (writer_App_wr_addr),
        .App_wr_din                 (writer_App_wr_din),
        .App_wr_dm                  (writer_App_wr_dm),

        .words_written              (writer_words_written_unused),
        .address_order_error_sticky (writer_address_order_error_sticky),
        .start_while_busy_sticky    (writer_start_while_busy_sticky)
    );


    assign transport_busy = writer_transport_busy;
    assign scaler_dst_ready =
        writer_pixel_ready;

    assign App_rd_en   = scaler_App_rd_en;
    assign App_rd_addr = scaler_App_rd_addr;

    assign App_wr_en   = writer_App_wr_en;
    assign App_wr_addr = writer_App_wr_addr;
    assign App_wr_din  = writer_App_wr_din;
    assign App_wr_dm   = writer_App_wr_dm;

endmodule
