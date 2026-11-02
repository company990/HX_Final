`timescale 1ns/1ps

// ============================================================================
// D5-D3-C1
// Final D5 image-memory path controller / SDRAM owner mux.
//
// This module joins the three already-verified D5 blocks:
//
//   1) source_frame_write_d5
//      TF/BMP -> native SOURCE_BUFFER
//
//   2) bilinear_frame_builder
//      SOURCE_BUFFER -> Bilinear -> fixed 640x480 DISPLAY_BUF0/1
//
//   3) frame_display_read_d5
//      DISPLAY_BUF0/1 -> sequential read -> video-domain rfifo
//
// The single SDRAM user port is handed over only at safe transaction
// boundaries:
//
//   DISPLAY
//      -> stop accepting NEW video-frame reads
//   DRAIN_READER
//      -> wait until all old display returns/FIFO words are drained
//   SOURCE_WRITE
//      -> allow TF/BMP native source write
//   BUILDER_CFG
//      -> configure Bilinear geometry
//   BUILDER_RUN
//      -> build complete 640x480 target frame
//   DISPLAY
//      -> resume video reads, using the newly committed display buffer
//
// The existing d5_sd_card_bmp control protocol is deliberately preserved:
//   - sd_write_req / sd_write_req_ack still control BMP source transfer.
//   - write_finish_toggle_out is now toggled only after the COMPLETE
//     640x480 DISPLAY frame has been built, not merely after source write.
//   - sd_disp_commit_toggle / sd_disp_buf_idx are observed so that
//     sd_disp_commit_ack_toggle is returned only after the new DISPLAY buffer
//     is genuinely active on the reader side.
//
// This lets the existing Fade state machine continue to hold black until the
// resized target frame is actually committed.
//
// Verilog-2001 compatible.
// ============================================================================

module d5_image_memory_path #(
    parameter ENABLE_AUDIO_SHARE = 0,
    parameter ADDR_BITS     = 21,
    parameter MEM_DATA_BITS = 32,

    parameter SOURCE_BASE   = 0,
    parameter SOURCE_WORDS  = 1048576,

    parameter DISP0_BASE    = 1048576,
    parameter DISP1_BASE    = 1355776,

    parameter FRAME_PIXELS  = 307200,

    // Diagnostic selector.  Keep 0 for production.
    // D6: 1=stripes, 2=green, 3=address pattern.
    // D7: 4=all-pattern SDRAM BIST, 5=verified green then display,
    //     6=direct-video green (no SDRAM owner).
    parameter DISPLAY_DIAG_MODE = 0
)(
    // ------------------------------------------------------------------------
    // Clocks / resets
    // ------------------------------------------------------------------------
    input wire audio_pause,
    output wire transport_busy,
    input                       mem_clk,
    input                       mem_rst,

    input                       sd_clk,
    input                       sd_rst,

    input                       video_clk,
    input                       video_rst,

    // Common asynchronous FIFO reset source, normally top.rst_all.
    input                       fifo_rst,

    // ------------------------------------------------------------------------
    // SDRAM controller
    // ------------------------------------------------------------------------
    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,

    output                      App_rd_en,
    output     [ADDR_BITS-1:0]  App_rd_addr,

    input                       Sdr_rd_en,
    input      [MEM_DATA_BITS-1:0] Sdr_rd_dout,

    output                      App_wr_en,
    output     [ADDR_BITS-1:0]  App_wr_addr,
    output     [MEM_DATA_BITS-1:0] App_wr_din,
    output     [3:0]            App_wr_dm,

    // ------------------------------------------------------------------------
    // Existing d5_sd_card_bmp source-stream side (sd_clk)
    // ------------------------------------------------------------------------
    input                       sd_write_req,
    output                      sd_write_req_ack,

    input      [ADDR_BITS-1:0]  sd_write_len,
    input                       sd_write_en,
    input      [MEM_DATA_BITS-1:0] sd_write_data,

    // Existing d5_sd_card_bmp selected idle display buffer.
    input      [1:0]            sd_write_buf_idx,

    // Quasi-static adaptive metadata produced by d5_sd_card_bmp.
    input      [15:0]           sd_src_width,
    input      [15:0]           sd_src_height,
    input      [15:0]           sd_dst_width,
    input      [15:0]           sd_dst_height,
    input      [15:0]           sd_x_offset,
    input      [15:0]           sd_y_offset,
    input                       sd_geometry_valid,

    // Replaces old frame_write_toggle_mem connection into d5_sd_card_bmp.
    // Toggle occurs only after target DISPLAY frame is complete.
    output reg                  write_finish_toggle_out,

    // Existing d5_sd_card_bmp display-commit transaction.
    input                       sd_disp_commit_toggle,
    input      [1:0]            sd_disp_buf_idx,
    output reg                  sd_disp_commit_ack_toggle,

    // ------------------------------------------------------------------------
    // Existing top.d5_video_timing_data / video_delay interface
    // ------------------------------------------------------------------------
    input                       video_read_req,
    input                       video_frame_tick,
    output                      video_read_req_ack,

    input                       video_read_en,
    output     [31:0]           video_read_data,
    output                      video_read_finish,

    // ------------------------------------------------------------------------
    // Debug / integration status
    // ------------------------------------------------------------------------
    output     [2:0]            mem_phase,

    output                      source_write_active,
    output                      builder_busy,
    output                      reader_idle,
    output                      reader_active_buffer_index,

    output reg                  geometry_error_sticky,
    output reg                  commit_ack_mismatch_sticky,

    output                      source_fifo_overflow_sticky,
    output                      source_fifo_underflow_sticky,
    output                      source_range_error_sticky,

    output                      reader_return_without_command_sticky,
    output                      reader_credit_overflow_sticky,
    output                      reader_frame_start_pending_sticky,
    output                      reader_command_order_error_sticky,
    output                      reader_fifo_underflow_sticky,
    output                      reader_fifo_overflow_sticky,

    // D7 BIST status/evidence (all originate in mem_clk).
    output                      d7_bist_running,
    output                      d7_bist_completed,
    output                      d7_bist_pass,
    output                      d7_bist_mismatch,
    output                      d7_bist_timeout,
    output                      d7_bist_sequence_error,
    output     [2:0]            d7_bist_test_id,
    output     [4:0]            d7_bist_state,
    output     [31:0]           d7_bist_write_attempt_count,
    output     [31:0]           d7_bist_write_accept_count,
    output     [31:0]           d7_bist_read_command_count,
    output     [31:0]           d7_bist_read_accept_count,
    output     [31:0]           d7_bist_read_return_count,
    output     [31:0]           d7_bist_compare_count,
    output     [31:0]           d7_bist_mismatch_count,
    output     [31:0]           d7_bist_timeout_count,
    output                      d7_bist_first_error_valid,
    output     [2:0]            d7_bist_first_error_test,
    output     [ADDR_BITS-1:0]  d7_bist_first_error_addr,
    output     [31:0]           d7_bist_first_error_expected,
    output     [31:0]           d7_bist_first_error_actual,
    output     [4:0]            d7_bist_first_timeout_state,
    output     [31:0]           d7_bist_error_xor_or,
    output     [31:0]           d7_bist_even_addr_error_count,
    output     [31:0]           d7_bist_odd_addr_error_count,
    output     [31:0]           d7_bist_addr_bit0_error_count,
    output     [31:0]           d7_bist_addr_bit1_error_count,
    output     [31:0]           d7_bist_addr_bit2_error_count,
    output     [31:0]           d7_bist_addr_bit3_error_count,

    // D7 display-read instrumentation.
    output     [31:0]           d7_display_app_rd_cmd_count,
    output     [31:0]           d7_display_sdr_return_count,
    output     [31:0]           d7_display_fifo_write_count,
    output     [31:0]           d7_display_video_pop_count,
    output     [31:0]           d7_video_pop_while_empty_count,
    output     [9:0]            d7_min_fifo_level,
    output     [9:0]            d7_max_fifo_level
);

    localparam ST_DISPLAY       = 3'd0;
    localparam ST_DRAIN_READER  = 3'd1;
    localparam ST_SOURCE_WRITE  = 3'd2;
    localparam ST_BUILDER_CFG   = 3'd3;
    localparam ST_BUILDER_WAIT  = 3'd4;
    localparam ST_BUILDER_START = 3'd5;
    localparam ST_BUILDER_RUN   = 3'd6;
    localparam ST_ERROR         = 3'd7;

    localparam D6_DIAG_ENABLED =
        (DISPLAY_DIAG_MODE >= 1) && (DISPLAY_DIAG_MODE <= 3);
    localparam D7_BIST_MODE = (DISPLAY_DIAG_MODE == 4);
    localparam D7_VERIFIED_GREEN_MODE = (DISPLAY_DIAG_MODE == 5);
    localparam D7_DIRECT_GREEN_MODE = (DISPLAY_DIAG_MODE == 6);
    localparam D7_BIST_ENABLED = D7_BIST_MODE || D7_VERIFIED_GREEN_MODE;
    localparam DIAG_ENABLED = (DISPLAY_DIAG_MODE != 0);

    reg [2:0] state;

    assign mem_phase = state;


    // ========================================================================
    // Synchronize source request + quasi-static metadata into MEM_CLK.
    //
    // d5_sd_card_bmp makes geometry valid before exposing the external write
    // request, and then holds the transaction until ACK.  Therefore these
    // buses are stable for many MEM_CLK cycles before we snapshot them.
    // ========================================================================

    reg sd_req_meta, sd_req_sync;

    reg [15:0] src_w_meta, src_w_sync;
    reg [15:0] src_h_meta, src_h_sync;
    reg [15:0] dst_w_meta, dst_w_sync;
    reg [15:0] dst_h_meta, dst_h_sync;
    reg [15:0] xoff_meta,  xoff_sync;
    reg [15:0] yoff_meta,  yoff_sync;
    reg        geom_meta,  geom_sync;

    reg [1:0]  write_idx_meta, write_idx_sync;

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            sd_req_meta <= 1'b0;
            sd_req_sync <= 1'b0;

            src_w_meta <= 16'd0;
            src_w_sync <= 16'd0;
            src_h_meta <= 16'd0;
            src_h_sync <= 16'd0;

            dst_w_meta <= 16'd0;
            dst_w_sync <= 16'd0;
            dst_h_meta <= 16'd0;
            dst_h_sync <= 16'd0;

            xoff_meta <= 16'd0;
            xoff_sync <= 16'd0;
            yoff_meta <= 16'd0;
            yoff_sync <= 16'd0;

            geom_meta <= 1'b0;
            geom_sync <= 1'b0;

            write_idx_meta <= 2'd0;
            write_idx_sync <= 2'd0;
        end
        else begin
            sd_req_meta <= sd_write_req;
            sd_req_sync <= sd_req_meta;

            src_w_meta <= sd_src_width;
            src_w_sync <= src_w_meta;
            src_h_meta <= sd_src_height;
            src_h_sync <= src_h_meta;

            dst_w_meta <= sd_dst_width;
            dst_w_sync <= dst_w_meta;
            dst_h_meta <= sd_dst_height;
            dst_h_sync <= dst_h_meta;

            xoff_meta <= sd_x_offset;
            xoff_sync <= xoff_meta;
            yoff_meta <= sd_y_offset;
            yoff_sync <= yoff_meta;

            geom_meta <= sd_geometry_valid;
            geom_sync <= geom_meta;

            write_idx_meta <= sd_write_buf_idx;
            write_idx_sync <= write_idx_meta;
        end
    end


    // Snapshot used for the complete source-write -> build transaction.
    reg [15:0] cfg_src_width;
    reg [15:0] cfg_src_height;
    reg [15:0] cfg_dst_width;
    reg [15:0] cfg_dst_height;
    reg [15:0] cfg_x_offset;
    reg [15:0] cfg_y_offset;
    reg        cfg_geometry_valid;
    reg        cfg_target_buffer_index;


    // ========================================================================
    // Source-write grant MEM_CLK -> SD_CLK.
    //
    // Until this grant reaches sd_clk, sd_write_req_ack cannot occur, so
    // d5_bmp_read cannot start flooding the write FIFO while the display reader
    // is still draining.
    // ========================================================================

    wire source_grant_mem;
    reg  source_grant_meta_sd;
    reg  source_grant_sync_sd;

    assign source_grant_mem =
        !DIAG_ENABLED &&
        (state == ST_SOURCE_WRITE);

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            source_grant_meta_sd <= 1'b0;
            source_grant_sync_sd <= 1'b0;
        end
        else begin
            source_grant_meta_sd <= source_grant_mem;
            source_grant_sync_sd <= source_grant_meta_sd;
        end
    end

    wire source_write_req_gated;

    assign source_write_req_gated =
        sd_write_req &&
        source_grant_sync_sd;


    // ========================================================================
    // SOURCE_BUFFER writer
    // ========================================================================

    wire pause_new = ENABLE_AUDIO_SHARE ? audio_pause : 1'b0;
    wire source_transport_busy, builder_transport_busy;
    reg [9:0] outstanding_reads;
    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) outstanding_reads <= 0;
        else case ({App_rd_en, Sdr_rd_en})
            2'b10: outstanding_reads <= outstanding_reads + 1'b1;
            2'b01: outstanding_reads <= outstanding_reads - 1'b1;
            default: outstanding_reads <= outstanding_reads;
        endcase
    end
    assign transport_busy = source_transport_busy | builder_transport_busy |
                            App_rd_en | App_wr_en | (outstanding_reads != 0);
    wire source_App_wr_en;
    wire [ADDR_BITS-1:0] source_App_wr_addr;
    wire [MEM_DATA_BITS-1:0] source_App_wr_din;
    wire [3:0] source_App_wr_dm;

    wire source_write_finish;

    source_frame_write_d5 #(
        .MEM_DATA_BITS (MEM_DATA_BITS),
        .ADDR_BITS     (ADDR_BITS),
        .BURST_BITS    (9),
        .BURST_SIZE    (128),
        .SOURCE_BASE   (SOURCE_BASE),
        .SOURCE_WORDS  (SOURCE_WORDS)
    ) u_source_frame_write_d5 (
        .transport_busy(source_transport_busy),
        .mem_clk                     (mem_clk),
        .mem_rst                     (mem_rst),

        .Sdr_init_done               (Sdr_init_done),
        .Sdr_init_ref_vld            (Sdr_init_ref_vld),
        .Sdr_busy                    (Sdr_busy),

        .read_port_busy              (!source_grant_mem | pause_new),

        .App_wr_en                   (source_App_wr_en),
        .App_wr_addr                 (source_App_wr_addr),
        .App_wr_din                  (source_App_wr_din),
        .App_wr_dm                   (source_App_wr_dm),

        .write_clk                   (sd_clk),
        .write_rst                   (sd_rst),
        .write_fifo_rst              (fifo_rst),

        .write_req                   (source_write_req_gated),
        .write_req_ack               (sd_write_req_ack),

        .write_len                   (sd_write_len),

        .write_en                    (sd_write_en),
        .write_data                  (sd_write_data),

        .write_finish                (source_write_finish),
        .source_write_active         (source_write_active),

        .write_fifo_full             (),
        .write_fifo_empty            (),

        .write_fifo_overflow_sticky  (source_fifo_overflow_sticky),
        .write_fifo_underflow_sticky (source_fifo_underflow_sticky),
        .source_range_error_sticky   (source_range_error_sticky)
    );


    // ========================================================================
    // Bilinear frame builder
    // ========================================================================

    reg builder_cfg_start;
    reg builder_build_start;

    wire builder_cfg_busy;
    wire builder_cfg_done;
    wire builder_cfg_valid;

    wire builder_build_done;
    wire builder_commit_valid;
    wire builder_commit_buffer_index;

    wire builder_App_rd_en;
    wire [ADDR_BITS-1:0] builder_App_rd_addr;

    wire builder_App_wr_en;
    wire [ADDR_BITS-1:0] builder_App_wr_addr;
    wire [31:0] builder_App_wr_din;
    wire [3:0] builder_App_wr_dm;

    wire builder_sdram_return_valid;

    bilinear_frame_builder #(
        .ENABLE_AUDIO_SHARE(ENABLE_AUDIO_SHARE),
        .ADDR_BITS    (ADDR_BITS),
        .DISP0_BASE   (DISP0_BASE),
        .DISP1_BASE   (DISP1_BASE),
        .FRAME_PIXELS (FRAME_PIXELS)
    ) u_bilinear_frame_builder (
        .audio_pause(pause_new), .transport_busy(builder_transport_busy),
        .clk                            (mem_clk),
        .rst                            (mem_rst),

        .cfg_start                      (builder_cfg_start),

        .src_base_addr                  (SOURCE_BASE[ADDR_BITS-1:0]),
        .src_width                      (cfg_src_width),
        .src_height                     (cfg_src_height),

        .dst_width                      (cfg_dst_width),
        .dst_height                     (cfg_dst_height),
        .x_offset                       (cfg_x_offset),
        .y_offset                       (cfg_y_offset),

        .geometry_valid                 (cfg_geometry_valid),

        .cfg_busy                       (builder_cfg_busy),
        .cfg_done                       (builder_cfg_done),
        .cfg_valid                      (builder_cfg_valid),

        .build_start                    (builder_build_start),
        .target_buffer_index            (cfg_target_buffer_index),

        .build_busy                     (builder_busy),
        .build_done                     (builder_build_done),

        .commit_valid                   (builder_commit_valid),
        .commit_buffer_index            (builder_commit_buffer_index),

        .Sdr_init_done                  (Sdr_init_done),
        .Sdr_init_ref_vld               (Sdr_init_ref_vld),
        .Sdr_busy                       (Sdr_busy),

        .App_rd_en                      (builder_App_rd_en),
        .App_rd_addr                    (builder_App_rd_addr),

        .Sdr_rd_en                      (builder_sdram_return_valid),
        .Sdr_rd_dout                    (Sdr_rd_dout),

        .App_wr_en                      (builder_App_wr_en),
        .App_wr_addr                    (builder_App_wr_addr),
        .App_wr_din                     (builder_App_wr_din),
        .App_wr_dm                      (builder_App_wr_dm),

        .scaler_frame_busy              (),
        .display_write_busy             (),
        .fetch_busy                     (),

        .fetch_unexpected_return_sticky (),
        .writer_address_order_error_sticky(),
        .writer_start_while_busy_sticky ()
    );


    // ========================================================================
    // D7 physical-SDRAM BIST.  Modes 4/5 give this block exclusive ownership.
    // Mode 4 keeps the port isolated after the five tests.  Mode 5 runs only
    // constant-green, verifies every word, then releases BUF1 to the reader.
    // ========================================================================

    wire d7_bist_App_wr_en;
    wire [ADDR_BITS-1:0] d7_bist_App_wr_addr;
    wire [31:0] d7_bist_App_wr_din;
    wire [3:0] d7_bist_App_wr_dm;
    wire d7_bist_App_rd_en;
    wire [ADDR_BITS-1:0] d7_bist_App_rd_addr;
    wire d7_bist_sdram_return_valid;
    wire d7_bist_release_port;
    wire d7_bist_commit_valid;
    reg  d7_bist_release_port_d;
    wire d7_bist_port_grant;

    assign d7_bist_port_grant =
        D7_BIST_MODE ||
        (D7_VERIFIED_GREEN_MODE && !d7_bist_release_port);

    assign d7_bist_commit_valid =
        D7_VERIFIED_GREEN_MODE &&
        d7_bist_release_port &&
        !d7_bist_release_port_d;

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst)
            d7_bist_release_port_d <= 1'b0;
        else
            d7_bist_release_port_d <= d7_bist_release_port;
    end

    generate if (D7_BIST_ENABLED) begin : diagnostics
    sdram_hardware_bist #(
        .ADDR_BITS      (ADDR_BITS),
        .FRAME_WORDS    (FRAME_PIXELS),
        .TARGET_BASE    (DISP1_BASE),
        .RUN_ALL_TESTS  (D7_BIST_MODE),
        .SETTLE_CYCLES  (32),
        .TIMEOUT_CYCLES (12500000)
    ) u_sdram_hardware_bist (
        .clk                         (mem_clk),
        .rst                         (mem_rst),
        .enable                      (D7_BIST_ENABLED),
        .port_grant                  (d7_bist_port_grant),
        .Sdr_init_done               (Sdr_init_done),
        .Sdr_init_ref_vld            (Sdr_init_ref_vld),
        .Sdr_busy                    (Sdr_busy),
        .App_wr_en                   (d7_bist_App_wr_en),
        .App_wr_addr                 (d7_bist_App_wr_addr),
        .App_wr_din                  (d7_bist_App_wr_din),
        .App_wr_dm                   (d7_bist_App_wr_dm),
        .App_rd_en                   (d7_bist_App_rd_en),
        .App_rd_addr                 (d7_bist_App_rd_addr),
        .Sdr_rd_en                   (d7_bist_sdram_return_valid),
        .Sdr_rd_dout                 (Sdr_rd_dout),
        .running                     (d7_bist_running),
        .completed                   (d7_bist_completed),
        .all_tests_pass              (d7_bist_pass),
        .data_mismatch               (d7_bist_mismatch),
        .timeout_error               (d7_bist_timeout),
        .sequence_error              (d7_bist_sequence_error),
        .release_port                (d7_bist_release_port),
        .test_id                     (d7_bist_test_id),
        .state_code                  (d7_bist_state),
        .write_attempt_count         (d7_bist_write_attempt_count),
        .write_accept_count          (d7_bist_write_accept_count),
        .read_command_count          (d7_bist_read_command_count),
        .read_accept_count           (d7_bist_read_accept_count),
        .read_return_count           (d7_bist_read_return_count),
        .compare_count               (d7_bist_compare_count),
        .mismatch_count              (d7_bist_mismatch_count),
        .timeout_count               (d7_bist_timeout_count),
        .first_error_valid           (d7_bist_first_error_valid),
        .first_error_test            (d7_bist_first_error_test),
        .first_error_addr            (d7_bist_first_error_addr),
        .first_error_expected        (d7_bist_first_error_expected),
        .first_error_actual          (d7_bist_first_error_actual),
        .first_timeout_state         (d7_bist_first_timeout_state),
        .error_xor_or                (d7_bist_error_xor_or),
        .even_addr_error_count       (d7_bist_even_addr_error_count),
        .odd_addr_error_count        (d7_bist_odd_addr_error_count),
        .addr_bit0_error_count       (d7_bist_addr_bit0_error_count),
        .addr_bit1_error_count       (d7_bist_addr_bit1_error_count),
        .addr_bit2_error_count       (d7_bist_addr_bit2_error_count),
        .addr_bit3_error_count       (d7_bist_addr_bit3_error_count)
    );
    end else begin : no_diagnostics
        assign d7_bist_App_wr_en=0;
        assign d7_bist_App_wr_addr=0;
        assign d7_bist_App_wr_din=0;
        assign d7_bist_App_wr_dm=0;
        assign d7_bist_App_rd_en=0;
        assign d7_bist_App_rd_addr=0;
        assign d7_bist_running=0;
        assign d7_bist_completed=0;
        assign d7_bist_pass=0;
        assign d7_bist_mismatch=0;
        assign d7_bist_timeout=0;
        assign d7_bist_sequence_error=0;
        assign d7_bist_release_port=0;
        assign d7_bist_test_id=0;
        assign d7_bist_state=0;
        assign d7_bist_write_attempt_count=0;
        assign d7_bist_write_accept_count=0;
        assign d7_bist_read_command_count=0;
        assign d7_bist_read_accept_count=0;
        assign d7_bist_read_return_count=0;
        assign d7_bist_compare_count=0;
        assign d7_bist_mismatch_count=0;
        assign d7_bist_timeout_count=0;
        assign d7_bist_first_error_valid=0;
        assign d7_bist_first_error_test=0;
        assign d7_bist_first_error_addr=0;
        assign d7_bist_first_error_expected=0;
        assign d7_bist_first_error_actual=0;
        assign d7_bist_first_timeout_state=0;
        assign d7_bist_error_xor_or=0;
        assign d7_bist_even_addr_error_count=0;
        assign d7_bist_odd_addr_error_count=0;
        assign d7_bist_addr_bit0_error_count=0;
        assign d7_bist_addr_bit1_error_count=0;
        assign d7_bist_addr_bit2_error_count=0;
        assign d7_bist_addr_bit3_error_count=0;
    end endgenerate



    // ========================================================================
    // D6 physical-SDRAM diagnostic framebuffer writer
    //
    // The reset-default display buffer is BUF0.  The diagnostic therefore
    // fills all of BUF1, commits BUF1 only after the final accepted word, then
    // permanently releases the SDRAM port to the normal display reader.
    // ========================================================================

    wire diag_App_wr_en;
    wire [ADDR_BITS-1:0] diag_App_wr_addr;
    wire [31:0] diag_App_wr_din;
    wire [3:0] diag_App_wr_dm;

    wire diag_fill_done;
    wire diag_commit_valid;
    wire diag_commit_buffer_index;
    wire diag_completed;

    wire owner_diag;

    assign owner_diag =
        D6_DIAG_ENABLED &&
        !diag_completed;

    display_buffer_pattern_test #(
        .ADDR_BITS           (ADDR_BITS),
        .FRAME_WORDS         (FRAME_PIXELS),
        .TARGET_BASE         (DISP1_BASE),
        .TARGET_BUFFER_INDEX (1),
        .PATTERN_MODE        (DISPLAY_DIAG_MODE)
    ) u_display_buffer_pattern_test (
        .clk                 (mem_clk),
        .rst                 (mem_rst),

        .enable              (D6_DIAG_ENABLED),
        .port_grant          (owner_diag),

        .Sdr_init_done       (Sdr_init_done),
        .Sdr_init_ref_vld    (Sdr_init_ref_vld),
        .Sdr_busy            (Sdr_busy),
        .App_rd_busy         (1'b0),

        .App_wr_en           (diag_App_wr_en),
        .App_wr_addr         (diag_App_wr_addr),
        .App_wr_din          (diag_App_wr_din),
        .App_wr_dm           (diag_App_wr_dm),

        .write_busy          (),
        .fill_done           (diag_fill_done),
        .commit_valid        (diag_commit_valid),
        .commit_buffer_index (diag_commit_buffer_index),
        .completed           (diag_completed),

        .word_index          (),
        .words_written       (),
        .range_error_sticky  ()
    );


    // ========================================================================
    // Fixed DISPLAY-buffer reader
    // ========================================================================

    wire reader_App_rd_en;
    wire [ADDR_BITS-1:0] reader_App_rd_addr;

    wire reader_sdram_return_valid;

    wire reader_commit_valid;
    wire reader_commit_buffer_index;

    assign reader_commit_valid =
        D6_DIAG_ENABLED ? diag_commit_valid :
        D7_VERIFIED_GREEN_MODE ? d7_bist_commit_valid :
        builder_commit_valid;

    assign reader_commit_buffer_index =
        D6_DIAG_ENABLED ? diag_commit_buffer_index :
        D7_VERIFIED_GREEN_MODE ? 1'b1 :
                       builder_commit_buffer_index;

    // New frame requests are admitted ONLY in ST_DISPLAY.
    // During ST_DRAIN_READER the already-started frame is allowed to finish,
    // but no new frame can begin.
    wire reader_video_read_req;
    wire reader_frame_activation;
    reg display_owned_valid;
    reg commit_ack_waiting;
    reg pending_build_commit;
    reg target_activated;

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            display_owned_valid<=0; pending_build_commit<=0; target_activated<=0;
        end else begin
            if (builder_commit_valid) begin
                pending_build_commit<=1; target_activated<=0;
            end
            if (reader_frame_activation) begin
                display_owned_valid<=1;
                if (reader_active_buffer_index==cfg_target_buffer_index) begin
                    pending_build_commit<=0; target_activated<=1;
                end
            end
        end
    end
    wire reader_diag_allowed;

    assign reader_diag_allowed =
        D6_DIAG_ENABLED ? !owner_diag :
        D7_VERIFIED_GREEN_MODE ? d7_bist_release_port :
        1'b0;

    assign reader_video_read_req =
        video_read_req &&
        (
            // Keep the existing read-request/ack CDC synchronizer present in
            // the BIST-only and direct-video diagnostic netlists.  Its local
            // SDRAM commands remain blocked by owner_reader below, so D7 BIST
            // still has exclusive access to the physical application port.
            D7_BIST_MODE ||
            D7_DIRECT_GREEN_MODE ||
            reader_diag_allowed ||
            (!DIAG_ENABLED && (state == ST_DISPLAY))
        );

    frame_display_read_d5 #(
        .ADDR_BITS    (ADDR_BITS),
        .FRAME_PIXELS (FRAME_PIXELS),
        .DISP0_BASE   (DISP0_BASE),
        .DISP1_BASE   (DISP1_BASE),
        .FRAME_LOCKED (1)
    ) u_frame_display_read_d5 (
        .mem_clk                         (mem_clk),
        .mem_rst                         (mem_rst),

        .Sdr_init_done                   (Sdr_init_done),
        .Sdr_init_ref_vld                (Sdr_init_ref_vld),
        .Sdr_busy                        (Sdr_busy),

        .App_wr_busy                     (pause_new),

        .App_rd_en                       (reader_App_rd_en),
        .App_rd_addr                     (reader_App_rd_addr),

        .Sdr_rd_en                       (reader_sdram_return_valid),
        .Sdr_rd_dout                     (Sdr_rd_dout),

        .commit_valid                    (reader_commit_valid),
        .commit_buffer_index             (reader_commit_buffer_index),

        .read_clk                        (video_clk),
        .read_rst                        (video_rst),
        .read_fifo_rst                   (fifo_rst),

        .read_req                        (video_read_req),
        .video_frame_tick                (video_frame_tick),
        .frame_admit_mem                 (reader_diag_allowed ||
                    (!DIAG_ENABLED && state==ST_DISPLAY &&
                     (!sd_req_sync || pending_build_commit || commit_ack_waiting))),
        .read_req_ack                    (video_read_req_ack),

        .read_en                         (video_read_en),
        .read_data                       (video_read_data),
        .read_finish                     (video_read_finish),

        .reader_idle                     (reader_idle),
        .active_buffer_index             (reader_active_buffer_index),
        .frame_activation_mem            (reader_frame_activation),

        .read_fifo_empty                 (),
        .read_fifo_full                  (),
        .read_fifo_wrusedw               (),
        .read_fifo_rdusedw               (),

        .return_without_command_sticky   (reader_return_without_command_sticky),
        .credit_overflow_sticky          (reader_credit_overflow_sticky),
        .frame_start_with_pending_sticky (reader_frame_start_pending_sticky),
        .command_order_error_sticky      (reader_command_order_error_sticky),
        .read_fifo_underflow_sticky      (reader_fifo_underflow_sticky),
        .rfifo_write_overflow_sticky     (reader_fifo_overflow_sticky),
        .display_app_rd_cmd_count        (d7_display_app_rd_cmd_count),
        .display_sdr_return_count        (d7_display_sdr_return_count),
        .display_fifo_write_count        (d7_display_fifo_write_count),
        .display_video_pop_count         (d7_display_video_pop_count),
        .video_pop_while_empty_count     (d7_video_pop_while_empty_count),
        .min_fifo_level                  (d7_min_fifo_level),
        .max_fifo_level                  (d7_max_fifo_level)
    );


    // ========================================================================
    // SDRAM ownership / command mux
    // ========================================================================

    wire owner_reader;
    wire owner_source;
    wire owner_builder;

    assign owner_reader =
        (
            reader_diag_allowed ||
            (!DIAG_ENABLED &&
             (
            (state == ST_DISPLAY) ||
            (state == ST_DRAIN_READER)
             ))
        );

    assign owner_source =
        !DIAG_ENABLED &&
        (state == ST_SOURCE_WRITE);

    assign owner_builder =
        !DIAG_ENABLED &&
        (state == ST_BUILDER_RUN);

    assign App_rd_en =
        d7_bist_port_grant ? d7_bist_App_rd_en :
        owner_reader  ? reader_App_rd_en :
        owner_builder ? builder_App_rd_en :
                        1'b0;

    assign App_rd_addr =
        d7_bist_port_grant ? d7_bist_App_rd_addr :
        owner_reader  ? reader_App_rd_addr :
        owner_builder ? builder_App_rd_addr :
                        {ADDR_BITS{1'b0}};

    assign App_wr_en =
        d7_bist_port_grant ? d7_bist_App_wr_en :
        owner_diag    ? diag_App_wr_en :
        owner_source  ? source_App_wr_en :
        owner_builder ? builder_App_wr_en :
                        1'b0;

    assign App_wr_addr =
        d7_bist_port_grant ? d7_bist_App_wr_addr :
        owner_diag    ? diag_App_wr_addr :
        owner_source  ? source_App_wr_addr :
        owner_builder ? builder_App_wr_addr :
                        {ADDR_BITS{1'b0}};

    assign App_wr_din =
        d7_bist_port_grant ? d7_bist_App_wr_din :
        owner_diag    ? diag_App_wr_din :
        owner_source  ? source_App_wr_din :
        owner_builder ? builder_App_wr_din :
                        {MEM_DATA_BITS{1'b0}};

    assign App_wr_dm =
        d7_bist_port_grant ? d7_bist_App_wr_dm :
        owner_diag    ? diag_App_wr_dm :
        owner_source  ? source_App_wr_dm :
        owner_builder ? builder_App_wr_dm :
                        4'b0000;

    // Route each SDRAM return only to the client that owns the read port.
    assign reader_sdram_return_valid =
        Sdr_rd_en &&
        owner_reader;

    assign builder_sdram_return_valid =
        Sdr_rd_en &&
        owner_builder;

    assign d7_bist_sdram_return_valid =
        Sdr_rd_en &&
        d7_bist_port_grant;


    // ========================================================================
    // Main ownership / build sequence
    // ========================================================================

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            state <= ST_DISPLAY;

            cfg_src_width  <= 16'd0;
            cfg_src_height <= 16'd0;
            cfg_dst_width  <= 16'd0;
            cfg_dst_height <= 16'd0;
            cfg_x_offset   <= 16'd0;
            cfg_y_offset   <= 16'd0;
            cfg_geometry_valid <= 1'b0;
            cfg_target_buffer_index <= 1'b0;

            builder_cfg_start   <= 1'b0;
            builder_build_start <= 1'b0;

            write_finish_toggle_out <= 1'b0;

            geometry_error_sticky <= 1'b0;
        end
        else begin
            builder_cfg_start   <= 1'b0;
            builder_build_start <= 1'b0;

            case (state)

                ST_DISPLAY: begin
                    if (sd_req_sync && !pending_build_commit && !commit_ack_waiting) begin
                        // Snapshot while d5_sd_card_bmp is holding write_req.
                        cfg_src_width  <= src_w_sync;
                        cfg_src_height <= src_h_sync;
                        cfg_dst_width  <= dst_w_sync;
                        cfg_dst_height <= dst_h_sync;
                        cfg_x_offset   <= xoff_sync;
                        cfg_y_offset   <= yoff_sync;
                        cfg_geometry_valid <= geom_sync;
                        cfg_target_buffer_index <= write_idx_sync[0];

                        if (display_owned_valid &&
                            write_idx_sync[0]==reader_active_buffer_index) begin
                            geometry_error_sticky<=1;
                            state<=ST_ERROR;
                        end else state <= ST_DRAIN_READER;
                    end
                end

                ST_DRAIN_READER: begin
                    // No NEW read request is admitted here; wait for the old
                    // display frame's command/return/FIFO pipeline to drain.
                    if (reader_idle)
                        state <= ST_SOURCE_WRITE;
                end

                ST_SOURCE_WRITE: begin
                    if (source_write_finish)
                        state <= ST_BUILDER_CFG;
                end

                ST_BUILDER_CFG: begin
                    builder_cfg_start <= 1'b1;
                    state <= ST_BUILDER_WAIT;
                end

                ST_BUILDER_WAIT: begin
                    if (builder_cfg_done) begin
                        if (builder_cfg_valid)
                            state <= ST_BUILDER_START;
                        else begin
                            geometry_error_sticky <= 1'b1;
                            state <= ST_ERROR;
                        end
                    end
                end

                ST_BUILDER_START: begin
                    builder_build_start <= 1'b1;
                    state <= ST_BUILDER_RUN;
                end

                ST_BUILDER_RUN: begin
                    if (builder_build_done) begin
                        // This is the event the old d5_sd_card_bmp state machine
                        // should interpret as "whole frame ready".
                        write_finish_toggle_out <=
                            ~write_finish_toggle_out;

                        state <= ST_DISPLAY;
                    end
                end

                default: begin
                    state <= ST_ERROR;
                end

            endcase
        end
    end


    // ========================================================================
    // Existing d5_sd_card_bmp commit-toggle ACK compatibility
    //
    // d5_sd_card_bmp toggles sd_disp_commit_toggle only after it observes the
    // new write_finish_toggle_out.  ACK is returned only after the D5 reader
    // has actually activated that target display buffer.
    // ========================================================================

    reg commit_tgl_meta_mem;
    reg commit_tgl_sync_mem;
    reg commit_tgl_seen_mem;

    reg [1:0] disp_idx_meta_mem;
    reg [1:0] disp_idx_sync_mem;

    reg commit_ack_target_buffer;
    reg commit_ack_target_toggle;

    always @(posedge mem_clk or posedge mem_rst) begin
        if (mem_rst) begin
            commit_tgl_meta_mem <= 1'b0;
            commit_tgl_sync_mem <= 1'b0;
            commit_tgl_seen_mem <= 1'b0;

            disp_idx_meta_mem <= 2'd0;
            disp_idx_sync_mem <= 2'd0;

            commit_ack_waiting <= 1'b0;
            commit_ack_target_buffer <= 1'b0;
            commit_ack_target_toggle <= 1'b0;

            sd_disp_commit_ack_toggle <= 1'b0;
            commit_ack_mismatch_sticky <= 1'b0;
        end
        else begin
            commit_tgl_meta_mem <= sd_disp_commit_toggle;
            commit_tgl_sync_mem <= commit_tgl_meta_mem;

            disp_idx_meta_mem <= sd_disp_buf_idx;
            disp_idx_sync_mem <= disp_idx_meta_mem;

            if (commit_tgl_sync_mem != commit_tgl_seen_mem) begin
                commit_tgl_seen_mem <= commit_tgl_sync_mem;

                commit_ack_waiting <= 1'b1;
                commit_ack_target_buffer <= disp_idx_sync_mem[0];
                commit_ack_target_toggle <= commit_tgl_sync_mem;
            end

            if (
                commit_ack_waiting &&
                target_activated &&
                (state == ST_DISPLAY) &&
                (reader_active_buffer_index == commit_ack_target_buffer)
            ) begin
                sd_disp_commit_ack_toggle <=
                    commit_ack_target_toggle;

                commit_ack_waiting <= 1'b0;
            end

            // Should never have a second SD commit while the previous one is
            // still waiting for the reader-side activation.
            if (
                commit_ack_waiting &&
                (commit_tgl_sync_mem != commit_tgl_seen_mem)
            )
                commit_ack_mismatch_sticky <= 1'b1;
        end
    end

endmodule
