// D5 visual pipeline using the single shared SD/SDRAM controller.
module d5_visual_path #(
    parameter [2:0] PRESENTATION_ICON_ID=3'd0,
    parameter [4:0] PRESENTATION_ICON_COL=5'd15,
    parameter [4:0] PRESENTATION_ICON_ROW=5'd7,
    parameter [11:0] PRESENTATION_HOTSPOT1={5'd1,5'd4,2'b00},
    parameter [11:0] PRESENTATION_HOTSPOT2={5'd1,5'd7,2'b01},
    parameter [11:0] PRESENTATION_HOTSPOT3={5'd13,5'd8,2'b11},
    parameter [11:0] PRESENTATION_HOTSPOT4={5'd15,5'd7,2'b11}
)(
    input wire clk,
    input wire rst_all,
    input wire sd_card_clk,
    input wire ext_mem_clk,
    input wire video_clk,
    input wire Sdr_init_done,
    input wire Sdr_init_ref_vld,
    input wire Sdr_busy,
    input wire Sdr_rd_en,
    input wire [31:0] Sdr_rd_dout,
    output wire App_rd_en,
    output wire [20:0] App_rd_addr,
    output wire App_wr_en,
    output wire [20:0] App_wr_addr,
    output wire [31:0] App_wr_din,
    output wire [3:0] App_wr_dm,
    output wire hs,
    output wire vs,
    output wire de,
    output wire [23:0] vout_data,
    output wire [3:0] state_code,
    output wire display_valid_active,
    output wire frame_write_toggle_mem,
    output wire video_read_en,
    output wire [31:0] video_read_data,
    input wire key1, key2, key_adjust, adjust_select,
    input wire presentation_mode, presentation_key_next_n,
    input wire signed [7:0] brightness_applied,
    output wire signed [7:0] brightness_target,
    output wire brightness_tick,
    input wire sd_init_done, sd_valid, sd_end,
    input wire [7:0] sd_data,
    output wire sd_req,
    output wire [31:0] sd_addr,
    output wire scan_done, bmp_found,
    input wire audio_pause,
    output wire transport_busy
);
wire rst_video;
// Match the one-clock synchronous font ROM latency at the HDMI boundary.
wire hs_pre_osd, vs_pre_osd, de_pre_osd, display_valid_active_pre_osd;
reg hs_font, vs_font, de_font, valid_font;
always @(posedge video_clk or posedge rst_video) begin
    if (rst_video) begin hs_font<=1; vs_font<=1; de_font<=0; valid_font<=0; end
    else begin
        hs_font<=hs_pre_osd; vs_font<=vs_pre_osd;
        de_font<=de_pre_osd; valid_font<=display_valid_active_pre_osd;
    end
end
assign hs=hs_font;
assign vs=vs_font;
assign de=de_font;
assign display_valid_active=valid_font;
parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;

// G1-C3-D3-C1:
// HDMI output remains fixed at 640x480, so the read side still consumes
// 307200 output pixels per video frame.
parameter [20:0] FRAME_PIXELS = 21'd307200;

// ===================== D5 final SDRAM layout =====================
// Native BMP source is always stored in SOURCE_BUFFER.
// Bilinear scaling then constructs a real fixed 640x480 frame in one of
// two DISPLAY buffers.  HDMI reads only the DISPLAY buffers sequentially.
parameter [20:0] SOURCE_BUF_ADDR  = 21'd0;
parameter [20:0] SOURCE_BUF_WORDS = 21'd1048576;

parameter [20:0] DISP0_ADDR       = 21'd1048576;
parameter [20:0] DISP1_ADDR       = 21'd1355776;

// Compile-time hardware diagnostic selector.  Keep 0 for production.
// D6: 1=stripes, 2=solid green, 3=address pattern through SDRAM display.
// D7: 4=SDRAM BIST/direct result, 5=verified green/display reader,
//     6=direct-video green (no SDRAM framebuffer).
localparam integer DISPLAY_DIAG_MODE = 0;
localparam D7_BIST_MODE = (DISPLAY_DIAG_MODE == 4);
localparam D7_VERIFIED_GREEN_MODE = (DISPLAY_DIAG_MODE == 5);
localparam D7_DIRECT_GREEN_MODE = (DISPLAY_DIAG_MODE == 6);


wire ext_mem_clk_sft;
wire hdmi_5x_clk;


wire [23:0] vout_data_raw;
wire [23:0] vout_data_adjusted;
wire [23:0] vout_data_fade;
wire [23:0] vout_data_osd_in;
wire [23:0] vout_data_osd;
wire        display_valid_src;
wire        display_valid_pending;

wire [6:0]  seg_data_0;
wire [6:0]  d7_seg_state;
wire [6:0]  d7_seg_err_lo;
wire [6:0]  d7_seg_err_hi;
wire [6:0]  d7_seg_addr_lo;
wire [6:0]  d7_seg_addr_hi;
// ===================== G1-B2 OSD status from sd_card_clk =====================
wire        osd_auto_play_src;
wire [1:0]  osd_img_idx_src;
wire [2:0]  osd_img_count_src;
// ===================== G1-B2 OSD status in video_clk =====================
wire        osd_auto_play_video;
wire [1:0]  osd_img_idx_video;
wire [2:0]  osd_img_count_video;
// ===================== G1-C1 Fade transition =====================

// SD-domain event toggles
wire fade_start_toggle_sd;
wire image_ready_toggle_sd;

// SD -> VIDEO converted pulses
wire fade_start_pulse_video;
wire image_ready_pulse_video;

// Fade-core VIDEO-domain pulses
wire fade_midpoint_pulse_video;
wire fade_done_pulse_video;

// VIDEO-domain events converted to toggles
reg  fade_midpoint_toggle_video;
reg  fade_done_toggle_video;

// VIDEO -> SD converted pulses
wire fade_midpoint_pulse_sd;
wire fade_done_pulse_sd;

// Fade status
wire       fade_transition_busy;
wire       fade_black_hold;
wire [5:0] fade_level;

// One pulse per video frame
reg  video_vs_d;
wire video_frame_tick;

// ===================== G1-C2 brightness / contrast control =====================
// Physical K3 is debounced in sd_card_clk domain.
// SW1 selects which parameter K3 changes:
//     0 -> brightness
//     1 -> contrast
// The selected button event is transferred to video_clk by toggle CDC,
// so the multi-bit brightness/contrast values stay entirely in video_clk domain.
wire key_adjust_press_sd;
reg  [1:0] adjust_select_sync_sd;
wire brightness_event_pulse_sd;
wire contrast_event_pulse_sd;
reg  brightness_event_toggle_sd;
reg  contrast_event_toggle_sd;
wire brightness_event_pulse_video;
wire contrast_event_pulse_video;
reg  brightness_event_pending_video;
reg  contrast_event_pending_video;
wire brightness_apply_pulse_video;
wire contrast_apply_pulse_video;
wire signed [8:0] image_brightness;
wire signed [8:0] image_brightness_target;
assign image_brightness = {brightness_applied[7],brightness_applied};
assign brightness_target = image_brightness_target[7:0];
assign brightness_tick = video_frame_tick;
wire        [9:0] image_contrast_gain;
wire [2:0] brightness_level;
wire [2:0] contrast_level;
wire        video_read_req;
wire        video_read_req_ack;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;

// D5: this toggle is generated only after the complete 640x480 DISPLAY
// target frame has been built.

// ===================== G1-C3-D3-C1 adaptive BMP metadata =====================
// Stable in sd_card_clk while one source image is being written.
// D3-C1 uses adaptive_write_len_sd on the SDRAM write side.
// The remaining geometry signals are reserved for the adaptive read mapper.
wire [15:0] adaptive_src_width_sd;
wire [15:0] adaptive_src_height_sd;
wire [20:0] adaptive_write_len_sd;
wire [15:0] adaptive_dst_width_sd;
wire [15:0] adaptive_dst_height_sd;
wire [15:0] adaptive_x_offset_sd;
wire [15:0] adaptive_y_offset_sd;
wire        adaptive_geometry_valid_sd;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;
wire        disp_buf_commit_toggle;
wire        disp_buf_commit_ack_toggle;

// ===================== D5 final image-memory path status =====================
wire [2:0]  d5_mem_phase;
wire        d5_source_write_active;
wire        d5_builder_busy;
wire        d5_reader_idle;
wire        d5_reader_active_buffer_index;

wire        d5_geometry_error_sticky;
wire        d5_commit_ack_mismatch_sticky;

wire        d5_source_fifo_overflow_sticky;
wire        d5_source_fifo_underflow_sticky;
wire        d5_source_range_error_sticky;

wire        d5_reader_return_without_command_sticky;
wire        d5_reader_credit_overflow_sticky;
wire        d5_reader_frame_start_pending_sticky;
wire        d5_reader_command_order_error_sticky;
wire        d5_reader_fifo_underflow_sticky;
wire        d5_reader_fifo_overflow_sticky;

wire        d7_bist_running;
wire        d7_bist_completed;
wire        d7_bist_pass;
wire        d7_bist_mismatch;
wire        d7_bist_timeout;
wire        d7_bist_sequence_error;
wire [2:0]  d7_bist_test_id;
wire [4:0]  d7_bist_state;
wire [31:0] d7_bist_write_attempt_count;
wire [31:0] d7_bist_write_accept_count;
wire [31:0] d7_bist_read_command_count;
wire [31:0] d7_bist_read_accept_count;
wire [31:0] d7_bist_read_return_count;
wire [31:0] d7_bist_compare_count;
wire [31:0] d7_bist_mismatch_count;
wire [31:0] d7_bist_timeout_count;
wire        d7_bist_first_error_valid;
wire [2:0]  d7_bist_first_error_test;
wire [ADDR_BITS-1:0] d7_bist_first_error_addr;
wire [31:0] d7_bist_first_error_expected;
wire [31:0] d7_bist_first_error_actual;
wire [4:0]  d7_bist_first_timeout_state;
wire [31:0] d7_bist_error_xor_or;
wire [31:0] d7_bist_even_addr_error_count;
wire [31:0] d7_bist_odd_addr_error_count;
wire [31:0] d7_bist_addr_bit0_error_count;
wire [31:0] d7_bist_addr_bit1_error_count;
wire [31:0] d7_bist_addr_bit2_error_count;
wire [31:0] d7_bist_addr_bit3_error_count;

wire [31:0] d7_display_app_rd_cmd_count;
wire [31:0] d7_display_sdr_return_count;
wire [31:0] d7_display_fifo_write_count;
wire [31:0] d7_display_video_pop_count;
wire [31:0] d7_video_pop_while_empty_count;
wire [9:0]  d7_min_fifo_level;
wire [9:0]  d7_max_fifo_level;

reg d7_pass_meta_video, d7_pass_video;
reg d7_mismatch_meta_video, d7_mismatch_video;
reg d7_timeout_meta_video, d7_timeout_video;
reg d7_sequence_meta_video, d7_sequence_video;
reg d7_reader_buf1_meta_video, d7_reader_buf1_video;
wire [23:0] d7_direct_status_rgb;
wire d7_use_direct_background;
wire diagnostic_display_valid_src;


wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 音频发射相关
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 统一复位：TF 图像链路 + HDMI 音频链路


wire rst_50;
wire rst_sd;
wire rst_mem;

wire rst_hdmi_5x;
wire rst_audio;

// D9 hardware diagnosis: other five digits show stable startup progress.
// Right to left: SD state, memory phase, source writer active, builder busy,
// sticky error summary, SDRAM init done. This variant changes no data path.
localparam D9_HWDEBUG_STAGE1 = 1'b1;
reg [2:0] hwdbg_phase_meta_50, hwdbg_phase_sync_50;
reg [3:0] hwdbg_flag_meta_50, hwdbg_flag_sync_50;
reg hwdbg_overflow_meta_50, hwdbg_overflow_sync_50;
wire hwdbg_error_50 = hwdbg_flag_sync_50[2] | hwdbg_overflow_sync_50;
// Collect only mem_clk sticky flags here. The source FIFO overflow flag is
// synchronized separately from sd_card_clk; the video-side underflow flag is
// intentionally excluded from this first-stage source/build diagnosis.
wire hwdbg_error_mem = d5_geometry_error_sticky |
    d5_commit_ack_mismatch_sticky |
    d5_source_fifo_underflow_sticky |
    d5_source_range_error_sticky |
    d5_reader_return_without_command_sticky |
    d5_reader_credit_overflow_sticky |
    d5_reader_frame_start_pending_sticky |
    d5_reader_command_order_error_sticky |
    d5_reader_fifo_overflow_sticky;
always @(posedge clk or posedge rst_50) begin
    if (rst_50) begin
        hwdbg_phase_meta_50 <= 3'b000;
        hwdbg_phase_sync_50 <= 3'b000;
        hwdbg_flag_meta_50 <= 4'b0000;
        hwdbg_flag_sync_50 <= 4'b0000;
        hwdbg_overflow_meta_50 <= 1'b0;
        hwdbg_overflow_sync_50 <= 1'b0;
    end else begin
        hwdbg_phase_meta_50 <= d5_mem_phase;
        hwdbg_phase_sync_50 <= hwdbg_phase_meta_50;
        hwdbg_flag_meta_50 <= {Sdr_init_done,hwdbg_error_mem,
                               d5_builder_busy,d5_source_write_active};
        hwdbg_flag_sync_50 <= hwdbg_flag_meta_50;
        hwdbg_overflow_meta_50 <= d5_source_fifo_overflow_sticky;
        hwdbg_overflow_sync_50 <= hwdbg_overflow_meta_50;
    end
end

// 保持你原来的 TF / SDRAM / video 时钟
reset_sync u_reset_sync_50 (
    .clk  (clk),
    .arst (rst_all),
    .srst (rst_50)
);

reset_sync u_reset_sync_sd (
    .clk  (sd_card_clk),
    .arst (rst_all),
    .srst (rst_sd)
);

reset_sync u_reset_sync_mem (
    .clk  (ext_mem_clk),
    .arst (rst_all),
    .srst (rst_mem)
);

reset_sync u_reset_sync_video (
    .clk  (video_clk),
    .arst (rst_all),
    .srst (rst_video)
);





// ===================== TF 多图扫描与缓存（双缓冲） =====================
d5_sd_card_bmp #(
    .FAST_SCAN(1), .KEYS_DEBOUNCED(1),
    .CLK_FREQ_HZ       (50_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd4),
    .BUFFER_WORDS      (SOURCE_BUF_WORDS)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_sd),

    .key_next          (key1),   // physical K2: manual next image
    .key_auto          (key2),   // physical K4: auto play ON/OFF

    // G1-C1 fade handshake
    .fade_midpoint_pulse(fade_midpoint_pulse_sd),
    .fade_done_pulse    (fade_done_pulse_sd),

    .fade_start_toggle  (fade_start_toggle_sd),
    .image_ready_toggle (image_ready_toggle_sd),

    .state_code        (state_code),
    
        // G1-B2 OSD status
    .osd_auto_play     (osd_auto_play_src),
    .osd_img_idx       (osd_img_idx_src),
    .osd_img_count     (osd_img_count_src),
    
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid_src),

    // G1-C3-D3-B/C1: real source size, source pixel count and fit geometry.
    .adaptive_src_width      (adaptive_src_width_sd),
    .adaptive_src_height     (adaptive_src_height_sd),
    .adaptive_write_len      (adaptive_write_len_sd),
    .adaptive_dst_width      (adaptive_dst_width_sd),
    .adaptive_dst_height     (adaptive_dst_height_sd),
    .adaptive_x_offset       (adaptive_x_offset_sd),
    .adaptive_y_offset       (adaptive_y_offset_sd),
    .adaptive_geometry_valid (adaptive_geometry_valid_sd),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),
    .disp_buf_commit_toggle(disp_buf_commit_toggle),
    .disp_buf_commit_ack_toggle(disp_buf_commit_ack_toggle),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .sd_init_done(sd_init_done), .sd_sec_read(sd_req), .sd_sec_read_addr(sd_addr),
    .sd_sec_read_data(sd_data), .sd_sec_read_data_valid(sd_valid), .sd_sec_read_end(sd_end),
    .O_scan_done(scan_done), .O_bmp_found(bmp_found)
);

d5_video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_video),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_video),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs_pre_osd),
    .vs_r              (vs_pre_osd),
    .de_r              (de_pre_osd),
    .vout_data         (vout_data_raw)
);
// ===================== G1-C1 video frame tick =====================
//
// VS is active-low in this video timing.
// Generate one pulse on the falling edge:
//
//      VS: 1 -> 0
//
// which corresponds to the beginning of a new frame.
//
always @(posedge video_clk or posedge rst_video) begin

    if (rst_video)
        video_vs_d <= 1'b1;
    else
        video_vs_d <= vs_pre_osd;

end

assign video_frame_tick =
    video_vs_d &&
    !vs_pre_osd;

// ===================== G1-C2 K3 + SW1 hardware control =====================
// SW1 is asynchronous to sd_card_clk. Synchronize it before sampling the mode.
always @(posedge sd_card_clk or posedge rst_sd) begin
    if (rst_sd)
        adjust_select_sync_sd <= 2'b00;
    else
        adjust_select_sync_sd <= {adjust_select_sync_sd[0], adjust_select};
end

// Physical K3 is active-low. Reuse the same 20 ms debounce module
// already used by the SD/BMP control path.
d5_key_press_debounce #(
    .CLK_FREQ_HZ (50_000_000),
    .DEBOUNCE_MS (0)
) u_key_adjust_debounce (
    .clk         (sd_card_clk),
    .rst         (rst_sd),
    .button_in   (key_adjust),
    .press_pulse (key_adjust_press_sd)
);

// Sample SW1 only when a debounced K3 press occurs.
assign brightness_event_pulse_sd =
    key_adjust_press_sd && !adjust_select_sync_sd[1];

assign contrast_event_pulse_sd =
    key_adjust_press_sd && adjust_select_sync_sd[1];

// Convert the SD-domain one-cycle events into toggles for reliable CDC.
always @(posedge sd_card_clk or posedge rst_sd) begin
    if (rst_sd) begin
        brightness_event_toggle_sd <= 1'b0;
        contrast_event_toggle_sd   <= 1'b0;
    end
    else begin
        if (brightness_event_pulse_sd)
            brightness_event_toggle_sd <= ~brightness_event_toggle_sd;

        if (contrast_event_pulse_sd)
            contrast_event_toggle_sd <= ~contrast_event_toggle_sd;
    end
end

// SD -> VIDEO: brightness event.
cdc_toggle_to_pulse u_brightness_event_cdc (
    .dst_clk          (video_clk),
    .dst_rst          (rst_video),
    .src_toggle_async (brightness_event_toggle_sd),
    .dst_pulse        (brightness_event_pulse_video)
);

// SD -> VIDEO: contrast event.
cdc_toggle_to_pulse u_contrast_event_cdc (
    .dst_clk          (video_clk),
    .dst_rst          (rst_video),
    .src_toggle_async (contrast_event_toggle_sd),
    .dst_pulse        (contrast_event_pulse_video)
);

// Hold an arrived event until the next frame boundary.
// This prevents a brightness/contrast change from splitting one displayed frame.
always @(posedge video_clk or posedge rst_video) begin
    if (rst_video) begin
        brightness_event_pending_video <= 1'b0;
        contrast_event_pending_video   <= 1'b0;
    end
    else begin
        if (brightness_event_pulse_video)
            brightness_event_pending_video <= 1'b1;
        else if (video_frame_tick && brightness_event_pending_video)
            brightness_event_pending_video <= 1'b0;

        if (contrast_event_pulse_video)
            contrast_event_pending_video <= 1'b1;
        else if (video_frame_tick && contrast_event_pending_video)
            contrast_event_pending_video <= 1'b0;
    end
end

assign brightness_apply_pulse_video =
    video_frame_tick && brightness_event_pending_video;

assign contrast_apply_pulse_video =
    video_frame_tick && contrast_event_pending_video;

// Keep the actual parameter state in video_clk domain.
// Reset defaults remain brightness=0 and contrast=1.0x.
image_adjust_control u_image_adjust_control (
    .clk                   (video_clk),
    .rst                   (rst_video),
    .brightness_next_pulse (brightness_apply_pulse_video),
    .contrast_next_pulse   (contrast_apply_pulse_video),
    .brightness            (image_brightness_target),
    .contrast_gain         (image_contrast_gain),
    .brightness_level      (brightness_level),
    .contrast_level        (contrast_level)
);
// D7 status is level-only CDC.  Data comparison itself remains entirely in
// ext_mem_clk; these synchronized sticky levels select a direct video color.
always @(posedge video_clk or posedge rst_video) begin
    if (rst_video) begin
        d7_pass_meta_video      <= 1'b0;
        d7_pass_video           <= 1'b0;
        d7_mismatch_meta_video  <= 1'b0;
        d7_mismatch_video       <= 1'b0;
        d7_timeout_meta_video   <= 1'b0;
        d7_timeout_video        <= 1'b0;
        d7_sequence_meta_video  <= 1'b0;
        d7_sequence_video       <= 1'b0;
        d7_reader_buf1_meta_video <= 1'b0;
        d7_reader_buf1_video      <= 1'b0;
    end
    else begin
        d7_pass_meta_video      <= d7_bist_pass;
        d7_pass_video           <= d7_pass_meta_video;
        d7_mismatch_meta_video  <= d7_bist_mismatch;
        d7_mismatch_video       <= d7_mismatch_meta_video;
        d7_timeout_meta_video   <= d7_bist_timeout;
        d7_timeout_video        <= d7_timeout_meta_video;
        d7_sequence_meta_video  <= d7_bist_sequence_error;
        d7_sequence_video       <= d7_sequence_meta_video;
        d7_reader_buf1_meta_video <= d5_reader_active_buffer_index;
        d7_reader_buf1_video      <= d7_reader_buf1_meta_video;
    end
end

assign d7_direct_status_rgb =
    D7_DIRECT_GREEN_MODE ? 24'h00FF00 :
    d7_sequence_video    ? 24'hFF00FF :
    d7_timeout_video     ? 24'hFFFF00 :
    d7_mismatch_video    ? 24'hFF0000 :
    d7_pass_video        ? 24'h00FF00 :
                           24'h0000FF;

assign d7_use_direct_background =
    D7_BIST_MODE ||
    D7_DIRECT_GREEN_MODE ||
    (D7_VERIFIED_GREEN_MODE &&
     !(d7_pass_video && d7_reader_buf1_video));

assign diagnostic_display_valid_src =
    (D7_BIST_MODE || D7_VERIFIED_GREEN_MODE || D7_DIRECT_GREEN_MODE) ?
    1'b1 :
    ((DISPLAY_DIAG_MODE != 0) ? d5_reader_active_buffer_index :
                                display_valid_src);

// Synchronize the SD/BMP-domain flag and commit it only at vertical blanking.
display_valid_cdc u_display_valid_cdc (
    .video_clk             (video_clk),
    .video_rst             (rst_video),
    .display_valid_src     (diagnostic_display_valid_src),
    .video_vs              (vs_pre_osd),
    .display_valid_pending (display_valid_pending),
    .display_valid_active  (display_valid_active_pre_osd)
);

// ===================== G1-B2 OSD status CDC =====================
osd_status_cdc u_osd_status_cdc (
    .src_auto_play   (osd_auto_play_src),
    .src_img_idx     (osd_img_idx_src),
    .src_img_count   (osd_img_count_src),

    .video_clk       (video_clk),
    .video_rst       (rst_video),
    .video_vs        (vs_pre_osd),

    .video_auto_play (osd_auto_play_video),
    .video_img_idx   (osd_img_idx_video),
    .video_img_count (osd_img_count_video)
);
// =========================================================
// G1-C1 SD -> VIDEO
// Fade-start event
// =========================================================

cdc_toggle_to_pulse u_fade_start_cdc (

    .dst_clk          (video_clk),
    .dst_rst          (rst_video),

    .src_toggle_async (fade_start_toggle_sd),

    .dst_pulse        (fade_start_pulse_video)

);
// =========================================================
// G1-C1 SD -> VIDEO
// New image ready event
// =========================================================

cdc_toggle_to_pulse u_image_ready_cdc (

    .dst_clk          (video_clk),
    .dst_rst          (rst_video),

    .src_toggle_async (image_ready_toggle_sd),

    .dst_pulse        (image_ready_pulse_video)

);
// =========================================================
// G1-C1 VIDEO-domain pulse -> toggle
//
// midpoint:
//     completely black
//
// done:
//     fade-in completely finished
// =========================================================

always @(posedge video_clk or posedge rst_video) begin

    if (rst_video) begin

        fade_midpoint_toggle_video <= 1'b0;
        fade_done_toggle_video     <= 1'b0;

    end
    else begin

        if (fade_midpoint_pulse_video)
            fade_midpoint_toggle_video <=
                ~fade_midpoint_toggle_video;

        if (fade_done_pulse_video)
            fade_done_toggle_video <=
                ~fade_done_toggle_video;

    end

end
// =========================================================
// G1-C1 VIDEO -> SD
// Black midpoint event
// =========================================================

cdc_toggle_to_pulse u_fade_midpoint_cdc (

    .dst_clk          (sd_card_clk),
    .dst_rst          (rst_sd),

    .src_toggle_async (fade_midpoint_toggle_video),

    .dst_pulse        (fade_midpoint_pulse_sd)

);


// =========================================================
// G1-C1 VIDEO -> SD
// Fade-complete event
// =========================================================

cdc_toggle_to_pulse u_fade_done_cdc (

    .dst_clk          (sd_card_clk),
    .dst_rst          (rst_sd),

    .src_toggle_async (fade_done_toggle_video),

    .dst_pulse        (fade_done_pulse_sd)

);
// ===================== G1-C1 Fade transition =====================
//
// Only the BMP/image layer is faded.
//
// The OSD is placed AFTER this module so that:
//
// MODE
// IMAGE
// English scrolling text
// Chinese scrolling text
//
// remain bright and readable during the transition.
//
// =============================================================
// G1-C2 Image Adjustment
//
// Processing order:
//
// SDRAM/BMP RGB
//      -> Contrast
//      -> Brightness
//      -> Fade
//      -> OSD
//      -> HDMI
//
// Hardware-controlled settings from image_adjust_control:
// K3 + SW1 select brightness or contrast.
// Reset defaults: brightness = 0, contrast = 1.0x = 256
// =============================================================

video_image_adjust u_video_image_adjust (

    .rgb_in        (vout_data_raw),

    .brightness    (image_brightness),
    .contrast_gain (image_contrast_gain),

    .rgb_out       (vout_data_adjusted)

);
video_transition_fade_hold u_video_transition_fade_hold (

    .clk              (video_clk),
    .rst              (rst_video),

    .frame_tick       (video_frame_tick),

    .transition_start (fade_start_pulse_video),

    .image_ready      (image_ready_pulse_video),

    .rgb_in(vout_data_adjusted),
    .rgb_out          (vout_data_fade),

    .transition_busy  (fade_transition_busy),
    .midpoint_pulse   (fade_midpoint_pulse_video),
    .transition_done  (fade_done_pulse_video),

    .fade_level       (fade_level),

    .black_hold       (fade_black_hold)

);
// ===================== G1-B OSD overlay =====================
// Overlay FPGA-generated text on the faded RGB pixel stream.
// In D6 diagnostic builds the framebuffer still comes through physical SDRAM,
// but adjustment/fade are bypassed so the expected black/white/green values
// cannot be hidden by an unrelated SD-card transition state.  OSD remains last.
assign vout_data_osd_in =
    d7_use_direct_background ? d7_direct_status_rgb :
    ((DISPLAY_DIAG_MODE != 0) ? vout_data_raw : vout_data_fade);

osd_overlay_640x480 #(
    .FONT_BRAM(1),.PRESENTATION_ENABLE(1),
    .PRESENTATION_ICON_ID(PRESENTATION_ICON_ID),
    .PRESENTATION_ICON_COL(PRESENTATION_ICON_COL),
    .PRESENTATION_ICON_ROW(PRESENTATION_ICON_ROW),
    .PRESENTATION_HOTSPOT1(PRESENTATION_HOTSPOT1),
    .PRESENTATION_HOTSPOT2(PRESENTATION_HOTSPOT2),
    .PRESENTATION_HOTSPOT3(PRESENTATION_HOTSPOT3),
    .PRESENTATION_HOTSPOT4(PRESENTATION_HOTSPOT4)
) u_osd_overlay_640x480 (
    .clk              (video_clk),
    .rst              (rst_video),
    .vs               (vs_pre_osd),
    .de               (de_pre_osd),
    .presentation_mode(presentation_mode),
    .presentation_key_next_n(presentation_key_next_n),

    .auto_play        (osd_auto_play_video),
    .img_idx          (osd_img_idx_video),
    .img_count        (osd_img_count_video),

    // G1-C2 brightness / contrast status
    .brightness_level (brightness_level),
    .contrast_level   (contrast_level),

    .rgb_in           (vout_data_osd_in),
    .rgb_out          (vout_data_osd)
);
// Keep black until the synchronized frame-boundary commit is active.
assign vout_data = display_valid_active ? vout_data_osd : 24'd0;

// ============================================================================
// D5 final image-memory path
//
// OLD:
//   frame_read_write
//     + source double-buffer write
//     + HDMI-time adaptive/random read mapper
//     + return mask / dummy black reads
//
// NEW:
//   TF/BMP -> SOURCE_BUFFER
//          -> Bilinear frame builder
//          -> DISPLAY_BUF0 / DISPLAY_BUF1
//          -> fixed sequential reader
//          -> existing video_delay / Fade / OSD / HDMI
// ============================================================================
d5_image_memory_path #(
    .ENABLE_AUDIO_SHARE(1),
    .ADDR_BITS      (ADDR_BITS),
    .MEM_DATA_BITS  (MEM_DATA_BITS),

    .SOURCE_BASE    (SOURCE_BUF_ADDR),
    .SOURCE_WORDS   (SOURCE_BUF_WORDS),

    .DISP0_BASE     (DISP0_ADDR),
    .DISP1_BASE     (DISP1_ADDR),

    .FRAME_PIXELS      (FRAME_PIXELS),
    .DISPLAY_DIAG_MODE (DISPLAY_DIAG_MODE)
) d5_image_memory_path_m0 (
    .audio_pause(audio_pause), .transport_busy(transport_busy),
    .mem_clk        (ext_mem_clk),
    .mem_rst        (rst_mem),

    .sd_clk         (sd_card_clk),
    .sd_rst         (rst_sd),

    .video_clk      (video_clk),
    .video_rst      (rst_video),

    // Common asynchronous reset for the two dual-clock image FIFOs.
    .fifo_rst       (rst_all),

    // ---------------- SDRAM controller ----------------
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),

    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    // ---------------- d5_sd_card_bmp source stream ----------------
    .sd_write_req      (sd_card_write_req),
    .sd_write_req_ack  (sd_card_write_req_ack),

    .sd_write_len      (adaptive_write_len_sd),
    .sd_write_en       (sd_card_write_en),
    .sd_write_data     (sd_card_write_data),

    // d5_sd_card_bmp already selects the idle display target:
    // first frame -> BUF0, then BUF0/BUF1 alternate.
    .sd_write_buf_idx  (write_buf_idx),

    .sd_src_width      (adaptive_src_width_sd),
    .sd_src_height     (adaptive_src_height_sd),
    .sd_dst_width      (adaptive_dst_width_sd),
    .sd_dst_height     (adaptive_dst_height_sd),
    .sd_x_offset       (adaptive_x_offset_sd),
    .sd_y_offset       (adaptive_y_offset_sd),
    .sd_geometry_valid (adaptive_geometry_valid_sd),

    // Full target DISPLAY frame completion -> existing d5_sd_card_bmp protocol.
    .write_finish_toggle_out (frame_write_toggle_mem),

    // Existing display-buffer commit transaction remains unchanged externally.
    .sd_disp_commit_toggle     (disp_buf_commit_toggle),
    .sd_disp_buf_idx           (disp_buf_idx),
    .sd_disp_commit_ack_toggle (disp_buf_commit_ack_toggle),

    // ---------------- existing video interface ----------------
    .video_read_req      (video_read_req),
    .video_frame_tick    (video_frame_tick),
    .video_read_req_ack  (video_read_req_ack),

    .video_read_en       (video_read_en),
    .video_read_data     (video_read_data),
    .video_read_finish   (),

    // ---------------- D5 debug/status ----------------
    .mem_phase                   (d5_mem_phase),

    .source_write_active         (d5_source_write_active),
    .builder_busy                (d5_builder_busy),
    .reader_idle                 (d5_reader_idle),
    .reader_active_buffer_index  (d5_reader_active_buffer_index),

    .geometry_error_sticky       (d5_geometry_error_sticky),
    .commit_ack_mismatch_sticky  (d5_commit_ack_mismatch_sticky),

    .source_fifo_overflow_sticky (d5_source_fifo_overflow_sticky),
    .source_fifo_underflow_sticky(d5_source_fifo_underflow_sticky),
    .source_range_error_sticky   (d5_source_range_error_sticky),

    .reader_return_without_command_sticky
                                 (d5_reader_return_without_command_sticky),
    .reader_credit_overflow_sticky
                                 (d5_reader_credit_overflow_sticky),
    .reader_frame_start_pending_sticky
                                 (d5_reader_frame_start_pending_sticky),
    .reader_command_order_error_sticky
                                 (d5_reader_command_order_error_sticky),
    .reader_fifo_underflow_sticky
                                 (d5_reader_fifo_underflow_sticky),
    .reader_fifo_overflow_sticky (d5_reader_fifo_overflow_sticky),

    .d7_bist_running             (d7_bist_running),
    .d7_bist_completed           (d7_bist_completed),
    .d7_bist_pass                (d7_bist_pass),
    .d7_bist_mismatch            (d7_bist_mismatch),
    .d7_bist_timeout             (d7_bist_timeout),
    .d7_bist_sequence_error      (d7_bist_sequence_error),
    .d7_bist_test_id             (d7_bist_test_id),
    .d7_bist_state               (d7_bist_state),
    .d7_bist_write_attempt_count (d7_bist_write_attempt_count),
    .d7_bist_write_accept_count  (d7_bist_write_accept_count),
    .d7_bist_read_command_count  (d7_bist_read_command_count),
    .d7_bist_read_accept_count   (d7_bist_read_accept_count),
    .d7_bist_read_return_count   (d7_bist_read_return_count),
    .d7_bist_compare_count       (d7_bist_compare_count),
    .d7_bist_mismatch_count      (d7_bist_mismatch_count),
    .d7_bist_timeout_count       (d7_bist_timeout_count),
    .d7_bist_first_error_valid   (d7_bist_first_error_valid),
    .d7_bist_first_error_test    (d7_bist_first_error_test),
    .d7_bist_first_error_addr    (d7_bist_first_error_addr),
    .d7_bist_first_error_expected(d7_bist_first_error_expected),
    .d7_bist_first_error_actual  (d7_bist_first_error_actual),
    .d7_bist_first_timeout_state (d7_bist_first_timeout_state),
    .d7_bist_error_xor_or        (d7_bist_error_xor_or),
    .d7_bist_even_addr_error_count(d7_bist_even_addr_error_count),
    .d7_bist_odd_addr_error_count (d7_bist_odd_addr_error_count),
    .d7_bist_addr_bit0_error_count(d7_bist_addr_bit0_error_count),
    .d7_bist_addr_bit1_error_count(d7_bist_addr_bit1_error_count),
    .d7_bist_addr_bit2_error_count(d7_bist_addr_bit2_error_count),
    .d7_bist_addr_bit3_error_count(d7_bist_addr_bit3_error_count),
    .d7_display_app_rd_cmd_count (d7_display_app_rd_cmd_count),
    .d7_display_sdr_return_count (d7_display_sdr_return_count),
    .d7_display_fifo_write_count (d7_display_fifo_write_count),
    .d7_display_video_pop_count  (d7_display_video_pop_count),
    .d7_video_pop_while_empty_count(d7_video_pop_while_empty_count),
    .d7_min_fifo_level           (d7_min_fifo_level),
    .d7_max_fifo_level           (d7_max_fifo_level)
);

endmodule
