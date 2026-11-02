
module top(
 output wire startup_clk50_alive,startup_video_alive,startup_ready,startup_lcd_done,
 output wire link_cs_n,link_sck,link_mosi,input wire link_miso,
 output wire [10:0] link_mosi_extra,input wire [10:0] link_miso_extra,

    inout wire [15:0] lcd_d,
    output wire lcd_cs_n,lcd_rs,lcd_wr_n,lcd_rd_n,lcd_rst_n,lcd_bl,
    inout wire tp_scl,tp_sda,tp_int_n,
    output wire tp_rst_n,

    input wire adjust_select, // SW1=C8: 0 brightness, 1 contrast
    input                       eth_rxc,
    input                       eth_rx_ctl,
    input [3:0]                 eth_rxd,
    output                      eth_txc,
    output                      eth_tx_ctl,
    output [3:0]                eth_txd,
    output                      eth_mdc,
    inout                       eth_mdio,
    output [1:0]                eth_link_speed,
    output                      eth_rx_overflow,
    output                      eth_tx_overflow,
    output                      eth_ip_error,
    output                      eth_arp_error,
    input                       clk,
    input                       rst_n,
    input                       pir_out,        // HC-SR501 人体在场 OUT(M16): 连续高=在场; 连续低 60s=音频软复位
    input                       sw3,            // Reserved while Ethernet is disabled; WAV source is TF/SD
    input uart_rx,
    output uart_tx,
    input spectrum_mic_sw, // SW2=C7: 1=MIC spectrum, 0=WAV spectrum
    input audio_key3_n, // Physical KEY3=B1, active low
    input audio_mode_sw, // SW4=C5: high audio keys, low image keys; SW2/SW3 unused
    input                       key1,           // 手动下一张
    input                       key2,           // 自动播放 开/关

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,

    // HUB75 (64x32 频谱屏) —— 逻辑编号已由用户固定(GPIOA0..A12/J1), package pin 暂缓
    output                      HUB75_R1, HUB75_G1, HUB75_B1,
    output                      HUB75_R2, HUB75_G2, HUB75_B2,
    output                      HUB75_A,  HUB75_B,  HUB75_C,  HUB75_D,
    output                      HUB75_CLK, HUB75_LAT, HUB75_OE
);
// PHY status remains internal; no external debug PADs.
wire eth_phy_ready,eth_phy_fault,eth_link_up,eth_full_duplex;
wire link_online,link_remote_mic,link_person,link_light_ready,link_light_fault;
wire[1:0] link_light_level;wire[11:0] link_light_adc;
wire[383:0] link_heights;wire link_heights_valid,link_spectrum_ready;
parameter FM_INTEGRATION_ENABLE=1;
parameter FM_SCAN_ENABLE=1;
parameter FM_TIMESHIFT_ENABLE = 1; // Independent compile-time bypass.
parameter AUDIO_DSP_ENABLE=1;
wire [127:0] dsp_status_video,dsp_status_lcd;wire dsp_status_valid,dsp_ui_update;
wire lcd_dsp_event,video_dsp_event;wire [3:0] lcd_dsp_id,video_dsp_id;
wire touch_v,touch_sd;
wire [3:0] touch_id_v,touch_id_sd;
wire compare_valid_video;wire [3:0] compare_cmd_video;

wire lcd_scan_event,video_scan_event,lcd_scan_page;
wire [3:0] lcd_scan_id,video_scan_id;
wire [127:0] scan_status_video,scan_status_lcd;
wire scan_status_valid,scan_ui_update,scan_point_valid;
wire [7:0] scan_generation,scan_index;wire [15:0] scan_data;
wire scan_point_lcd_valid;wire [31:0] scan_point_lcd;
reg [1:0] scan_page_sync;
wire video_clk;

wire rst_all;

wire wav_wav_play;

wire touch_music_start;


wire session_resetting;
wire lcd_fm_event,video_fm_event;wire [3:0] lcd_fm_id,video_fm_id;
wire fm_request_valid;wire [7:0] fm_request_seq;wire [2:0] fm_request_op;wire [15:0] fm_request_value;
wire [127:0] fm_status;wire fm_status_valid;wire [47:0] fm_ui_video,fm_ui_lcd;
wire fm_local_error,fm_ui_update;
reg link_mode_mic,legacy_mode_mic;
always @(posedge video_clk or posedge rst_all)
 if(rst_all)legacy_mode_mic<=0;
 else if(touch_v && touch_id_v==14)legacy_mode_mic<=1;
 else if(touch_v && touch_id_v==12)legacy_mode_mic<=0;
always @(posedge video_clk or posedge rst_all)
 if(rst_all)link_mode_mic<=0;
 else if(touch_v && touch_id_v==14)link_mode_mic<=1;
 else if(touch_v && touch_id_v==12)link_mode_mic<=0;
 else if(FM_INTEGRATION_ENABLE && fm_status_valid)link_mode_mic<=fm_status[104:103]==1;



// Ambient status is declared before UART connections to prevent implicit nets.
wire light_vs,light_de;
wire [23:0] light_rgb;
wire [11:0] light_filtered;
wire [1:0] light_level;
wire [4:0] light_gain;
wire light_ready,light_fault;
wire [11:0] light_raw;
wire [7:0] light_samples;
wire [6:0] light_frames;
wire [7:0] light_rejected,light_averages;

parameter TF_VIDEO_ENABLE = 1;
parameter NET_VIDEO_ENABLE = 1;
wire [95:0] net_video_pixels;wire [2:0] net_video_flags;
wire net_video_valid,net_video_ready,net_video_remote_active,net_video_remote_done,net_video_remote_error;
wire net_image_active,net_image_displayed,net_image_error;wire [1:0] net_image_credit_reason;
reg tf_route_hold;
wire net_video_selected=NET_VIDEO_ENABLE && link_online && fm_status[106:105]==3 && soft_video_run && !tf_route_hold;
wire net_video_enabled=net_video_selected && !video_session && !session_resetting;
reg [11:0] net_video_audio_count;reg net_video_due;
parameter TF_VIDEO_HQ_ENABLE = 1;
parameter TF_VIDEO_FAST_ENABLE = 1;
parameter TF_VIDEO_SPI_FAST_ENABLE = 1;
wire lcd_video_event,video_event;wire [3:0] lcd_video_id,video_id;
wire [63:0] tf_video_burst;wire tf_video_burst_valid,tf_video_burst_last,tf_video_burst_ready;
wire tf_video_profile_fast_sd;wire tf_video_profile_sd;wire tf_video_enable_sd,tf_video_sample;wire [3:0] tf_video_error_sd;
wire video_owned,video_pause_legacy,video_dma_busy,video_wr,video_rd;
wire [20:0] video_wr_addr,video_rd_addr;wire [31:0] video_wr_data;
wire [23:0] tf_video_rgb;wire tf_video_display,tf_video_under,tf_video_late,tf_video_fault;
wire [7:0] tf_video_frame_count;
reg [15:0] video_status_meta,video_status_lcd;

wire video_start;
wire video_stop;
wire video_pause_event;
wire net_video_pause_event=video_event && video_id==6 && net_video_selected;
wire video_session;
wire video_session_cmd;
wire media_session_play;
wire [87:0] media_session_name;
wire [23:0] media_bg;reg [23:0] image_bg_delayed;reg media_vs,media_de;
parameter MEM_DATA_BITS = 32;
parameter integer SW3_DEBOUNCE_CYCLES = 500000; // 10 ms at 50 MHz
parameter [31:0] ETH_LOCAL_IP = 32'hc0a8f001; // 192.168.240.1
parameter [47:0] ETH_LOCAL_MAC = 48'h020000000001;
parameter [31:0] ETH_PC_IP = 32'hc0a8f002;    // 192.168.240.2
parameter [15:0] ETH_LOCAL_PORT = 16'd1;
parameter [15:0] ETH_PC_PORT = 16'd50000;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter [20:0] FRAME_PIXELS  = 21'd307200;   // 640*480=307200, <2^21, 21bit 可容纳
parameter [20:0] BUF0_ADDR     = 21'd0;        // 显示缓冲 0 起始地址
parameter [20:0] BUF1_ADDR     = FRAME_PIXELS; // 显示缓冲 1 起始地址
parameter [20:0] BUF2_ADDR     = FRAME_PIXELS * 2;
parameter [20:0] BUF3_ADDR     = FRAME_PIXELS * 3;

// ===================== 音频可视化(扩展要求5) 工程级参数 =====================
// 所有 FFT/频谱参数统一在此集中配置，子模块一律以参数传递，不自行定义。
parameter FFT_N                = 1024;    // FFT 点数
parameter FFT_LOG2_N           = 10;      // log2(FFT_N)
parameter FFT_INPUT_BITS       = 16;      // 送入 FFT 的采样位宽(取24bit高16位)
parameter FFT_INTERNAL_BITS    = 26;      // FFT 实/虚部数据位宽(防溢出)
parameter FFT_COEF_BITS        = 16;      // 窗/旋转因子位宽 Q1.15
parameter FFT_SAMPLE_RATE      = 48000;   // 音频采样率 48kHz
parameter FFT_HOP_SIZE         = 256;     // 滑窗步进(每帧新采样数)=75% overlap
parameter SPECTRUM_BIN_NUM     = 64;      // HUB75 频带(柱)数
parameter SPECTRUM_BAND_SHIFT  = 3;       // 每带 2^3=8 bin (bin1..512 -> 64 带, ~0..24kHz)
parameter SPECTRUM_SCALE_SHIFT = 15;      // (保留未用)
parameter SPECTRUM_MAX_BAR     = 71;      // (保留未用)
parameter SPECTRUM_X_START     = 368;     // 频谱区左边界(右下角)
parameter SPECTRUM_X_END       = 623;     // 右边界(区宽160)
parameter SPECTRUM_Y_TOP       = 16;     // 区上边界
parameter SPECTRUM_Y_BOTTOM    = 111;     // 区下边界(=TOP+MAX_BAR-1)
parameter BAR_WIDTH            = 3;       // 柱宽
parameter BAR_GAP              = 2;       // 柱间距 (32*3+31*2=158<=160)
parameter SMOOTH_STEP          = 1;       // 柱高每 video_clk 平滑步长
parameter SPECTRUM_ENABLE        = 1;     // 1=启用频谱叠加; 0=final_rgb 直接透传 vout_data(诊断用)
parameter SPECTRUM_DEBUG_BYPASS  = 0;     // 1=spectrum_renderer 纯透传 I_rgb_bg(跳过整层绘制,诊断用)

// ===================== HUB75 频谱屏 (Phase3) 工程级参数 =====================
parameter HUB75_ENABLE           = 0;     // 1=启用 HUB75 输出
parameter HUB75_TEST_MODE        = 0;     // 1=固定测试图(阶梯); 0=FFT 频谱
parameter HUB75_SCAN_ROWS        = 16;    // 1/16 扫描地址数
parameter HUB75_WIDTH            = 64;
parameter HUB75_HEIGHT           = 32;
parameter HUB75_CLK_DIV          = 2;     // 每列移位用 video_clk 数(>=2, 建议2的幂)
parameter HUB75_OE_ACTIVE_LEVEL  = 0;     // OE 有效电平(0=低使能; 参考 EN_0 亮)
parameter HUB75_SMOOTH_STEP      = 1;     // 柱高动画步长(每次分析)
parameter HUB75_BRIGHTNESS       = 40;    // 每行 OE 亮保持的 video_clk 数(亮度)
parameter SPECTRUM_LOG_SHIFT    = 16;    // 幅值>>LOG_SHIFT 后 log 压缩(灵敏度, 上板调)
parameter SPECTRUM_HDMI_ENABLE  = 1;     // 1=HDMI 叠加 64 频带透明频谱(右下角); 0=纯 BMP

// ===== HDMI 频谱视觉美化参数(统一在 top, 下传 hdmi_spectrum_overlay) =====
parameter SPECTRUM_COLOR_MODE    = 1;     // 1=列渐变(蓝->红)
parameter SPECTRUM_FADE_ENABLE   = 1;     // 1=柱顶 4 级 fade 融入当前背景
parameter SPECTRUM_FADE_PERCENT  = 20;    // 文档用(实现≈18.75%~25%)
parameter SPECTRUM_FADE_LEVELS   = 4;     // 25/50/75/100%
parameter SPECTRUM_SMOOTH_STEP   = 1;
parameter [23:0] SPECTRUM_COLOR0 = 24'h0080FF;   // blue
parameter [23:0] SPECTRUM_COLOR1 = 24'h00FFFF;   // cyan
parameter [23:0] SPECTRUM_COLOR2 = 24'h00FF40;   // green
parameter [23:0] SPECTRUM_COLOR3 = 24'hFFFF00;   // yellow
parameter [23:0] SPECTRUM_COLOR4 = 24'hFF8000;   // orange
parameter [23:0] SPECTRUM_COLOR5 = 24'hFF2020;   // red

// ===================== 开机音乐 ROM+DDS (Version 9) 工程级参数(集中于此) =====================
parameter STARTUP_MUSIC_ENABLE        = 1;    // 1=首帧后就绪播开机音一次; 0=首帧后直接 normal(不播)
parameter STARTUP_MUSIC_BPM           = 120;  // 4/4 拍速(quarter=0.5s=24000 samples @48k)
parameter signed [23:0] STARTUP_MUSIC_AMP = 24'sd2000000; // DDS 方波幅度(<=2^23)
parameter STARTUP_MUSIC_ROM_DEPTH     = 128;  // 音符表深度(容纳 62 音 M1-5+M7-13, 无 M6/无 repeat)
parameter STARTUP_MUSIC_NOTE_NUM      = 62;   // 有效音符事件数(ROM addr 0..61)
parameter STARTUP_WAIT_FOR_PLL_LOCK   = 1;    // 1=等 audio PLL 锁定后才释放 player_rst(用已有 audio_pll_lock/rst_all); 0=仅等 rst_n
parameter STARTUP_AUDIO_SETTLE_MS     = 300;  // V12: 首帧 BMP 就绪后 Audio Settle(ms); 以 48k audio_valid 计; 0=不 settle
parameter STARTUP_DEBUG_DISPLAY       = 1;    // V26 Step8: 1=启用"单数码管启动态显示"(digit0 显示 0/1/2/3, 其他位息灭); 0=保持原 6 位数码管调试映射
                                              //   注: 本参数不再改变启动生命周期(路径A 必须完整走完三步走)

// V26 Step8 Phase-B: 最终 WAV 链参数(只接线, 禁止改地址)
parameter [20:0] AUDIO_BASE_ADDR = 21'd1703936; // D5 display buffers end at word 1662975; audio ring starts at 0x1A0000.
parameter        AUDIO_TEST_MODE      = 0;           // 1=WAV 不可用(UNAVAIL)时 audio_mux3 sel2 输出原 I2S TestTone(DEBUG); 0=静音

// ===================== HC-SR501 人体在场检测(音频软复位) 工程级参数 =====================
// 高电平须连续 >= 该毫秒数才认定"有人在场", 抑制线缆 EMI/电源毛刺(HC-SR501 真实触发高
// 电平 >= ~0.3s; 门限建议 10..100, 必须 < 300)。连续低 60s 判定缺勤 -> 复位音频链。
parameter integer PIR_PRESENCE_QUALIFY_MS = 20;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;

wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire [23:0] final_rgb;
wire [23:0] hdmi_overlay_rgb;   // hdmi_spectrum_overlay 输出
wire filtered_media_vs,filtered_media_de;wire [23:0] filtered_media_bg;
wire        display_valid;

// ===== 音频可视化(Phase3 HUB75)：分析链内部连线 =====
wire        spectrum_heights_valid;
wire [SPECTRUM_BIN_NUM*6-1:0] spectrum_heights;   // 64 列 x 6bit 柱高 0..31
wire        spectrum_busy;
wire [SPECTRUM_BIN_NUM*6-1:0] test_h;     // 测试图
wire [SPECTRUM_BIN_NUM*6-1:0] target_h;   // 目标柱高(测试/FFT 二选一)
wire [SPECTRUM_BIN_NUM*6-1:0] cur_h;      // 平滑后当前柱高

// Phase5: HDMI = 原 BMP + 64 频带透明频谱叠加(右下角, 无面板, 细柱)。
// HUB75 与 HDMI 共用同一份 cur_h(同一 FFT/64band), 不复制分析链。
// V27S3: Audio FSM 观测线网声明前移(频谱开关需引用, 避免前向引用)
wire [2:0] wav_ctl_phase;       // Audio FSM 相位: A_WAIT_SD=0/A_DELAY=1/A_MUSIC=2/A_SEARCH=3/A_PLAY=4/A_ERROR=5
wire       au_music_start;      // Audio FSM -> Startup Music 启动单拍
wire       au_wav_search;       // Audio FSM -> A_SEARCH(WAV locator/预取)
wire [2:0] au_digit;            // Audio FSM debug digit(0..4)
wire       su_spectrum_on;      // 旧 Video FSM 频谱开关(保留观测, 不再驱动显示门控)
// V27S3: 频谱开关改由 Audio FSM 驱动(Startup Music=A_MUSIC / WAV 播放=A_PLAY 时 ON;
//   等 SD/延时/搜索/错误=OFF)。旧耦合(O_spectrum_on 受 BMP 扫描完成影响)已拆除。
wire mic_adc_clk,mic_selected,mic_valid;
wire [23:0] mic_pcm;
wire spectrum_on = link_online && link_spectrum_ready && (link_remote_mic==link_mode_mic);
// Spectrum stage captures runtime enable with its matching background.
assign final_rgb = hdmi_overlay_rgb;

wire [3:0]  state_code;
wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
wire        frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;
wire [1:0]  disp_buf_idx_mem;


wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

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
wire        tone_audio_valid;
// video_pll produces 25 MHz. Generate exactly 48000 sample enables/s.
audio_sample_tick #(.CLK_HZ(25000000),.SAMPLE_HZ(48000)) u_sample_tick (
    .clk(video_clk), .rst(rst_all), .tick(audio_valid)
);
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
// ---- 开机音乐(ROM+DDS)与 MUX ----
wire        startup_active;       // 1=开机音播放中
wire        startup_done;         // 1=开机音已播完(永不重启)
wire signed [23:0] startup_left, startup_right;
wire signed [23:0] aud_mux_l, aud_mux_r;   // MUX 后 PCM(HDMI 音频 + FFT 共用)

// All user media commands now come from LCD/keys.
wire uart_play=1'b0;
wire panel_play;
wire panel_pause_cmd;
wire image_key3_n; // Reserved image KEY3 handoff, active low; no image action yet.

// ===================== HC-SR501 人体在场 -> 音频软复位(PIR) =====================
// 移植自附件 V27S14-R2/R3: 软复位只作用于音频私有链路(音频启动 FSM + WAV/开机音通路),
// 不触碰视频/TF 图像链与共享 SD 控制器。
//   连续高 >= PIR_PRESENCE_QUALIFY_MS(20ms) -> 认定在场, rst_soft=1(运行);
//   连续低 60s -> rst_soft=0(复位音频链并静音); 未达标短毛刺忽略且不清缺勤计时。
// BOARD_A: PIR sensing moved to B. Keep original below as comments.
// reg  middle_rst, middle_rst_sync;
// // startup_done comes from video_clk. Arm once after the actual boot song ends;
// // subsequent PIR/session resets must not restart this startup exemption.
// reg [1:0] pir_startup_done_sync;
// reg pir_armed;
// always @(posedge clk or negedge rst_n) begin
//     if (!rst_n) begin
//         pir_startup_done_sync <= 2'b00;
//         pir_armed <= 1'b0;
//     end else begin
//         pir_startup_done_sync <= {pir_startup_done_sync[0], startup_done};
//         if (pir_startup_done_sync[1]) pir_armed <= 1'b1;
//     end
// end
// reg  [23:0] pir_high_cnt;      // 连续高计数(尖峰抑制; 饱和保持)
// reg  [31:0] pir_low_count;     // 连续低计数(缺勤计时)
// reg  rst_soft;                 // 1=在场/运行; 0=缺勤/软复位
// localparam [31:0] PIR_LOW_TIMEOUT_LAST = 32'd2_999_999_999;   // 60s @50MHz
// localparam [23:0] PIR_PRESENCE_QUALIFY_CYCLES = PIR_PRESENCE_QUALIFY_MS * 50_000;
// wire pir_high_qualified = (pir_high_cnt == PIR_PRESENCE_QUALIFY_CYCLES);
// always @(posedge clk or negedge rst_n) begin
//     if (!rst_n) begin
//         middle_rst      <= 1'b0;
//         middle_rst_sync <= 1'b0;
//         pir_high_cnt    <= 24'd0;
//         pir_low_count   <= 32'd0;
//         rst_soft        <= 1'b1; // Global reset still applies independently.
//     end else begin
//         middle_rst      <= pir_out;
//         middle_rst_sync <= middle_rst;
// 
//         if (!pir_armed) begin
//             // Do not mute boot audio or accumulate absence during startup.
//             pir_high_cnt  <= 24'd0;
//             pir_low_count <= 32'd0;
//             rst_soft      <= 1'b1;
//         end else begin
//         if (middle_rst_sync == 1'b1) begin
//             if (pir_high_cnt != PIR_PRESENCE_QUALIFY_CYCLES)
//                 pir_high_cnt <= pir_high_cnt + 24'd1;
//         end else begin
//             pir_high_cnt <= 24'd0;
//         end
// 
//         if (pir_high_qualified) begin          // 在场: 运行, 清零缺勤计时
//             pir_low_count <= 32'd0;
//             rst_soft      <= 1'b1;
//         end else if (middle_rst_sync == 1'b1) begin
//             // 未达标短高脉冲进行中: 保持现状(不清计时、不倒计时)
//         end else if (rst_soft == 1'b1) begin   // 连续低: 60s 到 -> 软复位
//             if (pir_low_count == PIR_LOW_TIMEOUT_LAST) begin
//                 pir_low_count <= PIR_LOW_TIMEOUT_LAST;
//                 rst_soft      <= 1'b0;
//             end else begin
//                 pir_low_count <= pir_low_count + 32'd1;
//             end
//         end else begin
//             pir_low_count <= 32'd0;
//             rst_soft      <= 1'b0;
//         end
//         end
//     end
// end
// 
// Remote PIR, with the original one-shot boot-music exemption on A.
reg[1:0] link_person_sync,link_online_sync,link_boot_sync;
reg link_pir_armed;
// Receive video-domain status on the 50 MHz falling edge; keep both stages timed.
always @(negedge clk or negedge rst_n)begin
 if(!rst_n)begin link_person_sync<=3;link_online_sync<=0;link_boot_sync<=0;link_pir_armed<=0;end
 else begin
  link_person_sync<={link_person_sync[0],link_person};
  link_online_sync<={link_online_sync[0],link_online};
  link_boot_sync<={link_boot_sync[0],startup_done};
  if(link_boot_sync[1])link_pir_armed<=1;
 end
end
wire rst_soft=(!link_pir_armed || !link_online_sync[1]) ? 1'b1 : link_person_sync[1];
// rst_soft -> 三个时钟域: 快速置位, 释放沿对齐各域时钟负沿(工程既有"下降沿释放"约定,
// 与 audio_uart_session 同策略, 避开异步复位 recovery/removal 窗口)。
reg [1:0] soft_sd_ps, soft_mem_ps, soft_video_ps;
reg       soft_sd_run, soft_mem_run, soft_video_run;   // 1=该域音频链运行(0=缺勤复位)
always @(posedge sd_card_clk or negedge rst_n)
    if (!rst_n) soft_sd_ps    <= 2'b00; else soft_sd_ps    <= {soft_sd_ps[0],    rst_soft};
always @(posedge ext_mem_clk or negedge rst_n)
    if (!rst_n) soft_mem_ps   <= 2'b00; else soft_mem_ps   <= {soft_mem_ps[0],   rst_soft};
always @(posedge video_clk or negedge rst_n)
    if (!rst_n) soft_video_ps <= 2'b00; else soft_video_ps <= {soft_video_ps[0], rst_soft};
always @(negedge sd_card_clk or negedge rst_n)
    if (!rst_n) soft_sd_run    <= 1'b0; else soft_sd_run    <= soft_sd_ps[0];
always @(negedge ext_mem_clk or negedge rst_n)
    if (!rst_n) soft_mem_run   <= 1'b0; else soft_mem_run   <= soft_mem_ps[0];
always @(negedge video_clk or negedge rst_n)
    if (!rst_n) soft_video_run <= 1'b0; else soft_video_run <= soft_video_ps[0];

reg audio_paused;
wire panel_start = (panel_pause_cmd || touch_music_start) && !wav_wav_play && !session_resetting;
always @(posedge video_clk or posedge rst_all) begin
 if (rst_all) audio_paused<=0;
 else if (!soft_video_run) audio_paused<=0;   // PIR 缺勤: 清除暂停, 人回来自动播放
 else if (uart_play || media_session_play || panel_start || touch_music_start || (video_event && (video_id==4 || video_id==5 || video_id==7))) audio_paused<=0;
 else if ((((panel_pause_cmd || video_pause_event) && wav_wav_play) || net_video_pause_event) && !session_resetting)
  audio_paused<=~audio_paused;
end
wire[23:0] av_features,av_levels;wire av_event,av_enable,av_clap_enable,av_demo,av_step;
wire[1:0] av_strength;wire[4:0] av_pulse,av_ring,av_pulse_original,av_ring_original;
wire panel_image_next_n, panel_image_auto_n;
wire image_auto_short_n;
reg image_vs_d;
always @(posedge video_clk or posedge rst_all)
 if (rst_all) image_vs_d<=1'b1;
 else image_vs_d<=vs;
image_key4_chord u_image_key4_chord (
 .clk(video_clk),.rst(rst_all),.frame_tick(image_vs_d && !vs),
 .enabled(!audio_mode_sw),
 .key4_n(panel_image_auto_n),.key3_n(image_key3_n),
 .auto_n(image_auto_short_n)
);
reg presentation_sw_meta, presentation_sw_sync;
always @(posedge video_clk or posedge rst_all) begin
 if (rst_all) begin presentation_sw_meta<=0; presentation_sw_sync<=0; end
 else begin presentation_sw_meta<=sw3; presentation_sw_sync<=presentation_sw_meta; end
end
wire presentation_mode = av_demo || (presentation_sw_sync && !audio_mode_sw);
reg presentation_mode_sd_meta;
always @(negedge sd_card_clk or posedge rst_all) begin
 if (rst_all) presentation_mode_sd_meta<=0;
 else presentation_mode_sd_meta<=presentation_mode;
end
reg [1:0] image_keys_meta, image_keys_sync;
// First-stage image key capture on the opposite edge avoids same-edge hold skew.
always @(negedge sd_card_clk or posedge rst_all) begin
 if (rst_all) image_keys_meta<=2'b11;
 else image_keys_meta<={(presentation_mode ? 1'b1 : panel_image_next_n),image_auto_short_n};
end
always @(posedge sd_card_clk or posedge rst_all) begin
 if (rst_all) image_keys_sync<=2'b11;
 else image_keys_sync<=image_keys_meta;
end
wire [87:0] panel_wav_name;
wire [87:0] current_wav_name_c;
wire wav_play_finished;
wire [87:0] selected_wav_name, active_wav_name;
wire audio_session_rst_a,audio_session_rst_b,audio_session_rst_c;
wire manual_session;
wire sd_audio_idle, mem_audio_idle;

// PIR 软复位与会话复位合并(仅音频私有链): 会话复位经空闲排空握手后复位; PIR 缺勤立即复位并静音。
// 两路信号均对齐各域时钟负沿(会话复位=audio_uart_session 负沿释放; PIR=上面的负沿寄存器),
// 合并后释放沿仍在负沿, 保持既有 recovery/removal 余量。
wire audio_rst_sd    = audio_session_rst_a | ~soft_sd_run;    // A 域 sd_card_clk
wire audio_rst_mem   = audio_session_rst_b | ~soft_mem_run;   // B 域 ext_mem_clk
wire audio_rst_video = audio_session_rst_c | ~soft_video_run; // C 域 video_clk

// SW3 is asynchronous and mechanical. Synchronize it and require 10 ms of
// stable input before changing the active WAV source. Reset defaults to SD.
// Disabled for the suffix/D5 build; original implementation retained.
// localparam integer SW3_DB_BITS = $clog2(SW3_DEBOUNCE_CYCLES + 1);
// reg sw3_meta=0, sw3_sync=0;
// reg wav_source_net=0;
// reg [SW3_DB_BITS-1:0] sw3_db_count=0;
// always @(posedge sd_card_clk or negedge rst_n) begin
//     if (!rst_n) begin
//         sw3_meta       <= 1'b0;
//         sw3_sync       <= 1'b0;
//         wav_source_net <= 1'b0;
//         sw3_db_count   <= {SW3_DB_BITS{1'b0}};
//     end else begin
//         sw3_meta <= sw3;
//         sw3_sync <= sw3_meta;
//         if (sw3_sync == wav_source_net) begin
//             sw3_db_count <= {SW3_DB_BITS{1'b0}};
//         end else if (sw3_db_count == SW3_DEBOUNCE_CYCLES-1) begin
//             wav_source_net <= sw3_sync;
//             sw3_db_count   <= {SW3_DB_BITS{1'b0}};
//         end else begin
//             sw3_db_count <= sw3_db_count + 1'b1;
//         end
//     end
// end

wire wav_source_net = 1'b0; // Ethernet disabled; SW3 is reserved.

// Ethernet WAV source. app_clk and the WAV byte consumer both use the
// existing 50 MHz sd_card_clk, so the ready/valid stream has no CDC boundary.
wire [7:0]  net_file_byte;
wire        net_file_valid;
wire        net_file_end;
wire        net_file_ready;
wire        net_file_error;
wire [31:0] net_file_size;
wire [31:0] net_next_offset;
wire        net_app_reset;
wire        net_debug_rx_seen;
wire        net_debug_wack_sent;

// Disabled for the suffix/D5 build; original implementation retained.
// fpga_wav_top #(
//     .APP_CLK_HZ  (50000000),
//     .AUTO_SPEED  (1),
//     .SPEED       (2'b10),
//     .LOCAL_IP    (ETH_LOCAL_IP),
//     .LOCAL_MAC   (ETH_LOCAL_MAC),
//     .LOCAL_PORT  (ETH_LOCAL_PORT),
//     .PC_IP       (ETH_PC_IP),
//     .PC_PORT     (ETH_PC_PORT),
//     .LOOPBACK_MODE(0)
// ) u_fpga_wav (
//     .ref_clk_50       (clk),
//     .app_clk          (sd_card_clk),
//     .rst_n            (rst_n),
//     .file_source_enable(wav_source_net),
//     .rgmii_rxc        (eth_rxc),
//     .rgmii_rx_ctl     (eth_rx_ctl),
//     .rgmii_rxd        (eth_rxd),
//     .rgmii_txc        (eth_txc),
//     .rgmii_tx_ctl     (eth_tx_ctl),
//     .rgmii_txd        (eth_txd),
//     .phy_mdc          (eth_mdc),
//     .phy_mdio         (eth_mdio),
//     .file_byte        (net_file_byte),
//     .file_valid       (net_file_valid),
//     .file_ready       (net_file_ready),
//     .file_error       (net_file_error),
//     .file_size        (net_file_size),
//     .file_end         (net_file_end),
//     .next_offset      (net_next_offset),
//     .debug_rx_seen    (net_debug_rx_seen),
//     .debug_wack_sent  (net_debug_wack_sent),
//     .app_reset        (net_app_reset),
//     .phy_ready        (eth_phy_ready),
//     .phy_fault        (eth_phy_fault),
//     .link_up          (eth_link_up),
//     .full_duplex      (eth_full_duplex),
//     .link_speed       (eth_link_speed),
//     .rx_overflow      (eth_rx_overflow),
//     .tx_overflow      (eth_tx_overflow),
//     .ip_error         (eth_ip_error),
//     .arp_error        (eth_arp_error)
// );


assign {eth_txc,eth_tx_ctl,eth_txd,eth_mdc}=7'd0;
assign eth_mdio=1'bz;
assign {eth_phy_ready,eth_phy_fault,eth_link_up,eth_full_duplex,eth_link_speed,eth_rx_overflow,eth_tx_overflow,eth_ip_error,eth_arp_error}=10'd0;
assign net_file_byte=0; assign net_file_valid=0; assign net_file_end=0;
assign net_file_size=0; assign net_next_offset=0; assign net_app_reset=0;
assign net_debug_rx_seen=0; assign net_debug_wack_sent=0;

// ===================== V26 Step8: audio_wav_top_path(最终 WAV 链) 顶层连线 =====================
wire signed [23:0] wav_pcm_l, wav_pcm_r;   // audio_wav_top_path 统一 24bit PCM(video 域, HDMI/FFT 同源)
wire               wav_pcm_v;
// A 域 SD m1 client(<-> sd_read_arbiter.m1 <-> 唯一 sd_card_top)
wire        wav_sd_req;  wire [31:0] wav_sd_addr;
wire [7:0]  wav_sd_data; wire wav_sd_valid, wav_sd_end;
// B 域 Ring mem 口(接 audio_sdram_access.a_*; RING_AW=13 -> 8192 帧 32KB)
wire                wav_ring_wr_req; wire [12:0] wav_ring_wr_addr; wire [31:0] wav_ring_wr_data;
wire                wav_ring_wr_ready;
wire                wav_ring_rd_req; wire [12:0] wav_ring_rd_addr;
wire [31:0]         wav_ring_rd_data; wire wav_ring_rd_valid;
// C 域 MUX/控制/水位(观测)
wire [1:0]  wav_audio_sel;  wire wav_unlock_image;
// (V27S3: wav_ctl_phase / au_music_start / au_wav_search / au_digit 声明已前移至频谱开关处)
wire        wav_ready_c; wire wav_found_c; wire wav_err_c; wire wav_done_c;
wire [13:0] wav_ring_used; wire wav_f1_full, wav_f1_empty, wav_f2_full, wav_f2_empty;
// video 域: frame_read_write(V 主) <-> audio_sdram_access.v_* 口
wire                vrd_en; wire [ADDR_BITS-1:0] vrd_addr;
wire                vrd_valid; wire [MEM_DATA_BITS-1:0] vrd_data;
wire                vwr_en; wire [ADDR_BITS-1:0] vwr_addr;
wire [MEM_DATA_BITS-1:0] vwr_din; wire [3:0] vwr_dm;
wire        player_rst;
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

// ===================== Startup Image + Startup Music (V12) =====================
// 三阶段: WAIT_FIRST_FRAME(0) -> STARTUP_SETTLE(1) -> STARTUP_MUSIC(2) -> NORMAL_PLAY(3)
//  阶段0(首帧 BMP 未就绪): 无 Music/无 TestTone/无频谱/无切图(静音), 等待第一张 BMP。
//  阶段1(首帧就绪): 锁第一张 BMP(=Splash, 非黑屏); 300ms settle(静音/频谱关); 然后 Music+Spectrum。
//  阶段2(music done): 解锁并自动 0->1 切第二张; TestTone+频谱恢复; 手动/自动切图恢复。
// display_valid(sd 域)同步到 video_clk; 其高电平 = 第一张 BMP 已整帧写完成并提交(首帧就绪)。
// Hold reset for >5 ms after configuration / external reset / PLL lock loss.
// PLL frequencies, SDRAM core and HDMI core are unchanged.
// Release from 25 MHz, phase-related to the 125 MHz pixel/board-link logic.
// 2^17 clocks at 25 MHz retains the same approximately 5.24 ms interval.
assign rst_all = ~rst_n | ~audio_pll_lock;
// Independent activity indicators; these never gate the media datapath.
reg [24:0] startup_clk50_count=0;
reg [23:0] startup_video_count=0;
always @(posedge clk or negedge rst_n)
 if(!rst_n)startup_clk50_count<=0;else startup_clk50_count<=startup_clk50_count+1'b1;
always @(posedge video_clk or negedge rst_n)
 if(!rst_n)startup_video_count<=0;else startup_video_count<=startup_video_count+1'b1;
assign startup_clk50_alive=startup_clk50_count[24];
assign startup_video_alive=startup_video_count[23];
assign startup_ready=!rst_all;
reg dv_s0, dv_s1;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin dv_s0 <= 1'b0; dv_s1 <= 1'b0; end
    else begin dv_s0 <= display_valid; dv_s1 <= dv_s0; end
end
wire first_frame_ready = dv_s1;
// ---- V26 Step8: 启动容错判定信号(全部来自真实扫描/定位结果, 无固定 delay) ----
// bmp(sd 域): sd_card_bmp.O_scan_done / O_bmp_found(只观测输出, 扫描 FSM 未改)
wire bmp_scan_done_sd, bmp_found_sd;
wire sd_init_done;               // sd_card_top 初始化完成(sd 域; 声明上移, 供给启动判定用)
// 单比特 CDC: sd_card_clk -> video_clk(2FF, 带 rst_all 复位)
reg bsd_s0, bsd_s1, bfd_s0, bfd_s1, sid_s0, sid_s1;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        bsd_s0 <= 1'b0; bsd_s1 <= 1'b0;
        bfd_s0 <= 1'b0; bfd_s1 <= 1'b0;
        sid_s0 <= 1'b0; sid_s1 <= 1'b0;
    end else begin
        bsd_s0 <= bmp_scan_done_sd;  bsd_s1 <= bsd_s0;
        bfd_s0 <= bmp_found_sd;      bfd_s1 <= bfd_s0;
        sid_s0 <= sd_init_done;      sid_s1 <= sid_s0;
    end
end
wire bmp_scan_done_v = bsd_s1;   // video 域: BMP 扫描结束
wire bmp_found_v     = bfd_s1;   // video 域: 有 BMP(>=1)
wire sd_init_done_v  = sid_s1;   // video 域: SD 初始化完成
wire [1:0] su_digit;             // V26 Step8: 0=扫描中 1=BMP路径 2=WAV路径 3=无媒体
wire su_no_bmp;                  // V26 Step8: 已判定无 BMP
wire [2:0] su_state;            // 0..6 = SCAN/WAIT/SETTLE/MUSIC/NORMAL/NOWAV/NOMEDIA
wire startup_phase;             // SETTLE/MUSIC(锁第一张 BMP/key)
wire music_on;                  // MUSIC -> player I_start
wire test_tone_on;              // NORMAL(C4~C5 Test Tone 允许)
wire music_done_pulse;
startup_controller #(
    .STARTUP_MUSIC_ENABLE   (STARTUP_MUSIC_ENABLE),
    .STARTUP_AUDIO_SETTLE_MS(STARTUP_AUDIO_SETTLE_MS),
    .STARTUP_DEBUG_DISPLAY  (1'b0)   // V26 Step8: 生命周期恒按正常流程(数码管开关见顶层 STARTUP_DEBUG_DISPLAY)
) u_startup_controller (
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_pll_locked       (audio_pll_lock),
    .I_first_frame_ready(first_frame_ready),
    .I_sample_enable    (audio_valid),
    .I_music_done       (startup_done),
    .I_sd_init_done     (sd_init_done_v),
    .I_bmp_scan_done    (bmp_scan_done_v),
    .I_bmp_found        (bmp_found_v),
    .I_wav_found        (wav_found_c),
    .I_wav_err          (wav_err_c),
    .O_state            (su_state),
    .O_media_digit      (su_digit),
    .O_no_bmp           (su_no_bmp),
    .O_startup_phase    (startup_phase),
    .O_music_on         (music_on),
    .O_spectrum_on      (su_spectrum_on),   // V27S3: 不再驱动 spectrum_on(改由 Audio FSM)
    .O_test_tone_on     (test_tone_on),
    .O_done_pulse       (music_done_pulse)
);
// V26 Step8: 图片锁/解锁改由 audio_startup_wav_ctl 的 O_unlock_image(video 域)同步到 sd 域:
//   未解锁(=0: Startup Music / 等 WAV ready 期间)锁第一张 BMP(屏蔽 key_next/key_auto, 不提前切图);
//   解锁(=1: PLAY_WAV 或 WAV_UNAVAIL)沿自动注入一次"下一张"(按住 key_next 低电平 ~30ms > 去抖 20ms)
//   -> sd_card_bmp 载入并提交第二张(image_index 0->1 一次), 之后恢复手动/自动切换。sd_card_bmp FSM 未改。
reg sp_sd0, sp_sd1;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin sp_sd0 <= 1'b0; sp_sd1 <= 1'b0; end
    // V27S2: 图片解锁不再由音频 ctl 驱动(音视频解耦) => 图片/按键路径只依赖 Video
    else begin sp_sd0 <= 1'b1; sp_sd1 <= sp_sd0; end
end
reg hold_prev;
reg adv_on;
reg [23:0] adv_t;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin hold_prev <= 1'b0; adv_on <= 1'b0; adv_t <= 24'd0; end
    else begin
        hold_prev <= sp_sd1;
        if (!hold_prev && sp_sd1 && !adv_on) begin adv_on <= 1'b1; adv_t <= 24'd0; end  // 解锁沿 -> 自动切一张
        else if (adv_on) begin
            if (adv_t >= 24'd2_999_999) adv_on <= 1'b0;   // ~30ms @100MHz(>去抖 20ms)
            else adv_t <= adv_t + 24'd1;
        end
    end
end
wire key_next_eff = ~sp_sd1 ? 1'b1 : (adv_on ? 1'b0 : image_keys_sync[1]);  // 未解锁松键(锁图); 解锁沿自动按一次; 之后手动
wire key_auto_eff = ~sp_sd1 ? 1'b1 : image_keys_sync[0];                // 未解锁不允许 auto 切换

// 开机音乐播放器复位: 可选“等 PLL 锁定”(默认)或仅等 rst_n。
// PIR 注: 播放器刻意不并入软复位 —— "人回来不重播开机流程"由粘性 startup_done 与
//   audio_startup_wav_ctl 的 A_DELAY 快捷跳转实现; 若把播放器也软复位, 该粘性标志
//   会被清掉, 人回来后就会重播开机音乐。
assign player_rst = (STARTUP_WAIT_FOR_PLL_LOCK == 1) ? rst_all : ~rst_n;

// V11: 首帧提交(display_valid)前黑屏; 提交后显示当前缓冲(首张 BMP = Splash 保持到 music done, 之后正常轮播)。
// 不再用 startup_active 把画面置黑(Splash 不能黑屏)。


// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .clk2_out   (mic_adc_clk),
    .reset      (1'b0)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// ===================== V25 Step2: 共享唯一 SD 控制器 + SD Read Arbiter =====================
// sd_card_bmp(BMP client, m0) + 预留 WAV/FAT(m1, 本步不接) 经 sd_read_arbiter
// 共享唯一 sd_card_top(控制器上移到 top, 物理 SD IO 位置不变)。
// V26 Step8: SD 初始化完成 -> video 域 2FF 已在启动判定区实现(sd_init_done_v)
wire        bmp_sd_req;  wire [31:0] bmp_sd_addr;      // BMP -> arbiter m0
wire [7:0]  bmp_sd_data; wire bmp_sd_valid, bmp_sd_end;// arbiter m0 -> BMP
wire        ctr_sd_req;  wire [31:0] ctr_sd_addr;      // arbiter -> sd_card_top
wire [7:0]  ctr_sd_data; wire ctr_sd_valid, ctr_sd_end;// sd_card_top -> arbiter
wire [1:0]  sd_arb_owner; wire sd_arb_busy;

// ===================== TF 多图扫描与缓存（双缓冲） =====================
// BMP(m0,优先) + WAV/FAT(m1=V26 Step8 audio_wav_top_path) -> 唯一 sd_card_top
sd_read_arbiter u_sd_read_arbiter (
    .clk(sd_card_clk), .rst(rst_all), .I_init_done(sd_init_done),
    .m0_req(bmp_sd_req && !tf_video_enable_sd), .m0_addr(bmp_sd_addr),
    .m0_data(bmp_sd_data), .m0_valid(bmp_sd_valid), .m0_end(bmp_sd_end),
    .m1_req(wav_sd_req), .m1_addr(wav_sd_addr),
    .m1_data(wav_sd_data), .m1_valid(wav_sd_valid), .m1_end(wav_sd_end),
    .ctr_req(ctr_sd_req), .ctr_addr(ctr_sd_addr),
    .ctr_data(ctr_sd_data), .ctr_valid(ctr_sd_valid), .ctr_end(ctr_sd_end),
    .o_owner(sd_arb_owner), .o_busy(sd_arb_busy), .o_m1_idle(sd_audio_idle)
);

// 唯一 SD 控制器(物理 IO: SD_nCS/DCLK/MOSI/MISO 位置不变)
sd_card_top #(.SPI_FAST_ENABLE(TF_VIDEO_SPI_FAST_ENABLE)) sd_card_top_m0 (
    .clk(sd_card_clk), .rst(rst_all),
    .SD_nCS(sd_ncs), .SD_DCLK(sd_dclk), .SD_MOSI(sd_mosi), .SD_MISO(sd_miso),
    .sd_init_done(sd_init_done),
    .sd_sec_read(ctr_sd_req), .sd_sec_read_addr(ctr_sd_addr),
    .sd_sec_read_data(ctr_sd_data), .sd_sec_read_data_valid(ctr_sd_valid),
    .sd_sec_read_end(ctr_sd_end),
    .sd_sec_write(1'b0), .sd_sec_write_addr(32'd0),
    .sd_sec_write_data(8'd0), .sd_sec_write_data_req(), .sd_sec_write_end()
);

// Debug peripherals removed for the integrated demonstration firmware.
// Keep the constrained pins safely idle; no scan clock or UART serializer.
assign seg_sel=6'b111111;
assign seg_data=8'hff;
assign uart_tx=1'b1;

// ===================== 原图像时序与帧缓存 =====================
wire audio_mem_pause, video_mem_busy;
wire [2:0] transition_mode_video;
wire lcd_transition_prev_event,lcd_transition_next_event;
wire lcd_prev_image_event,lcd_next_image_event;
wire signed [7:0] image_brightness_target_pid, image_brightness_applied_pid;
wire image_brightness_tick_pid, ambient_gain_tick_pid;
wire [6:0] ambient_gain_target_pid, ambient_gain_applied_pid;
brightness_pid_pair u_brightness_pid(
 .clk(video_clk),.rst(rst_all),
 .tick_manual(image_brightness_tick_pid),.tick_gain(ambient_gain_tick_pid),
 .target_manual(image_brightness_target_pid),.target_gain(ambient_gain_target_pid),
 .value_manual(image_brightness_applied_pid),.value_gain(ambient_gain_applied_pid));
d5_visual_path #(
 .NET_VIDEO_ENABLE(NET_VIDEO_ENABLE),
 .PRESENTATION_ICON_ID(3'd0),.PRESENTATION_ICON_COL(5'd15),.PRESENTATION_ICON_ROW(5'd7),
 .PRESENTATION_HOTSPOT1({5'd1,5'd4,2'b00}),
 .PRESENTATION_HOTSPOT2({5'd1,5'd7,2'b01}),
 .PRESENTATION_HOTSPOT3({5'd13,5'd8,2'b11}),
 .PRESENTATION_HOTSPOT4({5'd15,5'd7,2'b11})
) u_d5_visual (
 .media_enable(net_video_enabled && net_video_remote_active),.media_pixels(net_video_pixels),.media_flags(net_video_flags),
 .media_valid(net_video_valid),.media_ready(net_video_ready),.media_due_toggle(net_video_due),
 .media_active(net_image_active),.media_displayed(net_image_displayed),.media_error(net_image_error),.media_credit_reason(net_image_credit_reason),
 .touch_valid_video(touch_v),.touch_id_video(touch_id_v),
 .compare_valid(compare_valid_video && compare_cmd_video<=4'd4),
 .compare_cmd(compare_cmd_video),
 .transition_prev_event(compare_valid_video && compare_cmd_video==4'd5),
 .transition_next_event(compare_valid_video && compare_cmd_video==4'd6),
 .transition_mode_display(transition_mode_video),
 .touch_valid_sd(touch_sd),.touch_id_sd(touch_id_sd),
 .brightness_target(image_brightness_target_pid),.brightness_tick(image_brightness_tick_pid),
 .brightness_applied(image_brightness_applied_pid),
 .av_enable(av_enable && spectrum_on),.av_mode_mic(link_mode_mic),.av_levels(av_levels),
 .av_pulse(av_pulse),.av_ring(av_ring),.av_step(av_step),
 .clk(clk), .rst_all(rst_all), .sd_card_clk(sd_card_clk), .ext_mem_clk(ext_mem_clk), .video_clk(video_clk),
 .key1(presentation_mode_sd_meta ? 1'b1 : image_keys_sync[1]), .key2(image_keys_sync[0]), .key_adjust(image_key3_n), .adjust_select(adjust_select),
 .key_adjust_reverse_n(panel_image_auto_n),
 .presentation_mode(presentation_mode),.presentation_key_next_n(panel_image_next_n),
 .sd_init_done(sd_init_done), .sd_req(bmp_sd_req), .sd_addr(bmp_sd_addr),
 .sd_data(bmp_sd_data), .sd_valid(bmp_sd_valid), .sd_end(bmp_sd_end),
 .scan_done(bmp_scan_done_sd), .bmp_found(bmp_found_sd),
 .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld), .Sdr_busy(Sdr_busy),
 .App_rd_en(vrd_en), .App_rd_addr(vrd_addr), .Sdr_rd_en(vrd_valid && !video_owned), .Sdr_rd_dout(vrd_data),
 .App_wr_en(vwr_en), .App_wr_addr(vwr_addr), .App_wr_din(vwr_din), .App_wr_dm(vwr_dm),
 .audio_pause(audio_mem_pause || video_pause_legacy), .transport_busy(video_mem_busy),
 .hs(hs), .vs(vs), .de(de), .vout_data(vout_data), .display_valid_active(display_valid), .state_code(state_code),
 .frame_write_toggle_mem(frame_write_toggle_mem),
 .video_read_en(video_read_en), .video_read_data(video_read_data)
);

wire ts_wr_req,ts_wr_active,ts_wr_ack,ts_rd_req,ts_rd_valid,ts_mem_fault;
wire [15:0] ts_wr_block,ts_rd_block;wire [1:0] ts_wr_word,ts_rd_word;
wire [31:0] ts_wr_data,ts_rd_data;
wire ts_capture_event,ts_raw_valid,ts_gap;wire [15:0] ts_raw_pcm;
wire ts_history_mode,ts_history_valid;
wire ts_replay_selected=ts_history_mode && fm_status[106:105]==2 && link_online;wire [1:0] ts_epoch;wire [15:0] ts_history_pcm;
wire [23:0] ts_lcd_status;
wire lcd_ts_event,video_ts_event;wire [3:0] lcd_ts_id,video_ts_id;
reg [1:0] ts_memory_ready;
always @(posedge video_clk or posedge rst_all)
 if(rst_all)ts_memory_ready<=0;else ts_memory_ready<={ts_memory_ready[0],Sdr_init_done};
reg [15:0] ts_frequency;
wire ts_station_change=fm_status_valid && fm_status[63:48]!=ts_frequency;
always @(posedge video_clk or posedge rst_all)
 if(rst_all)ts_frequency<=0;else if(fm_status_valid)ts_frequency<=fm_status[63:48];
fm_timeshift_path u_timeshift(.video_clk(video_clk),.mem_clk(ext_mem_clk),.lcd_clk(clk),.rst(rst_all),
 .capture_event(ts_capture_event && ts_memory_ready[1]),.raw_pcm(ts_raw_pcm),.raw_valid(ts_raw_valid),.packet_gap(ts_gap),
 .station_change(ts_station_change),.sample_tick(audio_valid),.command_valid(video_ts_event),.command(video_ts_id),
 .history_mode(ts_history_mode),.play_epoch(ts_epoch),.history_pcm(ts_history_pcm),.history_valid(ts_history_valid),.lcd_status(ts_lcd_status),
 .wr_req(ts_wr_req),.wr_block(ts_wr_block),.wr_word(ts_wr_word),.wr_active(ts_wr_active),.wr_data(ts_wr_data),.wr_ack(ts_wr_ack),
 .rd_req(ts_rd_req),.rd_block(ts_rd_block),.rd_valid(ts_rd_valid),.rd_word(ts_rd_word),.rd_data(ts_rd_data),.mem_fault(ts_mem_fault));
// ===== V26 Step8: audio_sdram_access 仲裁桥插入 frame_read_write(V) <-> sdram U3; A 侧接 WAV Ring =====
// 每物理 SDRAM 用户口仅一个驱动源(App_*/Sdr_* 由本桥独占); 视频优先/音频插空; AUDIO_BASE=0x0A0000。
audio_sdram_access #(
    .ADDRW     (21),
    .DATAW     (32),
    .DMW       (4),
    .AADDRW    (13),                    // ring mem addr 13bit(8192 帧 = 32KB)
    .AUDIO_BASE(AUDIO_BASE_ADDR),       // 21'd655360 = 0x0A0000(高于 BUF0/BUF1, 不重叠)
    .TAGDEPTH  (64),
    .TAGW      (6),.TIME_EXT(FM_TIMESHIFT_ENABLE && AUDIO_DSP_ENABLE && FM_INTEGRATION_ENABLE)
) u_audio_sdram_access (
    .audio_idle(mem_audio_idle),
    .Sdr_init_done(Sdr_init_done),.Sdr_init_ref_vld(Sdr_init_ref_vld),.Sdr_busy(Sdr_busy),
    .v_busy(video_owned ? video_dma_busy : video_mem_busy),.v_pause(audio_mem_pause),
    .clk(ext_mem_clk), .rst(rst_all),
    // -> sdram U3
    .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr), .App_wr_dm(App_wr_dm), .App_wr_din(App_wr_din),
    .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr),
    .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
    // V: frame_read_write
    .v_wr_en(video_owned ? video_wr : vwr_en), .v_wr_addr(video_owned ? video_wr_addr : vwr_addr),
    .v_wr_dm(video_owned ? 4'd0 : vwr_dm), .v_wr_din(video_owned ? video_wr_data : vwr_din),
    .v_rd_en(video_owned ? video_rd : vrd_en), .v_rd_addr(video_owned ? video_rd_addr : vrd_addr),
    .v_rd_valid(vrd_valid), .v_rd_data(vrd_data),
    // A: audio_wav_top_path ring_mem_*
    .a_wr_req(wav_ring_wr_req), .a_wr_addr(wav_ring_wr_addr), .a_wr_data(wav_ring_wr_data),
    .a_wr_ready(wav_ring_wr_ready),
    .a_rd_req(wav_ring_rd_req), .a_rd_addr(wav_ring_rd_addr),
    .a_rd_valid(wav_ring_rd_valid), .a_rd_data(wav_ring_rd_data),
    .t_wr_req(ts_wr_req),.t_wr_block(ts_wr_block),.t_wr_data(ts_wr_data),.t_wr_word(ts_wr_word),.t_wr_active(ts_wr_active),.t_wr_ack(ts_wr_ack),
    .t_rd_req(ts_rd_req),.t_rd_block(ts_rd_block),.t_rd_valid(ts_rd_valid),.t_rd_word(ts_rd_word),.t_rd_data(ts_rd_data),.t_fault(ts_mem_fault)
);

assign video_start=video_event && (video_id==0 || video_id==3) && !session_resetting;
assign video_stop=((video_event && video_id==2) || (net_video_selected && video_session)) && !session_resetting;
assign video_pause_event=video_event && video_id==1 && video_session && !session_resetting;
assign video_session=TF_VIDEO_ENABLE && current_wav_name_c[15];
assign video_session_cmd=video_start || video_stop;
assign media_session_play=panel_play || video_session_cmd;
assign media_session_name=video_session_cmd ? (video_start?88'h8000:88'd0) : panel_wav_name;
generate if(TF_VIDEO_ENABLE)begin:tf_video
 tf_video_frame_store #(.HQ_ENABLE(TF_VIDEO_HQ_ENABLE),.FAST_ENABLE(TF_VIDEO_FAST_ENABLE),.COMPACT_LAYOUT(1)) store(.sd_clk(sd_card_clk),.mem_clk(ext_mem_clk),.pixel_clk(video_clk),.rst(rst_all),
  .profile_fast_sd(tf_video_profile_fast_sd),.profile_high_sd(tf_video_profile_sd),.enable_sd(tf_video_enable_sd),.burst_data(tf_video_burst),.burst_valid(tf_video_burst_valid),
  .burst_last(tf_video_burst_last),.burst_ready(tf_video_burst_ready),
  .legacy_busy(video_mem_busy),.legacy_wr(vwr_en),.legacy_rd(vrd_en),.audio_pause(audio_mem_pause),
  .sdram_ready(Sdr_init_done),.sdram_refresh(Sdr_init_ref_vld),.sdram_busy(Sdr_busy),
  .owned(video_owned),.pause_legacy(video_pause_legacy),.busy(video_dma_busy),
  .wr_en(video_wr),.wr_addr(video_wr_addr),.wr_data(video_wr_data),
  .rd_en(video_rd),.rd_addr(video_rd_addr),.rd_valid(vrd_valid && video_owned),.rd_data(vrd_data),
  .vs(vs),.de(de),.sample_transfer(tf_video_sample && video_session),
  .play_enable(wav_wav_play && video_session),.paused(audio_paused),.rgb(tf_video_rgb),.display_active(tf_video_display),
  .underrun(tf_video_under),.late_frame(tf_video_late),.fault(tf_video_fault),.displayed_frames(tf_video_frame_count));
end else begin
 assign video_owned=0;assign video_pause_legacy=0;assign video_dma_busy=0;
 assign video_wr=0;assign video_rd=0;assign video_wr_addr=0;assign video_rd_addr=0;assign video_wr_data=0;
 assign tf_video_burst_ready=0;assign tf_video_rgb=0;assign tf_video_display=0;
 assign tf_video_under=0;assign tf_video_late=0;assign tf_video_fault=0;assign tf_video_frame_count=0;
end endgenerate
// Delay the old image path by the same one clock as the video cache read.
always @(posedge video_clk or posedge rst_all)begin
 if(rst_all)begin image_bg_delayed<=0;media_vs<=1;media_de<=0;end
 else begin image_bg_delayed<=vout_data;media_vs<=vs;media_de<=de;
end
end
assign media_bg=(tf_video_display && !net_image_active) ? tf_video_rgb : image_bg_delayed;
// Receive 25MHz-origin status at the falling 50MHz edge; LCD uses
// the next rising edge. Both 25MHz source edges align with 50MHz rising.
always @(negedge clk or posedge rst_all)begin
 if(rst_all)video_status_meta<=0;
 else video_status_meta<={net_video_selected,net_image_displayed,(net_video_remote_error || net_image_error),net_video_remote_done,net_video_remote_active,tf_video_profile_fast_sd,tf_video_profile_sd,tf_video_under,(tf_video_fault?4'hf:tf_video_error_sd),tf_video_late,
  tf_video_display,audio_paused,video_session};
end
always @(posedge clk or posedge rst_all)begin
 if(rst_all)video_status_lcd<=0;
 else video_status_lcd<=video_status_meta;
end
sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频：内部 I2S 测试音 =====================
hdmi_audio_tone_i2s_64fs #(
    .PHASE_INC (32'd39370534),
    .AMP       (24'sd2000000)
) u_hdmi_audio_tone_i2s_64fs (
    .I_mclk      (audio_mclk),
    .I_rst       (rst_all),
    .O_i2s_BCLK  (audio_i2s_bclk),
    .O_i2s_LRCK  (audio_i2s_lrck),
    .O_i2s_DOUT  (audio_i2s_dout)
);

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (tone_audio_valid),
    .O_audio_left_data  (audio_left_data),
    .O_audio_right_data (audio_right_data)
);

// ===================== 开机音乐(ROM+DDS)播放器 + 音频 MUX (V12) =====================
// player 为纯引擎(线性 62 音): I_start 由 startup_controller 在首帧+Settle 后置位(MUSIC), 播完 done。
startup_music_player #(
    .STARTUP_ENABLE(STARTUP_MUSIC_ENABLE),
    .NOTE_NUM     (STARTUP_MUSIC_NOTE_NUM),
    .AMP          (STARTUP_MUSIC_AMP),
    .ROM_DEPTH    (STARTUP_MUSIC_ROM_DEPTH),
    .ADDR_BITS    (7)
) u_startup_music_player (
    .I_clk            (video_clk),
    .I_rst            (player_rst),
    .I_pll_locked     (audio_pll_lock),
    .I_start          (au_music_start),    // V27S2: 由音频 FSM(A_MUSIC)启动 one-shot
    .I_sample_enable  (audio_valid),
    .O_startup_active (startup_active),
    .O_startup_done   (startup_done),
    .O_left           (startup_left),
    .O_right          (startup_right)
);

// ===================== V26 Step8 Phase-B: 最终 WAV 链正式接入(audio_wav_top_path) =====================
//   只接线不重设计: 内部 u_ctl(audio_startup_wav_ctl)->audio_mux3 四源互斥
//   (sel 0 startup / 1 wav / 2 testtone / 3 silence); WAV 48k 来自真实链(video_clk 域 audio_valid tick)。
//   aud_mux_l/r = 模块 o_pcm(video 域统一 24bit) -> HDMI Audio 与 FFT 同一份最终 PCM。
//   TestTone 仅 NORMAL 且 AUDIO_TEST_MODE=1 时被选中(sel2); 默认 0 静音。
//   Startup/Tone 源 = 现有 startup_music_player / I2S_receiver 恢复音, 均不修改, 仅门控/接线。
// AUDIO_TEST_MODE=0 时把 tone 输入常数化 => I2S 测试音链(hdmi_audio_tone_i2s_64fs /
//   I2S_receiver)与旧 startup_controller 成为死逻辑被综合自动裁剪(约 320 个触发器), 缓解布线拥塞;
//   =1 时行为不变(AUDIO_TEST_MODE=1 调试仍可用完整测试音链)。
wire signed [23:0] wav_tone_l = ((AUDIO_TEST_MODE == 1) && test_tone_on) ? audio_left_data  : 24'sd0;
wire signed [23:0] wav_tone_r = ((AUDIO_TEST_MODE == 1) && test_tone_on) ? audio_right_data : 24'sd0;
wire [15:0] wav_catalog_count, wav_catalog_index;
wire wav_catalog_valid;
audio_panel_control #(.ENUMERATE_WAV(1),.PAUSE_KEY_ENABLE(1),.AUTO_NEXT_ENABLE(1),.DEBOUNCE_CYCLES(125000)) u_audio_panel(
 .touch_valid(touch_v),.touch_id(touch_id_v),
 .clk(video_clk),.rst(rst_all),.mode_audio(audio_mode_sw),
 .key_next_n(key1),.key_prev_n(key2),.key_pause_n(audio_key3_n),
 .play_finished(wav_play_finished && !wav_err_c && !video_session),.session_busy(session_resetting),
 .external_play(uart_play),.current_name(current_wav_name_c),
 .catalog_valid(wav_catalog_valid),.catalog_count(wav_catalog_count),.catalog_index(wav_catalog_index),
 .pause_cmd(panel_pause_cmd),.image_pause_n(image_key3_n),
 .image_next_n(panel_image_next_n),.image_auto_n(panel_image_auto_n),
 .play_cmd(panel_play),.target_name(panel_wav_name)
);

// This controller latches LCD/key filenames and sequences the private
// audio resets; it is not a serial debug peripheral.
assign selected_wav_name=current_wav_name_c;
audio_uart_session #(.INITIAL_NAME(88'd0)) u_uart_session(
 .clk_a(sd_card_clk),.clk_b(ext_mem_clk),.clk_c(video_clk),.rst(rst_all),
 .play_cmd(uart_play | media_session_play | panel_start),
 .selected_name(media_session_play ? media_session_name : selected_wav_name),
 .sd_idle(sd_audio_idle),.mem_idle(mem_audio_idle && !video_owned),
 .active_name(active_wav_name),.manual(manual_session),.busy(session_resetting),
 .current_name_c(current_wav_name_c),
 .rst_a(audio_session_rst_a),.rst_b(audio_session_rst_b),.rst_c(audio_session_rst_c)
);
audio_wav_top_path #(
    .TF_VIDEO_FAST_ENABLE(TF_VIDEO_FAST_ENABLE),.TF_VIDEO_HQ_ENABLE(TF_VIDEO_HQ_ENABLE),.TF_VIDEO_ENABLE(TF_VIDEO_ENABLE),.DYNAMIC_NAME(2), .PAUSE_ENABLE(1),
    .AUDIO_FIFO_DEPTH(64),
    .RING_FRAMES    (8192),
    .RING_AW        (13),
    .RING_LOW       (2048),
    .RING_HIGH      (6144),
    .AUDIO_TEST_MODE(AUDIO_TEST_MODE)
) u_audio_wav_top_path (
    .clk_a(sd_card_clk), .clk_b(ext_mem_clk), .clk_c(video_clk),
    .i_target_name(active_wav_name), .i_pause(audio_paused),
    .rst_a(rst_all | audio_session_rst_a | (wav_source_net ? net_app_reset : 1'b0)),
    .rst_b(rst_all | audio_session_rst_b), .rst_c(rst_all | audio_session_rst_c),
    .rst_soft_a(soft_sd_run), .rst_soft_b(soft_mem_run), .rst_soft_c(soft_video_run),
    .source_sel(wav_source_net),
    .net_file_byte(net_file_byte), .net_file_valid(net_file_valid),
    .net_file_end(net_file_end), .net_file_size(net_file_size),
    .net_file_ready(net_file_ready), .net_file_error(net_file_error),
    // A: SD m1(唯一 sd_card_top + sd_read_arbiter.m1)
    .sd_m1_req(wav_sd_req), .sd_m1_addr(wav_sd_addr),
    .sd_m1_data(wav_sd_data), .sd_m1_valid(wav_sd_valid), .sd_m1_end(wav_sd_end),
    // B: Ring mem -> audio_sdram_access.a_* -> 共享 sdram U3
    .ring_mem_wr_req(wav_ring_wr_req), .ring_mem_wr_addr(wav_ring_wr_addr),
    .ring_mem_wr_data(wav_ring_wr_data), .ring_mem_wr_ready(wav_ring_wr_ready),
    .ring_mem_rd_req(wav_ring_rd_req), .ring_mem_rd_addr(wav_ring_rd_addr),
    .ring_mem_rd_data(wav_ring_rd_data), .ring_mem_rd_valid(wav_ring_rd_valid),
    // C: video 域控制(真实源)
    .i_audio_valid(audio_valid), .i_first_frame_ready(first_frame_ready),
    .i_startup_done(startup_done | manual_session),
    .i_sd_init_done(sd_init_done_v),   // V27S2: 音频启动唯一基准 = SD init done
    .i_wav_start(1'b0),                // V27S2: 解除 BMP->音频 触发(现由内部 A_SEARCH 触发)
    .i_no_bmp(1'b0),                   // V27S2: 音频不再依赖 BMP 判定
    .i_startup_l(startup_left), .i_startup_r(startup_right),
    .i_tone_l(wav_tone_l), .i_tone_r(wav_tone_r),
    // C: 统一 24bit PCM + WAV 帧(观测)
    .o_pcm_valid(wav_pcm_v), .o_pcm_left(wav_pcm_l), .o_pcm_right(wav_pcm_r),
    .o_wav_valid(), .o_wav_left(), .o_wav_right(),
    // C: MUX/状态/水位(观测)
    .o_audio_sel(wav_audio_sel), .o_wav_play(wav_wav_play),
    .o_unlock_image(wav_unlock_image), .o_ctl_phase(wav_ctl_phase),
    .o_music_start(au_music_start), .o_wav_search(au_wav_search),   // V27S2
    .o_audio_digit(au_digit),                                       // V27S2
    .o_wav_count_c(wav_catalog_count), .o_wav_index_c(wav_catalog_index), .o_catalog_valid_c(wav_catalog_valid),
    .o_wav_ready(wav_ready_c), .o_wav_found_c(wav_found_c),
    .o_wav_err_c(wav_err_c), .o_wav_done_c(wav_done_c), .o_play_finished(wav_play_finished),
    .o_ring_used(wav_ring_used), .o_fifo1_full(wav_f1_full),
    .o_fifo1_empty(wav_f1_empty), .o_fifo2_full(wav_f2_full),
    .o_fifo2_empty(wav_f2_empty),
    .video_enable_sd(tf_video_enable_sd),.video_burst_data(tf_video_burst),
    .video_burst_valid(tf_video_burst_valid),.video_burst_last(tf_video_burst_last),
    .video_burst_ready(tf_video_burst_ready),.video_sample_transfer(tf_video_sample),.video_error_sd(tf_video_error_sd),.video_display_ready_c(tf_video_display),.video_profile_fast_sd(tf_video_profile_fast_sd),.video_profile_high_sd(tf_video_profile_sd)
);
// HDMI Audio + FFT 共用同一份最终 PCM(audio_mux3 输出), 绝不另开第二条 FFT/分析链。
// PIR 缺勤时强制 0(双保险: 此时音频 FSM 也已被软复位保持在静音态)。
wire link_net_mode,link_net_valid;
always @(posedge video_clk or posedge rst_all)
 if(rst_all)begin net_video_audio_count<=0;net_video_due<=0;end
 else if(!net_video_enabled || !net_video_remote_active)begin net_video_audio_count<=0;net_video_due<=0;end
 else if(audio_valid && link_net_valid && !audio_paused)begin
  if(net_video_audio_count==2399)begin net_video_audio_count<=0;net_video_due<=!net_video_due;end
  else net_video_audio_count<=net_video_audio_count+1'b1;
 end
wire [31:0] link_net_pcm;
wire [23:0] selected_audio_l=(link_online && link_net_mode) ?
 (link_net_valid ? {(fm_status[106:105]==2 && AUDIO_DSP_ENABLE ? link_net_pcm[15:0] : link_net_pcm[31:16]),8'd0} : 24'd0) : wav_pcm_l;
wire [23:0] selected_audio_r=(link_online && link_net_mode) ?
 (link_net_valid ? {link_net_pcm[15:0],8'd0} : 24'd0) : wav_pcm_r;
generate if(FM_INTEGRATION_ENABLE)begin:unified_audio
 audio_source_guard #(.KEY_W(8)) guard(.clk(video_clk),.rst(rst_all),.tick(audio_valid),
  .key({(fm_status[106:105]==2 ? ts_epoch : 2'd0),
       (soft_video_run && (!video_session || fm_status[106:105]!=0 || (!audio_paused && !session_resetting))),
       link_online,link_net_mode,link_net_valid,fm_status[106:105]}),
  .left(soft_video_run ? selected_audio_l : 24'd0),.right(soft_video_run ? selected_audio_r : 24'd0),
  .out_left(aud_mux_l),.out_right(aud_mux_r));
end else begin:legacy_audio
 assign aud_mux_l=soft_video_run ? selected_audio_l : 24'd0;
 assign aud_mux_r=soft_video_run ? selected_audio_r : 24'd0;
end endgenerate

// ===================== 音频可视化(扩展要求5)：FFT 分析链 =====================
// 纯旁路复制 PCM，不阻塞原 HDMI 音频播放链路（audio_valid/left/right 不变）。
// FFT 输入使用 MUX 后 PCM(开机=开机音, 正常=normal), 保持“PCM 链统一/FFT 仅一份”。
wire [11:0] light_adc_data;
wire light_adc_toggle;
// BOARD_A: moved to BOARD_B; original instance retained.
// mic_spectrum_source u_mic_spectrum(
//  .clk_adc(mic_adc_clk),.clk_video(video_clk),.rst(rst_all),
//  .select_sw(spectrum_mic_sw),.selected(mic_selected),.pcm(mic_pcm),.valid(mic_valid),
//  .light_data(light_adc_data),.light_toggle(light_adc_toggle));
// The packed compatibility path is retained for optional HUB75/test builds.
localparam SPECTRUM_INDEXED = 1; // Atomic remote spectrum in two BRAM banks.
wire spectrum_band_valid;
wire [5:0] spectrum_band_addr, spectrum_band_height;
wire [5:0] spectrum_read_addr, spectrum_read_height;
wire spectrum_freeze;
// BOARD_A: moved to BOARD_B; original instance retained.
// audio_spectrum_analyzer #(
//     .FFT_N               (FFT_N),
//     .FFT_LOG2_N          (FFT_LOG2_N),
//     .FFT_INPUT_BITS      (FFT_INPUT_BITS),
//     .FFT_INTERNAL_BITS   (FFT_INTERNAL_BITS),
//     .FFT_COEF_BITS       (FFT_COEF_BITS),
//     .FFT_SAMPLE_RATE     (FFT_SAMPLE_RATE),
//     .FFT_HOP_SIZE        (FFT_HOP_SIZE),
//     .SPECTRUM_BIN_NUM    (SPECTRUM_BIN_NUM),
//     .SPECTRUM_BAND_SHIFT (SPECTRUM_BAND_SHIFT),
//     .SPECTRUM_SCALE_SHIFT(SPECTRUM_SCALE_SHIFT),
//     .SPECTRUM_MAX_BAR    (SPECTRUM_MAX_BAR),
//     .SPECTRUM_LOG_SHIFT  (SPECTRUM_LOG_SHIFT)
// ) u_audio_spectrum_analyzer (
//     .I_clk             (video_clk),          // 25MHz 逻辑时钟
//     .I_rst             (rst_all),
//     .I_audio_valid     (mic_selected ? mic_valid : audio_valid),        // 48kHz 采样有效
//     .I_audio_left_data (mic_selected ? mic_pcm : aud_mux_l),          // 24bit 左声道 PCM(MUX 后: 开机=开机音, 之后=normal)
//     .O_heights_valid   (spectrum_heights_valid),
//     .O_heights         (spectrum_heights),
//     .O_band_valid(spectrum_band_valid),
//     .O_band_addr(spectrum_band_addr), .O_band_height(spectrum_band_height),
//     .O_analysis_busy   (spectrum_busy)
// );

// ===================== Phase3: FFT 幅值 -> HUB75 频谱 =====================
// 列=bin1..64, log 压缩(参考 STM32 logf(x+3)*8.1), 平滑动画, 1/16 扫描白色显示。
// HUB75 逻辑信号编号已由用户固定(GPIOA0..A12 / J1-1..15)；package pin LOC 暂缓，
// 不在本文件臆测封装脚。
wire hub_upd;
wire hub_r1, hub_g1, hub_b1, hub_r2, hub_g2, hub_b2;
wire hub_a, hub_b, hub_c, hub_d, hub_clk, hub_lat, hub_oe;

// 测试图: 阶梯高度 0,1,..32,0..(col%33), 便于上板检查列/行/上下半区/地址
genvar tg;
generate
    for (tg = 0; tg < SPECTRUM_BIN_NUM; tg = tg + 1) begin : gen_testh
        assign test_h[tg*6 +: 6] = (tg % 33);
    end
endgenerate

assign target_h = (HUB75_TEST_MODE == 1) ? test_h : spectrum_heights;

// 测试模式下需周期产生“更新”以驱动平滑/控制器装载
reg [19:0] hub_tcnt;
reg        hub_tick;
assign hub_upd = (HUB75_TEST_MODE == 1) ? hub_tick : spectrum_heights_valid;
wire spectrum_upd;
// V12 注: 频谱更新门控(spectrum_upd)现由 V27S3 Audio FSM 的 spectrum_on 控制
//   (A_MUSIC/A_PLAY 才更新; FFT 从不 reset, 仅显示门控)
assign spectrum_upd = spectrum_on ? hub_upd : 1'b0;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        hub_tcnt <= 20'd0;
        hub_tick <= 1'b0;
    end else if (hub_tcnt >= 20'd249_999) begin
        hub_tcnt <= 20'd0;
        hub_tick <= 1'b1;
    end else begin
        hub_tcnt <= hub_tcnt + 1'b1;
        hub_tick <= 1'b0;
    end
end

// BOARD_A: moved to BOARD_B; original instance retained.
// spectrum_smoother #(
//     .INDEXED(SPECTRUM_INDEXED),
//     .SPECTRUM_BIN_NUM (SPECTRUM_BIN_NUM),
//     .HUB75_HEIGHT     (HUB75_HEIGHT),
//     .HUB75_SMOOTH_STEP(HUB75_SMOOTH_STEP)
// ) u_spectrum_smoother (
//     .I_clk     (video_clk),
//     .I_rst     (rst_all),
//     .I_update  (spectrum_upd),
//     .I_target  (target_h),
//     .I_band_valid(spectrum_band_valid), .I_band_addr(spectrum_band_addr),
//     .I_band_height(spectrum_band_height), .I_freeze(spectrum_freeze),
//     .I_read_addr(spectrum_read_addr), .O_read_height(spectrum_read_height),
//     .O_height  (cur_h)
// );

// Disabled for the suffix/D5 build; original implementation retained.
// hub75_controller #(
//     .HUB75_WIDTH      (HUB75_WIDTH),
//     .HUB75_SCAN_ROWS  (HUB75_SCAN_ROWS),
//     .HUB75_HEIGHT     (HUB75_HEIGHT),
//     .HUB75_CLK_DIV    (HUB75_CLK_DIV),
//     .HUB75_BRIGHTNESS (HUB75_BRIGHTNESS)
// ) u_hub75_controller (
//     .I_clk    (video_clk),
//     .I_rst    (rst_all),
//     .I_load   (spectrum_upd),
//     .I_height (cur_h),
//     .O_R1(hub_r1), .O_G1(hub_g1), .O_B1(hub_b1),
//     .O_R2(hub_r2), .O_G2(hub_g2), .O_B2(hub_b2),
//     .O_A(hub_a), .O_B(hub_b), .O_C(hub_c), .O_D(hub_d),
//     .O_CLK(hub_clk), .O_LAT(hub_lat), .O_OE(hub_oe)
// );


assign {hub_r1,hub_g1,hub_b1,hub_r2,hub_g2,hub_b2,hub_a,hub_b,hub_c,hub_d,hub_clk,hub_lat}=12'd0;
assign hub_oe=1'b1;
// HUB75_ENABLE 门控; OE 电平按 HUB75_OE_ACTIVE_LEVEL 取反(参考 EN 低有效=0)
assign HUB75_R1 = HUB75_ENABLE ? hub_r1 : 1'b0;
assign HUB75_G1 = HUB75_ENABLE ? hub_g1 : 1'b0;
assign HUB75_B1 = HUB75_ENABLE ? hub_b1 : 1'b0;
assign HUB75_R2 = HUB75_ENABLE ? hub_r2 : 1'b0;
assign HUB75_G2 = HUB75_ENABLE ? hub_g2 : 1'b0;
assign HUB75_B2 = HUB75_ENABLE ? hub_b2 : 1'b0;
assign HUB75_A  = HUB75_ENABLE ? hub_a  : 1'b0;
assign HUB75_B  = HUB75_ENABLE ? hub_b  : 1'b0;
assign HUB75_C  = HUB75_ENABLE ? hub_c  : 1'b0;
assign HUB75_D  = HUB75_ENABLE ? hub_d  : 1'b0;
assign HUB75_CLK = HUB75_ENABLE ? hub_clk : 1'b0;
assign HUB75_LAT = HUB75_ENABLE ? hub_lat : 1'b0;
assign HUB75_OE  = HUB75_ENABLE ?
                   ((HUB75_OE_ACTIVE_LEVEL == 1) ? ~hub_oe : hub_oe) : 1'b1;

// ===================== Phase5: HDMI 64band 透明频谱叠加(右下角) =====================
// 与 HUB75 共用 cur_h(同一份 FFT/64band/平滑); 无面板, 非柱像素 100% 透传原图。
parameter NET_WHITE_FILTER_ENABLE=1;
// Existing media timing remains matched. Runtime gating confines suppression
// to NET; TF/picture pixels are unchanged apart from the matched delay.
white_pixel_filter #(.ENABLE(NET_WHITE_FILTER_ENABLE),.REGISTER_OUTPUT(0),.RUNTIME_ENABLE(1)) net_impulse_filter(
 .clk(video_clk),.rst(rst_all),.in_rgb(media_bg),.in_de(media_de),.in_vs(media_vs),
 .out_rgb(filtered_media_bg),.out_de(filtered_media_de),.out_vs(filtered_media_vs),.filter_enable(net_video_enabled));
wire spectrum_pixel_vs,spectrum_pixel_de;
hdmi_spectrum_overlay #(
    .REGISTER_CURVE(1),.RUNTIME_ENABLE(1),
    .INDEXED(SPECTRUM_INDEXED),
    .SPECTRUM_BIN_NUM     (SPECTRUM_BIN_NUM),
    .SPECTRUM_X_START     (SPECTRUM_X_START),
    .SPECTRUM_X_END       (SPECTRUM_X_END),
    .SPECTRUM_Y_TOP       (SPECTRUM_Y_TOP),
    .SPECTRUM_Y_BOTTOM    (SPECTRUM_Y_BOTTOM),
    .SPECTRUM_BAR_WIDTH   (2),
    .SPECTRUM_BAR_GAP     (0),
    .SPECTRUM_HEIGHT_SCALE(2),
    .SPECTRUM_DEBUG_BYPASS(SPECTRUM_DEBUG_BYPASS),
    .SPECTRUM_COLOR_MODE  (SPECTRUM_COLOR_MODE),
    .SPECTRUM_FADE_ENABLE (SPECTRUM_FADE_ENABLE),
    .SPECTRUM_FADE_PERCENT(SPECTRUM_FADE_PERCENT),
    .SPECTRUM_FADE_LEVELS (SPECTRUM_FADE_LEVELS),
    .SPECTRUM_SMOOTH_STEP (SPECTRUM_SMOOTH_STEP),
    .SPECTRUM_COLOR0      (SPECTRUM_COLOR0),
    .SPECTRUM_COLOR1      (SPECTRUM_COLOR1),
    .SPECTRUM_COLOR2      (SPECTRUM_COLOR2),
    .SPECTRUM_COLOR3      (SPECTRUM_COLOR3),
    .SPECTRUM_COLOR4      (SPECTRUM_COLOR4),
    .SPECTRUM_COLOR5      (SPECTRUM_COLOR5)
) u_hdmi_spectrum_overlay (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_vs       (filtered_media_vs),
    .I_de       (filtered_media_de),
    .I_rgb_bg   (filtered_media_bg),
    .I_height   (cur_h),
    .O_read_addr(spectrum_read_addr), .I_read_height(spectrum_read_height),
    .O_freeze(spectrum_freeze),
    .I_enable(SPECTRUM_HDMI_ENABLE && spectrum_on),
    .O_vs(spectrum_pixel_vs),.O_de(spectrum_pixel_de),
    .O_rgb      (hdmi_overlay_rgb)
);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
// （Phase3：旧 spectrum_renderer HDMI 叠加实例已从数据路径移除，源码保留于
//   spectrum_renderer.v 备查；FFT 输出改由后续 HUB75 链路消费。）
always @(posedge video_clk or posedge rst_all)
 if(rst_all)scan_page_sync<=0;else scan_page_sync<={scan_page_sync[0],lcd_scan_page};
wire analyzer_vs,analyzer_de;wire [23:0] analyzer_rgb;
parameter FM_SHARED_FONT_ENABLE=1;
parameter BEAT_ENABLE=0; // Park prediction until its timing/accuracy benefit is demonstrated.
// Production beat receiver. Both boards retain their independent 25 MHz clock.
parameter signed [31:0] BEAT_AV_CALIBRATION_US=0;
wire [31:0] beat_now,beat_t1,beat_t2,beat_t3,beat_t4;
wire beat_packet_valid,beat_exchange;wire [255:0] beat_packet;
reg beat_enable;reg [1:0] beat_sensitivity;
always @(posedge video_clk)begin
 if(rst_all)begin beat_enable<=1;beat_sensitivity<=1;end
 else if(compare_valid_video)case(compare_cmd_video)
  11:beat_enable<=~beat_enable;
  12:beat_sensitivity<=beat_sensitivity==2 ? 0:beat_sensitivity+1'b1;
  default:;
 endcase
end
beat_timebase beat_clock(video_clk,rst_all,beat_now);
wire beat_snapshot_valid,beat_locked,beat_source_match,beat_observed,beat_strong;
wire [31:0] beat_next,beat_period,beat_timestamp;
wire [7:0] beat_bpm,beat_confidence,beat_phase;wire [2:0] beat_state;
wire [1:0] beat_source;wire [15:0] beat_epoch;
beat_receiver #(.SOURCE_HOLDS(1)) beat_rx(video_clk,rst_all,BEAT_ENABLE && beat_enable && link_online,beat_packet_valid,beat_packet,beat_now,
 beat_snapshot_valid,beat_locked,beat_source_match,beat_observed,beat_strong,
 beat_next,beat_period,beat_timestamp,beat_bpm,beat_confidence,beat_phase,beat_state,beat_source,beat_epoch);
wire beat_clock_valid;wire signed [31:0] beat_clock_offset;wire [15:0] beat_rtt;
beat_clock_sync #(.SAME_REMOTE_STAMP(1)) beat_sync(video_clk,rst_all || !link_online,beat_exchange,beat_now,
 beat_t1,beat_t2,beat_t3,beat_t4,beat_clock_valid,beat_clock_offset,beat_rtt);
wire beat_frame,beat_frame_strong,beat_fallback;wire [7:0] beat_pulse,beat_ring;wire [31:0] beat_scheduled;
wire beat_frame_tick=image_vs_d && !vs;
beat_visual_sync beat_animation(video_clk,rst_all,BEAT_ENABLE && beat_enable && av_enable && link_online,beat_frame_tick,
 beat_now,beat_snapshot_valid,beat_source_match && beat_clock_valid,beat_locked,
 beat_next,beat_period,beat_clock_offset,BEAT_AV_CALIBRATION_US,beat_observed,beat_strong,
 {av_levels[5:0],2'd0},beat_frame,beat_frame_strong,beat_fallback,beat_pulse,beat_ring,beat_scheduled);
assign av_pulse=(!beat_fallback && beat_enable) ? beat_pulse[7:3]:av_pulse_original;
// Independent observed-beat border; prediction stays compiled out.
assign av_ring=av_ring_original;
wire [10:0] hdmi_font_addr,lcd_font_addr;
wire [15:0] hdmi_font_bits,lcd_font_bits;
generate if(FM_SHARED_FONT_ENABLE && FM_INTEGRATION_ENABLE)begin:shared_fm_font
 fm_font_rom_dual glyphs(.clk_a(video_clk),.addr_a(hdmi_font_addr),.bits_a(hdmi_font_bits),
  .clk_b(clk),.addr_b(lcd_font_addr),.bits_b(lcd_font_bits));
end else begin assign hdmi_font_bits=0;assign lcd_font_bits=0;end endgenerate
generate if(FM_SCAN_ENABLE && FM_INTEGRATION_ENABLE)begin:analyzer_enabled
 fm_band_analyzer_hdmi #(.SHARED_FONT(FM_SHARED_FONT_ENABLE),.BG_LATENCY(1)) renderer(.clk(video_clk),.rst(rst_all),.enable(scan_page_sync[1]),.online(link_online),
 .vs(filtered_media_vs),.de(filtered_media_de),.rgb(final_rgb),.point_valid(scan_point_valid),.point_generation(scan_generation),.point_index(scan_index),.point_data(scan_data),
 .status(scan_status_video),.out_vs(analyzer_vs),.out_de(analyzer_de),.out_rgb(analyzer_rgb),.shared_font_addr(hdmi_font_addr),.shared_font_bits(hdmi_font_bits));
end else begin assign analyzer_vs=spectrum_pixel_vs;assign analyzer_de=spectrum_pixel_de;assign analyzer_rgb=final_rgb;assign hdmi_font_addr=0;end endgenerate

ambient_brightness u_ambient_brightness(
 .remote_online(link_online),.remote_ready(link_light_ready),.remote_fault(link_light_fault),
 .remote_level(link_light_level),.remote_adc(link_light_adc),.remote_valid(link_heights_valid),
 .gain_applied(ambient_gain_applied_pid),.gain_target(ambient_gain_target_pid),.gain_tick(ambient_gain_tick_pid),
 .clk(video_clk),.rst(rst_all),.presence_run(soft_video_run),.light_data(light_adc_data),.light_toggle(light_adc_toggle),
 .vs(analyzer_vs),.de(analyzer_de),.rgb(analyzer_rgb),.out_vs(light_vs),.out_de(light_de),.out_rgb(light_rgb),
 .filtered_adc(light_filtered),.level(light_level),.gain(light_gain),.ready(light_ready),.fault(light_fault),
 .raw_adc(light_raw),.sample_count(light_samples),.frame_count(light_frames),
 .rejected_count(light_rejected),.average_count(light_averages));
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (light_vs),
    .I_de          (light_de),
    .I_rgb         (light_rgb),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (aud_mux_l),
    .I_audio_right_data (aud_mux_r),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);


// BOARD_A: all sensor/algorithm inputs are safe until transport is implemented.
assign mic_selected=0; assign mic_pcm=0; assign mic_valid=0;
assign light_adc_data=0; assign light_adc_toggle=0;
assign spectrum_heights=0; assign spectrum_heights_valid=0; assign spectrum_busy=0;
assign spectrum_band_valid=0; assign spectrum_band_addr=0; assign spectrum_band_height=0;
assign cur_h=link_heights; // Indexed renderer consumes spectrum_read_height.
wire lcd_action_valid;
wire [3:0] lcd_action_id;
wire lcd_compare_valid;
wire [3:0] lcd_compare_cmd;


lcd_touch_demo_top #(.VIDEO_ENABLE(TF_VIDEO_ENABLE),.SHARED_FONT(FM_SHARED_FONT_ENABLE),.CLK_HZ(50000000),.FM_ENABLE(FM_INTEGRATION_ENABLE),.SCAN_ENABLE(FM_SCAN_ENABLE && FM_INTEGRATION_ENABLE),.DSP_ENABLE(AUDIO_DSP_ENABLE),.TIME_ENABLE(FM_TIMESHIFT_ENABLE && AUDIO_DSP_ENABLE && FM_INTEGRATION_ENABLE)) u_lcd_touch(
 .clk(clk),.rst_n(!rst_all),.lcd_d(lcd_d),.lcd_cs_n(lcd_cs_n),.lcd_rs(lcd_rs),
 .lcd_wr_n(lcd_wr_n),.lcd_rd_n(lcd_rd_n),.lcd_rst_n(lcd_rst_n),.lcd_bl(lcd_bl),
 .tp_scl(tp_scl),.tp_sda(tp_sda),.tp_int_n(tp_int_n),.tp_rst_n(tp_rst_n),
 .action_valid(lcd_action_valid),.action_id(lcd_action_id),
 .video_status(video_status_lcd),.video_action_valid(lcd_video_event),.video_action_id(lcd_video_id),
 .time_status(ts_lcd_status),.time_action_valid(lcd_ts_event),.time_action_id(lcd_ts_id),
 .fm_action_valid(lcd_fm_event),.fm_action_id(lcd_fm_id),.fm_status(fm_ui_lcd),
 .scan_point_valid(scan_point_lcd_valid),.scan_point(scan_point_lcd),.scan_status(scan_status_lcd),.scan_action_valid(lcd_scan_event),.scan_action_id(lcd_scan_id),.scan_page_active(lcd_scan_page),.dsp_status(dsp_status_lcd),.dsp_action_valid(lcd_dsp_event),.dsp_action_id(lcd_dsp_id),
 .compare_valid(lcd_compare_valid),.compare_cmd(lcd_compare_cmd),
 .transition_mode_video(transition_mode_video),
 .transition_prev_event(lcd_transition_prev_event),
 .transition_next_event(lcd_transition_next_event),
 .prev_image_event(lcd_prev_image_event),.next_image_event(lcd_next_image_event),
 .init_done(startup_lcd_done),.init_fault(),.touch_fault(),.touch_identified(),.shared_font_addr(lcd_font_addr),.shared_font_bits(lcd_font_bits));
touch_command_cdc u_tf_video_control(.src_clk(clk),.dst_clk(video_clk),.rst(rst_all),
 .src_valid(lcd_video_event),.src_id(lcd_video_id),.dst_valid(video_event),.dst_id(video_id));
touch_command_cdc u_touch_video(.src_clk(clk),.dst_clk(video_clk),.rst(rst_all),
 .src_valid(lcd_action_valid),.src_id(lcd_action_id),.dst_valid(touch_v),.dst_id(touch_id_v));
touch_command_cdc u_touch_sd(.src_clk(clk),.dst_clk(sd_card_clk),.rst(rst_all),
 .src_valid(lcd_action_valid),.src_id(lcd_action_id),.dst_valid(touch_sd),.dst_id(touch_id_sd));
touch_command_cdc u_compare_video(.src_clk(clk),.dst_clk(video_clk),.rst(rst_all),
 .src_valid(lcd_compare_valid || lcd_transition_prev_event || lcd_transition_next_event),
 .src_id(lcd_transition_prev_event ? 4'd5 :
         lcd_transition_next_event ? 4'd6 : lcd_compare_cmd),
 .dst_valid(compare_valid_video),.dst_id(compare_cmd_video));


// PCM is the real HDMI/WAV output; mode changes NEVER pause or mute audio.
// Queue routing while another numbered command is awaiting execution ACK.
reg ts_route_pending,ts_route_stop,ts_route_ready;
wire ts_route_send=ts_route_pending && ts_route_ready && link_online && !fm_request_valid &&
 !video_fm_event && !video_scan_event && !video_dsp_event && !touch_v;
always @(posedge video_clk or posedge rst_all)begin
 if(rst_all)begin ts_route_pending<=0;ts_route_stop<=0;ts_route_ready<=0;tf_route_hold<=0;end
 else begin
  if(fm_status_valid && fm_status[106:105]!=3)tf_route_hold<=0;
  if(fm_status_valid)ts_route_ready<=1;
  if(ts_route_send && (ts_route_stop || fm_status[101]))ts_route_pending<=0;
  if(video_ts_event && (video_ts_id==0 || video_ts_id==6))begin ts_route_pending<=1;ts_route_stop<=video_ts_id==6;end
  // TF video carries WAV PCM. Request the existing WAV route and suppress
  // NET's auto-stop while its confirmed source change crosses the link.
  if(video_start)begin tf_route_hold<=1;ts_route_pending<=1;ts_route_stop<=1;end
 end
end
touch_command_cdc u_ts_event_cdc(.src_clk(clk),.dst_clk(video_clk),.rst(rst_all),
 .src_valid(lcd_ts_event),.src_id(lcd_ts_id),.dst_valid(video_ts_event),.dst_id(video_ts_id));
touch_command_cdc u_fm_event_cdc(.src_clk(clk),.dst_clk(video_clk),.rst(rst_all),
 .src_valid(lcd_fm_event),.src_id(lcd_fm_id),.dst_valid(video_fm_event),.dst_id(video_fm_id));
touch_command_cdc u_scan_event_cdc(.src_clk(clk),.dst_clk(video_clk),.rst(rst_all),
 .src_valid(lcd_scan_event),.src_id(lcd_scan_id),.dst_valid(video_scan_event),.dst_id(video_scan_id));
// Complete 128-bit status is immutable until its next four-fragment update
// (>=16 PCM frames). Capture handshake is <0.3 us; keep destination flops.
board_link_mailbox #(.W(128),.SOURCE_HOLDS(1)) u_scan_status_cdc(video_clk,clk,rst_all,scan_status_valid,scan_status_video,scan_ui_update,scan_status_lcd);
// Existing link point outputs stay held until the next accepted point packet
// (>20us). Reuse the established falling-edge destination capture; no PHY,
// packet, pin, sampling-rate or clock changes.
board_link_mailbox #(.W(32),.SOURCE_HOLDS(1)) u_scan_point_cdc(video_clk,clk,rst_all,
 scan_point_valid,{scan_generation,scan_index,scan_data},scan_point_lcd_valid,scan_point_lcd);
touch_command_cdc u_dsp_event_cdc(.src_clk(clk),.dst_clk(video_clk),.rst(rst_all),
 .src_valid(lcd_dsp_event),.src_id(lcd_dsp_id),.dst_valid(video_dsp_event),.dst_id(video_dsp_id));
board_link_mailbox #(.W(128),.SOURCE_HOLDS(1)) u_dsp_status_cdc(video_clk,clk,rst_all,dsp_status_valid,dsp_status_video,dsp_ui_update,dsp_status_lcd);
fm_console_a #(.SCAN_ENABLE(FM_SCAN_ENABLE && FM_INTEGRATION_ENABLE),.DSP_ENABLE(AUDIO_DSP_ENABLE)) u_fm_console(.clk(video_clk),.rst(rst_all),.event_valid(video_fm_event || ts_route_send),.event_id(ts_route_send ? (ts_route_stop ? 4'd2 : fm_status[101] ? 4'd1 : 4'd0) : video_fm_id),
 .legacy_valid(touch_v && FM_INTEGRATION_ENABLE),.legacy_id(touch_id_v),
 .online(link_online),.status_valid(fm_status_valid),.status(fm_status),
 .request_valid(fm_request_valid),.request_seq(fm_request_seq),.request_op(fm_request_op),.request_value(fm_request_value),
 .ui_status(fm_ui_video),.local_error(fm_local_error),.scan_event_valid(video_scan_event),.scan_event_id(video_scan_id),.dsp_event_valid(video_dsp_event),.dsp_event_id(video_dsp_id));
board_link_mailbox #(.W(48)) u_fm_status_cdc(video_clk,clk,rst_all,
 1'b1,fm_ui_video,fm_ui_update,fm_ui_lcd);
board_link_a #(.NET_VIDEO_EXT(NET_VIDEO_ENABLE),.BEAT_EXT(BEAT_ENABLE),.INDEXED(1),.FM_EXT(FM_INTEGRATION_ENABLE),.SCAN_EXT(FM_SCAN_ENABLE && FM_INTEGRATION_ENABLE),.DSP_EXT(AUDIO_DSP_ENABLE),.TIME_EXT(FM_TIMESHIFT_ENABLE && AUDIO_DSP_ENABLE && FM_INTEGRATION_ENABLE)) u_board_link(
 .beat_now_us(beat_now),.beat_enable(beat_enable),.beat_sensitivity(beat_sensitivity),
 .beat_packet_valid(beat_packet_valid),.beat_packet(beat_packet),.beat_exchange_valid(beat_exchange),
 .beat_t1(beat_t1),.beat_t2(beat_t2),.beat_t3(beat_t3),.beat_t4(beat_t4),
 .clk50(clk),.clk25(video_clk),.rst(rst_all),.sample_tick(audio_valid),
 .clk_fast(hdmi_5x_clk),.miso_extra(link_miso_extra),.mosi_extra(link_mosi_extra),
 .video_enable(net_video_enabled),.video_play(!audio_paused),.video_pixels(net_video_pixels),.video_flags(net_video_flags),
 .video_valid(net_video_valid),.video_ready(net_video_ready),.remote_video_active(net_video_remote_active),
 .remote_video_done(net_video_remote_done),.remote_video_error(net_video_remote_error),
 .pcm(FM_INTEGRATION_ENABLE ? (soft_video_run ? wav_pcm_l : 24'd0) : aud_mux_l),.mode_mic(link_mode_mic),.boot_done(startup_done),.miso(link_miso),.cs_n(link_cs_n),.sck(link_sck),.mosi(link_mosi),
 .online(link_online),.remote_mic(link_remote_mic),.present(link_person),
 .light_ready(link_light_ready),.light_fault(link_light_fault),.light_level(link_light_level),.light_adc(link_light_adc),
 .heights(link_heights),.heights_valid(link_heights_valid),.spectrum_ready(link_spectrum_ready),
 .read_addr(spectrum_read_addr),.read_height(spectrum_read_height),.freeze(spectrum_freeze),
 .net_mode(link_net_mode),.net_valid(link_net_valid),.net_pcm(link_net_pcm),
 .av_features(av_features),.av_event(av_event),
 .requested_legacy_mic(legacy_mode_mic),.fm_request_valid(fm_request_valid),.fm_request_seq(fm_request_seq),.fm_request_op(fm_request_op),.fm_request_value(fm_request_value),
 .fm_status(fm_status),.fm_status_valid(fm_status_valid),
 .scan_point_valid(scan_point_valid),.scan_generation(scan_generation),.scan_index(scan_index),.scan_data(scan_data),
 .scan_status(scan_status_video),.scan_status_valid(scan_status_valid),.stereo_pcm(ts_replay_selected ? {ts_history_pcm,ts_history_pcm} : soft_video_run ? {wav_pcm_l[23:8],wav_pcm_r[23:8]} : 32'd0),.stereo_valid(ts_replay_selected ? ts_history_valid : soft_video_run),.history_mode(ts_replay_selected),.play_epoch(ts_epoch),.capture_event(ts_capture_event),.raw_valid(ts_raw_valid),.packet_gap(ts_gap),.raw_pcm(ts_raw_pcm),.dsp_status(dsp_status_video),.dsp_status_valid(dsp_status_valid));

audio_visual_control u_av_control(.clk(video_clk),.rst(rst_all),.frame_tick(image_vs_d && !vs),
 .cmd_valid(compare_valid_video),.cmd(compare_cmd_video),.online(spectrum_on),
 .mode_mic(link_mode_mic),.presentation_mode(presentation_mode),.features(av_features),.event_toggle(av_event),
 .enabled(av_enable),.strength(av_strength),.clap_enabled(av_clap_enable),.demo(av_demo),
 .levels(av_levels),.pulse(av_pulse_original),.ring(av_ring_original),.clap_step(av_step));
assign touch_music_start=touch_v && touch_id_v==12;
endmodule
