
module top(
 inout wire fm_sda,fm_scl,
 input wire link_cs_n,link_sck,link_mosi,output wire link_miso,
 input wire [10:0] link_mosi_extra,output wire [10:0] link_miso_extra,

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
// Original FULL_STABLE implementation retained below, disabled in BOARD_B.
// 
// 
// wire rst_all;
// // Ambient status is declared before UART connections to prevent implicit nets.
// wire light_vs,light_de;
// wire [23:0] light_rgb;
// wire [11:0] light_filtered;
// wire [1:0] light_level;
// wire [4:0] light_gain;
// wire light_ready,light_fault;
// wire [11:0] light_raw;
// wire [7:0] light_samples;
// wire [6:0] light_frames;
// wire [7:0] light_rejected,light_averages;
// 
// parameter MEM_DATA_BITS = 32;
// parameter integer SW3_DEBOUNCE_CYCLES = 500000; // 10 ms at 50 MHz
// parameter [31:0] ETH_LOCAL_IP = 32'hc0a8f001; // 192.168.240.1
// parameter [47:0] ETH_LOCAL_MAC = 48'h020000000001;
// parameter [31:0] ETH_PC_IP = 32'hc0a8f002;    // 192.168.240.2
// parameter [15:0] ETH_LOCAL_PORT = 16'd1;
// parameter [15:0] ETH_PC_PORT = 16'd50000;
// parameter ADDR_BITS     = 21;
// parameter BUSRT_BITS    = 10;
// parameter [20:0] FRAME_PIXELS  = 21'd307200;   // 640*480=307200, <2^21, 21bit 可容纳
// parameter [20:0] BUF0_ADDR     = 21'd0;        // 显示缓冲 0 起始地址
// parameter [20:0] BUF1_ADDR     = FRAME_PIXELS; // 显示缓冲 1 起始地址
// parameter [20:0] BUF2_ADDR     = FRAME_PIXELS * 2;
// parameter [20:0] BUF3_ADDR     = FRAME_PIXELS * 3;
// 
// // ===================== 音频可视化(扩展要求5) 工程级参数 =====================
// // 所有 FFT/频谱参数统一在此集中配置，子模块一律以参数传递，不自行定义。
// parameter FFT_N                = 1024;    // FFT 点数
// parameter FFT_LOG2_N           = 10;      // log2(FFT_N)
// parameter FFT_INPUT_BITS       = 16;      // 送入 FFT 的采样位宽(取24bit高16位)
// parameter FFT_INTERNAL_BITS    = 26;      // FFT 实/虚部数据位宽(防溢出)
// parameter FFT_COEF_BITS        = 16;      // 窗/旋转因子位宽 Q1.15
// parameter FFT_SAMPLE_RATE      = 48000;   // 音频采样率 48kHz
// parameter FFT_HOP_SIZE         = 256;     // 滑窗步进(每帧新采样数)=75% overlap
// parameter SPECTRUM_BIN_NUM     = 64;      // HUB75 频带(柱)数
// parameter SPECTRUM_BAND_SHIFT  = 3;       // 每带 2^3=8 bin (bin1..512 -> 64 带, ~0..24kHz)
// parameter SPECTRUM_SCALE_SHIFT = 15;      // (保留未用)
// parameter SPECTRUM_MAX_BAR     = 71;      // (保留未用)
// parameter SPECTRUM_X_START     = 368;     // 频谱区左边界(右下角)
// parameter SPECTRUM_X_END       = 623;     // 右边界(区宽160)
// parameter SPECTRUM_Y_TOP       = 16;     // 区上边界
// parameter SPECTRUM_Y_BOTTOM    = 111;     // 区下边界(=TOP+MAX_BAR-1)
// parameter BAR_WIDTH            = 3;       // 柱宽
// parameter BAR_GAP              = 2;       // 柱间距 (32*3+31*2=158<=160)
// parameter SMOOTH_STEP          = 1;       // 柱高每 video_clk 平滑步长
// parameter SPECTRUM_ENABLE        = 1;     // 1=启用频谱叠加; 0=final_rgb 直接透传 vout_data(诊断用)
// parameter SPECTRUM_DEBUG_BYPASS  = 0;     // 1=spectrum_renderer 纯透传 I_rgb_bg(跳过整层绘制,诊断用)
// 
// // ===================== HUB75 频谱屏 (Phase3) 工程级参数 =====================
// parameter HUB75_ENABLE           = 1;     // 1=启用 HUB75 输出
// parameter HUB75_TEST_MODE        = 0;     // 1=固定测试图(阶梯); 0=FFT 频谱
// parameter HUB75_SCAN_ROWS        = 16;    // 1/16 扫描地址数
// parameter HUB75_WIDTH            = 64;
// parameter HUB75_HEIGHT           = 32;
// parameter HUB75_CLK_DIV          = 2;     // 每列移位用 video_clk 数(>=2, 建议2的幂)
// parameter HUB75_OE_ACTIVE_LEVEL  = 0;     // OE 有效电平(0=低使能; 参考 EN_0 亮)
// parameter HUB75_SMOOTH_STEP      = 1;     // 柱高动画步长(每次分析)
// parameter HUB75_BRIGHTNESS       = 40;    // 每行 OE 亮保持的 video_clk 数(亮度)
// parameter SPECTRUM_LOG_SHIFT    = 16;    // 幅值>>LOG_SHIFT 后 log 压缩(灵敏度, 上板调)
// parameter SPECTRUM_HDMI_ENABLE  = 1;     // 1=HDMI 叠加 64 频带透明频谱(右下角); 0=纯 BMP
// 
// // ===== HDMI 频谱视觉美化参数(统一在 top, 下传 hdmi_spectrum_overlay) =====
// parameter SPECTRUM_COLOR_MODE    = 1;     // 1=列渐变(蓝->红)
// parameter SPECTRUM_FADE_ENABLE   = 1;     // 1=柱顶 4 级 fade 融入当前背景
// parameter SPECTRUM_FADE_PERCENT  = 20;    // 文档用(实现≈18.75%~25%)
// parameter SPECTRUM_FADE_LEVELS   = 4;     // 25/50/75/100%
// parameter SPECTRUM_SMOOTH_STEP   = 1;
// parameter [23:0] SPECTRUM_COLOR0 = 24'h0080FF;   // blue
// parameter [23:0] SPECTRUM_COLOR1 = 24'h00FFFF;   // cyan
// parameter [23:0] SPECTRUM_COLOR2 = 24'h00FF40;   // green
// parameter [23:0] SPECTRUM_COLOR3 = 24'hFFFF00;   // yellow
// parameter [23:0] SPECTRUM_COLOR4 = 24'hFF8000;   // orange
// parameter [23:0] SPECTRUM_COLOR5 = 24'hFF2020;   // red
// 
// // ===================== 开机音乐 ROM+DDS (Version 9) 工程级参数(集中于此) =====================
// parameter STARTUP_MUSIC_ENABLE        = 1;    // 1=首帧后就绪播开机音一次; 0=首帧后直接 normal(不播)
// parameter STARTUP_MUSIC_BPM           = 120;  // 4/4 拍速(quarter=0.5s=24000 samples @48k)
// parameter signed [23:0] STARTUP_MUSIC_AMP = 24'sd2000000; // DDS 方波幅度(<=2^23)
// parameter STARTUP_MUSIC_ROM_DEPTH     = 128;  // 音符表深度(容纳 62 音 M1-5+M7-13, 无 M6/无 repeat)
// parameter STARTUP_MUSIC_NOTE_NUM      = 62;   // 有效音符事件数(ROM addr 0..61)
// parameter STARTUP_WAIT_FOR_PLL_LOCK   = 1;    // 1=等 audio PLL 锁定后才释放 player_rst(用已有 audio_pll_lock/rst_all); 0=仅等 rst_n
// parameter STARTUP_AUDIO_SETTLE_MS     = 300;  // V12: 首帧 BMP 就绪后 Audio Settle(ms); 以 48k audio_valid 计; 0=不 settle
// parameter STARTUP_DEBUG_DISPLAY       = 1;    // V26 Step8: 1=启用"单数码管启动态显示"(digit0 显示 0/1/2/3, 其他位息灭); 0=保持原 6 位数码管调试映射
//                                               //   注: 本参数不再改变启动生命周期(路径A 必须完整走完三步走)
// 
// // V26 Step8 Phase-B: 最终 WAV 链参数(只接线, 禁止改地址)
// parameter [20:0] AUDIO_BASE_ADDR = 21'd1703936; // D5 display buffers end at word 1662975; audio ring starts at 0x1A0000.
// parameter        AUDIO_TEST_MODE      = 0;           // 1=WAV 不可用(UNAVAIL)时 audio_mux3 sel2 输出原 I2S TestTone(DEBUG); 0=静音
// 
// // ===================== HC-SR501 人体在场检测(音频软复位) 工程级参数 =====================
// // 高电平须连续 >= 该毫秒数才认定"有人在场", 抑制线缆 EMI/电源毛刺(HC-SR501 真实触发高
// // 电平 >= ~0.3s; 门限建议 10..100, 必须 < 300)。连续低 60s 判定缺勤 -> 复位音频链。
// parameter integer PIR_PRESENCE_QUALIFY_MS = 20;
// 
// wire Sdr_init_done;
// wire Sdr_init_ref_vld;
// wire Sdr_busy;
// 
// wire sd_card_clk;
// wire ext_mem_clk;
// wire ext_mem_clk_sft;
// wire video_clk;
// wire hdmi_5x_clk;
// 
// wire hs;
// wire vs;
// wire de;
// 
// wire [23:0] vout_data_raw;
// wire [23:0] vout_data;
// wire [23:0] final_rgb;
// wire [23:0] hdmi_overlay_rgb;   // hdmi_spectrum_overlay 输出
// wire        display_valid;
// 
// // ===== 音频可视化(Phase3 HUB75)：分析链内部连线 =====
// wire        spectrum_heights_valid;
// wire [SPECTRUM_BIN_NUM*6-1:0] spectrum_heights;   // 64 列 x 6bit 柱高 0..31
// wire        spectrum_busy;
// wire [SPECTRUM_BIN_NUM*6-1:0] test_h;     // 测试图
// wire [SPECTRUM_BIN_NUM*6-1:0] target_h;   // 目标柱高(测试/FFT 二选一)
// wire [SPECTRUM_BIN_NUM*6-1:0] cur_h;      // 平滑后当前柱高
// 
// // Phase5: HDMI = 原 BMP + 64 频带透明频谱叠加(右下角, 无面板, 细柱)。
// // HUB75 与 HDMI 共用同一份 cur_h(同一 FFT/64band), 不复制分析链。
// // V27S3: Audio FSM 观测线网声明前移(频谱开关需引用, 避免前向引用)
// wire [2:0] wav_ctl_phase;       // Audio FSM 相位: A_WAIT_SD=0/A_DELAY=1/A_MUSIC=2/A_SEARCH=3/A_PLAY=4/A_ERROR=5
// wire       au_music_start;      // Audio FSM -> Startup Music 启动单拍
// wire       au_wav_search;       // Audio FSM -> A_SEARCH(WAV locator/预取)
// wire [2:0] au_digit;            // Audio FSM debug digit(0..4)
// wire       su_spectrum_on;      // 旧 Video FSM 频谱开关(保留观测, 不再驱动显示门控)
// // V27S3: 频谱开关改由 Audio FSM 驱动(Startup Music=A_MUSIC / WAV 播放=A_PLAY 时 ON;
// //   等 SD/延时/搜索/错误=OFF)。旧耦合(O_spectrum_on 受 BMP 扫描完成影响)已拆除。
// wire mic_adc_clk,mic_selected,mic_valid;
// wire [23:0] mic_pcm;
// wire spectrum_on = mic_selected || (wav_ctl_phase == 3'd2) || (wav_ctl_phase == 3'd4);
// assign final_rgb = (SPECTRUM_HDMI_ENABLE && spectrum_on) ? hdmi_overlay_rgb : vout_data;
// 
// wire [3:0]  state_code;
// wire [6:0]  seg_data_0;
// 
// // ===== 调试探针（数码管显示 + 供 ChipWatcher 选取） =====
// // 1) display_valid（sd 域同步到 clk 后实时显示）
// // 2) write_finish 是否发生过（ext_mem_clk 域 frame_write_toggle_mem 沿）
// // 3) video_read_data 是否出现过非零（video_clk 域）
// reg  dbg_rd_ev;                 // video_clk 域：曾出现 video_read_en && |video_read_data
// reg [1:0] dbg_dv_sync;
// reg [2:0] dbg_wft_sync;
// reg [1:0] dbg_rd_sync;
// reg  dbg_dv;      /* synthesis keep="true" */   // 同步后 display_valid
// reg  dbg_wfp;     /* synthesis keep="true" */   // write_finish 曾发生
// reg  dbg_rdact;   /* synthesis keep="true" */   // video_read_data 曾有非零
// wire [6:0] seg_dv7, seg_wf7, seg_rd7;
// wire [6:0] seg_su7;   // V26 Step8: 启动态 0/1/2/3 的 7 段编码
// 
// wire        video_read_req;
// wire        video_read_req_ack;
// wire        video_read_en;
// wire [31:0] video_read_data;
// 
// wire        sd_card_write_en;
// wire [31:0] sd_card_write_data;
// wire        sd_card_write_req;
// wire        sd_card_write_req_ack;
// wire        frame_write_finish;
// wire        frame_write_toggle_mem;
// 
// wire [1:0]  write_buf_idx;
// wire [1:0]  disp_buf_idx;
// wire [1:0]  disp_buf_idx_mem;
// 
// 
// wire App_rd_en;
// wire [ADDR_BITS-1:0] App_rd_addr;
// wire Sdr_rd_en;
// wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
// wire App_wr_en;
// wire [ADDR_BITS-1:0] App_wr_addr;
// wire [MEM_DATA_BITS-1:0] App_wr_din;
// wire [3:0] App_wr_dm;
// 
// wire hs_0;
// wire vs_0;
// wire de_0;
// 
// // HDMI 1.4b 音频发射相关
// wire        audio_pll_lock;
// wire        audio_mclk;
// wire        audio_i2s_bclk;
// wire        audio_i2s_lrck;
// wire        audio_i2s_dout;
// wire        audio_valid;
// wire        tone_audio_valid;
// // video_pll produces 25 MHz. Generate exactly 48000 sample enables/s.
// audio_sample_tick #(.CLK_HZ(25000000),.SAMPLE_HZ(48000)) u_sample_tick (
//     .clk(video_clk), .rst(rst_all), .tick(audio_valid)
// );
// wire [23:0] audio_left_data;
// wire [23:0] audio_right_data;
// // ---- 开机音乐(ROM+DDS)与 MUX ----
// wire        startup_active;       // 1=开机音播放中
// wire        startup_done;         // 1=开机音已播完(永不重启)
// wire signed [23:0] startup_left, startup_right;
// wire signed [23:0] aud_mux_l, aud_mux_r;   // MUX 后 PCM(HDMI 音频 + FFT 共用)
// 
// // UART is in the 25 MHz video domain; W selects, P starts/restarts.
// wire uart_play, uart_select;
// wire panel_play;
// wire panel_pause_cmd;
// wire image_key3_n; // Reserved image KEY3 handoff, active low; no image action yet.
// 
// // ===================== HC-SR501 人体在场 -> 音频软复位(PIR) =====================
// // 移植自附件 V27S14-R2/R3: 软复位只作用于音频私有链路(音频启动 FSM + WAV/开机音通路),
// // 不触碰视频/TF 图像链与共享 SD 控制器。
// //   连续高 >= PIR_PRESENCE_QUALIFY_MS(20ms) -> 认定在场, rst_soft=1(运行);
// //   连续低 60s -> rst_soft=0(复位音频链并静音); 未达标短毛刺忽略且不清缺勤计时。
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
// // rst_soft -> 三个时钟域: 快速置位, 释放沿对齐各域时钟负沿(工程既有"下降沿释放"约定,
// // 与 audio_uart_session 同策略, 避开异步复位 recovery/removal 窗口)。
// reg [1:0] soft_sd_ps, soft_mem_ps, soft_video_ps;
// reg       soft_sd_run, soft_mem_run, soft_video_run;   // 1=该域音频链运行(0=缺勤复位)
// always @(posedge sd_card_clk or negedge rst_n)
//     if (!rst_n) soft_sd_ps    <= 2'b00; else soft_sd_ps    <= {soft_sd_ps[0],    rst_soft};
// always @(posedge ext_mem_clk or negedge rst_n)
//     if (!rst_n) soft_mem_ps   <= 2'b00; else soft_mem_ps   <= {soft_mem_ps[0],   rst_soft};
// always @(posedge video_clk or negedge rst_n)
//     if (!rst_n) soft_video_ps <= 2'b00; else soft_video_ps <= {soft_video_ps[0], rst_soft};
// always @(negedge sd_card_clk or negedge rst_n)
//     if (!rst_n) soft_sd_run    <= 1'b0; else soft_sd_run    <= soft_sd_ps[0];
// always @(negedge ext_mem_clk or negedge rst_n)
//     if (!rst_n) soft_mem_run   <= 1'b0; else soft_mem_run   <= soft_mem_ps[0];
// always @(negedge video_clk or negedge rst_n)
//     if (!rst_n) soft_video_run <= 1'b0; else soft_video_run <= soft_video_ps[0];
// 
// reg audio_paused;
// wire panel_start = panel_pause_cmd && !wav_wav_play && !session_resetting;
// always @(posedge video_clk or posedge rst_all) begin
//  if (rst_all) audio_paused<=0;
//  else if (!soft_video_run) audio_paused<=0;   // PIR 缺勤: 清除暂停, 人回来自动播放
//  else if (uart_play || panel_play || panel_start) audio_paused<=0;
//  else if (panel_pause_cmd && wav_wav_play && !session_resetting)
//   audio_paused<=~audio_paused;
// end
// wire panel_image_next_n, panel_image_auto_n;
// reg presentation_sw_meta, presentation_sw_sync;
// always @(posedge video_clk or posedge rst_all) begin
//  if (rst_all) begin presentation_sw_meta<=0; presentation_sw_sync<=0; end
//  else begin presentation_sw_meta<=sw3; presentation_sw_sync<=presentation_sw_meta; end
// end
// wire presentation_mode = presentation_sw_sync && !audio_mode_sw;
// reg presentation_mode_sd_meta;
// always @(negedge sd_card_clk or posedge rst_all) begin
//  if (rst_all) presentation_mode_sd_meta<=0;
//  else presentation_mode_sd_meta<=presentation_mode;
// end
// reg [1:0] image_keys_meta, image_keys_sync;
// // First-stage image key capture on the opposite edge avoids same-edge hold skew.
// always @(negedge sd_card_clk or posedge rst_all) begin
//  if (rst_all) image_keys_meta<=2'b11;
//  else image_keys_meta<={(presentation_mode ? 1'b1 : panel_image_next_n),panel_image_auto_n};
// end
// always @(posedge sd_card_clk or posedge rst_all) begin
//  if (rst_all) image_keys_sync<=2'b11;
//  else image_keys_sync<=image_keys_meta;
// end
// wire [87:0] panel_wav_name;
// wire [87:0] current_wav_name_c;
// wire wav_play_finished;
// wire [87:0] selected_wav_name, active_wav_name;
// wire audio_session_rst_a,audio_session_rst_b,audio_session_rst_c;
// wire manual_session, session_resetting;
// wire sd_audio_idle, mem_audio_idle;
// 
// // PIR 软复位与会话复位合并(仅音频私有链): 会话复位经空闲排空握手后复位; PIR 缺勤立即复位并静音。
// // 两路信号均对齐各域时钟负沿(会话复位=audio_uart_session 负沿释放; PIR=上面的负沿寄存器),
// // 合并后释放沿仍在负沿, 保持既有 recovery/removal 余量。
// wire audio_rst_sd    = audio_session_rst_a | ~soft_sd_run;    // A 域 sd_card_clk
// wire audio_rst_mem   = audio_session_rst_b | ~soft_mem_run;   // B 域 ext_mem_clk
// wire audio_rst_video = audio_session_rst_c | ~soft_video_run; // C 域 video_clk
// 
// // SW3 is asynchronous and mechanical. Synchronize it and require 10 ms of
// // stable input before changing the active WAV source. Reset defaults to SD.
// // Disabled for the suffix/D5 build; original implementation retained.
// // localparam integer SW3_DB_BITS = $clog2(SW3_DEBOUNCE_CYCLES + 1);
// // reg sw3_meta=0, sw3_sync=0;
// // reg wav_source_net=0;
// // reg [SW3_DB_BITS-1:0] sw3_db_count=0;
// // always @(posedge sd_card_clk or negedge rst_n) begin
// //     if (!rst_n) begin
// //         sw3_meta       <= 1'b0;
// //         sw3_sync       <= 1'b0;
// //         wav_source_net <= 1'b0;
// //         sw3_db_count   <= {SW3_DB_BITS{1'b0}};
// //     end else begin
// //         sw3_meta <= sw3;
// //         sw3_sync <= sw3_meta;
// //         if (sw3_sync == wav_source_net) begin
// //             sw3_db_count <= {SW3_DB_BITS{1'b0}};
// //         end else if (sw3_db_count == SW3_DEBOUNCE_CYCLES-1) begin
// //             wav_source_net <= sw3_sync;
// //             sw3_db_count   <= {SW3_DB_BITS{1'b0}};
// //         end else begin
// //             sw3_db_count <= sw3_db_count + 1'b1;
// //         end
// //     end
// // end
// 
// wire wav_source_net = 1'b0; // Ethernet disabled; SW3 is reserved.
// 
// // Ethernet WAV source. app_clk and the WAV byte consumer both use the
// // existing 50 MHz sd_card_clk, so the ready/valid stream has no CDC boundary.
// wire [7:0]  net_file_byte;
// wire        net_file_valid;
// wire        net_file_end;
// wire        net_file_ready;
// wire        net_file_error;
// wire [31:0] net_file_size;
// wire [31:0] net_next_offset;
// wire        net_app_reset;
// wire        net_debug_rx_seen;
// wire        net_debug_wack_sent;
// 
// // Disabled for the suffix/D5 build; original implementation retained.
// // fpga_wav_top #(
// //     .APP_CLK_HZ  (50000000),
// //     .AUTO_SPEED  (1),
// //     .SPEED       (2'b10),
// //     .LOCAL_IP    (ETH_LOCAL_IP),
// //     .LOCAL_MAC   (ETH_LOCAL_MAC),
// //     .LOCAL_PORT  (ETH_LOCAL_PORT),
// //     .PC_IP       (ETH_PC_IP),
// //     .PC_PORT     (ETH_PC_PORT),
// //     .LOOPBACK_MODE(0)
// // ) u_fpga_wav (
// //     .ref_clk_50       (clk),
// //     .app_clk          (sd_card_clk),
// //     .rst_n            (rst_n),
// //     .file_source_enable(wav_source_net),
// //     .rgmii_rxc        (eth_rxc),
// //     .rgmii_rx_ctl     (eth_rx_ctl),
// //     .rgmii_rxd        (eth_rxd),
// //     .rgmii_txc        (eth_txc),
// //     .rgmii_tx_ctl     (eth_tx_ctl),
// //     .rgmii_txd        (eth_txd),
// //     .phy_mdc          (eth_mdc),
// //     .phy_mdio         (eth_mdio),
// //     .file_byte        (net_file_byte),
// //     .file_valid       (net_file_valid),
// //     .file_ready       (net_file_ready),
// //     .file_error       (net_file_error),
// //     .file_size        (net_file_size),
// //     .file_end         (net_file_end),
// //     .next_offset      (net_next_offset),
// //     .debug_rx_seen    (net_debug_rx_seen),
// //     .debug_wack_sent  (net_debug_wack_sent),
// //     .app_reset        (net_app_reset),
// //     .phy_ready        (eth_phy_ready),
// //     .phy_fault        (eth_phy_fault),
// //     .link_up          (eth_link_up),
// //     .full_duplex      (eth_full_duplex),
// //     .link_speed       (eth_link_speed),
// //     .rx_overflow      (eth_rx_overflow),
// //     .tx_overflow      (eth_tx_overflow),
// //     .ip_error         (eth_ip_error),
// //     .arp_error        (eth_arp_error)
// // );
// 
// 
// assign {eth_txc,eth_tx_ctl,eth_txd,eth_mdc}=7'd0;
// assign eth_mdio=1'bz;
// assign {eth_phy_ready,eth_phy_fault,eth_link_up,eth_full_duplex,eth_link_speed,eth_rx_overflow,eth_tx_overflow,eth_ip_error,eth_arp_error}=10'd0;
// assign net_file_byte=0; assign net_file_valid=0; assign net_file_end=0;
// assign net_file_size=0; assign net_next_offset=0; assign net_app_reset=0;
// assign net_debug_rx_seen=0; assign net_debug_wack_sent=0;
// 
// // ===================== V26 Step8: audio_wav_top_path(最终 WAV 链) 顶层连线 =====================
// wire signed [23:0] wav_pcm_l, wav_pcm_r;   // audio_wav_top_path 统一 24bit PCM(video 域, HDMI/FFT 同源)
// wire               wav_pcm_v;
// // A 域 SD m1 client(<-> sd_read_arbiter.m1 <-> 唯一 sd_card_top)
// wire        wav_sd_req;  wire [31:0] wav_sd_addr;
// wire [7:0]  wav_sd_data; wire wav_sd_valid, wav_sd_end;
// // B 域 Ring mem 口(接 audio_sdram_access.a_*; RING_AW=13 -> 8192 帧 32KB)
// wire                wav_ring_wr_req; wire [12:0] wav_ring_wr_addr; wire [31:0] wav_ring_wr_data;
// wire                wav_ring_wr_ready;
// wire                wav_ring_rd_req; wire [12:0] wav_ring_rd_addr;
// wire [31:0]         wav_ring_rd_data; wire wav_ring_rd_valid;
// // C 域 MUX/控制/水位(观测)
// wire [1:0]  wav_audio_sel; wire wav_wav_play; wire wav_unlock_image;
// // (V27S3: wav_ctl_phase / au_music_start / au_wav_search / au_digit 声明已前移至频谱开关处)
// wire        wav_ready_c; wire wav_found_c; wire wav_err_c; wire wav_done_c;
// wire [13:0] wav_ring_used; wire wav_f1_full, wav_f1_empty, wav_f2_full, wav_f2_empty;
// // video 域: frame_read_write(V 主) <-> audio_sdram_access.v_* 口
// wire                vrd_en; wire [ADDR_BITS-1:0] vrd_addr;
// wire                vrd_valid; wire [MEM_DATA_BITS-1:0] vrd_data;
// wire                vwr_en; wire [ADDR_BITS-1:0] vwr_addr;
// wire [MEM_DATA_BITS-1:0] vwr_din; wire [3:0] vwr_dm;
// wire        player_rst;
// wire        acr_valid;
// wire [19:0] acr_cts;
// wire [19:0] acr_n;
// 
// wire        axis_s_user;
// wire        axis_s_valid;
// wire        axis_s_last;
// wire [23:0] axis_s_data;
// wire        axis_s_ready;
// 
// wire        edid_trig;
// wire        edid_valid;
// wire [7:0]  edid_data;
// 
// wire [9:0]  tmds_ch0_data;
// wire [9:0]  tmds_ch1_data;
// wire [9:0]  tmds_ch2_data;
// wire [9:0]  tmds_clk_data;
// 
// // ===================== Startup Image + Startup Music (V12) =====================
// // 三阶段: WAIT_FIRST_FRAME(0) -> STARTUP_SETTLE(1) -> STARTUP_MUSIC(2) -> NORMAL_PLAY(3)
// //  阶段0(首帧 BMP 未就绪): 无 Music/无 TestTone/无频谱/无切图(静音), 等待第一张 BMP。
// //  阶段1(首帧就绪): 锁第一张 BMP(=Splash, 非黑屏); 300ms settle(静音/频谱关); 然后 Music+Spectrum。
// //  阶段2(music done): 解锁并自动 0->1 切第二张; TestTone+频谱恢复; 手动/自动切图恢复。
// // display_valid(sd 域)同步到 video_clk; 其高电平 = 第一张 BMP 已整帧写完成并提交(首帧就绪)。
// // 统一复位：TF 图像链路 + HDMI 音频链路(声明置于首次使用之前, 复位逻辑不变)
// assign rst_all = ~rst_n | ~audio_pll_lock;
// reg dv_s0, dv_s1;
// always @(posedge video_clk or posedge rst_all) begin
//     if (rst_all) begin dv_s0 <= 1'b0; dv_s1 <= 1'b0; end
//     else begin dv_s0 <= display_valid; dv_s1 <= dv_s0; end
// end
// wire first_frame_ready = dv_s1;
// // ---- V26 Step8: 启动容错判定信号(全部来自真实扫描/定位结果, 无固定 delay) ----
// // bmp(sd 域): sd_card_bmp.O_scan_done / O_bmp_found(只观测输出, 扫描 FSM 未改)
// wire bmp_scan_done_sd, bmp_found_sd;
// wire sd_init_done;               // sd_card_top 初始化完成(sd 域; 声明上移, 供给启动判定用)
// // 单比特 CDC: sd_card_clk -> video_clk(2FF, 带 rst_all 复位)
// reg bsd_s0, bsd_s1, bfd_s0, bfd_s1, sid_s0, sid_s1;
// always @(posedge video_clk or posedge rst_all) begin
//     if (rst_all) begin
//         bsd_s0 <= 1'b0; bsd_s1 <= 1'b0;
//         bfd_s0 <= 1'b0; bfd_s1 <= 1'b0;
//         sid_s0 <= 1'b0; sid_s1 <= 1'b0;
//     end else begin
//         bsd_s0 <= bmp_scan_done_sd;  bsd_s1 <= bsd_s0;
//         bfd_s0 <= bmp_found_sd;      bfd_s1 <= bfd_s0;
//         sid_s0 <= sd_init_done;      sid_s1 <= sid_s0;
//     end
// end
// wire bmp_scan_done_v = bsd_s1;   // video 域: BMP 扫描结束
// wire bmp_found_v     = bfd_s1;   // video 域: 有 BMP(>=1)
// wire sd_init_done_v  = sid_s1;   // video 域: SD 初始化完成
// wire [1:0] su_digit;             // V26 Step8: 0=扫描中 1=BMP路径 2=WAV路径 3=无媒体
// wire su_no_bmp;                  // V26 Step8: 已判定无 BMP
// wire [2:0] su_state;            // 0..6 = SCAN/WAIT/SETTLE/MUSIC/NORMAL/NOWAV/NOMEDIA
// wire startup_phase;             // SETTLE/MUSIC(锁第一张 BMP/key)
// wire music_on;                  // MUSIC -> player I_start
// wire test_tone_on;              // NORMAL(C4~C5 Test Tone 允许)
// wire music_done_pulse;
// startup_controller #(
//     .STARTUP_MUSIC_ENABLE   (STARTUP_MUSIC_ENABLE),
//     .STARTUP_AUDIO_SETTLE_MS(STARTUP_AUDIO_SETTLE_MS),
//     .STARTUP_DEBUG_DISPLAY  (1'b0)   // V26 Step8: 生命周期恒按正常流程(数码管开关见顶层 STARTUP_DEBUG_DISPLAY)
// ) u_startup_controller (
//     .I_clk              (video_clk),
//     .I_rst              (rst_all),
//     .I_pll_locked       (audio_pll_lock),
//     .I_first_frame_ready(first_frame_ready),
//     .I_sample_enable    (audio_valid),
//     .I_music_done       (startup_done),
//     .I_sd_init_done     (sd_init_done_v),
//     .I_bmp_scan_done    (bmp_scan_done_v),
//     .I_bmp_found        (bmp_found_v),
//     .I_wav_found        (wav_found_c),
//     .I_wav_err          (wav_err_c),
//     .O_state            (su_state),
//     .O_media_digit      (su_digit),
//     .O_no_bmp           (su_no_bmp),
//     .O_startup_phase    (startup_phase),
//     .O_music_on         (music_on),
//     .O_spectrum_on      (su_spectrum_on),   // V27S3: 不再驱动 spectrum_on(改由 Audio FSM)
//     .O_test_tone_on     (test_tone_on),
//     .O_done_pulse       (music_done_pulse)
// );
// // V26 Step8: 图片锁/解锁改由 audio_startup_wav_ctl 的 O_unlock_image(video 域)同步到 sd 域:
// //   未解锁(=0: Startup Music / 等 WAV ready 期间)锁第一张 BMP(屏蔽 key_next/key_auto, 不提前切图);
// //   解锁(=1: PLAY_WAV 或 WAV_UNAVAIL)沿自动注入一次"下一张"(按住 key_next 低电平 ~30ms > 去抖 20ms)
// //   -> sd_card_bmp 载入并提交第二张(image_index 0->1 一次), 之后恢复手动/自动切换。sd_card_bmp FSM 未改。
// reg sp_sd0, sp_sd1;
// always @(posedge sd_card_clk or posedge rst_all) begin
//     if (rst_all) begin sp_sd0 <= 1'b0; sp_sd1 <= 1'b0; end
//     // V27S2: 图片解锁不再由音频 ctl 驱动(音视频解耦) => 图片/按键路径只依赖 Video
//     else begin sp_sd0 <= 1'b1; sp_sd1 <= sp_sd0; end
// end
// reg hold_prev;
// reg adv_on;
// reg [23:0] adv_t;
// always @(posedge sd_card_clk or posedge rst_all) begin
//     if (rst_all) begin hold_prev <= 1'b0; adv_on <= 1'b0; adv_t <= 24'd0; end
//     else begin
//         hold_prev <= sp_sd1;
//         if (!hold_prev && sp_sd1 && !adv_on) begin adv_on <= 1'b1; adv_t <= 24'd0; end  // 解锁沿 -> 自动切一张
//         else if (adv_on) begin
//             if (adv_t >= 24'd2_999_999) adv_on <= 1'b0;   // ~30ms @100MHz(>去抖 20ms)
//             else adv_t <= adv_t + 24'd1;
//         end
//     end
// end
// wire key_next_eff = ~sp_sd1 ? 1'b1 : (adv_on ? 1'b0 : image_keys_sync[1]);  // 未解锁松键(锁图); 解锁沿自动按一次; 之后手动
// wire key_auto_eff = ~sp_sd1 ? 1'b1 : image_keys_sync[0];                // 未解锁不允许 auto 切换
// 
// // 开机音乐播放器复位: 可选“等 PLL 锁定”(默认)或仅等 rst_n。
// // PIR 注: 播放器刻意不并入软复位 —— "人回来不重播开机流程"由粘性 startup_done 与
// //   audio_startup_wav_ctl 的 A_DELAY 快捷跳转实现; 若把播放器也软复位, 该粘性标志
// //   会被清掉, 人回来后就会重播开机音乐。
// assign player_rst = (STARTUP_WAIT_FOR_PLL_LOCK == 1) ? rst_all : ~rst_n;
// 
// // V11: 首帧提交(display_valid)前黑屏; 提交后显示当前缓冲(首张 BMP = Splash 保持到 music done, 之后正常轮播)。
// // 不再用 startup_active 把画面置黑(Splash 不能黑屏)。
// 
// 
// // 保持你原来的 TF / SDRAM / video 时钟
// sys_pll sys_pll_m0(
//     .refclk     (clk),
//     .clk0_out   (sd_card_clk),
//     .clk1_out   (ext_mem_clk),
//     .clk2_out   (ext_mem_clk_sft),
//     .reset      (1'b0)
// );
// 
// video_pll video_pll_m0(
//     .refclk     (clk),
//     .clk0_out   (video_clk),
//     .clk1_out   (hdmi_5x_clk),
//     .clk2_out   (mic_adc_clk),
//     .reset      (1'b0)
// );
// 
// // 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
// PLL_HDMI_AUDIO u_audio_pll(
//     .refclk     (clk),
//     .reset      (1'b0),
//     .extlock    (audio_pll_lock),
//     .clk0_out   (),
//     .clk1_out   (),
//     .clk2_out   (audio_mclk)
// );
// 
// // ===================== V25 Step2: 共享唯一 SD 控制器 + SD Read Arbiter =====================
// // sd_card_bmp(BMP client, m0) + 预留 WAV/FAT(m1, 本步不接) 经 sd_read_arbiter
// // 共享唯一 sd_card_top(控制器上移到 top, 物理 SD IO 位置不变)。
// // V26 Step8: SD 初始化完成 -> video 域 2FF 已在启动判定区实现(sd_init_done_v)
// wire        bmp_sd_req;  wire [31:0] bmp_sd_addr;      // BMP -> arbiter m0
// wire [7:0]  bmp_sd_data; wire bmp_sd_valid, bmp_sd_end;// arbiter m0 -> BMP
// wire        ctr_sd_req;  wire [31:0] ctr_sd_addr;      // arbiter -> sd_card_top
// wire [7:0]  ctr_sd_data; wire ctr_sd_valid, ctr_sd_end;// sd_card_top -> arbiter
// wire [1:0]  sd_arb_owner; wire sd_arb_busy;
// 
// // ===================== TF 多图扫描与缓存（双缓冲） =====================
// // BMP(m0,优先) + WAV/FAT(m1=V26 Step8 audio_wav_top_path) -> 唯一 sd_card_top
// sd_read_arbiter u_sd_read_arbiter (
//     .clk(sd_card_clk), .rst(rst_all), .I_init_done(sd_init_done),
//     .m0_req(bmp_sd_req), .m0_addr(bmp_sd_addr),
//     .m0_data(bmp_sd_data), .m0_valid(bmp_sd_valid), .m0_end(bmp_sd_end),
//     .m1_req(wav_sd_req), .m1_addr(wav_sd_addr),
//     .m1_data(wav_sd_data), .m1_valid(wav_sd_valid), .m1_end(wav_sd_end),
//     .ctr_req(ctr_sd_req), .ctr_addr(ctr_sd_addr),
//     .ctr_data(ctr_sd_data), .ctr_valid(ctr_sd_valid), .ctr_end(ctr_sd_end),
//     .o_owner(sd_arb_owner), .o_busy(sd_arb_busy), .o_m1_idle(sd_audio_idle)
// );
// 
// // 唯一 SD 控制器(物理 IO: SD_nCS/DCLK/MOSI/MISO 位置不变)
// sd_card_top sd_card_top_m0 (
//     .clk(sd_card_clk), .rst(rst_all),
//     .SD_nCS(sd_ncs), .SD_DCLK(sd_dclk), .SD_MOSI(sd_mosi), .SD_MISO(sd_miso),
//     .sd_init_done(sd_init_done),
//     .sd_sec_read(ctr_sd_req), .sd_sec_read_addr(ctr_sd_addr),
//     .sd_sec_read_data(ctr_sd_data), .sd_sec_read_data_valid(ctr_sd_valid),
//     .sd_sec_read_end(ctr_sd_end),
//     .sd_sec_write(1'b0), .sd_sec_write_addr(32'd0),
//     .sd_sec_write_data(8'd0), .sd_sec_write_data_req(), .sd_sec_write_end()
// );
// 
// seg_decoder seg_decoder_m0(
//     .bin_data          (state_code),
//     .seg_data          (seg_data_0)
// );
// 
// // 调试：数码管 digit0/1/2 显示 3 个探针的 0/1
// seg_decoder seg_dbg_dv_m0(
//     .bin_data          ({3'd0, dbg_dv}),
//     .seg_data          (seg_dv7)
// );
// seg_decoder seg_dbg_wf_m0(
//     .bin_data          ({3'd0, dbg_wfp}),
//     .seg_data          (seg_wf7)
// );
// seg_decoder seg_dbg_rd_m0(
//     .bin_data          ({3'd0, dbg_rdact}),
//     .seg_data          (seg_rd7)
// );
// 
// // ================= V27S13-R10-DIAG: 极简调试显示(只读, 不回馈任何控制/数据通路) =================
// //   STARTUP_DEBUG_DISPLAY=1 时:
// //     digit0 = 主流程状态码 0..4:
// //               0 = 系统加载(默认)        1 = 开始播放开机音乐  2 = 开始寻找 WAV
// //               3 = 找到 WAV             4 = 没找到 WAV/出错
// //     digit1 = 仅两种字符:
// //               A = FIFO#2 为空(wav_f2_empty==1)
// //               B = FSM 未跳转: 已 ready 却仍停在寻找 WAV = (au_digit==2) && wav_ready_c
// //               否则熄灭
// //   域核对(无新增 CDC, 全部复用既有 C 域信号):
// //     - au_digit[2:0](audio_startup_wav_ctl: 0=WAIT_SD/DELAY 1=MUSIC 2=SEARCH 3=PLAY 4=ERROR)
// //     - wav_found_c / wav_err_c: audio_wav_top_path 内已由 2FF 同步到 clk_c
// //     - wav_f2_empty: FIFO#2 **读侧(clk_c)** 空标志
// //     - wav_ready_c(=o_wav_ready): 工程既有 C 域信号, 不新造 ready 逻辑
// //   不再显示: BMP 搜索/arbiter owner/wav_done/milestone 及其它内部信息; 不新增 2FF/状态机。
// wire [2:0] main_code =
//     wav_err_c         ? 3'd4 :      // 没找到 WAV / 出错
//     wav_found_c       ? 3'd3 :      // 找到 WAV
//     (au_digit==3'd2)  ? 3'd2 :      // 正在寻找 WAV(A_SEARCH)
//     (au_digit==3'd1)  ? 3'd1 :      // 开始播放开机音乐(A_MUSIC)
//                         3'd0;        // 系统加载(默认)
// // Ethernet receive/WACK diagnostic on digit0:
// //   A = no UDP application byte has reached udp_wav_receiver
// //   B = a UDP message was seen, but no complete WACK has been emitted
// //   C = the complete 20-byte WACK was emitted to the UDP transmit interface
// reg [1:0] net_rx_seen_clk_sync;
// reg [1:0] net_wack_sent_clk_sync;
// always @(posedge clk or negedge rst_n) begin
//     if (!rst_n) begin
//         net_rx_seen_clk_sync   <= 2'b00;
//         net_wack_sent_clk_sync <= 2'b00;
//     end else begin
//         net_rx_seen_clk_sync   <= {net_rx_seen_clk_sync[0], net_debug_rx_seen};
//         net_wack_sent_clk_sync <= {net_wack_sent_clk_sync[0], net_debug_wack_sent};
//     end
// end
// wire [3:0] ethernet_diag_code = net_wack_sent_clk_sync[1] ? 4'hc :
//                                 net_rx_seen_clk_sync[1]   ? 4'hb : 4'ha;
// seg_decoder seg_dbg_sys_m0(
//     .bin_data          (ethernet_diag_code),
//     .seg_data          (seg_su7)
// );
// 
// // 第二位: A = FIFO#2 为空; B = FSM 未跳转(已 ready 却仍在 A_SEARCH); 否则熄灭
// //   注: 当 au_digit==2(仍在搜索)时——若 wav_err_c=1, ctl 必已转 A_ERROR => au_digit!=2, 故此时 err 必为 0;
// //       而 wav_ready_c=1(即 FIFO#2 非空)时, found_l 必然已置 1(数据晚于 found 到达) =>
// //       "au_digit==2 && wav_ready_c" 即等价于 "found && !err && !f2_empty 但 FSM 未跳"。
// wire fsm_stuck_dbg = (au_digit == 3'd2) && wav_ready_c;
// wire d1_show_a = wav_f2_empty;
// wire d1_show_b = fsm_stuck_dbg && ~wav_f2_empty;   // 严格按 if/else 优先级: A 优先于 B
// wire [3:0] d1_bin = d1_show_a ? 4'd10 : 4'd11;     // A=0xA('A') 否则 B=0xB('b')
// wire [6:0] seg_d1dig7;
// seg_decoder seg_dbg_d1_m0(.bin_data(d1_bin), .seg_data(seg_d1dig7));
// 
// wire [7:0] seg_d0_mux = (STARTUP_DEBUG_DISPLAY == 1) ? {1'b1, seg_su7}  : {1'b1,seg_dv7};   // d0: 主流程 0..4
// wire [7:0] seg_d1_mux = (STARTUP_DEBUG_DISPLAY == 1) ? 8'hff : {1'b1,seg_wf7};
// wire [7:0] seg_d2_mux = (STARTUP_DEBUG_DISPLAY == 1) ? 8'hff : {1'b1,seg_rd7};             // d2: 熄灭(简化)
// wire [7:0] seg_d3_mux = 8'hff;      // d3: 熄灭(简化)
// wire [7:0] seg_d4_mux = 8'hff;      // d4: 熄灭(简化)
// wire [7:0] seg_d5_mux = (STARTUP_DEBUG_DISPLAY == 1) ? 8'hff : {1'b1,seg_data_0};          // d5: 熄灭(简化)
// 
// seg_scan seg_scan_m0(
//     .clk               (clk),
//     .rst_n             (rst_n),
//     .seg_sel           (seg_sel),
//     .seg_data          (seg_data),
//     .seg_data_0        (seg_d0_mux),   // digit0(debug): 主流程 0..4
//     .seg_data_1        (seg_d1_mux),   // digit1(debug): A=FIFO2空 / B=FSM未跳转
//     .seg_data_2        (seg_d2_mux),   // digit2(debug): 熄灭
//     .seg_data_3        (seg_d3_mux),
//     .seg_data_4        (seg_d4_mux),
//     .seg_data_5        (seg_d5_mux)    // digit5(debug): 熄灭(或原 SD state_code)
// );
// 
// // ---- video_clk 域：检测“出现过非零 video_read_data” ----
// always @(posedge video_clk or posedge rst_all) begin
//     if (rst_all)
//         dbg_rd_ev <= 1'b0;
//     else if (video_read_en && (|video_read_data))
//         dbg_rd_ev <= 1'b1;
// end
// 
// // ---- clk(50M) 域：同步并锁存三个探针（数码管驱动） ----
// always @(posedge clk or negedge rst_n) begin
//     if (!rst_n) begin
//         dbg_dv_sync  <= 2'd0;
//         dbg_wft_sync <= 3'd0;
//         dbg_rd_sync  <= 2'd0;
//         dbg_dv       <= 1'b0;
//         dbg_wfp      <= 1'b0;
//         dbg_rdact    <= 1'b0;
//     end else begin
//         dbg_dv_sync  <= {dbg_dv_sync[0],  display_valid};
//         dbg_wft_sync <= {dbg_wft_sync[1:0], frame_write_toggle_mem};
//         dbg_rd_sync  <= {dbg_rd_sync[0],   dbg_rd_ev};
//         dbg_dv       <= dbg_dv_sync[1];
//         if (dbg_wft_sync[2] != dbg_wft_sync[1])  // toggle 沿 => write_finish 发生
//             dbg_wfp <= 1'b1;
//         if (dbg_rd_sync[1])
//             dbg_rdact <= 1'b1;
//     end
// end
// 
// // ===================== 原图像时序与帧缓存 =====================
// wire audio_mem_pause, video_mem_busy;
// wire signed [7:0] image_brightness_target_pid, image_brightness_applied_pid;
// wire image_brightness_tick_pid, ambient_gain_tick_pid;
// wire [6:0] ambient_gain_target_pid, ambient_gain_applied_pid;
// brightness_pid_pair u_brightness_pid(
//  .clk(video_clk),.rst(rst_all),
//  .tick_manual(image_brightness_tick_pid),.tick_gain(ambient_gain_tick_pid),
//  .target_manual(image_brightness_target_pid),.target_gain(ambient_gain_target_pid),
//  .value_manual(image_brightness_applied_pid),.value_gain(ambient_gain_applied_pid));
// d5_visual_path #(
//  .PRESENTATION_ICON_ID(3'd0),.PRESENTATION_ICON_COL(5'd15),.PRESENTATION_ICON_ROW(5'd7),
//  .PRESENTATION_HOTSPOT1({5'd1,5'd4,2'b00}),
//  .PRESENTATION_HOTSPOT2({5'd1,5'd7,2'b01}),
//  .PRESENTATION_HOTSPOT3({5'd13,5'd8,2'b11}),
//  .PRESENTATION_HOTSPOT4({5'd15,5'd7,2'b11})
// ) u_d5_visual (
//  .brightness_target(image_brightness_target_pid),.brightness_tick(image_brightness_tick_pid),
//  .brightness_applied(image_brightness_applied_pid),
//  .clk(clk), .rst_all(rst_all), .sd_card_clk(sd_card_clk), .ext_mem_clk(ext_mem_clk), .video_clk(video_clk),
//  .key1(presentation_mode_sd_meta ? 1'b1 : image_keys_sync[1]), .key2(image_keys_sync[0]), .key_adjust(image_key3_n), .adjust_select(adjust_select),
//  .presentation_mode(presentation_mode),.presentation_key_next_n(panel_image_next_n),
//  .sd_init_done(sd_init_done), .sd_req(bmp_sd_req), .sd_addr(bmp_sd_addr),
//  .sd_data(bmp_sd_data), .sd_valid(bmp_sd_valid), .sd_end(bmp_sd_end),
//  .scan_done(bmp_scan_done_sd), .bmp_found(bmp_found_sd),
//  .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld), .Sdr_busy(Sdr_busy),
//  .App_rd_en(vrd_en), .App_rd_addr(vrd_addr), .Sdr_rd_en(vrd_valid), .Sdr_rd_dout(vrd_data),
//  .App_wr_en(vwr_en), .App_wr_addr(vwr_addr), .App_wr_din(vwr_din), .App_wr_dm(vwr_dm),
//  .audio_pause(audio_mem_pause), .transport_busy(video_mem_busy),
//  .hs(hs), .vs(vs), .de(de), .vout_data(vout_data), .display_valid_active(display_valid), .state_code(state_code),
//  .frame_write_toggle_mem(frame_write_toggle_mem), .video_read_en(video_read_en), .video_read_data(video_read_data)
// );
// 
// // ===== V26 Step8: audio_sdram_access 仲裁桥插入 frame_read_write(V) <-> sdram U3; A 侧接 WAV Ring =====
// // 每物理 SDRAM 用户口仅一个驱动源(App_*/Sdr_* 由本桥独占); 视频优先/音频插空; AUDIO_BASE=0x0A0000。
// audio_sdram_access #(
//     .ADDRW     (21),
//     .DATAW     (32),
//     .DMW       (4),
//     .AADDRW    (13),                    // ring mem addr 13bit(8192 帧 = 32KB)
//     .AUDIO_BASE(AUDIO_BASE_ADDR),       // 21'd655360 = 0x0A0000(高于 BUF0/BUF1, 不重叠)
//     .TAGDEPTH  (64),
//     .TAGW      (6)
// ) u_audio_sdram_access (
//     .audio_idle(mem_audio_idle),
//     .Sdr_init_done(Sdr_init_done),.Sdr_init_ref_vld(Sdr_init_ref_vld),.Sdr_busy(Sdr_busy),
//     .v_busy(video_mem_busy),.v_pause(audio_mem_pause),
//     .clk(ext_mem_clk), .rst(rst_all),
//     // -> sdram U3
//     .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr), .App_wr_dm(App_wr_dm), .App_wr_din(App_wr_din),
//     .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr),
//     .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
//     // V: frame_read_write
//     .v_wr_en(vwr_en), .v_wr_addr(vwr_addr), .v_wr_dm(vwr_dm), .v_wr_din(vwr_din),
//     .v_rd_en(vrd_en), .v_rd_addr(vrd_addr),
//     .v_rd_valid(vrd_valid), .v_rd_data(vrd_data),
//     // A: audio_wav_top_path ring_mem_*
//     .a_wr_req(wav_ring_wr_req), .a_wr_addr(wav_ring_wr_addr), .a_wr_data(wav_ring_wr_data),
//     .a_wr_ready(wav_ring_wr_ready),
//     .a_rd_req(wav_ring_rd_req), .a_rd_addr(wav_ring_rd_addr),
//     .a_rd_valid(wav_ring_rd_valid), .a_rd_data(wav_ring_rd_data)
// );
// 
// sdram U3(
//     .Clk               (ext_mem_clk),
//     .Clk_sft           (ext_mem_clk_sft),
//     .Rst               (rst_all),
//     .Sdr_init_done     (Sdr_init_done),
//     .Sdr_init_ref_vld  (Sdr_init_ref_vld),
//     .Sdr_busy          (Sdr_busy),
//     .App_wr_en         (App_wr_en),
//     .App_wr_addr       (App_wr_addr),
//     .App_wr_dm         (App_wr_dm),
//     .App_wr_din        (App_wr_din),
//     .App_rd_en         (App_rd_en),
//     .App_rd_addr       (App_rd_addr),
//     .Sdr_rd_en         (Sdr_rd_en),
//     .Sdr_rd_dout       (Sdr_rd_dout)
// );
// 
// // ===================== 音频：内部 I2S 测试音 =====================
// hdmi_audio_tone_i2s_64fs #(
//     .PHASE_INC (32'd39370534),
//     .AMP       (24'sd2000000)
// ) u_hdmi_audio_tone_i2s_64fs (
//     .I_mclk      (audio_mclk),
//     .I_rst       (rst_all),
//     .O_i2s_BCLK  (audio_i2s_bclk),
//     .O_i2s_LRCK  (audio_i2s_lrck),
//     .O_i2s_DOUT  (audio_i2s_dout)
// );
// 
// I2S_receiver u_I2S_receiver(
//     .I_clk              (video_clk),
//     .I_rst              (rst_all),
//     .I_i2s_BCLK         (audio_i2s_bclk),
//     .I_i2s_LRCK         (audio_i2s_lrck),
//     .I_i2s_DOUT         (audio_i2s_dout),
//     .O_audio_valid      (tone_audio_valid),
//     .O_audio_left_data  (audio_left_data),
//     .O_audio_right_data (audio_right_data)
// );
// 
// // ===================== 开机音乐(ROM+DDS)播放器 + 音频 MUX (V12) =====================
// // player 为纯引擎(线性 62 音): I_start 由 startup_controller 在首帧+Settle 后置位(MUSIC), 播完 done。
// startup_music_player #(
//     .STARTUP_ENABLE(STARTUP_MUSIC_ENABLE),
//     .NOTE_NUM     (STARTUP_MUSIC_NOTE_NUM),
//     .AMP          (STARTUP_MUSIC_AMP),
//     .ROM_DEPTH    (STARTUP_MUSIC_ROM_DEPTH),
//     .ADDR_BITS    (7)
// ) u_startup_music_player (
//     .I_clk            (video_clk),
//     .I_rst            (player_rst),
//     .I_pll_locked     (audio_pll_lock),
//     .I_start          (au_music_start),    // V27S2: 由音频 FSM(A_MUSIC)启动 one-shot
//     .I_sample_enable  (audio_valid),
//     .O_startup_active (startup_active),
//     .O_startup_done   (startup_done),
//     .O_left           (startup_left),
//     .O_right          (startup_right)
// );
// 
// // ===================== V26 Step8 Phase-B: 最终 WAV 链正式接入(audio_wav_top_path) =====================
// //   只接线不重设计: 内部 u_ctl(audio_startup_wav_ctl)->audio_mux3 四源互斥
// //   (sel 0 startup / 1 wav / 2 testtone / 3 silence); WAV 48k 来自真实链(video_clk 域 audio_valid tick)。
// //   aud_mux_l/r = 模块 o_pcm(video 域统一 24bit) -> HDMI Audio 与 FFT 同一份最终 PCM。
// //   TestTone 仅 NORMAL 且 AUDIO_TEST_MODE=1 时被选中(sel2); 默认 0 静音。
// //   Startup/Tone 源 = 现有 startup_music_player / I2S_receiver 恢复音, 均不修改, 仅门控/接线。
// // AUDIO_TEST_MODE=0 时把 tone 输入常数化 => I2S 测试音链(hdmi_audio_tone_i2s_64fs /
// //   I2S_receiver)与旧 startup_controller 成为死逻辑被综合自动裁剪(约 320 个触发器), 缓解布线拥塞;
// //   =1 时行为不变(AUDIO_TEST_MODE=1 调试仍可用完整测试音链)。
// wire signed [23:0] wav_tone_l = ((AUDIO_TEST_MODE == 1) && test_tone_on) ? audio_left_data  : 24'sd0;
// wire signed [23:0] wav_tone_r = ((AUDIO_TEST_MODE == 1) && test_tone_on) ? audio_right_data : 24'sd0;
// wire [15:0] wav_catalog_count, wav_catalog_index;
// wire wav_catalog_valid;
// audio_panel_control #(.ENUMERATE_WAV(1),.PAUSE_KEY_ENABLE(1),.AUTO_NEXT_ENABLE(1),.DEBOUNCE_CYCLES(125000)) u_audio_panel(
//  .clk(video_clk),.rst(rst_all),.mode_audio(audio_mode_sw),
//  .key_next_n(key1),.key_prev_n(key2),.key_pause_n(audio_key3_n),
//  .play_finished(wav_play_finished && !wav_err_c),.session_busy(session_resetting),
//  .external_play(uart_play),.current_name(current_wav_name_c),
//  .catalog_valid(wav_catalog_valid),.catalog_count(wav_catalog_count),.catalog_index(wav_catalog_index),
//  .pause_cmd(panel_pause_cmd),.image_pause_n(image_key3_n),
//  .image_next_n(panel_image_next_n),.image_auto_n(panel_image_auto_n),
//  .play_cmd(panel_play),.target_name(panel_wav_name)
// );
// team2_uart_control #(.SUFFIX_ONLY(1),.INITIAL_NAME(88'd0),.CLK_FREQ_HZ(25000000),.BAUD(115200),.PANEL_ENABLE(1)) u_uart_select(
//  .clk(video_clk),.rst(rst_all),.uart_rx(uart_rx),.uart_tx(uart_tx),
//  .panel_play(panel_play),.panel_name(panel_wav_name),
//  .light_debug({light_averages,light_rejected,light_frames,light_raw,light_filtered,light_samples,light_gain,light_level,light_ready,light_fault}),
//  .cmd_next(),.cmd_auto(),.cmd_play(uart_play),.cmd_clear(),
//  .cmd_wav_rescan(uart_select),.wav_target_name(selected_wav_name),
//  .state_code({1'b0,wav_ctl_phase}),.bmp_count(3'd0),
//  .wav_found(wav_found_c),.wav_playing(wav_wav_play),
//  .fifo_underflow(1'b0),.fifo_overflow(1'b0),.fifo_level(10'd0),
//  .error_code(wav_err_c ? 8'h01 : 8'h00)
// );
// audio_uart_session #(.INITIAL_NAME(88'd0)) u_uart_session(
//  .clk_a(sd_card_clk),.clk_b(ext_mem_clk),.clk_c(video_clk),.rst(rst_all),
//  .play_cmd(uart_play | panel_play | panel_start),
//  .selected_name(panel_play ? panel_wav_name : selected_wav_name),
//  .sd_idle(sd_audio_idle),.mem_idle(mem_audio_idle),
//  .active_name(active_wav_name),.manual(manual_session),.busy(session_resetting),
//  .current_name_c(current_wav_name_c),
//  .rst_a(audio_session_rst_a),.rst_b(audio_session_rst_b),.rst_c(audio_session_rst_c)
// );
// audio_wav_top_path #(
//     .DYNAMIC_NAME(2), .PAUSE_ENABLE(1),
//     .AUDIO_FIFO_DEPTH(64),
//     .RING_FRAMES    (8192),
//     .RING_AW        (13),
//     .RING_LOW       (2048),
//     .RING_HIGH      (6144),
//     .AUDIO_TEST_MODE(AUDIO_TEST_MODE)
// ) u_audio_wav_top_path (
//     .clk_a(sd_card_clk), .clk_b(ext_mem_clk), .clk_c(video_clk),
//     .i_target_name(active_wav_name), .i_pause(audio_paused),
//     .rst_a(rst_all | audio_session_rst_a | (wav_source_net ? net_app_reset : 1'b0)),
//     .rst_b(rst_all | audio_session_rst_b), .rst_c(rst_all | audio_session_rst_c),
//     .rst_soft_a(soft_sd_run), .rst_soft_b(soft_mem_run), .rst_soft_c(soft_video_run),
//     .source_sel(wav_source_net),
//     .net_file_byte(net_file_byte), .net_file_valid(net_file_valid),
//     .net_file_end(net_file_end), .net_file_size(net_file_size),
//     .net_file_ready(net_file_ready), .net_file_error(net_file_error),
//     // A: SD m1(唯一 sd_card_top + sd_read_arbiter.m1)
//     .sd_m1_req(wav_sd_req), .sd_m1_addr(wav_sd_addr),
//     .sd_m1_data(wav_sd_data), .sd_m1_valid(wav_sd_valid), .sd_m1_end(wav_sd_end),
//     // B: Ring mem -> audio_sdram_access.a_* -> 共享 sdram U3
//     .ring_mem_wr_req(wav_ring_wr_req), .ring_mem_wr_addr(wav_ring_wr_addr),
//     .ring_mem_wr_data(wav_ring_wr_data), .ring_mem_wr_ready(wav_ring_wr_ready),
//     .ring_mem_rd_req(wav_ring_rd_req), .ring_mem_rd_addr(wav_ring_rd_addr),
//     .ring_mem_rd_data(wav_ring_rd_data), .ring_mem_rd_valid(wav_ring_rd_valid),
//     // C: video 域控制(真实源)
//     .i_audio_valid(audio_valid), .i_first_frame_ready(first_frame_ready),
//     .i_startup_done(startup_done | manual_session),
//     .i_sd_init_done(sd_init_done_v),   // V27S2: 音频启动唯一基准 = SD init done
//     .i_wav_start(1'b0),                // V27S2: 解除 BMP->音频 触发(现由内部 A_SEARCH 触发)
//     .i_no_bmp(1'b0),                   // V27S2: 音频不再依赖 BMP 判定
//     .i_startup_l(startup_left), .i_startup_r(startup_right),
//     .i_tone_l(wav_tone_l), .i_tone_r(wav_tone_r),
//     // C: 统一 24bit PCM + WAV 帧(观测)
//     .o_pcm_valid(wav_pcm_v), .o_pcm_left(wav_pcm_l), .o_pcm_right(wav_pcm_r),
//     .o_wav_valid(), .o_wav_left(), .o_wav_right(),
//     // C: MUX/状态/水位(观测)
//     .o_audio_sel(wav_audio_sel), .o_wav_play(wav_wav_play),
//     .o_unlock_image(wav_unlock_image), .o_ctl_phase(wav_ctl_phase),
//     .o_music_start(au_music_start), .o_wav_search(au_wav_search),   // V27S2
//     .o_audio_digit(au_digit),                                       // V27S2
//     .o_wav_count_c(wav_catalog_count), .o_wav_index_c(wav_catalog_index), .o_catalog_valid_c(wav_catalog_valid),
//     .o_wav_ready(wav_ready_c), .o_wav_found_c(wav_found_c),
//     .o_wav_err_c(wav_err_c), .o_wav_done_c(wav_done_c), .o_play_finished(wav_play_finished),
//     .o_ring_used(wav_ring_used), .o_fifo1_full(wav_f1_full),
//     .o_fifo1_empty(wav_f1_empty), .o_fifo2_full(wav_f2_full),
//     .o_fifo2_empty(wav_f2_empty)
// );
// // HDMI Audio + FFT 共用同一份最终 PCM(audio_mux3 输出), 绝不另开第二条 FFT/分析链。
// // PIR 缺勤时强制 0(双保险: 此时音频 FSM 也已被软复位保持在静音态)。
// assign aud_mux_l = soft_video_run ? wav_pcm_l : 24'sd0;
// assign aud_mux_r = soft_video_run ? wav_pcm_r : 24'sd0;
// 
// // ===================== 音频可视化(扩展要求5)：FFT 分析链 =====================
// // 纯旁路复制 PCM，不阻塞原 HDMI 音频播放链路（audio_valid/left/right 不变）。
// // FFT 输入使用 MUX 后 PCM(开机=开机音, 正常=normal), 保持“PCM 链统一/FFT 仅一份”。
// wire [11:0] light_adc_data;
// wire light_adc_toggle;
// mic_spectrum_source u_mic_spectrum(
//  .clk_adc(mic_adc_clk),.clk_video(video_clk),.rst(rst_all),
//  .select_sw(spectrum_mic_sw),.selected(mic_selected),.pcm(mic_pcm),.valid(mic_valid),
//  .light_data(light_adc_data),.light_toggle(light_adc_toggle));
// // The packed compatibility path is retained for optional HUB75/test builds.
// localparam SPECTRUM_INDEXED = (HUB75_ENABLE == 0 && HUB75_TEST_MODE == 0 && SPECTRUM_BIN_NUM == 64);
// wire spectrum_band_valid;
// wire [5:0] spectrum_band_addr, spectrum_band_height;
// wire [5:0] spectrum_read_addr, spectrum_read_height;
// wire spectrum_freeze;
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
// 
// // ===================== Phase3: FFT 幅值 -> HUB75 频谱 =====================
// // 列=bin1..64, log 压缩(参考 STM32 logf(x+3)*8.1), 平滑动画, 1/16 扫描白色显示。
// // HUB75 逻辑信号编号已由用户固定(GPIOA0..A12 / J1-1..15)；package pin LOC 暂缓，
// // 不在本文件臆测封装脚。
// wire hub_upd;
// wire hub_r1, hub_g1, hub_b1, hub_r2, hub_g2, hub_b2;
// wire hub_a, hub_b, hub_c, hub_d, hub_clk, hub_lat, hub_oe;
// 
// // 测试图: 阶梯高度 0,1,..32,0..(col%33), 便于上板检查列/行/上下半区/地址
// genvar tg;
// generate
//     for (tg = 0; tg < SPECTRUM_BIN_NUM; tg = tg + 1) begin : gen_testh
//         assign test_h[tg*6 +: 6] = (tg % 33);
//     end
// endgenerate
// 
// assign target_h = (HUB75_TEST_MODE == 1) ? test_h : spectrum_heights;
// 
// // 测试模式下需周期产生“更新”以驱动平滑/控制器装载
// reg [19:0] hub_tcnt;
// reg        hub_tick;
// assign hub_upd = (HUB75_TEST_MODE == 1) ? hub_tick : spectrum_heights_valid;
// wire spectrum_upd;
// // V12 注: 频谱更新门控(spectrum_upd)现由 V27S3 Audio FSM 的 spectrum_on 控制
// //   (A_MUSIC/A_PLAY 才更新; FFT 从不 reset, 仅显示门控)
// assign spectrum_upd = spectrum_on ? hub_upd : 1'b0;
// always @(posedge video_clk or posedge rst_all) begin
//     if (rst_all) begin
//         hub_tcnt <= 20'd0;
//         hub_tick <= 1'b0;
//     end else if (hub_tcnt >= 20'd249_999) begin
//         hub_tcnt <= 20'd0;
//         hub_tick <= 1'b1;
//     end else begin
//         hub_tcnt <= hub_tcnt + 1'b1;
//         hub_tick <= 1'b0;
//     end
// end
// 
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
//     .I_read_addr(spectrum_read_addr), .O_read_height(),
//     .O_height  (cur_h)
// );
// 
// // Disabled for the suffix/D5 build; original implementation retained.
// // hub75_controller #(
// //     .HUB75_WIDTH      (HUB75_WIDTH),
// //     .HUB75_SCAN_ROWS  (HUB75_SCAN_ROWS),
// //     .HUB75_HEIGHT     (HUB75_HEIGHT),
// //     .HUB75_CLK_DIV    (HUB75_CLK_DIV),
// //     .HUB75_BRIGHTNESS (HUB75_BRIGHTNESS)
// // ) u_hub75_controller (
// //     .I_clk    (video_clk),
// //     .I_rst    (rst_all),
// //     .I_load   (spectrum_upd),
// //     .I_height (cur_h),
// //     .O_R1(hub_r1), .O_G1(hub_g1), .O_B1(hub_b1),
// //     .O_R2(hub_r2), .O_G2(hub_g2), .O_B2(hub_b2),
// //     .O_A(hub_a), .O_B(hub_b), .O_C(hub_c), .O_D(hub_d),
// //     .O_CLK(hub_clk), .O_LAT(hub_lat), .O_OE(hub_oe)
// // );
// 
// 
// assign {hub_r1,hub_g1,hub_b1,hub_r2,hub_g2,hub_b2,hub_a,hub_b,hub_c,hub_d,hub_clk,hub_lat}=12'd0;
// assign hub_oe=1'b1;
// // HUB75_ENABLE 门控; OE 电平按 HUB75_OE_ACTIVE_LEVEL 取反(参考 EN 低有效=0)
// assign HUB75_R1 = HUB75_ENABLE ? hub_r1 : 1'b0;
// assign HUB75_G1 = HUB75_ENABLE ? hub_g1 : 1'b0;
// assign HUB75_B1 = HUB75_ENABLE ? hub_b1 : 1'b0;
// assign HUB75_R2 = HUB75_ENABLE ? hub_r2 : 1'b0;
// assign HUB75_G2 = HUB75_ENABLE ? hub_g2 : 1'b0;
// assign HUB75_B2 = HUB75_ENABLE ? hub_b2 : 1'b0;
// assign HUB75_A  = HUB75_ENABLE ? hub_a  : 1'b0;
// assign HUB75_B  = HUB75_ENABLE ? hub_b  : 1'b0;
// assign HUB75_C  = HUB75_ENABLE ? hub_c  : 1'b0;
// assign HUB75_D  = HUB75_ENABLE ? hub_d  : 1'b0;
// assign HUB75_CLK = HUB75_ENABLE ? hub_clk : 1'b0;
// assign HUB75_LAT = HUB75_ENABLE ? hub_lat : 1'b0;
// assign HUB75_OE  = HUB75_ENABLE ?
//                    ((HUB75_OE_ACTIVE_LEVEL == 1) ? ~hub_oe : hub_oe) : 1'b1;
// 
// // ===================== Phase5: HDMI 64band 透明频谱叠加(右下角) =====================
// // 与 HUB75 共用 cur_h(同一份 FFT/64band/平滑); 无面板, 非柱像素 100% 透传原图。
// hdmi_spectrum_overlay #(
//     .INDEXED(SPECTRUM_INDEXED),
//     .SPECTRUM_BIN_NUM     (SPECTRUM_BIN_NUM),
//     .SPECTRUM_X_START     (SPECTRUM_X_START),
//     .SPECTRUM_X_END       (SPECTRUM_X_END),
//     .SPECTRUM_Y_TOP       (SPECTRUM_Y_TOP),
//     .SPECTRUM_Y_BOTTOM    (SPECTRUM_Y_BOTTOM),
//     .SPECTRUM_BAR_WIDTH   (2),
//     .SPECTRUM_BAR_GAP     (0),
//     .SPECTRUM_HEIGHT_SCALE(2),
//     .SPECTRUM_DEBUG_BYPASS(SPECTRUM_DEBUG_BYPASS),
//     .SPECTRUM_COLOR_MODE  (SPECTRUM_COLOR_MODE),
//     .SPECTRUM_FADE_ENABLE (SPECTRUM_FADE_ENABLE),
//     .SPECTRUM_FADE_PERCENT(SPECTRUM_FADE_PERCENT),
//     .SPECTRUM_FADE_LEVELS (SPECTRUM_FADE_LEVELS),
//     .SPECTRUM_SMOOTH_STEP (SPECTRUM_SMOOTH_STEP),
//     .SPECTRUM_COLOR0      (SPECTRUM_COLOR0),
//     .SPECTRUM_COLOR1      (SPECTRUM_COLOR1),
//     .SPECTRUM_COLOR2      (SPECTRUM_COLOR2),
//     .SPECTRUM_COLOR3      (SPECTRUM_COLOR3),
//     .SPECTRUM_COLOR4      (SPECTRUM_COLOR4),
//     .SPECTRUM_COLOR5      (SPECTRUM_COLOR5)
// ) u_hdmi_spectrum_overlay (
//     .I_clk      (video_clk),
//     .I_rst      (rst_all),
//     .I_vs       (vs),
//     .I_de       (de),
//     .I_rgb_bg   (vout_data),
//     .I_height   (cur_h),
//     .O_read_addr(spectrum_read_addr), .I_read_height(spectrum_read_height),
//     .O_freeze(spectrum_freeze),
//     .O_rgb      (hdmi_overlay_rgb)
// );
// 
// audio_arc_calculate #(
//     .ACR_N         (6144)
// ) u_audio_arc_calculate (
//     .I_clk         (video_clk),
//     .I_rst         (rst_all),
//     .I_audio_valid (audio_valid),
//     .O_acr_valid   (acr_valid),
//     .O_acr_cts     (acr_cts),
//     .O_acr_n       (acr_n)
// );
// 
// // ===================== RGB/DE 转 AXIS 视频 =====================
// // （Phase3：旧 spectrum_renderer HDMI 叠加实例已从数据路径移除，源码保留于
// //   spectrum_renderer.v 备查；FFT 输出改由后续 HUB75 链路消费。）
// ambient_brightness u_ambient_brightness(
//  .gain_applied(ambient_gain_applied_pid),.gain_target(ambient_gain_target_pid),.gain_tick(ambient_gain_tick_pid),
//  .clk(video_clk),.rst(rst_all),.presence_run(soft_video_run),.light_data(light_adc_data),.light_toggle(light_adc_toggle),
//  .vs(vs),.de(de),.rgb(final_rgb),.out_vs(light_vs),.out_de(light_de),.out_rgb(light_rgb),
//  .filtered_adc(light_filtered),.level(light_level),.gain(light_gain),.ready(light_ready),.fault(light_fault),
//  .raw_adc(light_raw),.sample_count(light_samples),.frame_count(light_frames),
//  .rejected_count(light_rejected),.average_count(light_averages));
// video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
//     .I_clk         (video_clk),
//     .I_rst         (rst_all),
//     .I_vs          (light_vs),
//     .I_de          (light_de),
//     .I_rgb         (light_rgb),
//     .O_video_user  (axis_s_user),
//     .O_video_valid (axis_s_valid),
//     .O_video_last  (axis_s_last),
//     .O_video_data  (axis_s_data)
// );
// 
// // 上电后自动打一拍，触发一次 EDID 读取
// startup_pulse #(
//     .CNT_MAX(20'd100000)
// ) u_startup_pulse (
//     .I_clk   (video_clk),
//     .I_rst   (rst_all),
//     .O_pulse (edid_trig)
// );
// 
// // ===================== 带音频的 HDMI 1.4b 发射 =====================
// hdmi_1_4b_transmitter_core_wrapper #(
//     .DEVICE                 ( "EG"       ),
//     .HTOTAL                 ( 800        ),
//     .HSA                    ( 96         ),
//     .HFP                    ( 16         ),
//     .HBP                    ( 48         ),
//     .HACTIVE                ( 640        ),
//     .VTOTAL                 ( 525        ),
//     .VSA                    ( 2          ),
//     .VFP                    ( 10         ),
//     .VBP                    ( 33         ),
//     .VACTIVE                ( 480        ),
//     .VIDEO_VIC              ( 1          ),
//     .VIDEO_TPG              ( "Disable"  ),
//     .VIDEO_FORMAT           ( "RGB"      ),
//     .AUDIO_SAMPLE_RATE      ( "48K"      ),
//     .IIC_SCL_DIV            ( 250        )
// ) u_hdmi_1_4b_transmitter_core_wrapper(
//     .I_pixel_clk        (video_clk),
//     .I_rst              (rst_all),
//     .I_edid_read_trig   (edid_trig),
//     .O_edid_read_valid  (edid_valid),
//     .O_edid_read_data   (edid_data),
// 
//     .I_axis_s_user      (axis_s_user),
//     .I_axis_s_valid     (axis_s_valid),
//     .I_axis_s_last      (axis_s_last),
//     .I_axis_s_data      (axis_s_data),
//     .O_axis_s_ready     (axis_s_ready),
// 
//     .I_audio_valid      (audio_valid),
//     .I_audio_left_data  (aud_mux_l),
//     .I_audio_right_data (aud_mux_r),
//     .I_acr_valid        (acr_valid),
//     .I_acr_cts          (acr_cts),
//     .I_acr_n            (acr_n),
// 
//     .O_video_locked     (),
//     .O_ddc_scl          (HDMI_DDC_SCL),
//     .IO_ddc_sda         (HDMI_DDC_SDA),
// 
//     .O_ch0_tmds_data    (tmds_ch0_data),
//     .O_ch1_tmds_data    (tmds_ch1_data),
//     .O_ch2_tmds_data    (tmds_ch2_data),
//     .O_clk_tmds_data    (tmds_clk_data)
// );
// 
// hdmi_phy_wrapper #(
//     .DEVICE ( "EG" )
// ) u_hdmi2phy_wrapper(
//     .I_pixel_clk        (video_clk),
//     .I_serial_clk       (hdmi_5x_clk),
//     .I_rst              (rst_all),
//     .I_tmds_channel_0   (tmds_ch0_data),
//     .I_tmds_channel_1   (tmds_ch1_data),
//     .I_tmds_channel_2   (tmds_ch2_data),
//     .I_tmds_channel_clk (tmds_clk_data),
//     .O_tmds_ch0_p       (HDMI_D0_P),
//     .O_tmds_ch1_p       (HDMI_D1_P),
//     .O_tmds_ch2_p       (HDMI_D2_P),
//     .O_tmds_clk_p       (HDMI_CLK_P)
// );
// 
// Active BOARD_B wiring
wire link_requested_mic;
wire link_online,link_mode_mic,link_source_change,link_pcm_valid,link_boot_done;
wire[23:0] link_pcm;

parameter MEM_DATA_BITS = 32;
parameter integer SW3_DEBOUNCE_CYCLES = 500000;
parameter [31:0] ETH_LOCAL_IP = 32'hc0a8f001;
parameter [47:0] ETH_LOCAL_MAC = 48'h020000000001;
parameter [31:0] ETH_PC_IP = 32'hc0a8f002;
parameter [15:0] ETH_LOCAL_PORT = 16'd1;
parameter [15:0] ETH_PC_PORT = 16'd50000;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter [20:0] FRAME_PIXELS  = 21'd307200;
parameter [20:0] BUF0_ADDR     = 21'd0;
parameter [20:0] BUF1_ADDR     = FRAME_PIXELS;
parameter [20:0] BUF2_ADDR     = FRAME_PIXELS * 2;
parameter [20:0] BUF3_ADDR     = FRAME_PIXELS * 3;
parameter FFT_N                = 1024;
parameter FFT_LOG2_N           = 10;
parameter FFT_INPUT_BITS       = 16;
parameter FFT_INTERNAL_BITS    = 26;
parameter FFT_COEF_BITS        = 16;
parameter FFT_SAMPLE_RATE      = 48000;
parameter FFT_HOP_SIZE         = 256;
parameter SPECTRUM_BIN_NUM     = 64;
parameter SPECTRUM_BAND_SHIFT  = 3;
parameter SPECTRUM_SCALE_SHIFT = 15;
parameter SPECTRUM_MAX_BAR     = 71;
parameter SPECTRUM_X_START     = 368;
parameter SPECTRUM_X_END       = 623;
parameter SPECTRUM_Y_TOP       = 16;
parameter SPECTRUM_Y_BOTTOM    = 111;
parameter BAR_WIDTH            = 3;
parameter BAR_GAP              = 2;
parameter SMOOTH_STEP          = 1;
parameter SPECTRUM_ENABLE        = 1;
parameter SPECTRUM_DEBUG_BYPASS  = 0;
parameter HUB75_ENABLE           = 1;
parameter HUB75_TEST_MODE        = 0;
parameter HUB75_SCAN_ROWS        = 16;
parameter HUB75_WIDTH            = 64;
parameter HUB75_HEIGHT           = 32;
parameter HUB75_CLK_DIV          = 4;
parameter HUB75_OE_ACTIVE_LEVEL  = 0;
parameter HUB75_SMOOTH_STEP      = 1;
parameter HUB75_BRIGHTNESS       = 15;
parameter HUB75_LAT_CYCLES       = 8;
parameter HUB75_LAT_OE_CYCLES    = 14;
parameter HUB75_OE_DEAD_CYCLES   = 14;
// Production defaults to live FFT. Diagnostic logic is removed at elaboration.
parameter HUB75_TEST_ENABLE     = 0;
parameter HUB75_TEST_ROW_ONLY   = 1;
parameter [3:0] HUB75_TEST_PATTERN = 4'd1;
parameter SPECTRUM_LOG_SHIFT    = 16;
parameter SPECTRUM_HDMI_ENABLE  = 1;
parameter SPECTRUM_COLOR_MODE    = 1;
parameter SPECTRUM_FADE_ENABLE   = 1;
parameter SPECTRUM_FADE_PERCENT  = 20;
parameter SPECTRUM_FADE_LEVELS   = 4;
parameter SPECTRUM_SMOOTH_STEP   = 1;
parameter [23:0] SPECTRUM_COLOR0 = 24'h0080FF;
parameter [23:0] SPECTRUM_COLOR1 = 24'h00FFFF;
parameter [23:0] SPECTRUM_COLOR2 = 24'h00FF40;
parameter [23:0] SPECTRUM_COLOR3 = 24'hFFFF00;
parameter [23:0] SPECTRUM_COLOR4 = 24'hFF8000;
parameter [23:0] SPECTRUM_COLOR5 = 24'hFF2020;
parameter STARTUP_MUSIC_ENABLE        = 1;
parameter STARTUP_MUSIC_BPM           = 120;
parameter signed [23:0] STARTUP_MUSIC_AMP = 24'sd2000000;
parameter STARTUP_MUSIC_ROM_DEPTH     = 128;
parameter STARTUP_MUSIC_NOTE_NUM      = 62;
parameter STARTUP_WAIT_FOR_PLL_LOCK   = 1;
parameter STARTUP_AUDIO_SETTLE_MS     = 300;
parameter STARTUP_DEBUG_DISPLAY       = 1;
parameter [20:0] AUDIO_BASE_ADDR = 21'd1703936;
parameter        AUDIO_TEST_MODE      = 0;
parameter integer PIR_PRESENCE_QUALIFY_MS = 20;
reg  middle_rst, middle_rst_sync;
// startup_done comes from video_clk. Arm once after the actual boot song ends;
// subsequent PIR/session resets must not restart this startup exemption.
reg [1:0] pir_startup_done_sync;
reg pir_armed;
// The tuner/link additions change placement. Sampling the 25 MHz boot flag
// on the falling 50 MHz edge supplies a real half-cycle hold margin rather
// than relying on a short same-edge route or adding a timing exception.
always @(negedge clk or negedge rst_n) begin
    if (!rst_n) pir_startup_done_sync<=2'b00;
    else pir_startup_done_sync<={pir_startup_done_sync[0],(!link_online || link_boot_done)};
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pir_armed <= 1'b0;
    end else begin
        pir_armed <= pir_startup_done_sync[1]; // A boot exemption also clears the absence timer below.
    end
end
reg  [23:0] pir_high_cnt;      // 连续高计数(尖峰抑制; 饱和保持)
reg  [31:0] pir_low_count;     // 连续低计数(缺勤计时)
reg  rst_soft;                 // 1=在场/运行; 0=缺勤/软复位
localparam [31:0] PIR_LOW_TIMEOUT_LAST = 32'd2_999_999_999;   // 60s @50MHz
localparam [23:0] PIR_PRESENCE_QUALIFY_CYCLES = PIR_PRESENCE_QUALIFY_MS * 50_000;
wire pir_high_qualified = (pir_high_cnt == PIR_PRESENCE_QUALIFY_CYCLES);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        middle_rst      <= 1'b0;
        middle_rst_sync <= 1'b0;
        pir_high_cnt    <= 24'd0;
        pir_low_count   <= 32'd0;
        rst_soft        <= 1'b1; // Global reset still applies independently.
    end else begin
        middle_rst      <= pir_out;
        middle_rst_sync <= middle_rst;

        if (!pir_armed) begin
            // Do not mute boot audio or accumulate absence during startup.
            pir_high_cnt  <= 24'd0;
            pir_low_count <= 32'd0;
            rst_soft      <= 1'b1;
        end else begin
        if (middle_rst_sync == 1'b1) begin
            if (pir_high_cnt != PIR_PRESENCE_QUALIFY_CYCLES)
                pir_high_cnt <= pir_high_cnt + 24'd1;
        end else begin
            pir_high_cnt <= 24'd0;
        end

        if (pir_high_qualified) begin          // 在场: 运行, 清零缺勤计时
            pir_low_count <= 32'd0;
            rst_soft      <= 1'b1;
        end else if (middle_rst_sync == 1'b1) begin
            // 未达标短高脉冲进行中: 保持现状(不清计时、不倒计时)
        end else if (rst_soft == 1'b1) begin   // 连续低: 60s 到 -> 软复位
            if (pir_low_count == PIR_LOW_TIMEOUT_LAST) begin
                pir_low_count <= PIR_LOW_TIMEOUT_LAST;
                rst_soft      <= 1'b0;
            end else begin
                pir_low_count <= pir_low_count + 32'd1;
            end
        end else begin
            pir_low_count <= 32'd0;
            rst_soft      <= 1'b0;
        end
        end
    end
end



// BOARD_B: minimal wiring around the unchanged acquisition/algorithm modules.
wire video_clk,hdmi_5x_clk,mic_adc_clk,audio_mclk,audio_pll_lock;
wire rst_all=~rst_n | ~audio_pll_lock;
// Proven FM I2C initialization retained; control arrives from A LCD.
parameter FM_SCAN_ENABLE=1;
parameter NET_VIDEO_ENABLE=1;
parameter FM_TIMESHIFT_ENABLE = 1; // Independent compile-time bypass.
parameter AUDIO_DSP_ENABLE=1;
wire dsp_command,dsp_command_ready,dsp_command_done;wire [15:0] dsp_command_value;wire [3:0] dsp_command_error;
wire [31:0] link_stereo_pcm;wire dsp_link_capable,dsp_link_valid;
wire dsp_enabled,dsp_agc,dsp_comp,dsp_lim,dsp_gate,dsp_post;
wire [19:0] dsp_gains;wire [1:0] dsp_threshold,dsp_ratio,dsp_speed,dsp_target;
wire [63:0] dsp_control_status,dsp_engine_status;wire [127:0] dsp_status={dsp_control_status,dsp_engine_status};
wire dsp_ready,dsp_out_valid;wire signed [23:0] dsp_out_left,dsp_out_right;wire [9:0] dsp_max_cycles;

parameter FM_ENABLE=1,FM_INTEGRATION_ENABLE=1,FM_ADC_ENABLE=1;
parameter FM_DEFAULT_FREQUENCY=968,FM_DEFAULT_VOLUME=15,FM_INIT_TABLE=0;
parameter FM_PCM_GAIN_SHIFT=3; // default x8 (+6 dB vs x4); FM PCM only, smoothed and saturated

wire [3:0] video_acked_groups;


wire chip_fm_valid,chip_fm_ready;
wire [2:0] chip_fm_op;
wire [15:0] chip_fm_value;
wire [79:0] fm_debug;
wire [2:0] fm_gain_shift;wire rf_quiet_toggle;

wire [127:0] fm_system_status;
wire [3:0] fm_error;
wire fm_powered,fm_initialized,fm_tuned,fm_comm;
wire link_fm_valid,link_fm_ready;
wire [7:0] link_fm_seq;wire [2:0] link_fm_op;wire [15:0] link_fm_value;
wire [1:0] managed_audio_source,managed_fft_source;
wire mic_monitor;
wire [23:0] fm_pcm,fm_raw_pcm;wire fm_sample_valid,fm_raw_valid,fm_adc_fault,fm_adc_clipped,fm_gain_clipped;
fm_pcm_gain u_fm_gain(.clk(video_clk),.rst(rst_all),.in_pcm(fm_raw_pcm),.in_valid(fm_raw_valid),
 .gain_shift(fm_gain_shift),.out_pcm(fm_pcm),.out_valid(fm_sample_valid),.clipped(fm_gain_clipped));
reg [1:0] adc_fault_sync,adc_clip_sync;
always @(negedge video_clk or posedge rst_all)
 if(rst_all)begin adc_fault_sync<=0;adc_clip_sync<=0;end
 else begin adc_fault_sync<={adc_fault_sync[0],fm_adc_fault};adc_clip_sync<={adc_clip_sync[0],fm_adc_clipped};end
wire fm_scan_busy,fm_scan_valid,fm_scan_ready,fm_scan_done;
wire [15:0] fm_scan_value,scan_read_data;
wire [7:0] scan_read_index;
wire [127:0] fm_scan_status;wire [3:0] fm_scan_error;
wire scan_chip_valid;wire [2:0] scan_chip_op;wire [15:0] scan_chip_value;
generate if(FM_SCAN_ENABLE && FM_ENABLE)begin:scan_enabled
 fm_band_scanner scanner(.clk(video_clk),.rst(rst_all),.command_valid(fm_scan_valid),.command_ready(fm_scan_ready),
 .command_value(fm_scan_value),.command_done(fm_scan_done),.command_error(fm_scan_error),
 .chip_status(fm_debug),.chip_ready(chip_fm_ready),.chip_error(fm_error),.chip_valid(scan_chip_valid),.chip_op(scan_chip_op),.chip_value(scan_chip_value),
 .busy(fm_scan_busy),.status(fm_scan_status),.read_index(scan_read_index),.read_data(scan_read_data));
end else begin
 assign fm_scan_busy=0;assign fm_scan_ready=1;assign fm_scan_done=fm_scan_valid;assign fm_scan_error=7;
 assign fm_scan_status=0;assign scan_read_data=0;assign scan_chip_valid=0;assign scan_chip_op=0;assign scan_chip_value=0;
end endgenerate
wire adc_bridge_overflow;
fm_system_control #(.FM_ENABLE(FM_ENABLE),.SCAN_ENABLE(FM_SCAN_ENABLE),.FM_GAIN_DEFAULT(FM_PCM_GAIN_SHIFT),.DSP_ENABLE(AUDIO_DSP_ENABLE)) u_fm_system(
 .clk(video_clk),.rst(rst_all),.legacy_net(net_selected),.legacy_mic(link_requested_mic),
 .link_valid(link_fm_valid),.link_ready(link_fm_ready),.link_op(link_fm_op),.link_value(link_fm_value),.link_seq(link_fm_seq),
 .uart_valid(1'b0),.uart_ready(),.uart_op(3'd0),.uart_value(16'd0),
 .chip_valid(chip_fm_valid),.chip_ready(chip_fm_ready),.chip_op(chip_fm_op),.chip_value(chip_fm_value),.chip_error(fm_error),
 .chip_status(fm_debug),.fm_powered(fm_powered),.adc_fault(adc_fault_sync[1]),.adc_clipped(adc_clip_sync[1] || fm_gain_clipped),.overflow(adc_bridge_overflow),
 .audio_source(managed_audio_source),.fft_source(managed_fft_source),.mic_monitor(mic_monitor),.status(fm_system_status),.busy(),.scan_busy(fm_scan_busy),.scan_valid(fm_scan_valid),.scan_value(fm_scan_value),
 .scan_ready(fm_scan_ready),.scan_done(fm_scan_done),.scan_error(fm_scan_error),.dsp_valid(dsp_command),.dsp_value(dsp_command_value),.dsp_ready(dsp_command_ready),.dsp_done(dsp_command_done),.dsp_error(dsp_command_error),.fm_gain_shift(fm_gain_shift),.rf_quiet_toggle(rf_quiet_toggle));
generate if(FM_ENABLE)begin:fm_enabled
 rda5807m_ctrl #(.CLK_HZ(25000000),.DEFAULT_FREQUENCY(FM_DEFAULT_FREQUENCY),
  .DEFAULT_VOLUME(FM_DEFAULT_VOLUME),.INIT_TABLE(FM_INIT_TABLE)) u_fm(
  .clk(video_clk),.rst(rst_all),.cmd_valid(fm_scan_busy?scan_chip_valid:chip_fm_valid),.cmd_ready(chip_fm_ready),
  .cmd_op(fm_scan_busy?scan_chip_op:chip_fm_op),.cmd_value(fm_scan_busy?scan_chip_value:chip_fm_value),
  .busy(),.init_done(fm_initialized),.comm_ok(fm_comm),.tuned(fm_tuned),.powered(fm_powered),.error(fm_error),
  .frequency(),.volume(),.mute(),.status_a(),.status_b(),.debug_status(fm_debug),.fm_scl(fm_scl),.fm_sda(fm_sda));

end else begin:fm_disabled
 assign fm_sda=1'bz;assign fm_scl=1'bz;

 assign chip_fm_ready=1;assign fm_error=7;assign fm_debug=80'h07000000000000000000;
 assign fm_powered=0;assign fm_initialized=0;assign fm_tuned=0;assign fm_comm=0;
end endgenerate
wire mic_selected,mic_valid;wire [23:0] mic_pcm;
wire [11:0] light_adc_data;wire light_adc_toggle;
wire [11:0] light_filtered,light_raw;wire[1:0] light_level;
wire [4:0] light_gain;wire light_ready,light_fault;
wire [7:0] light_samples,light_rejected,light_averages;wire[6:0] light_frames;
wire [6:0] ambient_gain_target_pid,ambient_gain_applied_pid;wire ambient_gain_tick_pid;
wire [383:0] spectrum_heights,cur_h;wire spectrum_heights_valid,spectrum_busy;
wire spectrum_band_valid;wire[5:0] spectrum_band_addr,spectrum_band_height,spectrum_read_height;
wire[5:0] spectrum_read_addr=light_samples[5:0];
wire spectrum_freeze;
wire [5:0] spectrum_hub_addr,spectrum_hub_height,spectrum_link_addr,spectrum_link_height;
wire spectrum_commit;
wire spectrum_upd=spectrum_heights_valid;
wire[383:0] target_h=spectrum_heights;
localparam SPECTRUM_INDEXED=0; // Restore the original packed HUB75 smoothing path.
wire vs;
reg [1:0] soft_video_ps;
reg soft_video_run;
always @(posedge video_clk or negedge rst_n)
 if(!rst_n) soft_video_ps<=0;else soft_video_ps<={soft_video_ps[0],rst_soft};
always @(negedge video_clk or negedge rst_n)
 if(!rst_n)soft_video_run<=0;else soft_video_run<=soft_video_ps[0];
// Existing timing generator retained ONLY as the ambient watchdog/PID cadence.
d5_video_timing_data u_sensor_frame_tick(.video_clk(video_clk),.rst(rst_all),
 .read_req(),.read_req_ack(1'b0),.hs(),.vs(vs),.de());
brightness_pid_pair u_brightness_pid(.clk(video_clk),.rst(rst_all),
 .tick_manual(1'b0),.tick_gain(ambient_gain_tick_pid),.target_manual(8'sd0),
 .target_gain(ambient_gain_target_pid),.value_manual(),.value_gain(ambient_gain_applied_pid));
ambient_brightness u_ambient_brightness(
 .gain_applied(ambient_gain_applied_pid),.gain_target(ambient_gain_target_pid),.gain_tick(ambient_gain_tick_pid),
 .clk(video_clk),.rst(rst_all),.presence_run(soft_video_run),.light_data(light_adc_data),.light_toggle(light_adc_toggle),
 .vs(vs),.de(1'b0),.rgb(24'd0),.out_vs(),.out_de(),.out_rgb(),
 .filtered_adc(light_filtered),.level(light_level),.gain(light_gain),.ready(light_ready),.fault(light_fault),
 .raw_adc(light_raw),.sample_count(light_samples),.frame_count(light_frames),
 .rejected_count(light_rejected),.average_count(light_averages));
// Ethernet stays enabled independently of the FFT display source switch.
wire [3:0] wav_debug_state,wav_debug_checks,wav_debug_tx;
wire [15:0] wav_debug_length,wav_debug_count;
wire phy_debug_valid;
wire [3:0] phy_debug_step,phy_debug_reason;
wire [15:0] phy_debug_read,phy_debug_expected;
wire net_reset, net_valid, net_ready, net_error, net_end, net_seen, net_wack;
wire [7:0] net_byte;
wire [31:0] net_size, net_offset;
wire net_pcm_valid,net_done;
wire signed [15:0] net_left,net_right;
wire [3:0] net_error_code,net_state;
wire net_queue_ready,net_queue_valid,net_take;
wire [31:0] net_queue_data;
wire [11:0] net_queue_level;
wire link_video_enable,link_video_play,av_active,av_error,av_ended,av_file_end;
wire [3:0] link_video_epoch,av_slots;reg [3:0] av_epoch_d;
wire [7:0] av_byte;wire av_valid,av_ready;
wire [31:0] av_size,av_pcm,av_queue_data;
wire [95:0] av_pixels;wire [2:0] av_flags;
wire av_pixels_valid,av_pixels_ready,av_pcm_valid,av_pcm_ready,av_queue_valid;
wire [11:0] av_queue_level;
reg av_active_d;
always @(posedge video_clk or posedge rst_all)
 if(rst_all)begin av_active_d<=0;av_epoch_d<=0;end
 else begin av_active_d<=av_active;av_epoch_d<=link_video_epoch;end
wire av_source_change=av_active!=av_active_d;
wire av_reset=rst_all || net_reset || !link_video_enable || av_epoch_d!=link_video_epoch;
generate if(NET_VIDEO_ENABLE)begin:net_video
 media25_demux demux(.clk(video_clk),.rst(av_reset),.data(av_byte),.valid(av_valid),
  .ready(av_ready),.file_size(av_size),.error(av_error),.active(av_active),
  .pixels(av_pixels),.pixels_valid(av_pixels_valid),.pixels_ready(av_pixels_ready),.pixels_flags(av_flags),
  .pcm(av_pcm),.pcm_valid(av_pcm_valid),.pcm_ready(av_pcm_ready),.ended(av_ended),
  .slot_count(av_slots),.idle_ne(),.diag(),.diag_error(),.diag_complete());
 pcm_elastic_fifo audio(.clk(video_clk),.rst(av_reset),.in_data(av_pcm),.in_valid(av_pcm_valid),
  .in_ready(av_pcm_ready),.out_data(av_queue_data),.out_valid(av_queue_valid),
  .out_ready(net_queue_ready && av_active),.level(av_queue_level));
end else begin:no_net_video
 assign av_ready=0;assign av_error=0;assign av_active=0;assign av_ended=0;
 assign av_pixels=0;assign av_flags=0;assign av_pixels_valid=0;
 assign av_pcm=0;assign av_pcm_valid=0;assign av_pcm_ready=0;
 assign av_queue_data=0;assign av_queue_valid=0;assign av_queue_level=0;
end endgenerate
board_pcm_queue u_net_queue(.clk(video_clk),.rst(rst_all | net_reset | av_source_change),
 .in_data(av_active ? av_queue_data : {net_left,net_right}),
 .in_valid(av_active ? av_queue_valid : net_pcm_valid),.in_ready(net_queue_ready),
 .stream_end(av_active ? av_ended : net_end),.data(net_queue_data),.valid(net_queue_valid),.take(net_take),.level(net_queue_level));
fpga_wav_top #(.NET_VIDEO_ENABLE(NET_VIDEO_ENABLE),.APP_CLK_HZ(25000000),.AUTO_SPEED(1),.SPEED(2'b10),
 .LOCAL_IP(ETH_LOCAL_IP),.LOCAL_MAC(ETH_LOCAL_MAC),.LOCAL_PORT(ETH_LOCAL_PORT),
 .PC_IP(ETH_PC_IP),.PC_PORT(ETH_PC_PORT)) u_fpga_wav(
 .ref_clk_50(clk),.app_clk(video_clk),.rst_n(rst_n && !rst_all),.file_source_enable(1'b1),
 .video_source_enable(NET_VIDEO_ENABLE && link_video_enable),.video_byte(av_byte),
 .video_valid(av_valid),.video_ready(av_ready),.video_error(av_error),.video_size(av_size),
 .video_end(av_file_end),.video_active(),.video_slots(av_slots),
 .video_obs_payload_byte(),.video_obs_commit_byte(),.video_obs_drop_pkt(),.video_obs_commit_pkt(),
 .rgmii_rxc(eth_rxc),.rgmii_rx_ctl(eth_rx_ctl),.rgmii_rxd(eth_rxd),
 .rgmii_txc(eth_txc),.rgmii_tx_ctl(eth_tx_ctl),.rgmii_txd(eth_txd),
 .phy_mdc(eth_mdc),.phy_mdio(eth_mdio),
 .file_byte(net_byte),.file_valid(net_valid),.file_ready(net_ready),.file_error(net_error),
 .file_size(net_size),.file_end(net_end),.next_offset(net_offset),
 .debug_rx_seen(net_seen),.debug_wack_sent(net_wack),.app_reset(net_reset),
 .wav_debug_state(wav_debug_state),.wav_debug_checks(wav_debug_checks),
 .wav_debug_tx(wav_debug_tx),.wav_debug_length(wav_debug_length),.wav_debug_count(wav_debug_count),
 .phy_debug_valid(phy_debug_valid),.phy_debug_step(phy_debug_step),
 .phy_debug_reason(phy_debug_reason),.phy_debug_read(phy_debug_read),.phy_debug_expected(phy_debug_expected),
 .phy_ready(eth_phy_ready),.phy_fault(eth_phy_fault),.link_up(eth_link_up),
 .full_duplex(eth_full_duplex),.link_speed(eth_link_speed),
 .rx_overflow(eth_rx_overflow),.tx_overflow(eth_tx_overflow),.ip_error(eth_ip_error),.arp_error(eth_arp_error));
wav_stream_pcm_pipeline u_net_pcm(
 .clk(video_clk),.rst(rst_all | net_reset),.I_start(1'b1),.I_file_size(net_size),
 .I_byte(net_byte),.I_byte_valid(net_valid),.I_stream_end(net_end),.O_byte_ready(net_ready),
 .I_sample_req(net_queue_ready && !av_active),.pcm_valid(net_pcm_valid),.pcm_left(net_left),.pcm_right(net_right),
 .wav_done(net_done),.O_error(net_error),.O_error_code(net_error_code),.O_file_found(),.O_state(net_state));
// SW3=0 preserves A/B link and microphone selection; SW3=1 selects PC WAV.
reg [1:0] net_select_sync;
reg [17:0] net_select_count;
reg net_selected,net_selected_d;
always @(posedge video_clk or posedge rst_all) begin
 if(rst_all) begin net_select_sync<=0;net_select_count<=0;net_selected<=0;net_selected_d<=0;end
 else begin
  net_select_sync<={net_select_sync[0],sw3};net_selected_d<=net_selected;
  if(net_select_sync[1]==net_selected) net_select_count<=0;
  else if(net_select_count==249999) begin net_selected<=net_select_sync[1];net_select_count<=0;end
  else net_select_count<=net_select_count+1'b1;
 end
end
wire fft_source_change;
reg [1:0] previous_fft_source,previous_audio_source;
reg previous_fm_usable,fft_clear_d;
wire fm_usable=!fm_scan_busy && FM_ENABLE && FM_ADC_ENABLE && fm_powered && fm_initialized && fm_tuned && fm_comm && !adc_fault_sync[1];
always @(posedge video_clk or posedge rst_all)
 if(rst_all)begin previous_fft_source<=0;previous_audio_source<=0;previous_fm_usable<=0;fft_clear_d<=0;end
 else begin previous_fft_source<=managed_fft_source;previous_audio_source<=managed_audio_source;previous_fm_usable<=fm_usable;fft_clear_d<=fft_source_change;end
wire adc_bridge_reset=rst_all || !link_online || managed_audio_source!=previous_audio_source;
wire [23:0] adc_bridge_input=managed_audio_source==2 ? (fm_usable && !fm_debug[40] ? fm_pcm : 24'd0) : mic_pcm;
wire adc_bridge_valid=managed_audio_source==2 ? fm_sample_valid : mic_valid;
wire [15:0] adc_return_pcm;wire adc_return_ready;
pcm_rate_bridge u_adc_bridge(.clk(video_clk),.rst(adc_bridge_reset),.pcm(adc_bridge_input),
 .sample_valid(adc_bridge_valid && (managed_audio_source==1 || managed_audio_source==2)),
 .request(link_pcm_valid && (managed_audio_source==1 || managed_audio_source==2)),
 .out_pcm(adc_return_pcm),.ready(adc_return_ready),.overflow(adc_bridge_overflow));
wire remote_pcm_take;
wire net_audio_play=!av_active || link_video_play;
assign net_take=net_audio_play && (managed_audio_source==3 ? (AUDIO_DSP_ENABLE && dsp_link_capable ? link_pcm_valid && net_queue_valid : remote_pcm_take) : managed_fft_source==3 && link_pcm_valid && net_queue_valid);
wire dsp_active=AUDIO_DSP_ENABLE && dsp_link_capable && dsp_engine_status[63] && !dsp_engine_status[61];
wire time_capable,history_mode;wire [1:0] play_epoch;
reg previous_history;reg [1:0] previous_play_epoch;
always @(posedge video_clk or posedge rst_all)
 if(rst_all)begin previous_history<=0;previous_play_epoch<=0;end
 else begin previous_history<=history_mode;previous_play_epoch<=play_epoch;end
wire time_flush=managed_audio_source==2 && (history_mode!=previous_history || play_epoch!=previous_play_epoch);
wire history_selected=time_capable && history_mode && managed_audio_source==2;
wire [15:0] fm_selected_pcm=history_selected ? link_stereo_pcm[31:16] : adc_return_pcm;
wire fm_selected_valid=history_selected ? dsp_link_valid && fm_usable : adc_return_ready && fm_usable;
wire dsp_input_available=managed_audio_source==0 ? dsp_link_valid : managed_audio_source==3 ? net_queue_valid && net_audio_play : managed_audio_source==2 ? fm_selected_valid : adc_return_ready;
wire signed [23:0] dsp_input_left=!dsp_input_available ? 24'd0 : managed_audio_source==0 ? {link_stereo_pcm[31:16],8'd0} : managed_audio_source==3 ? {net_queue_data[31:16],8'd0} : {fm_selected_pcm,8'd0};
wire signed [23:0] dsp_input_right=!dsp_input_available ? 24'd0 : managed_audio_source==0 ? {link_stereo_pcm[15:0],8'd0} : managed_audio_source==3 ? {net_queue_data[15:0],8'd0} : {fm_selected_pcm,8'd0};
wire dsp_flush=time_flush || (managed_audio_source!=previous_audio_source) || av_source_change || !link_online || (dsp_command && dsp_command_value==16'h400e);
function [15:0] dsp_round16;
 input signed [23:0] value;reg signed [24:0] rounded;
 begin rounded=$signed(value)+25'sd128;
 if(rounded>25'sd8388352)dsp_round16=16'h7fff;
 else if(rounded< -25'sd8388608)dsp_round16=16'h8000;
 else dsp_round16=rounded[23:8];end
endfunction
wire [31:0] dsp_return;wire dsp_return_valid,dsp_pair_current;
audio_dsp_return_cache u_dsp_return(.clk(video_clk),.rst(rst_all),
 .in_accept(link_pcm_valid && dsp_ready && dsp_link_capable),.in_available(dsp_input_available),.flush(dsp_flush),
 .source(managed_audio_source),.source_live(link_online && (managed_audio_source!=2 || fm_usable)),
 .out_valid(dsp_out_valid),.left(dsp_out_left),.right(dsp_out_right),
 .pcm(dsp_return),.valid(dsp_return_valid),.pair_current(dsp_pair_current));
generate if(AUDIO_DSP_ENABLE)begin:dsp_system
 audio_dsp_control control(.clk(video_clk),.rst(rst_all),.command_valid(dsp_command),.command_value(dsp_command_value),
 .command_ready(dsp_command_ready),.command_done(dsp_command_done),.command_error(dsp_command_error),
 .enabled(dsp_enabled),.agc_enabled(dsp_agc),.compress_enabled(dsp_comp),.limiter_enabled(dsp_lim),.gate_enabled(dsp_gate),.analysis_post(dsp_post),
 .gains(dsp_gains),.threshold_select(dsp_threshold),.ratio_select(dsp_ratio),.speed_select(dsp_speed),.target_select(dsp_target),.status(dsp_control_status));
 audio_dsp_engine engine(.clk(video_clk),.rst(rst_all),.in_valid(link_pcm_valid && dsp_link_capable),.flush(dsp_flush),
 .in_left(dsp_input_left),.in_right(dsp_input_right),.in_ready(dsp_ready),.enabled(dsp_enabled),
 .agc_enabled(dsp_agc),.compress_enabled(dsp_comp),.limiter_enabled(dsp_lim),.gate_enabled(dsp_gate),.gains(dsp_gains),
 .threshold_select(dsp_threshold),.ratio_select(dsp_ratio),.speed_select(dsp_speed),.target_select(dsp_target),
 .out_valid(dsp_out_valid),.out_left(dsp_out_left),.out_right(dsp_out_right),.status(dsp_engine_status),.max_cycles(dsp_max_cycles));
end else begin
 assign dsp_command_ready=1;assign dsp_command_done=dsp_command;assign dsp_command_error=7;
 assign dsp_enabled=0;assign dsp_agc=0;assign dsp_comp=0;assign dsp_lim=0;assign dsp_gate=0;assign dsp_post=0;
 assign dsp_gains=20'h44444;assign dsp_threshold=0;assign dsp_ratio=0;assign dsp_speed=0;assign dsp_target=0;
 assign dsp_control_status=0;assign dsp_engine_status=0;assign dsp_ready=0;assign dsp_out_valid=0;
 assign dsp_out_left=0;assign dsp_out_right=0;assign dsp_max_cycles=0;
end endgenerate
wire remote_pcm_mode=dsp_active || managed_audio_source!=0;
wire remote_pcm_valid=dsp_active ? dsp_return_valid : managed_audio_source==3 ? net_queue_valid && net_audio_play : managed_audio_source==2 ? fm_selected_valid : adc_return_ready;
wire [31:0] selected_play_data=dsp_active ? dsp_return : managed_audio_source==3 ? net_queue_data : {fm_selected_pcm,fm_selected_pcm};
wire fm_split=time_capable && managed_audio_source==2;
wire raw_capture_valid=fm_split && adc_return_ready && fm_usable && !adc_bridge_overflow;
// FM DSP inputs are identical mono; retain its right output directly.
wire [31:0] remote_pcm_data={fm_split ? adc_return_pcm : selected_play_data[31:16],selected_play_data[15:0]};
wire[23:0] av_features;wire av_event;
audio_visual_features u_av_features(.clk(video_clk),.rst(rst_all | fft_source_change),
 .band_valid(spectrum_band_valid),.band_addr(spectrum_band_addr),.band_height(spectrum_band_height),
 .mode_mic(managed_fft_source==1),.bass(av_features[23:18]),.mid(av_features[17:12]),
 .treble(av_features[11:6]),.energy(av_features[5:0]),.event_toggle(av_event));
wire dsp_fft_match=dsp_active && managed_fft_source==managed_audio_source;
reg previous_dsp_post,previous_dsp_enabled,previous_dsp_match;
always @(posedge video_clk or posedge rst_all)
 if(rst_all)begin previous_dsp_post<=0;previous_dsp_enabled<=0;previous_dsp_match<=0;end
 else begin previous_dsp_post<=dsp_post;previous_dsp_enabled<=dsp_enabled;previous_dsp_match<=dsp_fft_match;end
assign fft_source_change=time_flush || (dsp_post!=previous_dsp_post) || (dsp_enabled!=previous_dsp_enabled) || (dsp_fft_match!=previous_dsp_match) || (managed_fft_source!=previous_fft_source) || (managed_fft_source==2 && fm_usable!=previous_fm_usable) || link_source_change;
wire fft_input_valid=dsp_fft_match ? (dsp_post ? dsp_out_valid && dsp_pair_current : link_pcm_valid && dsp_input_available) : managed_fft_source==1 ? mic_valid : managed_fft_source==2 ? (history_selected ? link_pcm_valid && fm_selected_valid : fm_sample_valid && fm_usable) : managed_fft_source==3 ? net_take : link_pcm_valid;
wire [23:0] fft_input_pcm=dsp_fft_match ? (dsp_post ? {dsp_round16(dsp_out_left),8'd0} : dsp_input_left) : managed_fft_source==1 ? mic_pcm : managed_fft_source==2 ? (history_selected ? {fm_selected_pcm,8'd0} : fm_debug[40] ? 24'd0 : fm_pcm) : managed_fft_source==3 ? {net_queue_data[31:16],8'd0} : link_pcm;
// UART diagnostics and seven-segment displays removed by user request.
assign uart_tx=1'b1;
assign seg_sel=6'b111111;
assign seg_data=8'hff;
// Retained top-level physical ports: inactive peripherals are safely idle.
assign sd_ncs=1'b1;assign sd_dclk=0;assign sd_mosi=1'b1;
assign {HDMI_CLK_P,HDMI_D0_P,HDMI_D1_P,HDMI_D2_P}=0;
assign HDMI_DDC_SCL=1'b1;assign HDMI_DDC_SDA=1'bz;


// HUB75 output idle tie-offs replaced by the original controller below.
// Debug read is provided by the independent BRAM port below.

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .clk2_out   (mic_adc_clk),
    .reset      (1'b0)
);
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);
mic_spectrum_source #(.FM_ADC_ENABLE(FM_ENABLE && FM_ADC_ENABLE)) u_mic_spectrum(
 .clk_adc(mic_adc_clk),.clk_video(video_clk),.rst(rst_all),
 .select_sw(1'b1),.selected(mic_selected),.pcm(mic_pcm),.valid(mic_valid),
 .light_data(light_adc_data),.light_toggle(light_adc_toggle),
 .fm_pcm(fm_raw_pcm),.fm_valid(fm_raw_valid),.fm_fault(fm_adc_fault),.fm_clipped(fm_adc_clipped));
parameter BEAT_ENABLE=1;
wire [31:0] beat_now,beat_center;
wire [7:0] beat_mag,beat_bpm,beat_confidence,beat_phase;
wire beat_onset,beat_predict,beat_strong,beat_locked,beat_ambiguity,beat_error;
wire [31:0] beat_stamp,beat_period,beat_next;wire [2:0] beat_state;wire [13:0] beat_flux_value;
wire beat_enable;wire [1:0] beat_sensitivity;
beat_timebase beat_clock(video_clk,rst_all,beat_now);
beat_core beat_analysis(video_clk,rst_all,(BEAT_ENABLE && beat_enable),fft_source_change,beat_now,beat_center,
 spectrum_band_valid,spectrum_band_addr,beat_mag,beat_sensitivity,
 beat_onset,beat_predict,beat_strong,beat_locked,beat_ambiguity,beat_error,
 beat_stamp,beat_period,beat_next,beat_bpm,beat_confidence,beat_phase,beat_state,beat_flux_value);
wire beat_meta_valid,beat_meta_ready;wire [255:0] beat_meta;
wire beat_meta_advance;
reg [2:0] beat_frame_count;
reg beat_frames_seen,beat_flux_seen,beat_stream_fault;
always @(posedge video_clk) begin
 if(rst_all || fft_source_change || !beat_enable) begin
  beat_frame_count<=0;beat_frames_seen<=0;beat_flux_seen<=0;beat_stream_fault<=0;
 end else begin
  if(spectrum_band_valid && spectrum_band_addr==63) begin beat_frame_count<=beat_frame_count+1'b1;beat_frames_seen<=1;end
  if(beat_flux_value!=0)beat_flux_seen<=1;
  if(beat_error)beat_stream_fault<=1;
 end
end

beat_snapshot #(.SERIAL_OUTPUT(1),.COARSE_TELEMETRY(1)) beat_telemetry(video_clk,rst_all,(BEAT_ENABLE && beat_enable),fft_source_change,managed_fft_source,
 beat_now,beat_next,beat_period,beat_stamp,beat_onset,beat_predict,beat_strong,beat_locked,
 beat_ambiguity,beat_error,managed_fft_source==managed_audio_source,beat_bpm,beat_confidence,beat_phase,beat_state,
 beat_meta_ready,beat_meta_valid,beat_meta,beat_meta_advance,
 {beat_frame_count,beat_frames_seen,beat_flux_seen,beat_stream_fault});

audio_spectrum_analyzer #(
    .FFT_N               (FFT_N),
    .FFT_LOG2_N          (FFT_LOG2_N),
    .FFT_INPUT_BITS      (FFT_INPUT_BITS),
    .FFT_INTERNAL_BITS   (FFT_INTERNAL_BITS),
    .FFT_COEF_BITS       (FFT_COEF_BITS),
    .FFT_SAMPLE_RATE     (FFT_SAMPLE_RATE),
    .FFT_HOP_SIZE        (FFT_HOP_SIZE),
    .SPECTRUM_BIN_NUM    (SPECTRUM_BIN_NUM),
    .SPECTRUM_BAND_SHIFT (SPECTRUM_BAND_SHIFT),
    .SPECTRUM_SCALE_SHIFT(SPECTRUM_SCALE_SHIFT),
    .SPECTRUM_MAX_BAR    (SPECTRUM_MAX_BAR),
    .SPECTRUM_LOG_SHIFT  (SPECTRUM_LOG_SHIFT)
) u_audio_spectrum_analyzer (
 .I_beat_now_us(beat_now),.O_beat_center_us(beat_center),.O_beat_magnitude(beat_mag),
    .I_clk             (video_clk),          // 25MHz 逻辑时钟
    .I_rst             (rst_all | fft_source_change),
    .I_audio_valid     (fft_input_valid),        // 48kHz 采样有效
    .I_audio_left_data (fft_input_pcm),          // 24bit 左声道 PCM(MUX 后: 开机=开机音, 之后=normal)
    .O_heights_valid   (spectrum_heights_valid),
    .O_heights         (spectrum_heights),
    .O_band_valid(spectrum_band_valid),
    .O_band_addr(spectrum_band_addr), .O_band_height(spectrum_band_height),
    .O_analysis_busy   (spectrum_busy)
);
spectrum_store_multi #(.STEP(HUB75_SMOOTH_STEP)) u_spectrum_store(
 .clk(video_clk),.rst(rst_all || fft_source_change),.update(spectrum_upd),
 .band_valid(spectrum_band_valid),.band_addr(spectrum_band_addr),.band_height(spectrum_band_height),
 .freeze(spectrum_freeze),.hub_addr(spectrum_hub_addr),.link_addr(spectrum_link_addr),.debug_addr(spectrum_read_addr),
 .hub_height(spectrum_hub_height),.link_height(spectrum_link_height),.debug_height(spectrum_read_height),.committed(spectrum_commit));



// Restored original 64x32 / 1:16 HUB75 white spectrum, shared MIC FFT only.
wire hub_r1,hub_g1,hub_b1,hub_r2,hub_g2,hub_b2;
wire hub_a,hub_b,hub_c,hub_d,hub_clk,hub_lat,hub_oe;
wire [5:0] hub_display_height;
wire hub_row_test,hub_blank_top,hub_blank_bottom;
wire [3:0] hub_row_sel;
generate if(HUB75_TEST_ENABLE)begin:hub_diagnostics
 reg [3:0] switches_meta,switches_sync,mode_q;
 reg previous_oe;
 wire [3:0] requested_mode=switches_sync[2] ? 4'd3 : switches_sync[3] ? 4'd0 :
                            4'd11+{2'b00,switches_sync[1:0]};
 // Optional diagnostic switches are synchronized and committed at frame wrap.
 always @(posedge video_clk or posedge rst_all)begin
  if(rst_all)begin switches_meta<=0;switches_sync<=0;mode_q<=0;previous_oe<=1;end
  else begin
   switches_meta<={audio_mode_sw,sw3,spectrum_mic_sw,adjust_select};
   switches_sync<=switches_meta;previous_oe<=hub_oe;
   if(hub_oe && !previous_oe && {hub_d,hub_c,hub_b,hub_a}==15)mode_q<=requested_mode;
  end
 end
 wire [3:0] selected_mode=HUB75_TEST_ROW_ONLY ? mode_q:HUB75_TEST_PATTERN;
 wire [5:0] test_height;
 hub75_test_pattern patterns(.clk(video_clk),.rst(rst_all),.mode(selected_mode),
  .addr(spectrum_hub_addr),.height(test_height),.row_test(hub_row_test),.row_sel(hub_row_sel),
  .blank_top(hub_blank_top),.blank_bottom(hub_blank_bottom));
 assign hub_display_height=selected_mode==0 ? spectrum_hub_height:test_height;
end else begin:hub_production
 assign hub_display_height=spectrum_hub_height;
 assign hub_row_test=0;assign hub_row_sel=0;assign hub_blank_top=0;assign hub_blank_bottom=0;
end endgenerate
wire hub_oe_effective=hub_oe || (hub_row_test && {hub_d,hub_c,hub_b,hub_a}!=hub_row_sel);
hub75_controller #(
    .INDEXED(1),
    .HUB75_WIDTH      (HUB75_WIDTH),
    .HUB75_SCAN_ROWS  (HUB75_SCAN_ROWS),
    .HUB75_HEIGHT     (HUB75_HEIGHT),
    .HUB75_CLK_DIV    (HUB75_CLK_DIV),
    .HUB75_BRIGHTNESS (HUB75_BRIGHTNESS),
    .HUB75_LAT_CYCLES(HUB75_LAT_CYCLES),
    .HUB75_LAT_OE_CYCLES(HUB75_LAT_OE_CYCLES),
    .HUB75_OE_DEAD_CYCLES(HUB75_OE_DEAD_CYCLES)
) u_hub75_controller (
    .I_clk    (video_clk),
    .I_rst    (rst_all),
    .I_load   (spectrum_upd || fft_clear_d),
    .I_height (384'd0),.O_height_addr(spectrum_hub_addr),.I_read_height(hub_display_height),
    .O_R1(hub_r1), .O_G1(hub_g1), .O_B1(hub_b1),
    .O_R2(hub_r2), .O_G2(hub_g2), .O_B2(hub_b2),
    .O_A(hub_a), .O_B(hub_b), .O_C(hub_c), .O_D(hub_d),
    .O_CLK(hub_clk), .O_LAT(hub_lat), .O_OE(hub_oe)
);


assign HUB75_R1 = HUB75_ENABLE && !hub_blank_top ? hub_r1 : 1'b0;
assign HUB75_G1 = HUB75_ENABLE && !hub_blank_top ? hub_g1 : 1'b0;
assign HUB75_B1 = HUB75_ENABLE && !hub_blank_top ? hub_b1 : 1'b0;
assign HUB75_R2 = HUB75_ENABLE && !hub_blank_bottom ? hub_r2 : 1'b0;
assign HUB75_G2 = HUB75_ENABLE && !hub_blank_bottom ? hub_g2 : 1'b0;
assign HUB75_B2 = HUB75_ENABLE && !hub_blank_bottom ? hub_b2 : 1'b0;
assign HUB75_A  = HUB75_ENABLE ? hub_a  : 1'b0;
assign HUB75_B  = HUB75_ENABLE ? hub_b  : 1'b0;
assign HUB75_C  = HUB75_ENABLE ? hub_c  : 1'b0;
assign HUB75_D  = HUB75_ENABLE ? hub_d  : 1'b0;
assign HUB75_CLK = HUB75_ENABLE ? hub_clk : 1'b0;
assign HUB75_LAT = HUB75_ENABLE ? hub_lat : 1'b0;
assign HUB75_OE  = HUB75_ENABLE ?
                   ((HUB75_OE_ACTIVE_LEVEL == 1) ? ~hub_oe_effective : hub_oe_effective) : 1'b1;



board_link_b #(.NET_VIDEO_EXT(NET_VIDEO_ENABLE),.INDEXED_SPECTRUM(1),.BUFFER_BEAT(0),.SERIAL_BEAT(1),.BEAT_EXT(BEAT_ENABLE),.FM_EXT(FM_INTEGRATION_ENABLE),.SCAN_EXT(FM_SCAN_ENABLE && FM_INTEGRATION_ENABLE),.DSP_EXT(AUDIO_DSP_ENABLE),.TIME_EXT(FM_TIMESHIFT_ENABLE && AUDIO_DSP_ENABLE && FM_INTEGRATION_ENABLE)) u_board_link(
 .beat_now_us(beat_now),.beat_packet(beat_meta),.beat_packet_valid(beat_meta_valid),.beat_packet_advance(beat_meta_advance),.beat_packet_ready(beat_meta_ready),.beat_enable(beat_enable),.beat_sensitivity(beat_sensitivity),.clk50(clk),.clk25(video_clk),.rst(rst_all),
 .cs_n(link_cs_n),.sck(link_sck),.mosi(link_mosi),.miso(link_miso),
 .clk_fast(hdmi_5x_clk),.mosi_extra(link_mosi_extra),.miso_extra(link_miso_extra),
 .video_pixels(av_pixels),.video_flags(av_flags),.video_valid(av_pixels_valid),.video_ready(av_pixels_ready),
 .video_active(av_active),.video_done(av_ended),.video_error(av_error),
 .video_enable(link_video_enable),.video_play(link_video_play),.video_epoch(link_video_epoch),.acknowledged_groups(video_acked_groups),
 .spectrum_live_height(spectrum_link_height),.spectrum_live_addr(spectrum_link_addr),.spectrum_freeze(spectrum_freeze),
 .heights(384'd0),.heights_valid(spectrum_heights_valid),
 .present(soft_video_run),.light_ready(light_ready),.light_fault(light_fault),.light_level(light_level),.light_adc(light_filtered),
 .online(link_online),.mode_mic(link_mode_mic),.source_change(link_source_change),.pcm(link_pcm),.pcm_valid(link_pcm_valid),.boot_done(link_boot_done),
 .net_mode(remote_pcm_mode),.net_valid(remote_pcm_valid),.net_pcm(remote_pcm_data),.net_take(remote_pcm_take),
 .av_features(av_features),.av_event(av_event),.fm_status(fm_system_status),
 .requested_mode_mic(link_requested_mic),.analysis_mic(managed_fft_source==1),
 .analysis_change((managed_fft_source!=previous_fft_source) || (managed_fft_source==2 && fm_usable!=previous_fm_usable)),
 .fm_cmd_valid(link_fm_valid),.fm_cmd_ready(link_fm_ready),.fm_cmd_seq(link_fm_seq),.fm_cmd_op(link_fm_op),.fm_cmd_value(link_fm_value),.scan_status(fm_scan_status),.scan_read_data(scan_read_data),.scan_read_index(scan_read_index),.dsp_status(dsp_status),.stereo_pcm(link_stereo_pcm),.dsp_capable(dsp_link_capable),.dsp_pcm_valid(dsp_link_valid),.raw_capture_valid(raw_capture_valid),.time_capable(time_capable),.history_mode(history_mode),.play_epoch(play_epoch));

endmodule

// Optional diagnostics from HUB75_LED_20261030. Not instantiated in production.
// Height is synchronous, matching spectrum_store_multi's one-cycle BRAM port.
module hub75_test_pattern #(parameter integer CLK_HZ=25000000)(
 input wire clk,rst,input wire [3:0] mode,input wire [5:0] addr,
 output reg [5:0] height,output reg row_test,output reg [3:0] row_sel,
 output reg blank_top,blank_bottom
);
 localparam integer HALF_CYCLES=CLK_HZ<2 ? 1:CLK_HZ/2;
 localparam integer CW=HALF_CYCLES<2 ? 1:$clog2(HALF_CYCLES);
 reg [CW-1:0] count;reg [5:0] sweep_height,walk_column;
 reg [5:0] next_height;
 always @*begin
  next_height=0;row_test=0;row_sel=0;blank_top=0;blank_bottom=0;
  case(mode)
   1:next_height=0;2:next_height=16;3:next_height=32;
   4:next_height=15;5:next_height=17;6:next_height=8;7:next_height=1;
   8:next_height=sweep_height;
   9:begin next_height=32;blank_bottom=1;end
   10:begin next_height=32;blank_top=1;end
   11:begin next_height=32;row_test=1;row_sel=14;blank_bottom=1;end
   12:begin next_height=32;row_test=1;row_sel=15;blank_bottom=1;end
   13:begin next_height=32;row_test=1;row_sel=0;blank_top=1;end
   14:begin next_height=32;row_test=1;row_sel=1;blank_top=1;end
   15:next_height=addr==walk_column ? 32:0;
   default:next_height=0;
  endcase
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin count<=0;sweep_height<=0;walk_column<=0;height<=0;end
  else begin
   height<=next_height;
   if(count==HALF_CYCLES-1)begin
    count<=0;sweep_height<=sweep_height==32 ? 0:sweep_height+1'b1;
    walk_column<=walk_column+1'b1;
   end else count<=count+1'b1;
  end
 end
endmodule
