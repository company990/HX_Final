//=====================================================================
// startup_controller.v  (V12 Startup Image + Startup Music 三阶段生命周期)
// ---------------------------------------------------------------------
// V26 Step8 容错重构: 启动先"扫描 BMP + WAV", 依真实扫描结果选 3 条启动路径,
//  并用单个数码管显示 0/1/2/3(见 O_media_digit)。
//
// 判定输入(全部为真实信号, 无固定 delay):
//   I_bmp_scan_done = sd_card_bmp.O_scan_done (bmp_read.scan_done: 找到目标张数 or 扫到末尾)
//   I_bmp_found     = sd_card_bmp.O_bmp_found (scan_done 时刻 img_found_count!=0)
//   I_wav_found     = audio_wav_top_path.o_wav_found_c (FAT 定位到 /TF_AUDIO/MUSIC.WAV, 粘性)
//   I_wav_err       = audio_wav_top_path.o_wav_err_c   (定位失败/未找到/格式错, 粘性)
//   I_sd_init_done  = sd_card_top.sd_init_done (未完成前不判定, 防"提前报错")
//
// 路径(状态编码; 0..3 与旧版一致, 5/6 为新增):
//   --- 先扫描(复位后处于 S_WAIT, digit 0): 等 SD 初始化 + BMP 扫描结束 ---
//   0 = S_WAIT      (digit 0/1) 扫描中(digit0); 有 BMP 后等首帧(digit1)
//   --- 找到 BMP(>=1 张) -> 进入原来的正常启动流程(100% 原语义) ---
//   1 = S_SETTLE    (digit 1) 首帧就绪: 锁首图, settle 计满, 静音/频谱 OFF
//   2 = S_MUSIC     (digit 1) Music+Spectrum ON, 播一轮
//   3 = S_NORM      (digit 1) music done: 解锁/切图, TestTone+Spectrum ON
//   --- 无 BMP 但 WAV 可用 -> 直接启动音频大循环, 不播 Startup Music ---
//   5 = S_NOWAV     (digit 2) 不等 BMP: WAV 播放路径 + 频谱 ON(music OFF)
//   --- BMP/WAV 都不可用 -> 保持静音错误态, 不死锁 ---
//   6 = S_NOMEDIA   (digit 3) 静音 + 频谱 OFF + 不等待
//
// STARTUP_DEBUG_DISPLAY(本模块参数)已不再改变生命周期(避免破坏路径A): 顶层同名参数
//  改为"单数码管启动态显示"开关, 传给本模块的恒为 0。
//=====================================================================
module startup_controller #(
    parameter integer STARTUP_MUSIC_ENABLE    = 1,   // 0=首帧后直接 normal(不播音乐)
    parameter integer STARTUP_AUDIO_SETTLE_MS = 300, // 首帧就绪后 settle(以 48k audio_valid 计), 0=不 settle
    parameter integer STARTUP_DEBUG_DISPLAY   = 0    // 保留: 1=music done 后停留 MUSIC(V26 Step8 起顶层恒传 0)
)(
    input  wire I_clk,               // video_clk
    input  wire I_rst,               // rst_all
    input  wire I_pll_locked,        // audio_pll_lock
    input  wire I_first_frame_ready, // 首帧 BMP 已提交(video 域同步后 display_valid)
    input  wire I_sample_enable,     // 48k audio_valid
    input  wire I_music_done,        // player O_startup_done
    input  wire I_sd_init_done,      // V26 Step8: SD 初始化完成(真实信号)
    input  wire I_bmp_scan_done,     // V26 Step8: BMP 扫描结束(找到 or 扫完)
    input  wire I_bmp_found,         // V26 Step8: 有 >=1 张 BMP(scan_done 时刻锁存)
    input  wire I_wav_found,         // V26 Step8: WAV 已定位(粘性)
    input  wire I_wav_err,           // V26 Step8: WAV 不可用(粘性)
    output wire [2:0] O_state,       // 0..6 状态编码(见文件头; 0..3 与旧版一致)
    output wire [1:0] O_media_digit, // 单数码管: 0=扫描中 1=BMP路径 2=WAV路径 3=无媒体
    output wire O_no_bmp,            // 1=已判定无 BMP(digit2/3) -> 通知 WAV ctl 不等首帧
    output wire O_startup_phase,     // 1=SETTLE/MUSIC(锁第一张 BMP/key)
    output wire O_music_on,          // 1=MUSIC(player I_start)
    output wire O_spectrum_on,       // 1=MUSIC/NORMAL/NOWAV(HDMI/HUB75 频谱使能)
    output wire O_test_tone_on,      // 1=NORMAL(C4~C5 Test Tone 允许)
    output wire O_done_pulse         // MUSIC->NORMAL 单拍(debug/自动切图参考)
);
    localparam [2:0] S_WAIT  = 3'd0,   // 复位态(与旧版一致)
                     S_SETTLE= 3'd1,
                     S_MUSIC = 3'd2,
                     S_NORM  = 3'd3,
                     S_NOWAV = 3'd5,
                     S_NOMEDIA=3'd6;
    localparam [23:0] SETTLE_SAMPLES = STARTUP_AUDIO_SETTLE_MS * 48;

    reg [2:0]  st;
    reg [23:0] scnt;
    reg [2:0]  st_prev;
    reg        done_pulse;

    // 扫描/判定真实信号
    wire scan_ok     = I_sd_init_done && I_bmp_scan_done;  // BMP 扫描已结束(可判定)
    wire wav_decided = I_wav_found | I_wav_err;            // WAV 定位已出结论
    // 仍在扫描中: BMP 未扫完, 或已确认无 BMP 但 WAV 还未出结论 -> digit 0
    wire scan_pending = !scan_ok || (!I_bmp_found && !wav_decided);

    assign O_state        = st;
    assign O_media_digit  = (st == S_NOWAV)   ? 2'd2 :
                            (st == S_NOMEDIA) ? 2'd3 :
                            ((st == S_WAIT) && scan_pending) ? 2'd0 : 2'd1;
    assign O_no_bmp       = (st == S_NOWAV) || (st == S_NOMEDIA);
    assign O_startup_phase= (st == S_SETTLE) || (st == S_MUSIC);
    assign O_music_on     = (st == S_MUSIC);
    assign O_spectrum_on  = (st == S_MUSIC) || (st == S_NORM) || (st == S_NOWAV);
    assign O_test_tone_on = (st == S_NORM);
    assign O_done_pulse   = done_pulse;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            st         <= S_WAIT;
            scnt       <= 24'd0;
            st_prev    <= S_WAIT;
            done_pulse <= 1'b0;
        end else begin
            st_prev    <= st;
            done_pulse <= 1'b0;
            // MUSIC -> NORMAL 时产生单拍(用于 debug/自动切图)
            if (st_prev == S_MUSIC && st == S_NORM)
                done_pulse <= 1'b1;

            case (st)
                S_WAIT: begin
                    // 阶段0: 先等扫描结束(bmp_scan_done); 扫描期间不播/不切图(digit 0)。
                    //  扫描结束后的分支(BMP 优先 / WAV 直启 / 无媒体)与旧版
                    //  "首帧就绪 -> settle/music/normal" 语义完全兼容。
                    scnt <= 24'd0;
                    if (scan_ok) begin
                        if (I_bmp_found) begin
                            // 路径A: 原逻辑语义不变(仍等 PLL + 首帧)
                            if (I_pll_locked && I_first_frame_ready) begin
                                if (STARTUP_MUSIC_ENABLE == 0) begin
                                    st <= S_NORM;               // 不播音乐: 首帧后直接 normal
                                end else if (SETTLE_SAMPLES == 0) begin
                                    st <= S_MUSIC;
                                end else begin
                                    st <= S_SETTLE;
                                end
                            end
                        end else if (I_wav_found && !I_wav_err) begin
                            st <= S_NOWAV;                  // 路径B: WAV 直启
                        end else if (I_wav_err) begin
                            st <= S_NOMEDIA;                // 路径C: 静音错误态
                        end
                        // else: WAV 仍在定位 -> 留 S_WAIT(digit 保持 0)
                    end
                end
                S_SETTLE: begin
                    // 阶段1: Audio Settle(48k 计数); 期间静音/频谱 OFF/锁首图
                    if (I_sample_enable) begin
                        if (scnt >= (SETTLE_SAMPLES - 24'd1))
                            st <= S_MUSIC;
                        else
                            scnt <= scnt + 24'd1;
                    end
                end
                S_MUSIC: begin
                    // 阶段1: Music+Spectrum ON(第一张 BMP 保持); 等整曲 done
                    if (I_music_done) begin
                        if (STARTUP_DEBUG_DISPLAY == 1)
                            st <= S_MUSIC;              // 调试: 停留(音乐已静音)
                        else
                            st <= S_NORM;
                    end
                end
                S_NORM: begin
                    // 阶段2: normal(TestTone+Spectrum+图片切换); 保持到复位
                end
                S_NOWAV: begin
                    // 路径B: 无 BMP, WAV 可用 -> 不等首帧/不播 Startup Music;
                    //        音频 mux 由 audio_startup_wav_ctl 接管(ready 前静音),
                    //        频谱 ON(与 NORMAL 一致), TestTone OFF。
                end
                S_NOMEDIA: begin
                    // 路径C: BMP 与可用 WAV 都不存在 -> 保持静音/频谱 OFF/无等待(不死锁)。
                    //        音频 mux 由 ctl 停在 WAV_UNAVAIL(静音)。
                end
                default: st <= S_WAIT;
            endcase
        end
    end
endmodule
