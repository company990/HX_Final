//=====================================================================
// audio_startup_wav_ctl.v   (V27 Step2: 音频启动 FSM — 与 Video/BMP 完全解耦)
//
// 目标(Step2): 音频启动链不再等待任何视频/图片信号。
//   新链路:  sd_init_done -> 参数化延迟 -> Startup Music one-shot
//            -> Music done -> FAT32 ROOT 搜 /MUSIC.WAV -> WAV 播放 -> 结束/错误
//
// 音频 FSM(状态编码 = O_phase, 同时作为 debug digit 依据):
//   A_WAIT_SD(0) : 等 I_sd_init_done=1      (digit 0) 静音; WAV client 不发 SD 请求
//   A_DELAY(1)   : SD init 后参数化延时      (digit 0) 静音; 以 48k sample_enable 计
//   A_MUSIC(2)   : 发 O_music_start 单拍 -> Startup Music one-shot (digit 1) mux=startup
//   A_SEARCH(3)  : 发 O_wav_search(电平) 启动 ROOT locator + 预取 (digit 2) 静音等待
//   A_PLAY(4)    : O_wav_play=1, mux=WAV      (digit 3)
//   A_ERROR(5)   : 未找到/无效/播完 -> 稳定静音 (digit 4)
//
// V27S14-R3 软复位续播: 开机流程(A_DELAY+A_MUSIC)在本上电周期内只执行一次。
//   开机音乐播完过一次(在 A_MUSIC 内看到 I_startup_done)后, 置位粘性标志 boot_flow_done
//   (仅 rst_n/硬复位清除, 软复位 rst_soft 不清除); 之后任一次软复位释放(rst_soft 0->1)都
//   从 A_WAIT_SD 直接进入 A_SEARCH(跳过 A_DELAY/A_MUSIC), 即回到"播放音频"(digit 3),
//   不再重复播放开机流程; 首次上电(标志=0)仍完整走 A_DELAY+A_MUSIC。
//
// 本模块**没有任何 BMP/Video 输入**(无 first_frame_ready/display_valid/bmp_scan_done/
// bmp_found) => 结构上不可能等待视频。
//
// 与 audio_mux3 的接口保持不变: O_audio_sel = 0 startup / 1 wav / 2 tone / 3 silence
// SD 共享原则: 只有在 A_SEARCH 之后才 O_wav_search=1, 之前 WAV client 保持 idle，
//             因此不会在 A_WAIT_SD/A_DELAY/A_MUSIC 抢 SD(Startup Music 为内部 ROM, 不占 SD)。
//=====================================================================
module audio_startup_wav_ctl #(
    parameter integer AUDIO_START_DELAY_MS = 1000, // SD init done 后的音频启动延时(ms, 以 48k tick 计)
    parameter integer THRESH_FRAMES        = 1,    // WAV 起播安全水位(帧)
    parameter integer WCNT_W               = 8,    // I_wav_used 位宽
    parameter integer AUDIO_TEST_MODE      = 0     // 1=WAV 不可用时输出 TestTone(DEBUG)
)(
    input  wire clk,
    input  wire rst_n,       // active-low hardware/PLL reset
    input  wire rst_soft,    // active-low HC-SR501 reset: 1=person/run, 0=no person/reset

    // ---- 启动/事件输入(全部音频域, 与 Video 无关) ----
    input  wire I_sd_init_done,          // V27S2 新增: SD 初始化完成(已同步到本域)
    input  wire I_sample_enable,         // 48k 采样 tick(=audio_valid), 用于 ms 级延时计数
    input  wire I_startup_done,          // Startup Music 播完(粘性) [旧名保留]
    // Enter WAV search immediately for a live network stream. This prevents
    // UDP WACK generation from being blocked by the boot delay and melody.
    input  wire I_skip_boot_flow,
    input  wire I_wav_found,             // ROOT locator: 找到 /MUSIC.WAV
    input  wire I_wav_error,             // ROOT locator: 未找到/非法/链错误
    input  wire [WCNT_W-1:0] I_wav_used, // 音频 ring 水位(帧)
    input  wire I_wav_play_done,         // 播放完成
    // ---- 兼容保留(本步起不再使用, 仅为旧 TB 端口兼容) ----
    input  wire I_first_frame_ready,     // (不再使用: 音频不等首帧)
    input  wire I_no_bmp,                // (不再使用: 音频不等 BMP 判定)

    // ---- 输出 ----
    output reg  [1:0] O_audio_sel,       // 0 startup / 1 wav / 2 tone / 3 silence
    output reg        O_wav_play,        // WAV consumer 使能
    output reg        O_music_start,     // V27S2 新增: 单拍, 启动 Startup Music one-shot
    output reg        O_wav_search,      // V27S2 新增: 电平, 启动 ROOT locator + 预取
    output wire       O_prefetch_start,  // 旧名保留 = O_wav_search
    output reg        O_unlock_image,    // 保留输出(Step2 起 top 不再用它锁图)
    output reg  [2:0] O_digit,           // V27S2 新增 debug: 0=WAIT/DELAY 1=Music 2=Search 3=Play 4=Error
    output wire [2:0] O_phase            // = 状态编码(0..5)
);
    localparam [2:0] A_WAIT_SD = 3'd0,
                     A_DELAY   = 3'd1,
                     A_MUSIC   = 3'd2,
                     A_SEARCH  = 3'd3,
                     A_PLAY    = 3'd4,
                     A_ERROR   = 3'd5;

    // 延时(ms) -> 48k sample 数
    localparam integer DELAY_SAMPLES = AUDIO_START_DELAY_MS * 48;
    localparam [15:0] DELAY_LAST = (DELAY_SAMPLES > 0) ? (DELAY_SAMPLES - 1) : 0;

    reg [2:0]  st;
    reg [15:0] dcnt;             // 延时计数
    reg        played;           // 本状态内是否已播完(WAV play done)

    wire wav_ready   = I_wav_found && !I_wav_error && (I_wav_used >= THRESH_FRAMES);
    wire wav_decided = I_wav_found | I_wav_error;      // locator 已出结论
    wire wav_bad     = wav_decided && (I_wav_error | ~I_wav_found);

    assign O_phase          = st;
    assign O_prefetch_start = O_wav_search;   // 旧名兼容

    // ---- V27S14-R3: "本上电周期内开机流程已完成" 粘性标志(仅 rst_n 清除) ----
    // 置位时机 = 在 A_MUSIC 内等到 Music done(与离开 A_MUSIC 同一拍); 软复位不清除。
    // 依据该标志, 软复位释放后 A_WAIT_SD 直接跳 A_SEARCH, 不再重播开机流程。
    reg boot_flow_done;
    always @(posedge clk or negedge rst_n) begin
        if (rst_n == 1'b0) boot_flow_done <= 1'b0;
        else if (rst_soft && (st == A_MUSIC) && I_startup_done)
            boot_flow_done <= 1'b1;
    end

    always @(posedge clk or negedge rst_n or negedge rst_soft) begin
        if ((rst_n == 1'b0) || (rst_soft == 1'b0)) begin
            st              <= A_WAIT_SD;
            dcnt            <= 16'd0;
            played          <= 1'b0;
            O_audio_sel     <= 2'd3;
            O_wav_play      <= 1'b0;
            O_music_start   <= 1'b0;
            O_wav_search    <= 1'b0;
            O_unlock_image  <= 1'b0;
            O_digit         <= 3'd0;
        end else begin
            O_music_start <= 1'b0;          // 单拍脉冲默认 0

            case (st)
                // ---------------- 0: 等 SD 初始化(不碰 SD/WAV) ----------------
                A_WAIT_SD: begin
                    O_audio_sel    <= 2'd3;     // 静音
                    O_wav_play     <= 1'b0;
                    O_wav_search   <= 1'b0;     // WAV client 保持 idle
                    O_unlock_image <= 1'b0;
                    O_digit        <= 3'd0;
                    dcnt           <= 16'd0;
                    // V27S14-R3: 开机流程已走过 => 软复位释放后直接回搜索/播放(A_SEARCH),
                    // 不再重走 A_DELAY/A_MUSIC; 首次上电(标志=0)保持原三步走。
                    if (I_sd_init_done)
                        st <= (boot_flow_done || I_skip_boot_flow) ? A_SEARCH : A_DELAY;
                end

                // ---------------- 1: 参数化延时(以 48k tick 计) ----------------
                A_DELAY: begin
                    O_audio_sel    <= 2'd3;
                    O_wav_play     <= 1'b0;
                    O_wav_search   <= 1'b0;
                    O_unlock_image <= 1'b0;
                    O_digit        <= 3'd0;
                    if (I_startup_done) begin
                        st <= A_SEARCH; dcnt <= 0;
                    end else if (I_sample_enable) begin
                        // V27S5 修复: 转入 A_MUSIC 时**同时清零 dcnt**。
                        // 原写法只做 st<=A_MUSIC, dcnt 仍为 DELAY_LAST(1000ms -> 47999),
                        // 而 A_MUSIC 的发脉冲条件是 dcnt==0 => O_music_start 永不发出
                        // => 播放器收不到 start => startup_done 恒 0 => 卡在 A_MUSIC(digit 恒 1)。
                        // 注: st / dcnt 各自只在本分支赋值一次, 无双重赋值冲突。
                        if (dcnt >= DELAY_LAST) begin
                            st   <= A_MUSIC;
                            dcnt <= 16'd0;
                        end else begin
                            dcnt <= dcnt + 16'd1;
                        end
                    end
                end

                // ---------------- 2: Startup Music one-shot ----------------
                A_MUSIC: begin
                    O_audio_sel    <= 2'd0;     // mux = Startup Music
                    O_wav_play     <= 1'b0;
                    O_wav_search   <= 1'b0;     // 不申请 SD(音乐是内部 ROM)
                    O_unlock_image <= 1'b0;
                    O_digit        <= 3'd1;
                    if (I_startup_done) begin
                        st <= A_SEARCH;         // Music done -> 搜 WAV
                    end else if (dcnt == 16'd0) begin
                        O_music_start <= 1'b1;  // 进入本状态第一拍发启动脉冲
                        dcnt          <= 16'd1;
                    end
                end

                // ---------------- 3: ROOT 搜索 /MUSIC.WAV(此时才允许用 SD) ----------------
                A_SEARCH: begin
                    O_audio_sel    <= 2'd3;     // 搜索期静音
                    O_wav_play     <= 1'b0;
                    O_wav_search   <= 1'b1;     // 电平: 触发 locator + 预取
                    O_unlock_image <= 1'b1;
                    O_digit        <= 3'd2;
                    if (wav_ready) begin
                        st <= A_PLAY;
                    end else if (wav_bad) begin
                        st <= A_ERROR;
                    end
                end

                // ---------------- 4: WAV 播放 ----------------
                A_PLAY: begin
                    O_wav_search   <= 1'b1;     // 保持预取(流式续读)
                    O_unlock_image <= 1'b1;
                    O_digit        <= 3'd3;
                    if (I_wav_error) begin
                        st <= A_ERROR;
                        O_audio_sel <= 2'd3;
                        O_wav_play <= 1'b0;
                    end else if (I_wav_play_done || played) begin
                        played       <= 1'b1;
                        O_audio_sel  <= 2'd3;   // 播完静音, 不循环
                        O_wav_play   <= 1'b0;
                        if (I_wav_error) st <= A_ERROR;
                    end else begin
                        O_audio_sel <= 2'd1;    // WAV
                        O_wav_play  <= 1'b1;
                    end
                end

                // ---------------- 5: 稳定静音错误态(未找到/无效) ----------------
                A_ERROR: begin
                    O_audio_sel    <= (AUDIO_TEST_MODE == 1) ? 2'd2 : 2'd3;
                    O_wav_play     <= 1'b0;
                    O_wav_search   <= 1'b1;
                    O_unlock_image <= 1'b1;
                    O_digit        <= 3'd4;
                end

                default: st <= A_WAIT_SD;
            endcase
        end
    end
endmodule
