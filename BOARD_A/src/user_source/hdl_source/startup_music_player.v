//=====================================================================
// startup_music_player.v  (开机音乐 DDS 播放器 - V12: 线性 62 音, 一次即 DONE)
// 数据源 = startup_music_rom.v (开机音乐(2).mscz 简化: M1-5+M7-13, 62 音, 无 M6/无 repeat)
//
// V12 语义: 纯音乐引擎, 不知道 HDMI/BMP/图片/循环。
//   I_start=1(top/startup_controller 在首帧+settle 后给) -> 从 ROM[0](F4) 开始,
//   线性播放 ROM addr 0..NOTE_NUM-1(62 音), 播完 -> O_startup_done=1, active=0, PCM=0。
//   不循环; 不重复; 保持到复位。
//
// 工作时钟 I_clk=video_clk(稳定); duration 以 48k sample enable(audio_valid)计数;
// DDS: phase_acc[31] 方波 +-AMP; 无 DSP/无 sin ROM/无 BRAM。
//=====================================================================
module startup_music_player #(
    parameter integer STARTUP_ENABLE = 1,               // 0=完全关闭(不播, done 常 1)
    parameter integer NOTE_NUM       = 62,              // 有效音符事件数(ROM 0..NOTE_NUM-1)
    parameter signed [23:0] AMP      = 24'sd2000000,
    parameter integer ROM_DEPTH      = 128,
    parameter integer ADDR_BITS      = 7
)(
    input  wire             I_clk,
    input  wire             I_rst,
    input  wire             I_pll_locked,      // PLL 未锁保持静止(reset 语义兜底)
    input  wire             I_start,           // 1=允许启动(top controller 置位, MUSIC 态)
    input  wire             I_sample_enable,   // 1 clk 48kHz 采样脉冲
    output wire             O_startup_active,  // 1=播放中
    output wire             O_startup_done,    // 1=整曲播完(直到复位)
    output wire signed [23:0] O_left,
    output wire signed [23:0] O_right
);

    localparam [1:0] S_IDLE = 2'd0, S_PLAY = 2'd1, S_DONE = 2'd2;
    localparam [ADDR_BITS-1:0] LAST = NOTE_NUM - 1;   // 62 音 => addr 61 为末音

    reg  [1:0] S_state;
    reg  [ADDR_BITS-1:0] S_addr;
    reg  [15:0] S_cnt;                 // 当前音符剩余 sample enable(含本次)
    reg  [31:0] S_phase_acc;
    reg  signed [23:0] S_out;
    reg         S_done;

    // 目标(下一)音符地址: 线性 addr+1(末音自身, 不进 TERM)
    wire [ADDR_BITS-1:0] t_addr = (S_addr == LAST) ? S_addr : (S_addr + {{(ADDR_BITS-1){1'b0}},1'b1});

    wire [31:0] W_step, W_tstep;
    wire [15:0] W_dur,  W_tdur;

    startup_music_rom #(
        .ROM_DEPTH(ROM_DEPTH),
        .ADDR_BITS(ADDR_BITS)
    ) u_rom (
        .I_addr       (S_addr),
        .O_phase_step (W_step),
        .O_duration   (W_dur),
        .I_taddr      (t_addr),
        .O_t_phase_step(W_tstep),
        .O_t_duration (W_tdur)
    );

    wire active = (S_state == S_PLAY) && (I_pll_locked == 1'b1) && (STARTUP_ENABLE == 1);

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_state     <= S_IDLE;
            S_addr      <= {ADDR_BITS{1'b0}};
            S_cnt       <= 16'd0;
            S_phase_acc <= 32'd0;
            S_out       <= 24'sd0;
            S_done      <= 1'b0;
        end else if (!I_pll_locked) begin
            // PLL 未锁定: 静止(IDLE), 全清
            S_state     <= S_IDLE;
            S_addr      <= {ADDR_BITS{1'b0}};
            S_cnt       <= 16'd0;
            S_phase_acc <= 32'd0;
            S_out       <= 24'sd0;
            S_done      <= 1'b0;
        end else if (STARTUP_ENABLE == 0) begin
            S_state <= S_IDLE;
            S_out   <= 24'sd0;
            S_done  <= 1'b1;
        end else begin
            case (S_state)
                S_IDLE: begin
                    // I_start 由 controller 在 Splash+Settle 后给出 -> 从 ROM[0](F4) 开始
                    if (I_start) begin
                        S_state     <= S_PLAY;
                        S_addr      <= {ADDR_BITS{1'b0}};
                        S_cnt       <= W_dur;      // 首音 duration(addr=0)
                        S_phase_acc <= 32'd0;
                        S_done      <= 1'b0;
                        S_out       <= 24'sd0;
                    end
                end
                S_PLAY: begin
                    if (I_sample_enable) begin
                        // ---- 产生当前音符样本(方波) ----
                        S_out       <= S_phase_acc[31] ? AMP : -AMP;
                        S_phase_acc <= S_phase_acc + W_step;
                        // ---- 时长推进 ----
                        if (S_cnt == 16'd0) begin
                            S_cnt <= W_dur - 16'd1;   // 保险: 首音尚未载入
                        end else if (S_cnt == 16'd1) begin
                            // 本音符最后一音已产生 -> 线性下一音; 末音结束 -> DONE(不循环)
                            if (S_addr == LAST) begin
                                S_state <= S_DONE;
                                S_done  <= 1'b1;
                                S_out   <= 24'sd0;
                            end else begin
                                S_addr <= S_addr + {{(ADDR_BITS-1){1'b0}},1'b1};
                                S_cnt  <= W_tdur;       // dur(addr+1)
                            end
                        end else begin
                            S_cnt <= S_cnt - 16'd1;
                        end
                    end
                end
                S_DONE: begin
                    S_out <= 24'sd0;              // 忽略 I_start(不重复播放)
                end
                default: S_state <= S_IDLE;
            endcase
        end
    end

    assign O_startup_active = active;
    assign O_startup_done   = S_done;
    assign O_left           = active ? S_out : 24'sd0;
    assign O_right          = active ? S_out : 24'sd0;

endmodule

