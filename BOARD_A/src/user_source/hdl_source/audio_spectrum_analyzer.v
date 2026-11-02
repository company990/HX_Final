//=====================================================================
// audio_spectrum_analyzer.v
// 音频频谱分析链顶层（纯旁路，不阻塞原 HDMI 音频/视频）：
//
//   I2S_receiver(video_clk域, 48kHz)
//     -> audio_valid / audio_left_data
//     -> 1024 点双缓冲采集（仅在 audio_valid=1 采样，Fs=48kHz）
//     -> Hann 加窗（Q1.15 ROM 半表对称，w[n]=w[1023-n]）
//     -> fft_1024_engine（自研 radix-2 DIT，位反转装载 -> 自然 bin 序）
//     -> sqrt(Re^2+Im^2)（sqrt_integer）
//     -> 正频率 bin 1..512 -> 32 频带（取带内最大幅度）
//     -> 线性缩放 -> 32 个 8bit 柱高
//
// 实时性：FFT 计算约 2~3ms << 21.33ms(1024@48kHz)，双缓冲保证采集
//         与计算不互相阻塞；原音频播放链路不受影响（旁路复制数据）。
// 只分析左声道（测试音左右相同）。
//
// 所有工程级参数由 top 传入（禁止子模块自行定义工程级宏）。
//=====================================================================
module audio_spectrum_analyzer #(
    parameter integer FFT_N                = 1024,    // 点数
    parameter integer FFT_LOG2_N           = 10,      // log2(FFT_N)
    parameter integer FFT_INPUT_BITS       = 16,      // 采样位宽(有符号)
    parameter integer FFT_INTERNAL_BITS    = 26,      // FFT 实/虚部位宽
    parameter integer FFT_COEF_BITS        = 16,      // 窗/旋转因子位宽
    parameter integer FFT_SAMPLE_RATE      = 48000,   // 采样率(文档用)
    parameter integer FFT_HOP_SIZE         = 256,     // 滑窗步进(每帧新采样数)
    parameter integer SPECTRUM_BIN_NUM     = 64,      // HUB75 频带(柱)数
    parameter integer SPECTRUM_BAND_SHIFT  = 3,       // 每带 2^3=8 个 bin (bin1..512 -> 64 带)
    parameter integer SPECTRUM_SCALE_SHIFT = 15,      // (保留未用)
    parameter integer SPECTRUM_MAX_BAR     = 71,      // (保留未用)
    parameter integer SPECTRUM_LOG_SHIFT   = 16       // 幅度>>LOG_SHIFT 后 log 压缩(灵敏度调参)
)(
    input  wire                          I_clk,       // video_clk 25MHz
    input  wire                          I_rst,       // 高有效
    input  wire                          I_audio_valid,
    input  wire signed [23:0]            I_audio_left_data,

    output reg                           O_heights_valid,          // 新一帧就绪脉冲
    output reg [SPECTRUM_BIN_NUM*6-1:0] O_heights,               // 每列 6bit 柱高 0..31
    output reg O_band_valid,
    output reg [5:0] O_band_addr,
    output reg [5:0] O_band_height,
    output reg                           O_analysis_busy
);

    localparam integer N       = FFT_N;                  // 1024
    localparam integer AW      = FFT_LOG2_N;             // 10
    localparam integer MAG_BITS= 26;                     // 幅度位宽
    localparam integer SQR_IN  = 2*MAG_BITS;             // 52：Re^2+Im^2
    localparam integer BAR_BITS= 6;                      // 输出柱高位宽(0..31)
    localparam integer HOP     = FFT_HOP_SIZE;           // 默认 256

    // ============ 采样裁剪：24bit -> 高 16bit [23:8] ============
    wire signed [FFT_INPUT_BITS-1:0] S_sample_in =
        I_audio_left_data[23 -: FFT_INPUT_BITS];

    // ================================================================
    // 1) 1024x16bit 连续环形缓冲（独立于图片双缓冲，只存一份）
    //    - 每个 audio_valid 采样持续写入并回绕，不停止采集
    //    - 前 1024 点暖机；此后每 HOP 个新采样触发一次分析（Hop=256 => 75% overlap）
    //    - 触发时快照 S_base=下一个写指针(=窗口最旧样本)，窗口 n=0..1023=(base+n)
    //      即为“最旧..最新”时间顺序
    // ================================================================
    reg signed [FFT_INPUT_BITS-1:0] S_ring [0:N-1];
    reg [AW-1:0] S_wr;          // 下一个写指针 0..1023
    reg [AW-1:0] S_hop_cnt;     // 距上次触发的新采样数
    reg          S_warm;        // 是否已收满 1024 点
    reg [AW-1:0] S_base;        // 窗口最旧样本地址
    reg          S_hop_evt;     // 分析触发脉冲
    reg          S_pend;        // 忙时错过的触发待处理标志

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_wr      <= {AW{1'b0}};
            S_hop_cnt <= {AW{1'b0}};
            S_warm    <= 1'b0;
            S_base    <= {AW{1'b0}};
            S_hop_evt <= 1'b0;
            // 注意：S_pend 只允许由主 FSM always 复位/驱动（单一驱动），
            // 不得在本 Ring 采样块中出现，否则构成多重时序驱动。
        end else begin
            S_hop_evt <= 1'b0;
            if (I_audio_valid) begin
                S_ring[S_wr] <= S_sample_in;      // 连续采集，不停
                if (!S_warm) begin
                    // 暖机完成：写满 1024 点后允许首次分析（窗口 base=0）
                    if (S_wr == (N-1)) begin
                        S_warm    <= 1'b1;
                        S_hop_cnt <= {AW{1'b0}};
                        S_base    <= {AW{1'b0}};
                        S_hop_evt <= 1'b1;
                    end
                end else begin
                    // 每 HOP 个新采样触发（本采样为触发前的第 HOP 个）
                    if (S_hop_cnt == (HOP-1)) begin
                        S_hop_cnt <= {AW{1'b0}};
                        S_base    <= (S_wr == (N-1)) ? {AW{1'b0}} : (S_wr + 1'b1);
                        S_hop_evt <= 1'b1;
                    end else begin
                        S_hop_cnt <= S_hop_cnt + 1'b1;
                    end
                end
                S_wr <= (S_wr == (N-1)) ? {AW{1'b0}} : (S_wr + 1'b1);
            end
        end
    end

    // 环形缓冲同步读口（1 拍延迟）
    reg [AW-1:0] S_rd_addr;
    reg signed [FFT_INPUT_BITS-1:0] S_ring_q;
    always @(posedge I_clk) begin
        S_ring_q <= S_ring[S_rd_addr];
    end

    // ================================================================
    // 2) Hann 窗 ROM（半表）+ 加窗乘法
    //    表索引对称：n<512 用 n；n>=512 用 1023-n（n 为采集序号）
    // ================================================================
    reg [AW-1:0] S_hann_addr;
    wire [FFT_COEF_BITS-1:0] S_hann_q;
    hann_window_rom u_hann (
        .clka (I_clk),
        .addra(S_hann_addr[8:0]),   // ROM 半表 512 点
        .doa  (S_hann_q)
    );
    wire signed [FFT_COEF_BITS-1:0] hq_s = S_hann_q;

    // 加窗对称映射（半表）：addr = n<512 ? n : 1023-n

    // 加窗：win = (sample * hann + 2^(C-2)) >>> (C-1)  （Q1.15 四舍五入）
    wire signed [FFT_INPUT_BITS+FFT_COEF_BITS-1:0] win_prod =
        $signed(S_ring_q) * $signed(hq_s);
    wire signed [FFT_INPUT_BITS+FFT_COEF_BITS-1:0] win_rs =
        (win_prod + (1 << (FFT_COEF_BITS-2))) >>> (FFT_COEF_BITS-1);
    wire signed [FFT_INPUT_BITS-1:0] win_s16 =
        win_rs[FFT_INPUT_BITS-1:0];
    wire signed [FFT_INTERNAL_BITS-1:0] S_win_ext =
        {{(FFT_INTERNAL_BITS-FFT_INPUT_BITS){win_s16[FFT_INPUT_BITS-1]}},
          win_s16};

    // 装载按位反转地址，使 FFT 输出自然 bin 序
    function [AW-1:0] bitrev;
        input [AW-1:0] v;
        integer i;
        begin
            bitrev = {AW{1'b0}};
            for (i = 0; i < AW; i = i + 1)
                bitrev[i] = v[AW-1-i];
        end
    endfunction

    // ================================================================
    // 3) FFT 引擎
    // ================================================================
    reg                                   S_lw_en;
    reg [AW-1:0]                          S_lw_addr;
    reg signed [FFT_INTERNAL_BITS-1:0]    S_lw_re;
    reg signed [FFT_INTERNAL_BITS-1:0]    S_lw_im;
    reg                                   S_go;
    reg [AW-1:0]                          S_rd_fft;
    wire                                  S_fft_busy;
    wire                                  S_fft_done;
    wire signed [FFT_INTERNAL_BITS-1:0]   S_fft_re;
    wire signed [FFT_INTERNAL_BITS-1:0]   S_fft_im;

    fft_1024_engine #(
        .LOG2_N    (FFT_LOG2_N),
        .DATA_BITS (FFT_INTERNAL_BITS),
        .COEF_BITS (FFT_COEF_BITS)
    ) u_fft (
        .I_clk     (I_clk),
        .I_rst     (I_rst),
        .I_lw_en   (S_lw_en),
        .I_lw_addr (S_lw_addr),
        .I_lw_re   (S_lw_re),
        .I_lw_im   (S_lw_im),
        .I_go      (S_go),
        .O_busy    (S_fft_busy),
        .O_done    (S_fft_done),
        .I_rd_addr (S_rd_fft),
        .O_rd_re   (S_fft_re),
        .O_rd_im   (S_fft_im)
    );

    // ================================================================
    // 4) 整数平方根（sqrt(Re^2+Im^2)）
    // ================================================================
    reg [SQR_IN-1:0] S_sqrt_val;
    reg              S_sqrt_start;
    wire             S_sqrt_busy;
    wire             S_sqrt_done;
    wire [MAG_BITS-1:0] S_sqrt_res;
    sqrt_integer #(
        .IN_WIDTH  (SQR_IN),
        .OUT_WIDTH (MAG_BITS)
    ) u_sqrt (
        .I_clk    (I_clk),
        .I_rst    (I_rst),
        .I_start  (S_sqrt_start),
        .I_value  (S_sqrt_val),
        .O_busy   (S_sqrt_busy),
        .O_done   (S_sqrt_done),
        .O_result (S_sqrt_res)
    );

    // 平方和（Re,Im 26bit 有符号 -> <=2^51，52bit 无符号送 sqrt）
    wire signed [SQR_IN-1:0] re2 = $signed(S_fft_re) * $signed(S_fft_re);
    wire signed [SQR_IN-1:0] im2 = $signed(S_fft_im) * $signed(S_fft_im);
    wire [SQR_IN-1:0] magsum = re2 + im2;

    // 带内峰值比较用 8bit: mag8 = sat8(sqrt结果 >> LOG_SHIFT)(与末级旧逻辑同式)
    wire [7:0] mag8 = ((S_sqrt_res >> SPECTRUM_LOG_SHIFT) > 8'd255) ?
                      8'd255 : S_sqrt_res[SPECTRUM_LOG_SHIFT +: 8];

    // ================================================================
    // 5) Ordered-bin peak reduction and indexed height publication.
    // Quantization and the exact sqrt magnitude are unchanged.
    // ================================================================
    // Bins are visited in monotonically increasing band order.
    // Keep one peak and stream the completed band; the legacy packed output
    // is a shift register and is eliminated when its port is unused.
    reg [7:0] S_peak;
    reg [7:0] S_completed_peak;
    // ================================================================
    // 6) 主状态机
    // ================================================================
    localparam ST_IDLE  = 4'd0;
    localparam ST_LOAD  = 4'd1;   // 加窗装载（4 拍/样本）
    localparam ST_GO    = 4'd2;   // 启动 FFT
    localparam ST_WAIT  = 4'd3;   // 等 FFT
    localparam ST_MAG   = 4'd4;   // 逐 bin 求幅度
    localparam ST_SCALE = 4'd5;   // 峰值->柱高

    reg [3:0] S_state;
    reg [AW-1:0] S_ld_idx;
    reg [1:0]    S_ld_st;         // 0..3
    reg [AW-1:0] S_bin;           // bin 1..(N>>1)=512
    reg [1:0]    S_mst;           // 0..3
    reg [6:0]    S_band;          // 频带号 0..63

    // 频带号 = (bin-1) >> BAND_SHIFT
    function [5:0] display_band;
        input [9:0] b;
        begin
            case (b)
                10'd2: display_band = 6'd0;
                10'd3: display_band = 6'd0;
                10'd4: display_band = 6'd0;
                10'd5: display_band = 6'd1;
                10'd6: display_band = 6'd1;
                10'd7: display_band = 6'd1;
                10'd8: display_band = 6'd2;
                10'd9: display_band = 6'd2;
                10'd10: display_band = 6'd3;
                10'd11: display_band = 6'd3;
                10'd12: display_band = 6'd3;
                10'd13: display_band = 6'd4;
                10'd14: display_band = 6'd4;
                10'd15: display_band = 6'd4;
                10'd16: display_band = 6'd5;
                10'd17: display_band = 6'd5;
                10'd18: display_band = 6'd6;
                10'd19: display_band = 6'd6;
                10'd20: display_band = 6'd6;
                10'd21: display_band = 6'd7;
                10'd22: display_band = 6'd7;
                10'd23: display_band = 6'd7;
                10'd24: display_band = 6'd8;
                10'd25: display_band = 6'd8;
                10'd26: display_band = 6'd9;
                10'd27: display_band = 6'd9;
                10'd28: display_band = 6'd9;
                10'd29: display_band = 6'd10;
                10'd30: display_band = 6'd10;
                10'd31: display_band = 6'd10;
                10'd32: display_band = 6'd11;
                10'd33: display_band = 6'd11;
                10'd34: display_band = 6'd12;
                10'd35: display_band = 6'd12;
                10'd36: display_band = 6'd12;
                10'd37: display_band = 6'd13;
                10'd38: display_band = 6'd13;
                10'd39: display_band = 6'd13;
                10'd40: display_band = 6'd14;
                10'd41: display_band = 6'd14;
                10'd42: display_band = 6'd15;
                10'd43: display_band = 6'd15;
                10'd44: display_band = 6'd15;
                10'd45: display_band = 6'd16;
                10'd46: display_band = 6'd16;
                10'd47: display_band = 6'd16;
                10'd48: display_band = 6'd17;
                10'd49: display_band = 6'd17;
                10'd50: display_band = 6'd18;
                10'd51: display_band = 6'd18;
                10'd52: display_band = 6'd18;
                10'd53: display_band = 6'd19;
                10'd54: display_band = 6'd19;
                10'd55: display_band = 6'd19;
                10'd56: display_band = 6'd20;
                10'd57: display_band = 6'd20;
                10'd58: display_band = 6'd21;
                10'd59: display_band = 6'd21;
                10'd60: display_band = 6'd21;
                10'd61: display_band = 6'd22;
                10'd62: display_band = 6'd22;
                10'd63: display_band = 6'd22;
                10'd64: display_band = 6'd23;
                10'd65: display_band = 6'd23;
                10'd66: display_band = 6'd24;
                10'd67: display_band = 6'd24;
                10'd68: display_band = 6'd24;
                10'd69: display_band = 6'd25;
                10'd70: display_band = 6'd25;
                10'd71: display_band = 6'd25;
                10'd72: display_band = 6'd26;
                10'd73: display_band = 6'd26;
                10'd74: display_band = 6'd27;
                10'd75: display_band = 6'd27;
                10'd76: display_band = 6'd27;
                10'd77: display_band = 6'd28;
                10'd78: display_band = 6'd28;
                10'd79: display_band = 6'd28;
                10'd80: display_band = 6'd29;
                10'd81: display_band = 6'd29;
                10'd82: display_band = 6'd30;
                10'd83: display_band = 6'd30;
                10'd84: display_band = 6'd30;
                10'd85: display_band = 6'd31;
                10'd86: display_band = 6'd31;
                10'd87: display_band = 6'd32;
                10'd88: display_band = 6'd32;
                10'd89: display_band = 6'd32;
                10'd90: display_band = 6'd33;
                10'd91: display_band = 6'd33;
                10'd92: display_band = 6'd33;
                10'd93: display_band = 6'd34;
                10'd94: display_band = 6'd34;
                10'd95: display_band = 6'd35;
                10'd96: display_band = 6'd35;
                10'd97: display_band = 6'd35;
                10'd98: display_band = 6'd36;
                10'd99: display_band = 6'd36;
                10'd100: display_band = 6'd36;
                10'd101: display_band = 6'd37;
                10'd102: display_band = 6'd37;
                10'd103: display_band = 6'd38;
                10'd104: display_band = 6'd38;
                10'd105: display_band = 6'd38;
                10'd106: display_band = 6'd39;
                10'd107: display_band = 6'd39;
                10'd108: display_band = 6'd39;
                10'd109: display_band = 6'd40;
                10'd110: display_band = 6'd40;
                10'd111: display_band = 6'd41;
                10'd112: display_band = 6'd41;
                10'd113: display_band = 6'd41;
                10'd114: display_band = 6'd42;
                10'd115: display_band = 6'd42;
                10'd116: display_band = 6'd42;
                10'd117: display_band = 6'd43;
                10'd118: display_band = 6'd43;
                10'd119: display_band = 6'd44;
                10'd120: display_band = 6'd44;
                10'd121: display_band = 6'd44;
                10'd122: display_band = 6'd45;
                10'd123: display_band = 6'd45;
                10'd124: display_band = 6'd45;
                10'd125: display_band = 6'd46;
                10'd126: display_band = 6'd46;
                10'd127: display_band = 6'd47;
                10'd128: display_band = 6'd47;
                10'd129: display_band = 6'd47;
                10'd130: display_band = 6'd48;
                10'd131: display_band = 6'd48;
                10'd132: display_band = 6'd48;
                10'd133: display_band = 6'd49;
                10'd134: display_band = 6'd49;
                10'd135: display_band = 6'd50;
                10'd136: display_band = 6'd50;
                10'd137: display_band = 6'd50;
                10'd138: display_band = 6'd51;
                10'd139: display_band = 6'd51;
                10'd140: display_band = 6'd51;
                10'd141: display_band = 6'd52;
                10'd142: display_band = 6'd52;
                10'd143: display_band = 6'd53;
                10'd144: display_band = 6'd53;
                10'd145: display_band = 6'd53;
                10'd146: display_band = 6'd54;
                10'd147: display_band = 6'd54;
                10'd148: display_band = 6'd54;
                10'd149: display_band = 6'd55;
                10'd150: display_band = 6'd55;
                10'd151: display_band = 6'd56;
                10'd152: display_band = 6'd56;
                10'd153: display_band = 6'd56;
                10'd154: display_band = 6'd57;
                10'd155: display_band = 6'd57;
                10'd156: display_band = 6'd57;
                10'd157: display_band = 6'd58;
                10'd158: display_band = 6'd58;
                10'd159: display_band = 6'd59;
                10'd160: display_band = 6'd59;
                10'd161: display_band = 6'd59;
                10'd162: display_band = 6'd60;
                10'd163: display_band = 6'd60;
                10'd164: display_band = 6'd60;
                10'd165: display_band = 6'd61;
                10'd166: display_band = 6'd61;
                10'd167: display_band = 6'd62;
                10'd168: display_band = 6'd62;
                10'd169: display_band = 6'd62;
                10'd170: display_band = 6'd63;
                10'd171: display_band = 6'd63;
                default: display_band = 6'd0;
            endcase
        end
    endfunction
    wire [5:0] band_of_bin = display_band(S_bin);

    // 频带峰值(已是 >>LOG_SHIFT 饱和 8bit)直接送 log 表
    wire [7:0] col_m8 = S_completed_peak;
    wire band_last = (S_bin==10'd171) || (display_band(S_bin+10'd1)!=band_of_bin);

    // STM32 参考 log 表: h = min(31, floor(8.1*ln(m+3))), m=0..255 (≥43 饱和 31)
    function [5:0] map_h;
        input [7:0] m;
        begin
            if (m >= 8'd43)      map_h = 6'd31;
            else if (m >= 8'd38) map_h = 6'd30;
            else if (m >= 8'd33) map_h = 6'd29;
            else if (m >= 8'd29) map_h = 6'd28;
            else if (m >= 8'd26) map_h = 6'd27;
            else if (m >= 8'd22) map_h = 6'd26;
            else if (m >= 8'd19) map_h = 6'd25;
            else if (m >= 8'd17) map_h = 6'd24;
            else if (m >= 8'd15) map_h = 6'd23;
            else if (m >= 8'd13) map_h = 6'd22;
            else if (m >= 8'd11) map_h = 6'd21;
            else if (m >= 8'd9)  map_h = 6'd20;
            else if (m >= 8'd8)  map_h = 6'd19;
            else if (m >= 8'd7)  map_h = 6'd18;
            else if (m >= 8'd6)  map_h = 6'd17;
            else if (m >= 8'd5)  map_h = 6'd16;
            else if (m >= 8'd4)  map_h = 6'd15;
            else if (m >= 8'd3)  map_h = 6'd14;
            else if (m >= 8'd2)  map_h = 6'd13;
            else if (m >= 8'd1)  map_h = 6'd11;
            else                 map_h = 6'd8;
        end
    endfunction


    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_state      <= ST_IDLE;
            S_ld_idx     <= {AW{1'b0}};
            S_ld_st      <= 2'd0;
            S_bin        <= {AW{1'b0}};
            S_mst        <= 2'd0;
            S_band       <= 6'd0;
            S_rd_addr    <= {AW{1'b0}};
            S_hann_addr  <= {AW{1'b0}};
            S_lw_en      <= 1'b0;
            S_lw_addr    <= {AW{1'b0}};
            S_lw_re      <= {FFT_INTERNAL_BITS{1'b0}};
            S_lw_im      <= {FFT_INTERNAL_BITS{1'b0}};
            S_go         <= 1'b0;
            S_rd_fft     <= {AW{1'b0}};
            S_sqrt_start <= 1'b0;
            S_sqrt_val   <= {SQR_IN{1'b0}};
            O_heights_valid <= 1'b0;
            O_analysis_busy <= 1'b0;
            S_pend       <= 1'b0;      // 单一驱动：仅主 FSM 复位/锁存/消费
            S_peak <= 0;
            S_completed_peak <= 0;
            O_heights <= 0;
            O_band_valid <= 0;
            O_band_addr <= 0;
            O_band_height <= 0;
        end else begin
            O_heights_valid <= 1'b0;
            O_band_valid <= 1'b0;
            // 事件锁存：任意时刻到来的 hop 触发都记录到 S_pend 不丢失，
            // 统一由 ST_IDLE 消费（此处是 S_pend 在本块的唯一驱动点）。
            if (S_hop_evt) S_pend <= 1'b1;
            case (S_state)
                // ---------------- IDLE ----------------
                ST_IDLE: begin
                    O_analysis_busy <= 1'b0;
                    S_lw_en <= 1'b0;
                    S_go    <= 1'b0;
                    if (S_pend) begin
                        // 消费待处理触发；若同一拍又来了新 S_hop_evt，
                        // 则保留 S_pend 给下一帧（不丢事件、不会漏触发）
                        if (!S_hop_evt) S_pend <= 1'b0;
                        S_ld_idx <= {AW{1'b0}};
                        S_ld_st  <= 2'd0;
                        S_state  <= ST_LOAD;
                        O_analysis_busy <= 1'b1;
                    end
                end

                // ---------------- LOAD：加窗 -> 写 Re(bitrev)，Im=0 ----------------
                ST_LOAD: begin
                    case (S_ld_st)
                        2'd0: begin
                            // 按时间顺序读窗口：addr = (base+n)（10bit 自动回绕）
                            // Hann 半表按对称映射到 0..511
                            S_rd_addr   <= S_base + S_ld_idx;
                            S_hann_addr <= (S_ld_idx < (N>>1)) ?
                                           S_ld_idx : ((N-1) - S_ld_idx);
                            S_lw_en     <= 1'b0;
                            S_ld_st     <= 2'd1;
                        end
                        2'd1: begin
                            S_ld_st <= 2'd2;            // 等采样/窗出数
                        end
                        2'd2: begin
                            // 数据已有效（S_sample_q/S_hann_q），锁存写数据
                            S_lw_en   <= 1'b1;
                            S_lw_addr <= bitrev(S_ld_idx);
                            S_lw_re   <= S_win_ext;
                            S_lw_im   <= {FFT_INTERNAL_BITS{1'b0}};
                            S_ld_st   <= 2'd3;
                        end
                        2'd3: begin
                            // 引擎已在上一拍写入，清使能并推进
                            S_lw_en <= 1'b0;
                            if (S_ld_idx == (N-1)) begin
                                S_state <= ST_GO;
                            end else begin
                                S_ld_idx <= S_ld_idx + 1'b1;
                                S_ld_st  <= 2'd0;
                            end
                        end
                        default: S_ld_st <= 2'd0;
                    endcase
                end

                // ---------------- GO ----------------
                ST_GO: begin
                    S_go    <= 1'b1;
                    S_state <= ST_WAIT;
                end

                // ---------------- WAIT：等 FFT ----------------
                ST_WAIT: begin
                    S_go <= 1'b0;
                    if (S_fft_done) begin
                        S_bin  <= 10'd2;      // 从 bin1 开始（跳过 DC）
                        S_mst  <= 2'd0;
                        S_state<= ST_MAG;
                    end
                end

                // ---------------- MAG ----------------
                ST_MAG: begin
                    case (S_mst)
                        2'd0: begin
                            S_rd_fft <= S_bin;           // 请求 bin（1 拍出数）
                            S_mst    <= 2'd1;
                        end
                        2'd1: S_mst <= 2'd2;             // 等引擎出数
                        2'd2: begin                      // 出数有效，发起 sqrt
                            S_sqrt_start <= 1'b1;
                            S_sqrt_val   <= magsum;
                            S_mst        <= 2'd3;
                        end
                        2'd3: begin                      // 轮询 sqrt 完成
                            S_sqrt_start <= 1'b0;
                            if (S_sqrt_done) begin
                                // 频带取最大幅度: band = (bin-1)>>3, bin1..512 -> 0..63
                                if (band_last || S_bin == 10'd171) begin
                                    S_completed_peak <= (mag8 > S_peak) ? mag8 : S_peak;
                                    S_peak <= 0;
                                    S_band <= band_of_bin;
                                    S_state <= ST_SCALE;
                                end else begin
                                    if (mag8 > S_peak) S_peak <= mag8;
                                    S_bin <= S_bin + 1'b1;
                                    S_mst <= 0;
                                end
                            end
                        end
                        default: S_mst <= 2'd0;
                    endcase
                end

                // ---------------- SCALE：频带峰值 >> LOG_SHIFT -> log 表 -> 6bit 柱高 ----------------
                // (显示层 log 压缩, 参考 STM32 logf(x+3)*8.1; 幅度层仍为精确 sqrt 峰值)
                ST_SCALE: begin
                    O_heights <= {map_h(col_m8), O_heights[SPECTRUM_BIN_NUM*6-1:6]};
                    O_band_valid <= 1'b1;
                    O_band_addr <= S_band[5:0];
                    O_band_height <= map_h(col_m8);
                    if (S_bin == 10'd171) begin
                        S_state <= ST_IDLE;
                        O_heights_valid <= 1'b1;
                    end else begin
                        S_bin <= S_bin + 1'b1;
                        S_mst <= 0;
                        S_state <= ST_MAG;
                    end
                end

                default: S_state <= ST_IDLE;
            endcase
        end
    end

endmodule
