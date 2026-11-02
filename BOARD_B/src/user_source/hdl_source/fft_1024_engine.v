//=====================================================================
// fft_1024_engine.v
// 自研 1024 点 radix-2 离散傅里叶变换引擎（DIT，原位计算），不使用任何 IP。
//
// 算法：
//   - 外部把加窗后的时域样本按“位反转地址”写入 Re RAM（Im 写 0），
//     使 FFT 输出为自然 bin 顺序（bin k 即频率 k*Fs/N）。
//   - 标准 Cooley-Tukey DIT：10 级蝶形，每级 512 个蝶形。
//     第 s 级（len=2^s, half=len/2, tstep=2^(LOG2_N-s)）：
//         p = pb+j, q = p+half, 旋转因子索引 tw = j*tstep (0..511)
//         X[p] = u+v,  X[q] = u-v,  v = X[q] * W_1024^tw
//   - 蝶形微码（每蝶约 6 拍，25MHz 下完成整帧仅 ~2ms << 21.3ms）：
//         ph0: 读 X[p]        ph1: 读 X[q] 且锁存 u/旋转因子
//         ph2: 复数乘法 v      ph3: u+v / u-v
//         ph4: 写回 X[p]      ph5: 写回 X[q] 并推进计数器
//
// 位宽策略：
//   DATA_BITS = FFT_INPUT_BITS + LOG2_N（默认 16+10=26）有符号，
//   保证任意 16bit 输入 10 级蝶形都不溢出（|X[k]| <= N*max|x| <= 2^25）。
//   COEF_BITS = 16（旋转因子 Q1.15），乘积取整/饱和回 DATA_BITS。
//
// 时钟域：video_clk(25MHz)；与 48kHz 采样率无关（输入由 I_go 使能）。
// 数据通路全寄存器化，组合路径仅为 26x16 乘法+加法，25MHz 时序收敛容易。
//=====================================================================
module fft_1024_engine #(
    parameter integer LOG2_N    = 10,          // 1024 点
    parameter integer DATA_BITS = 26,          // 实/虚部定点位宽(有符号)
    parameter integer COEF_BITS = 16           // 旋转因子位宽 Q1.15
)(
    input  wire                             I_clk,
    input  wire                             I_rst,     // 高有效

    // ---- 数据装载口（仅引擎 IDLE 时有效，按位反转地址写入）----
    input  wire                             I_lw_en,
    input  wire [LOG2_N-1:0]                I_lw_addr,
    input  wire signed [DATA_BITS-1:0]      I_lw_re,
    input  wire signed [DATA_BITS-1:0]      I_lw_im,

    // ---- 控制 ----
    input  wire                             I_go,      // 装载完成后启动
    output reg                              O_busy,
    output reg                              O_done,    // 1 拍完成脉冲

    // ---- 结果读口（IDLE 后，自然 bin 序，同步读 1 拍延迟）----
    input  wire [LOG2_N-1:0]                I_rd_addr,
    output wire signed [DATA_BITS-1:0]      O_rd_re,
    output wire signed [DATA_BITS-1:0]      O_rd_im
);

    localparam integer N  = (1 << LOG2_N);             // 1024
    localparam integer AW = LOG2_N;                    // 10
    localparam integer LW = LOG2_N + 1;                // 11（len/计数用）
    localparam integer PW = DATA_BITS + COEF_BITS;     // 42
    localparam integer PWX = PW + 1;                   // 43（乘法/加法余量）

    localparam ST_IDLE = 1'b0;
    localparam ST_RUN  = 1'b1;

    // ============ 状态/计数器 ============
    reg              S_state;
    reg [3:0]        S_ph;          // 蝶形微码 0..5
    reg [3:0]        S_stage;       // 1..LOG2_N
    reg [LW-1:0]     S_len;         // 当前级块长 2^stage
    reg [LW-1:0]     S_half;        // len/2
    reg [LW-1:0]     S_tstep;       // 旋转因子步进 2^(LOG2_N-stage)
    reg [LW-1:0]     S_pb;          // 当前块基址
    reg [AW-1:0]     S_j;           // 块内蝶形序号 0..half-1
    reg [LW-1:0]     S_p, S_q;      // 当前蝶形两操作数地址
    reg [AW:0]       S_tw;          // 旋转因子索引 0..511

    // ============ RAM 访问选择（组合） ============
    reg [AW-1:0]                 c_addr;
    reg                          c_we;
    reg signed [DATA_BITS-1:0]   c_wre;
    reg signed [DATA_BITS-1:0]   c_wim;
    wire fft_run = (S_state == ST_RUN);

    // ============ 蝶形数据通路寄存器 ============
    reg signed [DATA_BITS-1:0] S_U_re, S_U_im;            // u = X[p]
    reg signed [COEF_BITS-1:0] S_Tw_c, S_Tw_s;            // 旋转因子
    reg signed [DATA_BITS-1:0] S_V_re, S_V_im;            // v = X[q]*W
    reg signed [DATA_BITS-1:0] S_W1_re, S_W1_im;          // u+v
    reg signed [DATA_BITS-1:0] S_W2_re, S_W2_im;          // u-v

    always @(*) begin
        if (fft_run) begin
            c_addr = {AW{1'b0}};
            c_we   = 1'b0;
            c_wre  = {DATA_BITS{1'b0}};
            c_wim  = {DATA_BITS{1'b0}};
            case (S_ph)
                4'd0: c_addr = S_p[AW-1:0];
                4'd1: c_addr = S_q[AW-1:0];
                4'd2: c_addr = S_q[AW-1:0];
                4'd3: c_addr = S_q[AW-1:0];
                4'd4: begin c_addr = S_p[AW-1:0]; c_we = 1'b1;
                            c_wre = S_W1_re; c_wim = S_W1_im; end
                4'd5: begin c_addr = S_q[AW-1:0]; c_we = 1'b1;
                            c_wre = S_W2_re; c_wim = S_W2_im; end
                default: ;
            endcase
        end else begin
            c_addr = I_lw_en ? I_lw_addr : I_rd_addr;
            c_we   = I_lw_en;
            c_wre  = I_lw_re;
            c_wim  = I_lw_im;
        end
    end

    // ============ 复数工作 RAM（推断为块 RAM，单口同步读） ============
    reg signed [DATA_BITS-1:0] S_mem_re [0:N-1];
    reg signed [DATA_BITS-1:0] S_mem_im [0:N-1];
    reg signed [DATA_BITS-1:0] S_q_re;   // 读数据（1 拍延迟）
    reg signed [DATA_BITS-1:0] S_q_im;

    always @(posedge I_clk) begin
        if (c_we)
            S_mem_re[c_addr] <= c_wre;
        S_q_re <= S_mem_re[c_addr];
    end
    always @(posedge I_clk) begin
        if (c_we)
            S_mem_im[c_addr] <= c_wim;
        S_q_im <= S_mem_im[c_addr];
    end

    // ============ 旋转因子 ROM ============
    wire [COEF_BITS-1:0] S_tw_cos_do;
    wire [COEF_BITS-1:0] S_tw_sin_do;
    wire [AW-1:0] tw_addr = fft_run ? S_tw[AW-1:0] : {AW{1'b0}};

    fft_twiddle_rom u_tw_rom (
        .clka    (I_clk),
        .addra   (tw_addr[8:0]),
        .doa_cos (S_tw_cos_do),
        .doa_sin (S_tw_sin_do)
    );

    // ============ 蝶形数据通路（组合） ============
    // ---- 复数乘法组合：v = (a+jb)*(c - js) = (ac+bs)+j(bc-as) ----
    wire signed [DATA_BITS-1:0] ar = S_q_re;
    wire signed [DATA_BITS-1:0] ai = S_q_im;
    wire signed [PWX-1:0] arx = {{(PWX-DATA_BITS){ar[DATA_BITS-1]}}, ar};
    wire signed [PWX-1:0] aix = {{(PWX-DATA_BITS){ai[DATA_BITS-1]}}, ai};
    wire signed [PWX-1:0] cxs = {{(PWX-COEF_BITS){S_Tw_c[COEF_BITS-1]}}, S_Tw_c};
    wire signed [PWX-1:0] sxs = {{(PWX-COEF_BITS){S_Tw_s[COEF_BITS-1]}}, S_Tw_s};

    wire signed [PWX-1:0] pr_ac = arx * cxs;
    wire signed [PWX-1:0] pr_bs = aix * sxs;
    wire signed [PWX-1:0] pr_bc = aix * cxs;
    wire signed [PWX-1:0] pr_as = arx * sxs;

    // Q1.15 取整：商 = (x * coef) / 2^15，四舍五入加 2^(COEF_BITS-2) = 2^14
    localparam signed [PWX-1:0] RNDC =
        {{(PWX-COEF_BITS+1){1'b0}}, 1'b1, {(COEF_BITS-2){1'b0}}};

    wire signed [PWX-1:0] sum_re = pr_ac + pr_bs;
    wire signed [PWX-1:0] sum_im = pr_bc - pr_as;
    wire signed [PWX-1:0] shr_re = (sum_re + RNDC) >>> (COEF_BITS-1);
    wire signed [PWX-1:0] shr_im = (sum_im + RNDC) >>> (COEF_BITS-1);

    // 溢出检测并饱和回 DATA_BITS
    wire ovf_hi_re = ~shr_re[DATA_BITS-1] & (|shr_re[PWX-1:DATA_BITS]);
    wire ovf_lo_re =  shr_re[DATA_BITS-1] & (~&shr_re[PWX-1:DATA_BITS]);
    wire ovf_hi_im = ~shr_im[DATA_BITS-1] & (|shr_im[PWX-1:DATA_BITS]);
    wire ovf_lo_im =  shr_im[DATA_BITS-1] & (~&shr_im[PWX-1:DATA_BITS]);

    // W^0 = 1：直接旁路乘法，避免 0x7FFF(≈0.99997) 的量化衰减
    // 使 DC/整 bin 情况下残留误差为零级。
    wire S_tw_is0 = (S_tw == {(AW+1){1'b0}});

    wire signed [DATA_BITS-1:0] v_re_c =
        S_tw_is0 ? S_q_re :
        (ovf_hi_re ? {1'b0,{(DATA_BITS-1){1'b1}}} :
         ovf_lo_re ? {1'b1,{(DATA_BITS-1){1'b0}}} :
                     shr_re[DATA_BITS-1:0]);
    wire signed [DATA_BITS-1:0] v_im_c =
        S_tw_is0 ? S_q_im :
        (ovf_hi_im ? {1'b0,{(DATA_BITS-1){1'b1}}} :
         ovf_lo_im ? {1'b1,{(DATA_BITS-1){1'b0}}} :
                     shr_im[DATA_BITS-1:0]);

    // ---- u±v（扩 1 位求和再饱和） ----
    wire signed [DATA_BITS:0] uxr = {{1{S_U_re[DATA_BITS-1]}}, S_U_re};
    wire signed [DATA_BITS:0] uxi = {{1{S_U_im[DATA_BITS-1]}}, S_U_im};
    wire signed [DATA_BITS:0] vxr = {{1{S_V_re[DATA_BITS-1]}}, S_V_re};
    wire signed [DATA_BITS:0] vxi = {{1{S_V_im[DATA_BITS-1]}}, S_V_im};
    wire signed [DATA_BITS:0] w1s_re = uxr + vxr;
    wire signed [DATA_BITS:0] w1s_im = uxi + vxi;
    wire signed [DATA_BITS:0] w2s_re = uxr - vxr;
    wire signed [DATA_BITS:0] w2s_im = uxi - vxi;

    // 通用饱和：输入宽 DATA_BITS+1 -> 输出 DATA_BITS
    function automatic signed [DATA_BITS-1:0] f_clamp;
        input signed [DATA_BITS:0] x;
        begin
            if (x[DATA_BITS] != x[DATA_BITS-1]) begin
                if (x[DATA_BITS])
                    f_clamp = {1'b1, {(DATA_BITS-1){1'b0}}};
                else
                    f_clamp = {1'b0, {(DATA_BITS-1){1'b1}}};
            end else
                f_clamp = x[DATA_BITS-1:0];
        end
    endfunction

    wire signed [DATA_BITS-1:0] w1re_c = f_clamp(w1s_re);
    wire signed [DATA_BITS-1:0] w1im_c = f_clamp(w1s_im);
    wire signed [DATA_BITS-1:0] w2re_c = f_clamp(w2s_re);
    wire signed [DATA_BITS-1:0] w2im_c = f_clamp(w2s_im);

    // ============ 主状态机 ============
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_state <= ST_IDLE;
            S_ph    <= 4'd0;
            S_stage <= 4'd0;
            S_len   <= {LW{1'b0}};
            S_half  <= {LW{1'b0}};
            S_tstep <= {LW{1'b0}};
            S_pb    <= {LW{1'b0}};
            S_j     <= {AW{1'b0}};
            S_p     <= {LW{1'b0}};
            S_q     <= {LW{1'b0}};
            S_tw    <= {(AW+1){1'b0}};
            O_busy  <= 1'b0;
            O_done  <= 1'b0;
            S_U_re  <= {DATA_BITS{1'b0}};
            S_U_im  <= {DATA_BITS{1'b0}};
            S_Tw_c  <= {COEF_BITS{1'b0}};
            S_Tw_s  <= {COEF_BITS{1'b0}};
            S_V_re  <= {DATA_BITS{1'b0}};
            S_V_im  <= {DATA_BITS{1'b0}};
            S_W1_re <= {DATA_BITS{1'b0}};
            S_W1_im <= {DATA_BITS{1'b0}};
            S_W2_re <= {DATA_BITS{1'b0}};
            S_W2_im <= {DATA_BITS{1'b0}};
        end else begin
            O_done <= 1'b0;
            case (S_state)
                ST_IDLE: begin
                    O_busy <= 1'b0;
                    if (I_go) begin
                        O_busy  <= 1'b1;
                        S_state <= ST_RUN;
                        S_ph    <= 4'd0;
                        S_stage <= 4'd1;
                        S_len   <= 2;
                        S_half  <= 1;
                        S_tstep <= (N >> 1);
                        S_pb    <= {LW{1'b0}};
                        S_j     <= {AW{1'b0}};
                        S_tw    <= {(AW+1){1'b0}};
                        S_p     <= {LW{1'b0}};
                        S_q     <= 1;
                    end
                end

                ST_RUN: begin
                    case (S_ph)
                        4'd0: S_ph <= 4'd1;                       // 等 X[p]
                        4'd1: begin                                // 锁 u、旋转因子；读 X[q]
                            S_U_re <= S_q_re;
                            S_U_im <= S_q_im;
                            S_Tw_c <= S_tw_cos_do;
                            S_Tw_s <= S_tw_sin_do;
                            S_ph   <= 4'd2;
                        end
                        4'd2: begin                                // 复数乘 v
                            S_V_re <= v_re_c;
                            S_V_im <= v_im_c;
                            S_ph   <= 4'd3;
                        end
                        4'd3: begin                                // u±v
                            S_W1_re <= w1re_c;
                            S_W1_im <= w1im_c;
                            S_W2_re <= w2re_c;
                            S_W2_im <= w2im_c;
                            S_ph    <= 4'd4;
                        end
                        4'd4: S_ph <= 4'd5;                        // 写 X[p]
                        4'd5: begin                                // 写 X[q] 并推进
                            if (S_j < (S_half - 1)) begin
                                S_j   <= S_j + {{(AW-1){1'b0}},1'b1};
                                S_p   <= S_p + 1'b1;
                                S_q   <= S_q + 1'b1;
                                S_tw  <= S_tw + S_tstep;
                                S_ph  <= 4'd0;
                            end else if ((S_pb + S_len) >= N) begin
                                // 本级最后一块结束
                                if (S_stage == LOG2_N) begin
                                    O_done  <= 1'b1;
                                    O_busy  <= 1'b0;
                                    S_state <= ST_IDLE;
                                end else begin
                                    S_stage <= S_stage + 1'b1;
                                    S_len   <= S_len  << 1;
                                    S_half  <= S_half << 1;
                                    S_tstep <= S_tstep >> 1;
                                    S_pb    <= {LW{1'b0}};
                                    S_j     <= {AW{1'b0}};
                                    S_tw    <= {(AW+1){1'b0}};
                                    S_p     <= {LW{1'b0}};
                                    S_q     <= S_half << 1;
                                    S_ph    <= 4'd0;
                                end
                            end else begin
                                // 本级下一块
                                S_j   <= {AW{1'b0}};
                                S_tw  <= {(AW+1){1'b0}};
                                S_p   <= S_pb + S_len;
                                S_q   <= S_pb + S_len + S_half;
                                S_pb  <= S_pb + S_len;
                                S_ph  <= 4'd0;
                            end
                        end
                        default: S_ph <= 4'd0;
                    endcase
                end
                default: S_state <= ST_IDLE;
            endcase
        end
    end

    // ============ 读出口（自然 bin 序，1 拍延迟） ============
    assign O_rd_re = S_q_re;
    assign O_rd_im = S_q_im;

endmodule
