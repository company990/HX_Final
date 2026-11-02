//=====================================================================
// hub75_controller.v  (Phase3: 独立 HUB75 频谱屏)
//
//  64x32, 1/16 扫描, 4-bit 地址 A/B/C/D。
//  单趟扫描: 地址 a(0..15)
//     - 上半区行 y=a  -> R1/G1/B1
//     - 下半区行 y=a+16 -> R2/G2/B2
//  颜色: 第一版固定白色 => R=G=B=lit；柱外 RGB=0。OE 只管整行显示时间。
//  柱自下而上 (y 顶=0): 像素 lit 当 y >= (HUB75_HEIGHT - height[col])。
//
//  无 64x32 帧存(BRAM 保持低位): 扫描时按 (row, col, height[col]) 现算。
//  亮度: OE 使能保持 HUB75_BRIGHTNESS 个 I_clk (计数器, 非 busy-wait)。
//  时序: OE灭 -> 设地址 -> CLK x 64 -> LAT -> OE亮(BRIGHTNESS) -> OE灭 -> 下一行
//
//  所有工程参数由 top 传入。
//=====================================================================
module hub75_controller #(
    parameter integer HUB75_WIDTH      = 64,   // 列数
    parameter integer HUB75_SCAN_ROWS  = 16,   // 扫描地址行数(1/16)
    parameter integer HUB75_HEIGHT     = 32,   // 行数(上下半区各 SCAN_ROWS)
    parameter integer HUB75_CLK_DIV    = 2,    // 每列移位的 I_clk 数(>=2)
    parameter integer HUB75_BRIGHTNESS = 24    // 每行 OE 亮保持的 I_clk 数
)(
    input  wire                           I_clk,   // 25MHz video_clk
    input  wire                           I_rst,   // 高有效
    input  wire                           I_load,  // 新高度提交(采样 I_height 到内部快照)
    input  wire [HUB75_WIDTH*6-1:0]       I_height,// 64 柱 x 6bit (0..HUB75_HEIGHT)

    output reg                            O_R1, O_G1, O_B1,
    output reg                            O_R2, O_G2, O_B2,
    output reg                            O_A, O_B, O_C, O_D,
    output reg                            O_CLK, O_LAT, O_OE
);
    localparam integer DIV = HUB75_CLK_DIV < 2 ? 2 : HUB75_CLK_DIV;
    localparam integer SHIFT_TOTAL = HUB75_WIDTH * DIV;   // 每行移位总 I_clk

    // 高度快照(64 x 6bit)
    reg [5:0] h [0:HUB75_WIDTH-1];
    integer k;
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            for (k = 0; k < HUB75_WIDTH; k = k + 1) h[k] <= 6'd0;
        end else if (I_load) begin
            for (k = 0; k < HUB75_WIDTH; k = k + 1)
                h[k] <= I_height[k*6 +: 6];
        end
    end

    // 行地址 / 状态 / 子计数
    localparam [1:0] S_ADDR  = 2'd0;
    localparam [1:0] S_SHIFT = 2'd1;
    localparam [1:0] S_LAT   = 2'd2;
    localparam [1:0] S_DWELL = 2'd3;
    reg [1:0] st;
    reg [5:0] row;      // 0..SCAN_ROWS-1
    reg [15:0] sub;     // 通用子计数

    reg  o_r1,o_g1,o_b1,o_r2,o_g2,o_b2;
    reg  o_a,o_b,o_c,o_d;
    reg  o_clk,o_lat,o_oe;

    // 组合: 当前列(移位子计数 sub 期间推导) 的亮灭
    // col = sub/DIV ; 仅当 sub 处于有效列区间时取 h[col]
    wire [6:0] col    = sub[15:0] / DIV;              // 0..63
    wire [5:0] hval   = (sub < SHIFT_TOTAL) ? h[col] : 6'd0;
    wire [5:0] lim    = HUB75_HEIGHT[5:0] - hval;     // 32 - h
    wire       top_lit    = (sub < SHIFT_TOTAL) && (row >= lim);
    wire       bottom_lit = (sub < SHIFT_TOTAL) && ((row + 6'd16) >= lim);

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            st <= S_ADDR; row <= 6'd0; sub <= 16'd0;
            o_r1<=0;o_g1<=0;o_b1<=0;o_r2<=0;o_g2<=0;o_b2<=0;
            o_a<=0;o_b<=0;o_c<=0;o_d<=0;
            o_clk<=0;o_lat<=0;o_oe<=1'b1;             // OE=1 => 灭
        end else begin
            case (st)
            // ---------- 设地址, 灭显示 ----------
            S_ADDR: begin
                o_oe <= 1'b1;                         // 灭
                o_lat <= 1'b0;
                o_clk <= 1'b0;
                o_a <= row[0]; o_b <= row[1]; o_c <= row[2]; o_d <= row[3];
                sub <= 16'd0;
                st  <= S_SHIFT;
            end
            // ---------- 移位 64 列 ----------
            S_SHIFT: begin
                if (sub == SHIFT_TOTAL) begin
                    o_clk <= 1'b0;
                    st <= S_LAT; sub <= 16'd0;
                end else begin
                    if ((sub % DIV) == 0) begin
                        // 数据稳定(此时 O_CLK 为低), 白色 => 6 线同值
                        o_r1 <= top_lit;    o_g1 <= top_lit;    o_b1 <= top_lit;
                        o_r2 <= bottom_lit; o_g2 <= bottom_lit; o_b2 <= bottom_lit;
                        o_clk <= 1'b0;
                    end else if ((sub % DIV) == (DIV-1)) begin
                        o_clk <= 1'b1;                   // 上升沿采样
                    end
                    sub <= sub + 1'b1;
                end
            end
            // ---------- 锁存 ----------
            S_LAT: begin
                if (sub == 0) begin
                    o_lat <= 1'b1; sub <= 1'b1;
                end else begin
                    o_lat <= 1'b0;
                    o_oe  <= 1'b0;                       // 打开显示
                    sub <= 16'd0;
                    st  <= S_DWELL;
                end
            end
            // ---------- 亮度保持 ----------
            S_DWELL: begin
                if (sub < HUB75_BRIGHTNESS) begin
                    sub <= sub + 1'b1;
                end else begin
                    o_oe <= 1'b1;                        // 灭, 准备下一行
                    if (row == (HUB75_SCAN_ROWS-1)) row <= 6'd0;
                    else                               row <= row + 1'b1;
                    st <= S_ADDR;
                end
            end
            default: st <= S_ADDR;
            endcase
        end
    end

    always @* begin
        O_R1 = o_r1; O_G1 = o_g1; O_B1 = o_b1;
        O_R2 = o_r2; O_G2 = o_g2; O_B2 = o_b2;
        O_A = o_a;   O_B = o_b;   O_C = o_c;   O_D = o_d;
        O_CLK = o_clk; O_LAT = o_lat; O_OE = o_oe;
    end
endmodule
