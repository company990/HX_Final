//=====================================================================
// spectrum_renderer.v (Phase 2)
// 频谱柱 RGB 叠加层（工作于 video_clk 25MHz）。
//
//  - 无面板背景：只有“柱体像素”才覆盖原图，其余像素一律透传 I_rgb_bg。
//  - 区域固定到右下角小区域(默认 X=464..623, Y=400..470，160x71)，
//    32 根柱按 BAR_WIDTH=3 / BAR_GAP=2 均匀排列（32*3+31*2=158 <= 160）。
//  - 高度平滑：target(每帧更新) -> current(每 video_clk 以 SMOOTH_STEP 逼近)。
//  - target 在 vsync 消隐期提交(防撕裂)；current 平滑到 target。
//
//  区域/柱宽/间距/平滑步长等全部由 top 传入。
//=====================================================================
module spectrum_renderer #(
    parameter integer SPECTRUM_BIN_NUM   = 32,
    parameter integer SPECTRUM_X_START   = 464,
    parameter integer SPECTRUM_X_END     = 623,   // 160 宽
    parameter integer SPECTRUM_Y_TOP     = 400,
    parameter integer SPECTRUM_Y_BOTTOM  = 470,   // 71 高
    parameter integer BAR_WIDTH          = 3,     // 柱宽
    parameter integer BAR_GAP            = 2,     // 柱间距
    parameter integer SPECTRUM_MAX_BAR   = 71,    // 柱高上限 = 区域高
    parameter integer SMOOTH_STEP        = 1,     // 每 video_clk 高度变化步长
    parameter integer SPECTRUM_DEBUG_BYPASS = 0   // 1=纯透传(诊断用)
)(
    input  wire                          I_clk,   // video_clk
    input  wire                          I_rst,   // 高有效
    input  wire                          I_vs,    // 与 I_rgb_bg 对齐
    input  wire                          I_de,
    input  wire [23:0]                   I_rgb_bg,// 背景(原图)RGB

    input  wire                          I_heights_valid,
    input  wire [SPECTRUM_BIN_NUM*8-1:0] I_heights,

    output wire [23:0]                   O_rgb
);

    localparam integer PITCH = BAR_WIDTH + BAR_GAP;  // =5

    // ============ 有效区坐标重建（同前，仅用于区域/柱判定） ============
    reg S_de_d;
    reg S_vs_d;
    reg [9:0] S_x;
    reg [9:0] S_y;
    reg       S_active_seen;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_de_d <= 1'b0; S_vs_d <= 1'b0;
            S_x    <= 10'd0; S_y <= 10'd0;
            S_active_seen <= 1'b0;
        end else begin
            S_de_d <= I_de;
            S_vs_d <= I_vs;
            if (I_vs != S_vs_d) begin
                S_y <= 10'd0; S_x <= 10'd0; S_active_seen <= 1'b0;
            end else if (I_de) begin
                if (!S_de_d) begin
                    if (S_active_seen) S_y <= S_y + 1'b1;
                    else               S_active_seen <= 1'b1;
                end
                S_x <= S_x + 1'b1;
            end else begin
                S_x <= 10'd0;
            end
        end
    end

    // ============ 高度：pending -> target(vsync 提交) -> current(平滑) ============
    reg [7:0] S_h_pend [0:SPECTRUM_BIN_NUM-1];  // 分析输出(随时可写)
    reg [7:0] S_tgt    [0:SPECTRUM_BIN_NUM-1];  // 目标高度
    reg [7:0] S_cur    [0:SPECTRUM_BIN_NUM-1];  // 当前(平滑)高度

    integer bi;
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            for (bi = 0; bi < SPECTRUM_BIN_NUM; bi = bi + 1) begin
                S_h_pend[bi] <= 8'd0;
                S_tgt   [bi] <= 8'd0;
                S_cur   [bi] <= 8'd0;
            end
        end else begin
            if (I_heights_valid)
                for (bi = 0; bi < SPECTRUM_BIN_NUM; bi = bi + 1)
                    S_h_pend[bi] <= I_heights[bi*8 +: 8];

            if (I_vs != S_vs_d)              // 消隐期提交 target
                for (bi = 0; bi < SPECTRUM_BIN_NUM; bi = bi + 1)
                    S_tgt[bi] <= S_h_pend[bi];

            // 每 video_clk 平滑一步（current 向 target 移动 SMOOTH_STEP）
            for (bi = 0; bi < SPECTRUM_BIN_NUM; bi = bi + 1) begin
                if      (S_cur[bi] < S_tgt[bi])
                    S_cur[bi] <= S_cur[bi] + SMOOTH_STEP[7:0];
                else if (S_cur[bi] > S_tgt[bi])
                    S_cur[bi] <= S_cur[bi] - SMOOTH_STEP[7:0];
                else
                    S_cur[bi] <= S_cur[bi];
            end
        end
    end

    // ============ 组合判定（仅柱体像素覆盖，绝无面板） ============
    wire inx  = (S_x >= SPECTRUM_X_START) && (S_x <= SPECTRUM_X_END);
    wire iny  = (S_y >= SPECTRUM_Y_TOP) && (S_y <= SPECTRUM_Y_BOTTOM);
    wire in_reg = I_de && inx && iny;

    wire [8:0] xoff = S_x - SPECTRUM_X_START;          // 0..159
    // band = xoff/PITCH, celloff = xoff%PITCH（常量除/余，综合为小规模）
    wire [4:0] band    = inx ? (xoff / PITCH) : 5'd0;
    wire [3:0] celloff = inx ? (xoff % PITCH) : 4'd0;

    wire bar_ok = in_reg &&
                  (celloff < BAR_WIDTH) &&
                  (S_cur[band] != 8'd0) &&
                  ((S_y + S_cur[band]) >= (SPECTRUM_Y_BOTTOM + 1));

    // ============ 柱体颜色（常量彩虹表） ============
    function [23:0] bar_color;
        input [4:0] band;
        begin
            case (band)
                5'd0 : bar_color = 24'h0000FF;
                5'd1 : bar_color = 24'h001EFF;
                5'd2 : bar_color = 24'h003CFF;
                5'd3 : bar_color = 24'h005AFF;
                5'd4 : bar_color = 24'h0077FF;
                5'd5 : bar_color = 24'h0095FF;
                5'd6 : bar_color = 24'h00B3FF;
                5'd7 : bar_color = 24'h00D1FF;
                5'd8 : bar_color = 24'h00EEFF;
                5'd9 : bar_color = 24'h00FFF2;
                5'd10: bar_color = 24'h00FFD4;
                5'd11: bar_color = 24'h00FFB6;
                5'd12: bar_color = 24'h00FF99;
                5'd13: bar_color = 24'h00FF7B;
                5'd14: bar_color = 24'h00FF5D;
                5'd15: bar_color = 24'h00FF3F;
                5'd16: bar_color = 24'h00FF22;
                5'd17: bar_color = 24'h00FF04;
                5'd18: bar_color = 24'h1AFF00;
                5'd19: bar_color = 24'h38FF00;
                5'd20: bar_color = 24'h55FF00;
                5'd21: bar_color = 24'h73FF00;
                5'd22: bar_color = 24'h91FF00;
                5'd23: bar_color = 24'hAFFF00;
                5'd24: bar_color = 24'hCCFF00;
                5'd25: bar_color = 24'hEAFF00;
                5'd26: bar_color = 24'hFFF600;
                5'd27: bar_color = 24'hFFD800;
                5'd28: bar_color = 24'hFFBB00;
                5'd29: bar_color = 24'hFF9D00;
                5'd30: bar_color = 24'hFF7F00;
                default: bar_color = 24'hFF6100;
            endcase
        end
    endfunction

    // 只有真正的柱体像素覆盖原图；其余（含区域内非柱）一律透传 I_rgb_bg
    assign O_rgb = SPECTRUM_DEBUG_BYPASS ? I_rgb_bg :
                   (bar_ok ? bar_color(band) : I_rgb_bg);

endmodule
