//=====================================================================
// spectrum_log_mapper.v  (Phase3)
// 把 FFT 每列幅值(16bit 量级, 由 analyzer 输出)按 STM32 参考公式映射为柱高:
//     h = min(31, floor(8.1 * ln(m_norm + 3)))
// 其中 m_norm = magnitude >> MAG_SHIFT (钳位 0..255)。
// 定点对照(区间实现)见文件头注释; ≥43 即饱和 31 —— 与原程序灵敏度一致。
//
// 纯组合、逐列独立; 64 列并行。不占 BRAM。
//=====================================================================
module spectrum_log_mapper #(
    parameter integer SPECTRUM_BIN_NUM = 64,
    parameter integer MAG_BITS         = 16,   // 输入每列幅值位宽
    parameter integer MAG_SHIFT        = 6     // 幅值右移(灵敏度/增益调参)
)(
    input  wire [SPECTRUM_BIN_NUM*MAG_BITS-1:0] I_mag,   // 64 列幅值
    output wire [SPECTRUM_BIN_NUM*6-1:0]         O_height // 64 列柱高 0..31
);

    // floor(8.1*ln(m+3))  对照表(0..255):
    // >=43:31 >=38:30 >=33:29 >=29:28 >=26:27 >=22:26 >=19:25 >=17:24 >=15:23
    // >=13:22 >=11:21 >=9:20 >=8:19 >=7:18 >=6:17 >=5:16 >=4:15 >=3:14 >=2:13
    // >=1:11  else:8
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

    genvar g;
    generate
        for (g = 0; g < SPECTRUM_BIN_NUM; g = g + 1) begin : gen_col
            wire [MAG_BITS-1:0] mi = I_mag[g*MAG_BITS +: MAG_BITS];
            wire [15:0]         shifted = mi >> MAG_SHIFT;
            wire [7:0]          mn = (shifted > 16'd255) ? 8'd255 : shifted[7:0];
            assign O_height[g*6 +: 6] = map_h(mn);
        end
    endgenerate
endmodule
