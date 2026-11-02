//=====================================================================
// hann_window_rom.v
// Hann 窗系数 ROM（半表 512 点）：
//     w[n] = 0.5*(1 - cos(2*pi*n/(N-1))),  N=1024, n=0..511
// 利用对称性 w[n] = w[1023-n]（n>511 时由读地址端映射到 1023-n）。
// 系数为 Q1.15 有符号定点（0..~0x7FFF），由 gen_fft_roms.ps1 生成
// fft_hann.mif。时钟域：video_clk，同步读，约 1 拍延迟。
//=====================================================================
module hann_window_rom (
    input  wire        clka,        // video_clk
    input  wire [8:0]  addra,       // 0..511（已按对称映射后的表内地址）
    output wire [15:0] doa          // Q1.15 窗系数
);

    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A (16),
        .ADDR_WIDTH_A (9),
        .DATA_DEPTH_A (512),
        .DATA_WIDTH_B (16),
        .ADDR_WIDTH_B (9),
        .DATA_DEPTH_B (512),
        .MODE         ("SP"),
        .REGMODE_A    ("NOREG"),
        .IMPLEMENT    ("9K"),
        .DEBUGGABLE   ("NO"),
        .PACKABLE     ("NO"),
        .INIT_FILE    ("fft_hann.mif"),
        .FILL_ALL     ("NONE")
    ) u_hann_rom (
        .dia   (16'd0),
        .dib   (16'd0),
        .addra (addra),
        .addrb (9'd0),
        .cea   (1'b1),
        .ceb   (1'b0),
        .ocea  (1'b0),
        .oceb  (1'b0),
        .clka  (clka),
        .clkb  (1'b0),
        .wea   (1'b0),
        .web   (1'b0),
        .bea   (1'b0),
        .beb   (1'b0),
        .rsta  (1'b0),
        .rstb  (1'b0),
        .doa   (doa),
        .dob   ()
    );

endmodule
