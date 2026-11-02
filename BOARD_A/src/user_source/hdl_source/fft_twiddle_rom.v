//=====================================================================
// fft_twiddle_rom.v
// 旋转因子 ROM：W_1024^k = e^{-j2pi*k/1024}，k = 0..511
//   共 512 点，与 top 参数 FFT_N=1024 / 内部蝶形索引 k 对应。
//   实部 cos、虚部(-sin) 均为 Q1.15 有符号定点，由
//   python/gen_fft_roms.ps1 生成 fft_twiddle_cos.mif / fft_twiddle_sin.mif。
// 时钟域：video_clk(25MHz)，同步读（地址->数据约 1 拍延迟，与官方
//   AUDIO_SAMPLE_ROM 封装一致的 EG_LOGIC_BRAM 用法）。
//=====================================================================
module fft_twiddle_rom (
    input  wire        clka,        // video_clk
    input  wire [8:0]  addra,       // 0..511
    output wire [15:0] doa_cos,     // Q1.15 cos
    output wire [15:0] doa_sin      // Q1.15 -sin(即虚部系数)
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
        .INIT_FILE    ("fft_twiddle_cos.mif"),
        .FILL_ALL     ("NONE")
    ) u_cos_rom (
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
        .doa   (doa_cos),
        .dob   ()
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
        .INIT_FILE    ("fft_twiddle_sin.mif"),
        .FILL_ALL     ("NONE")
    ) u_sin_rom (
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
        .doa   (doa_sin),
        .dob   ()
    );

endmodule
