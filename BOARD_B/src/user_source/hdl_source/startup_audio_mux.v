//=====================================================================
// startup_audio_mux.v  (开机音 / 正常音 音频 MUX)
//  - select_startup=1 : 输出 startup 左/右
//  - select_startup=0 : 输出 normal 左/右 (normal 即现有 I2S 恢复 PCM)
//  - 纯组合选择; 切换点由 player 在 48kHz sample 边界改变 select_startup
//  - 保证: 开机音结束后 select_startup=0, normal 独享; 两者绝不叠加
//=====================================================================
module startup_audio_mux (
    input  wire             I_select_startup,   // 1=startup 音源
    input  wire signed [23:0] I_startup_left,
    input  wire signed [23:0] I_startup_right,
    input  wire signed [23:0] I_normal_left,
    input  wire signed [23:0] I_normal_right,
    output wire signed [23:0] O_left,
    output wire signed [23:0] O_right
);
    assign O_left  = I_select_startup ? I_startup_left  : I_normal_left;
    assign O_right = I_select_startup ? I_startup_right : I_normal_right;
endmodule
