//=====================================================================
// audio_mux3.v  (Phase22 最终 Audio MUX: Startup | WAV | TestTone | SILENT)
//   - sel: 0=startup 1=wav 2=testtone(DEBUG) 3=silence
//   - 纯组合选择, 每个 48k sample 边界只选一路 => 三源绝不叠加。
//   - 未选中源即使有 valid 也不会进输出(且各源仅在其有效拍被使能)。
// 输出 24bit(接 HDMI Audio 16->24 上采样/FFT 前的 MUX 输出)。
//=====================================================================
module audio_mux3 (
    input  wire [1:0] I_sel,              // 0 startup / 1 wav / 2 tone / 3 silence
    input  wire signed [23:0] I_startup_left,  I_startup_right,
    input  wire signed [23:0] I_wav_left,      I_wav_right,
    input  wire signed [23:0] I_tone_left,     I_tone_right,
    output reg  signed [23:0] O_left,
    output reg  signed [23:0] O_right
);
    always @(*) begin
        O_left  = 24'sd0;
        O_right = 24'sd0;
        case (I_sel)
            2'd0: begin O_left = I_startup_left;  O_right = I_startup_right; end
            2'd1: begin O_left = I_wav_left;      O_right = I_wav_right;     end
            2'd2: begin O_left = I_tone_left;     O_right = I_tone_right;    end
            default: begin O_left = 24'sd0; O_right = 24'sd0; end
        endcase
    end
endmodule
