//=====================================================================
// startup_music_rom.v : startup music note ROM (control data, NOT PCM)
// Source : 开机音乐(2).mscz (user-parsed), 4/4 @ BPM120, 简化:
//   只播放 M1..M5 + M7..M13 (删除第6小节; 删除全部 repeat 关系)
//   62 个有效音符事件(addr 0..61), 线性一次; TERM = addr>=62.
// entry = {duration[15:0], phase_step[31:0]} 48bit.
// phase_step = round(F*2^32/48000); duration in 48kHz sample enables.
// F4e/G4e/A4e/F4e/C5dq/A4e | G4q/C5q/G4q/F4e/D4e | A4dq/F4e/E4h | ...
//=====================================================================
module startup_music_rom #(
    parameter integer ROM_DEPTH   = 128,
    parameter integer ADDR_BITS   = 7
)(
    input  wire [ADDR_BITS-1:0] I_addr,      // current note
    output wire [31:0]          O_phase_step,
    output wire [15:0]          O_duration,
    input  wire [ADDR_BITS-1:0] I_taddr,     // target(next) note read(供无缝切音)
    output wire [31:0]          O_t_phase_step,
    output wire [15:0]          O_t_duration
);

    // Lossless event code: duration index and phase-step index.
    function [47:0] rom_get;
      input [ADDR_BITS-1:0] a;
      reg [6:0] code;
      reg [31:0] phase;
      reg [15:0] duration;
      begin
        case(a)
          7'd0:code=7'h14;
          7'd1:code=7'h15;
          7'd2:code=7'h16;
          7'd3:code=7'h14;
          7'd4:code=7'h38;
          7'd5:code=7'h16;
          7'd6:code=7'h25;
          7'd7:code=7'h28;
          7'd8:code=7'h25;
          7'd9:code=7'h14;
          7'd10:code=7'h12;
          7'd11:code=7'h36;
          7'd12:code=7'h14;
          7'd13:code=7'h43;
          7'd14:code=7'h14;
          7'd15:code=7'h13;
          7'd16:code=7'h22;
          7'd17:code=7'h23;
          7'd18:code=7'h14;
          7'd19:code=7'h15;
          7'd20:code=7'h21;
          7'd21:code=7'h24;
          7'd22:code=7'h15;
          7'd23:code=7'h16;
          7'd24:code=7'h27;
          7'd25:code=7'h17;
          7'd26:code=7'h16;
          7'd27:code=7'h15;
          7'd28:code=7'h14;
          7'd29:code=7'h44;
          7'd30:code=7'h16;
          7'd31:code=7'h17;
          7'd32:code=7'h28;
          7'd33:code=7'h28;
          7'd34:code=7'h28;
          7'd35:code=7'h28;
          7'd36:code=7'h18;
          7'd37:code=7'h19;
          7'd38:code=7'h18;
          7'd39:code=7'h17;
          7'd40:code=7'h26;
          7'd41:code=7'h26;
          7'd42:code=7'h26;
          7'd43:code=7'h26;
          7'd44:code=7'h16;
          7'd45:code=7'h17;
          7'd46:code=7'h16;
          7'd47:code=7'h15;
          7'd48:code=7'h24;
          7'd49:code=7'h24;
          7'd50:code=7'h14;
          7'd51:code=7'h13;
          7'd52:code=7'h22;
          7'd53:code=7'h23;
          7'd54:code=7'h13;
          7'd55:code=7'h14;
          7'd56:code=7'h25;
          7'd57:code=7'h15;
          7'd58:code=7'h16;
          7'd59:code=7'h15;
          7'd60:code=7'h14;
          7'd61:code=7'h44;
          default:code=0;
        endcase
        case(code[3:0])
          4'd0:phase=32'h00000000;
          4'd1:phase=32'h01653650;
          4'd2:phase=32'h0190f19c;
          4'd3:phase=32'h01c20e09;
          4'd4:phase=32'h01dcd0bb;
          4'd5:phase=32'h021735ee;
          4'd6:phase=32'h0258bf26;
          4'd7:phase=32'h027c76bb;
          4'd8:phase=32'h02ca6921;
          4'd9:phase=32'h0321e6b7;
          default:phase=0;
        endcase
        case(code[6:4])
          3'd0:duration=16'h0000;
          3'd1:duration=16'h2ee0;
          3'd2:duration=16'h5dc0;
          3'd3:duration=16'h8ca0;
          3'd4:duration=16'hbb80;
          default:duration=0;
        endcase
        rom_get={duration,phase};
      end
    endfunction
    wire [47:0] W = rom_get(I_addr);
    assign O_phase_step = W[31:0];
    assign O_duration   = W[47:32];

    // second (target) read for seamless next-note transition
    wire [47:0] Wt = rom_get(I_taddr);
    assign O_t_phase_step = Wt[31:0];
    assign O_t_duration   = Wt[47:32];
endmodule

