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

    function [47:0] rom_get;
        input [ADDR_BITS-1:0] a;
        begin
            case (a)
                // ---- Measure 1 ----
                7'd0: rom_get = 48'h2EE0_01DCD0BB;  // F4  e
                7'd1: rom_get = 48'h2EE0_021735EE;  // G4  e
                7'd2: rom_get = 48'h2EE0_0258BF26;  // A4  e
                7'd3: rom_get = 48'h2EE0_01DCD0BB;  // F4  e
                7'd4: rom_get = 48'h8CA0_02CA6921;  // C5  dq
                7'd5: rom_get = 48'h2EE0_0258BF26;  // A4  e
                // ---- Measure 2 ----
                7'd6: rom_get = 48'h5DC0_021735EE;  // G4  q
                7'd7: rom_get = 48'h5DC0_02CA6921;  // C5  q
                7'd8: rom_get = 48'h5DC0_021735EE;  // G4  q
                7'd9: rom_get = 48'h2EE0_01DCD0BB;  // F4  e
                7'd10: rom_get = 48'h2EE0_0190F19C; // D4  e
                // ---- Measure 3 ----
                7'd11: rom_get = 48'h8CA0_0258BF26; // A4  dq
                7'd12: rom_get = 48'h2EE0_01DCD0BB; // F4  e
                7'd13: rom_get = 48'hBB80_01C20E09; // E4  h
                // ---- Measure 4 ----
                7'd14: rom_get = 48'h2EE0_01DCD0BB; // F4  e
                7'd15: rom_get = 48'h2EE0_01C20E09; // E4  e
                7'd16: rom_get = 48'h5DC0_0190F19C; // D4  q
                7'd17: rom_get = 48'h5DC0_01C20E09; // E4  q
                7'd18: rom_get = 48'h2EE0_01DCD0BB; // F4  e
                7'd19: rom_get = 48'h2EE0_021735EE; // G4  e
                // ---- Measure 5 ----
                7'd20: rom_get = 48'h5DC0_01653650; // C4  q
                7'd21: rom_get = 48'h5DC0_01DCD0BB; // F4  q
                7'd22: rom_get = 48'h2EE0_021735EE; // G4  e
                7'd23: rom_get = 48'h2EE0_0258BF26; // A4  e
                7'd24: rom_get = 48'h5DC0_027C76BB; // Bb4 q
                // (Measure 6 deleted)
                // ---- Measure 7 ----
                7'd25: rom_get = 48'h2EE0_027C76BB; // Bb4 e
                7'd26: rom_get = 48'h2EE0_0258BF26; // A4  e
                7'd27: rom_get = 48'h2EE0_021735EE; // G4  e
                7'd28: rom_get = 48'h2EE0_01DCD0BB; // F4  e
                7'd29: rom_get = 48'hBB80_01DCD0BB; // F4  h
                // ---- Measure 8 ----
                7'd30: rom_get = 48'h2EE0_0258BF26; // A4  e
                7'd31: rom_get = 48'h2EE0_027C76BB; // Bb4 e
                7'd32: rom_get = 48'h5DC0_02CA6921; // C5  q
                7'd33: rom_get = 48'h5DC0_02CA6921; // C5  q
                7'd34: rom_get = 48'h5DC0_02CA6921; // C5  q
                // ---- Measure 9 ----
                7'd35: rom_get = 48'h5DC0_02CA6921; // C5  q
                7'd36: rom_get = 48'h2EE0_02CA6921; // C5  e
                7'd37: rom_get = 48'h2EE0_0321E6B7; // D5  e
                7'd38: rom_get = 48'h2EE0_02CA6921; // C5  e
                7'd39: rom_get = 48'h2EE0_027C76BB; // Bb4 e
                7'd40: rom_get = 48'h5DC0_0258BF26; // A4  q
                // ---- Measure 10 ----
                7'd41: rom_get = 48'h5DC0_0258BF26; // A4  q
                7'd42: rom_get = 48'h5DC0_0258BF26; // A4  q
                7'd43: rom_get = 48'h5DC0_0258BF26; // A4  q
                7'd44: rom_get = 48'h2EE0_0258BF26; // A4  e
                7'd45: rom_get = 48'h2EE0_027C76BB; // Bb4 e
                // ---- Measure 11 ----
                7'd46: rom_get = 48'h2EE0_0258BF26; // A4  e
                7'd47: rom_get = 48'h2EE0_021735EE; // G4  e
                7'd48: rom_get = 48'h5DC0_01DCD0BB; // F4  q
                7'd49: rom_get = 48'h5DC0_01DCD0BB; // F4  q
                7'd50: rom_get = 48'h2EE0_01DCD0BB; // F4  e
                7'd51: rom_get = 48'h2EE0_01C20E09; // E4  e
                // ---- Measure 12 ----
                7'd52: rom_get = 48'h5DC0_0190F19C; // D4  q
                7'd53: rom_get = 48'h5DC0_01C20E09; // E4  q
                7'd54: rom_get = 48'h2EE0_01C20E09; // E4  e
                7'd55: rom_get = 48'h2EE0_01DCD0BB; // F4  e
                7'd56: rom_get = 48'h5DC0_021735EE; // G4  q
                // ---- Measure 13 ----
                7'd57: rom_get = 48'h2EE0_021735EE; // G4  e
                7'd58: rom_get = 48'h2EE0_0258BF26; // A4  e
                7'd59: rom_get = 48'h2EE0_021735EE; // G4  e
                7'd60: rom_get = 48'h2EE0_01DCD0BB; // F4  e  (末第2音 改 F4, 节奏不变)
                7'd61: rom_get = 48'hBB80_01DCD0BB; // F4  h  (末音   改 F4, 节奏不变)
                // terminator (never stepped into)
                default: rom_get = 48'h0000_00000000;
            endcase
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

