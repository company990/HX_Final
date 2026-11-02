// UART filename selection for the verified WAV48 audio path.
// W commits a name only; P starts/restarts playback; S reports status and name.
// N/A outputs are retained for interface compatibility, unused by this top.
// Commands:
//   N = next BMP
//   A = auto BMP on/off
//   P = play currently selected WAV
//   L = ambient-light diagnostic snapshot (hexadecimal values)
//   S = status
//   C = clear faults
//   W:<8.3-name> = select WAV by FAT32 short filename, e.g. W:TEST_4~1.WAV
//     or W TEST_4~1.WAV. Press Enter after the filename.
// The selected FAT 8.3 name is stored as 11 bytes: 8-byte base + 3-byte extension,
// space padded and upper-cased.
module team2_uart_control #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD = 115200,
    parameter integer SUFFIX_ONLY = 0,
    parameter [87:0] INITIAL_NAME = "MUSIC   WAV",
    parameter integer PANEL_ENABLE = 0
)(
    input        clk,
    input        rst,
    input        panel_play,
    input [87:0] panel_name,
    input        uart_rx,
    output       uart_tx,
    output reg   cmd_next,
    output reg   cmd_auto,
    output reg   cmd_play,
    output reg   cmd_clear,
    output reg   cmd_wav_rescan,
    output reg [87:0] wav_target_name,
    input  [3:0] state_code,
    input  [2:0] bmp_count,
    input        wav_found,
    input        wav_playing,
    input        fifo_underflow,
    input        fifo_overflow,
    input  [9:0] fifo_level,
    input  [63:0] light_debug, // video-clock snapshot: frames/raw/average/count/gain/level/ready/fault
    input  [7:0] error_code
);
    wire rx_valid;
    wire [7:0] rx_data;
    reg tx_start;
    wire [7:0] status_byte;
    wire tx_busy;
    reg status_pending;
    reg light_pending,light_sending;
    reg [63:0] light_snapshot;
    reg [5:0] idx;
    reg sending;
    reg tx_busy_d;

    reg [87:0] name_edit;
    reg name_invalid;
    reg name_error;
    reg naming;
    reg got_dot;
    reg [3:0] base_idx;
    reg [2:0] ext_idx;

    uart_rx_8n1 #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD(BAUD)) u_rx(.clk(clk),.rst(rst),.rx(uart_rx),.valid(rx_valid),.data(rx_data));
    uart_tx_8n1 #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD(BAUD)) u_tx(.clk(clk),.rst(rst),.tx_start(tx_start),.tx_data(status_byte),.tx_busy(tx_busy),.tx(uart_tx));

    function [7:0] hexchar;
        input [3:0] v;
        begin
            if (v < 10) hexchar = "0" + v;
            else hexchar = "A" + (v-10);
        end
    endfunction

    function [7:0] upper_char;
        input [7:0] c;
        begin
            if ((c >= "a") && (c <= "z"))
                upper_char = c - 8'd32;
            else
                upper_char = c;
        end
    endfunction

    function [7:0] status_char;
        input [5:0] n;
        begin
            case(n)
                6'd0: status_char = 8'h54;
                6'd1: status_char = 8'h32;
                6'd2: status_char = 8'h20;
                6'd3: status_char = 8'h53;
                6'd4: status_char = 8'h54;
                6'd5: status_char = 8'h3D;
                6'd6: status_char = 8'd0;
                6'd7: status_char = 8'h20;
                6'd8: status_char = 8'h57;
                6'd9: status_char = 8'h3D;
                6'd10: status_char = wav_found ? "1" : "0";
                6'd11: status_char = wav_playing ? "1" : "0";
                6'd12: status_char = 8'h20;
                6'd13: status_char = 8'h45;
                6'd14: status_char = 8'h3D;
                6'd15: status_char = 8'd0;
                6'd16: status_char = 8'd0;
                6'd17: status_char = 8'h20;
                6'd18: status_char = 8'h4E;
                6'd19: status_char = 8'h3D;
                6'd20: status_char = SUFFIX_ONLY ? 8'd65 : wav_target_name[87:80];
                6'd21: status_char = SUFFIX_ONLY ? 8'd85 : wav_target_name[79:72];
                6'd22: status_char = SUFFIX_ONLY ? 8'd84 : wav_target_name[71:64];
                6'd23: status_char = SUFFIX_ONLY ? 8'd79 : wav_target_name[63:56];
                6'd24: status_char = SUFFIX_ONLY ? 8'd32 : wav_target_name[55:48];
                6'd25: status_char = SUFFIX_ONLY ? 8'd32 : wav_target_name[47:40];
                6'd26: status_char = SUFFIX_ONLY ? 8'd32 : wav_target_name[39:32];
                6'd27: status_char = SUFFIX_ONLY ? 8'd32 : wav_target_name[31:24];
                6'd28: status_char = SUFFIX_ONLY ? 8'd87 : wav_target_name[23:16];
                6'd29: status_char = SUFFIX_ONLY ? 8'd65 : wav_target_name[15:8];
                6'd30: status_char = SUFFIX_ONLY ? 8'd86 : wav_target_name[7:0];
                6'd31: status_char = 8'h0D;
                6'd32: status_char = 8'h0A;
                default: status_char = " ";
            endcase
        end
    endfunction

        function [7:0] light_char;
        input [5:0] n;
        begin case(n)
            6'd0: light_char = 8'h4C;
            6'd1: light_char = 8'h20;
            6'd2: light_char = 8'h52;
            6'd3: light_char = 8'h3D;
            6'd4: light_char = 8'd0;
            6'd5: light_char = 8'd0;
            6'd6: light_char = 8'd0;
            6'd7: light_char = 8'h20;
            6'd8: light_char = 8'h41;
            6'd9: light_char = 8'h3D;
            6'd10: light_char = 8'd0;
            6'd11: light_char = 8'd0;
            6'd12: light_char = 8'd0;
            6'd13: light_char = 8'h20;
            6'd14: light_char = 8'h43;
            6'd15: light_char = 8'h3D;
            6'd16: light_char = 8'd0;
            6'd17: light_char = 8'd0;
            6'd18: light_char = 8'h20;
            6'd19: light_char = 8'h47;
            6'd20: light_char = 8'h3D;
            6'd21: light_char = 8'd0;
            6'd22: light_char = 8'd0;
            6'd23: light_char = 8'h20;
            6'd24: light_char = 8'h53;
            6'd25: light_char = 8'h3D;
            6'd26: light_char = 8'd0;
            6'd27: light_char = 8'h20;
            6'd28: light_char = 8'h56;
            6'd29: light_char = 8'h3D;
            6'd30: light_char = 8'd0;
            6'd31: light_char = 8'h20;
            6'd32: light_char = 8'h46;
            6'd33: light_char = 8'h3D;
            6'd34: light_char = 8'd0;
            6'd35: light_char = 8'h20;
            6'd36: light_char = 8'h48;
            6'd37: light_char = 8'h3D;
            6'd38: light_char = 8'd0;
            6'd39: light_char = 8'd0;
            6'd40: light_char = " ";
            6'd41: light_char = "X";
            6'd42: light_char = "=";
            6'd43: light_char = 8'd0;
            6'd44: light_char = 8'd0;
            6'd45: light_char = " ";
            6'd46: light_char = "B";
            6'd47: light_char = "=";
            6'd48: light_char = 8'd0;
            6'd49: light_char = 8'd0;
            6'd50: light_char = 8'h0D;
            6'd51: light_char = 8'h0A;
            default: light_char = " ";
        endcase end
    endfunction
    // Select one nibble before ASCII conversion, sharing the hex encoder.
    reg hex_selected;
    reg [3:0] hex_nibble;
    always @* begin
        hex_selected=1'b1;hex_nibble=4'd0;
        if(light_sending) begin
            case(idx)
                6'd4:hex_nibble=light_snapshot[40:37];
                6'd5:hex_nibble=light_snapshot[36:33];
                6'd6:hex_nibble=light_snapshot[32:29];
                6'd10:hex_nibble=light_snapshot[28:25];
                6'd11:hex_nibble=light_snapshot[24:21];
                6'd12:hex_nibble=light_snapshot[20:17];
                6'd16:hex_nibble=light_snapshot[16:13];
                6'd17:hex_nibble=light_snapshot[12:9];
                6'd21:hex_nibble={3'b0,light_snapshot[8]};
                6'd22:hex_nibble=light_snapshot[7:4];
                6'd26:hex_nibble={2'b0,light_snapshot[3:2]};
                6'd30:hex_nibble={3'b0,light_snapshot[1]};
                6'd34:hex_nibble={3'b0,light_snapshot[0]};
                6'd38:hex_nibble={1'b0,light_snapshot[47:45]};
                6'd39:hex_nibble=light_snapshot[44:41];
                6'd43:hex_nibble=light_snapshot[55:52];
                6'd44:hex_nibble=light_snapshot[51:48];
                6'd48:hex_nibble=light_snapshot[63:60];
                6'd49:hex_nibble=light_snapshot[59:56];
                default:hex_selected=1'b0;
            endcase
        end else begin
            case(idx)
                6'd6:hex_nibble=state_code;
                6'd15:hex_nibble=name_error ? 4'h3 : error_code[7:4];
                6'd16:hex_nibble=name_error ? 4'h0 : error_code[3:0];
                default:hex_selected=1'b0;
            endcase
        end
    end
    assign status_byte=hex_selected ? hexchar(hex_nibble) : (light_sending ? light_char(idx) : status_char(idx));

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cmd_next        <= 1'b0;
            cmd_auto        <= 1'b0;
            cmd_play        <= 1'b0;
            cmd_clear       <= 1'b0;
            cmd_wav_rescan  <= 1'b0;
            tx_start        <= 1'b0;
            status_pending  <= 1'b0;
            light_pending<=0;light_sending<=0;light_snapshot<=0;
            idx             <= 6'd0;
            sending         <= 1'b0;
            tx_busy_d        <= 1'b0;
            name_edit <= {11{8'h20}};
            name_invalid <= 1'b0;
            name_error <= 1'b0;
            naming          <= 1'b0;
            got_dot         <= 1'b0;
            base_idx        <= 4'd0;
            ext_idx         <= 3'd0;
            // Boot-compatible default; W selects and P starts a new session.
            wav_target_name <= INITIAL_NAME;
        end else begin
            cmd_next       <= 1'b0;
            cmd_auto       <= 1'b0;
            cmd_play       <= 1'b0;
            cmd_clear      <= 1'b0;
            cmd_wav_rescan <= 1'b0;
            tx_start       <= 1'b0;
            tx_busy_d      <= tx_busy;

            if (rx_valid) begin
                if (!naming) begin
                    case (rx_data)
                        "N","n": cmd_next <= 1'b1;
                        "A","a": cmd_auto <= 1'b1;
                        "P","p": begin cmd_play <= 1'b1; status_pending <= 1'b1; light_pending <= 1'b0; end
                        "C","c": begin cmd_clear <= 1'b1; name_error <= 1'b0; end
                        "S","s": begin status_pending <= 1'b1; light_pending <= 1'b0; end
                        "L","l": begin status_pending <= 1'b1; light_pending <= 1'b1; end
                        "W","w": begin
                            naming   <= 1'b1;
                            got_dot  <= 1'b0;
                            base_idx <= 4'd0;
                            ext_idx  <= 3'd0;
                            // Pre-fill all 11 positions with spaces.
                            name_edit <= {11{8'h20}};
                            name_invalid <= 1'b0;
                        end
                        default: ;
                    endcase
                end else begin
                    // Filename entry mode. Allow optional ':' or leading spaces.
                    if ((rx_data == 8'h0D) || (rx_data == 8'h0A)) begin
                        naming <= 1'b0;
                        if (!name_invalid && (base_idx != 0) && got_dot &&
                            (ext_idx == 3) && (name_edit[23:0] == "WAV")) begin
                            wav_target_name <= SUFFIX_ONLY ? 88'd0 : name_edit;
                            cmd_wav_rescan <= 1'b1;
                            name_error <= 1'b0;
                        end else begin
                            name_error <= 1'b1;
                        end
                        status_pending <= 1'b1; light_pending <= 1'b0;
                    end else if (rx_data == 8'h1B) begin
                        naming <= 1'b0;
                    end else if (((rx_data == ":") || (rx_data == " ")) &&
                                 (base_idx == 0) && !got_dot) begin
                        // Ignore optional separator after W.
                    end else if ((rx_data == ".") && !got_dot) begin
                        if (base_idx != 0) begin
                            got_dot <= 1'b1;
                            ext_idx <= 3'd0;
                        end
                    end else if ((rx_data <= 8'h20) || (rx_data >= 8'h7F) ||
                                 (rx_data == ".") || (rx_data == ":") ||
                                 (rx_data == 8'h22) || (rx_data == "*") ||
                                 (rx_data == "/") || (rx_data == 8'h5C) ||
                                 (rx_data == "?") || (rx_data == "<") ||
                                 (rx_data == ">") || (rx_data == "|") ||
                                 (rx_data == "+") || (rx_data == ",") ||
                                 (rx_data == ";") || (rx_data == "=") ||
                                 (rx_data == "[") || (rx_data == "]")) begin
                        name_invalid <= 1'b1;
                    end else if (!got_dot) begin
                        if (base_idx < 8) begin
                            case (base_idx)
                                4'd0: name_edit[87:80] <= upper_char(rx_data);
                                4'd1: name_edit[79:72] <= upper_char(rx_data);
                                4'd2: name_edit[71:64] <= upper_char(rx_data);
                                4'd3: name_edit[63:56] <= upper_char(rx_data);
                                4'd4: name_edit[55:48] <= upper_char(rx_data);
                                4'd5: name_edit[47:40] <= upper_char(rx_data);
                                4'd6: name_edit[39:32] <= upper_char(rx_data);
                                4'd7: name_edit[31:24] <= upper_char(rx_data);
                                default: ;
                            endcase
                            base_idx <= base_idx + 1'b1;
                        end else name_invalid <= 1'b1;
                    end else begin
                        if (ext_idx < 3) begin
                            case (ext_idx)
                                3'd0: name_edit[23:16] <= upper_char(rx_data);
                                3'd1: name_edit[15:8]  <= upper_char(rx_data);
                                3'd2: name_edit[7:0]   <= upper_char(rx_data);
                                default: ;
                            endcase
                            ext_idx <= ext_idx + 1'b1;
                        end else name_invalid <= 1'b1;
                    end
                end
            end

            if (status_pending && !sending) begin
                sending        <= 1'b1;
                light_sending<=light_pending;light_snapshot<=light_debug;
                idx            <= 6'd0;
                status_pending <= 1'b0;
            end

            // Timing-light UART handshake:
            // 1) launch exactly one tx_start pulse while TX is idle;
            // 2) keep idx/status_byte unchanged until uart_tx_8n1 acknowledges
            //    the character by raising tx_busy;
            // 3) advance idx only on the rising edge of tx_busy.
            //
            // This fixes the old "every other status character disappears"
            // bug while adding only one small register (tx_busy_d).
            if (sending) begin
                if (!tx_busy && !tx_start)
                    tx_start <= 1'b1;

                if (tx_busy && !tx_busy_d) begin
                    if (idx == (light_sending ? 6'd51 : 6'd32))
                        sending <= 1'b0;
                    else
                        idx <= idx + 1'b1;
                end
            end
            // The physical panel wins a same-cycle name commit. Keep S/N and
            // a later UART P consistent with the last confirmed panel song.
            // Do not emit another cmd_play: top already handles panel_play.
            if (PANEL_ENABLE && panel_play) begin
                wav_target_name <= panel_name;
                name_error <= 1'b0;
                naming <= 1'b0;
                status_pending <= 1'b1; light_pending <= 1'b0;
            end
        end
    end
endmodule
