`timescale 1ns/1ps
// Shares the existing UART's decoded bytes and transmit byte interface.
// F:0968<Enter> (96.8MHz), V:15<Enter>, M:1<Enter>; I restart; Q status.
module fm_uart_control(
    input wire clk,rst,rx_valid,rx_allowed,
    input wire [7:0] rx_data,
    output reg cmd_valid,
    input wire cmd_ready,
    output reg [2:0] cmd_op,
    output reg [15:0] cmd_value,
    input wire [79:0] fm_status,
    output wire tx_valid,
    input wire tx_ready,
    output reg [7:0] tx_data
);
    reg parsing,invalid,parser_error;
    reg [2:0] parse_op;
    reg [1:0] extension_kind;
    reg [2:0] digits;
    reg [15:0] value;
    reg query_pending,sending;
    reg [4:0] tx_index;
    reg [79:0] snapshot;
    reg [3:0] nibble;
    assign tx_valid=sending;
    always @* begin
        nibble=0;
        if(tx_index>=3 && tx_index<=22) nibble=snapshot[(22-tx_index)*4 +: 4];
        case(tx_index)
            0:tx_data="F";1:tx_data="M";2:tx_data=" ";
            23:tx_data=13;24:tx_data=10;
            default:tx_data=nibble<10 ? "0"+nibble : "A"+nibble-10;
        endcase
    end
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            parsing<=0;invalid<=0;parser_error<=0;parse_op<=0;extension_kind<=0;digits<=0;value<=0;
            cmd_valid<=0;cmd_op<=0;cmd_value<=0;query_pending<=0;
            sending<=0;tx_index<=0;snapshot<=0;
        end else begin
            if(cmd_valid && cmd_ready) cmd_valid<=0;
            if(rx_valid && rx_allowed) begin
                if(!parsing) begin
                    case(rx_data)
                        "F","f","V","v","M","m","P","p","A","a": begin
                            parsing<=1;invalid<=0;digits<=0;value<=0;
                            extension_kind<=0;
                            parse_op<=rx_data=="F" || rx_data=="f" ? 0 :
                                      rx_data=="V" || rx_data=="v" ? 1 :
                                      rx_data=="M" || rx_data=="m" ? 2 :
                                      rx_data=="P" || rx_data=="p" ? 6 : 7;
                        end
                        "G","g","H","h","Z","z":begin
                            parsing<=1;invalid<=0;digits<=0;value<=0;parse_op<=7;
                            extension_kind<=rx_data=="G" || rx_data=="g" ? 1 : rx_data=="H" || rx_data=="h" ? 2 : 3;
                        end
                        "I","i": begin
                            if(!cmd_valid || cmd_ready) begin cmd_valid<=1;cmd_op<=3;cmd_value<=0;parser_error<=0;end
                            else parser_error<=1;
                        end
                        "U","u","D","d":begin
                            if(!cmd_valid || cmd_ready)begin cmd_valid<=1;cmd_op<=rx_data=="U" || rx_data=="u" ? 4 : 5;cmd_value<=0;parser_error<=0;end
                            else parser_error<=1;
                        end
                        "Q","q": query_pending<=1;
                        default:;
                    endcase
                end else if(rx_data==13 || rx_data==10) begin
                    parsing<=0;
                    if(!invalid && digits!=0 &&
                       (parse_op!=7 || (extension_kind==0 && value<=3) ||
                        (extension_kind==1 && value<=4) || (extension_kind==2 && value<=3) ||
                        (extension_kind==3 && value==1)) && (!cmd_valid || cmd_ready)) begin
                        cmd_valid<=1;cmd_op<=parse_op;
                        // A:0..3 selects matching HDMI/FFT; MIC monitor stays off.
                        if(parse_op!=7)cmd_value<=value;
                        else case(extension_kind)
                         0:cmd_value<={12'd0,value[1:0],value[1:0]};
                         1:cmd_value<=16'h1000 | value;
                         2:cmd_value<=16'h1100 | value;
                         3:cmd_value<=16'h1201;
                        endcase
                        parser_error<=0;
                    end else parser_error<=1;
                end else if(rx_data==27) parsing<=0;
                else if((rx_data==":" || rx_data==" ") && digits==0) begin end
                else if(rx_data>="0" && rx_data<="9" && digits<4) begin
                    // Decimal accumulation uses LUT adders, preserving FFT DSPs.
                    value<=(value<<3)+(value<<1)+(rx_data-8'd48);digits<=digits+1'b1;
                end else invalid<=1;
            end
            if(query_pending && !sending) begin
                snapshot<=fm_status;
                snapshot[39:38]<={parser_error,cmd_valid};
                query_pending<=0;sending<=1;tx_index<=0;
            end
            if(sending && tx_ready) begin
                if(tx_index==24) sending<=0;else tx_index<=tx_index+1'b1;
            end
        end
    end
endmodule
