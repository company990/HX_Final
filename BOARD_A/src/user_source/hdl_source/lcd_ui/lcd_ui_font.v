// LCD-only native DengXian glyphs. Existing shared HDMI font is untouched.
module lcd_ui_font(input clk,input [10:0] addr,output [23:0] bits);
`ifdef FM_FONT_SIM
 localparam FONT_FILE="fm_lcd_native24_home.dat";
`else
 localparam FONT_FILE="fm_lcd_native24_home.mif";
`endif

 EG_LOGIC_BRAM #(.DATA_WIDTH_A(24),.DATA_WIDTH_B(24),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
  .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),
  .INIT_FILE(FONT_FILE),.FILL_ALL("NONE")) memory(
  .dia(24'd0),.addra(addr),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(bits),
  .dib(24'd0),.addrb(11'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob());
endmodule
module lcd_ui_ascii_font(input clk,input [11:0] addr,output [15:0] bits);
`ifdef FM_FONT_SIM
 localparam FONT_FILE="fm_lcd_native24_ascii.dat";
`else
 localparam FONT_FILE="fm_lcd_native24_ascii.mif";
`endif
 function [10:0] glyph_address;input [11:0] a;reg [6:0] c;reg [5:0] slot;
 begin c=a[11:5];slot=c==107?0:c==122?28:c[5:0];glyph_address={slot,a[4:0]};end endfunction
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
  .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),
  .INIT_FILE(FONT_FILE),.FILL_ALL("NONE")) memory(
  .dia(16'd0),.addra(glyph_address(addr)),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(bits),
  .dib(16'd0),.addrb(11'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob());
endmodule
module lcd_ui_hero_font(input clk,input [12:0] addr,output [31:0] bits);
`ifdef FM_FONT_SIM
 localparam FONT_FILE="fm_lcd_native48_hero.dat";
`else
 localparam FONT_FILE="fm_lcd_native48_hero.mif";
`endif
 function [4:0] hero_slot;input [6:0] c;begin case(c)
 7'd32:hero_slot=5'd0;
 7'd45:hero_slot=5'd1;
 7'd46:hero_slot=5'd2;
 7'd48:hero_slot=5'd3;
 7'd49:hero_slot=5'd4;
 7'd50:hero_slot=5'd5;
 7'd51:hero_slot=5'd6;
 7'd52:hero_slot=5'd7;
 7'd53:hero_slot=5'd8;
 7'd54:hero_slot=5'd9;
 7'd55:hero_slot=5'd10;
 7'd56:hero_slot=5'd11;
 7'd57:hero_slot=5'd12;
 7'd58:hero_slot=5'd13;
 7'd63:hero_slot=5'd14;
 7'd69:hero_slot=5'd15;
 7'd70:hero_slot=5'd16;
 7'd72:hero_slot=5'd17;
 7'd73:hero_slot=5'd18;
 7'd76:hero_slot=5'd19;
 7'd77:hero_slot=5'd20;
 7'd80:hero_slot=5'd21;
 7'd83:hero_slot=5'd22;
 7'd86:hero_slot=5'd23;
 7'd88:hero_slot=5'd24;
 7'd90:hero_slot=5'd25;
 default:hero_slot=5'd14;endcase end endfunction

 EG_LOGIC_BRAM #(.DATA_WIDTH_A(32),.DATA_WIDTH_B(32),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
  .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),
  .INIT_FILE(FONT_FILE),.FILL_ALL("NONE")) memory(
  .dia(32'd0),.addra({hero_slot(addr[12:6]),addr[5:0]}),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(bits),
  .dib(32'd0),.addrb(11'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob());
endmodule
module lcd_ui_label(input wire [4:0] label_id,output reg [39:0] codes,output reg [2:0] length);
always @* begin codes=0;length=0;case(label_id)
0:begin codes=40'h0304000000;length=3'd2;end // 音频播放
1:begin codes=40'h0107000000;length=3'd2;end // 数字音效
2:begin codes=40'h0809000000;length=3'd2;end // 时移收听
3:begin codes=40'h0c02000000;length=3'd2;end // 视频播放
4:begin codes=40'h2103000000;length=3'd2;end // 广播电台
5:begin codes=40'h0d0e0f0000;length=3'd3;end // 上一首
6:begin codes=40'h0304121011;length=3'd5;end // 暂停/播放
7:begin codes=40'h130e0f0000;length=3'd3;end // 下一首
8:begin codes=40'h0d0e140000;length=3'd3;end // 上一张
9:begin codes=40'h1516121703;length=3'd5;end // 手动/轮播
10:begin codes=40'h130e140000;length=3'd3;end // 下一张
11:begin codes=40'h1800000000;length=3'd1;end // --
12:begin codes=40'h191a000000;length=3'd2;end // 亮度
13:begin codes=40'h1b00000000;length=3'd1;end // ++
14:begin codes=40'h1800000000;length=3'd1;end // --
15:begin codes=40'h1c1d1a0000;length=3'd3;end // 对比度
16:begin codes=40'h1b00000000;length=3'd1;end // ++
17:begin codes=40'h011e1f2000;length=3'd4;end // 音乐纯享
18:begin codes=40'h2103222300;length=3'd4;end // 广播电台
19:begin codes=40'h2425262700;length=3'd4;end // 激情开麦
default:begin codes=0;length=0;end
endcase end endmodule
