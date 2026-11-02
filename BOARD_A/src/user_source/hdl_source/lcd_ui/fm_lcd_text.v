// Direct, immutable 2048-byte caption ROM. Same address and clock latency.
module fm_lcd_text(input clk,input [10:0] addr,output [7:0] character);
`ifdef FM_FONT_SIM
 localparam CAPTION_FILE="fm_lcd_captions.dat";
`else
 localparam CAPTION_FILE="fm_lcd_captions.mif";
`endif
 wire [8:0] q;assign character=q[7:0];
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(9),.DATA_WIDTH_B(9),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
  .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE(CAPTION_FILE),.FILL_ALL("NONE")) captions_rom(
  .dia(9'd0),.addra(addr),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(q),
  .dib(9'd0),.addrb(11'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob());
endmodule
