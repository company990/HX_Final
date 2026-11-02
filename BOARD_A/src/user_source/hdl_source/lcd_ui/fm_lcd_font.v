// Shared 16px sans-serif glyphs; unchanged compact ROM addressing and one-cycle latency.
module fm_lcd_font(input clk,input [10:0] addr,output [15:0] bits);
 fm_font_rom_dual rom(.clk_a(clk),.addr_a(addr),.bits_a(bits),
  .clk_b(clk),.addr_b(11'd0),.bits_b());
endmodule

module fm_font_rom_dual(input clk_a,input [10:0] addr_a,output [15:0] bits_a,
 input clk_b,input [10:0] addr_b,output [15:0] bits_b);
`ifdef FM_FONT_SIM
 localparam FONT_FILE="fm_lcd_font_raw.dat";
`else
 localparam FONT_FILE="fm_lcd_font_raw.mif";
`endif
function [9:0] glyph_address;
 input [10:0] address;reg [6:0] c;reg [5:0] slot;
 begin c=address[10:4];slot=c==107 ? 0:c==122 ? 28:c[5:0];glyph_address={slot,address[3:0]};end
endfunction
function supported;input [6:0] c;begin
 supported=(c>=32 && c<=63) || (c>=65 && c<=91) || (c>=93 && c<=95) || c==107 || c==122;
end endfunction
reg va,vb;
always @(posedge clk_a)va<=supported(addr_a[10:4]);
always @(posedge clk_b)vb<=supported(addr_b[10:4]);


wire [17:0] qa,qb;
 assign bits_a=va ? qa[15:0]:16'd0;assign bits_b=vb ? qb[15:0]:16'd0;
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(18),.DATA_WIDTH_B(18),
  .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
  .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),
  .FORCE_KEEP("ON"),.IMPLEMENT("9K"),.INIT_FILE(FONT_FILE),.FILL_ALL("NONE")) shared_rom(
  .dia(18'd0),.addra(glyph_address(addr_a)),.cea(1'b1),.ocea(1'b0),.clka(clk_a),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(qa),
  .dib(18'd0),.addrb(glyph_address(addr_b)),.ceb(1'b1),.oceb(1'b0),.clkb(clk_b),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(qb));
endmodule
