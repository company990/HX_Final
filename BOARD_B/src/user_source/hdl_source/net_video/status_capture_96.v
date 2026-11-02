// Atomic 96-bit capture in three independent 32K RAMs; one clock, write-through.
module status_capture_96(input clk,rst,load,input [95:0] data,input [1:0] index,output [31:0] q);
`ifdef PRIORITY12_SIM
 reg [95:0] held;
 always @(posedge clk or posedge rst)if(rst)held<=0;else if(load)held<=data;
 wire [95:0] stored=held;wire initialized=1'b1;
`else
 reg initialized;wire [95:0] stored;
 always @(posedge clk or posedge rst)if(rst)initialized<=0;else if(load)initialized<=1;
 genvar bank;
 generate for(bank=0;bank<3;bank=bank+1)begin:g_bank
  EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(1),.ADDR_WIDTH_B(1),
   .DATA_DEPTH_A(2),.DATA_DEPTH_B(2),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
   .WRITEMODE_A("WRITETHROUGH"),.WRITEMODE_B("WRITETHROUGH"),.RESETMODE("SYNC"),
   .IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
   .dia(data[bank*32+:16]),.addra(1'b0),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(load && !rst),.rsta(1'b0),.bea(1'b0),.doa(stored[bank*32+:16]),
   .dib(data[bank*32+16+:16]),.addrb(1'b1),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(load && !rst),.rstb(1'b0),.beb(1'b0),.dob(stored[bank*32+16+:16]));
 end endgenerate
`endif
 reg [31:0] selected;
 always @*case(index)1:selected=stored[31:0];2:selected=stored[63:32];3:selected=stored[95:64];default:selected=0;endcase
 assign q=initialized?selected:32'd0;
endmodule
