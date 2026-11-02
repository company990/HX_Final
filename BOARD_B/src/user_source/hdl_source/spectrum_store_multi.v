// Four 64x6 BRAM tables. The compact EQ ROM makes the target BRAM available
// again, saving LUTs while preserving every table entry and read/write cycle.
// Updates retain
// the original saturating +/-step function; independent readers avoid 64-way
// packed-vector muxes. There is still exactly one FFT and one smoothing pass.
module spectrum_store_multi #(parameter STEP=1)(
 input clk,rst,update,input band_valid,input [5:0] band_addr,band_height,
 input freeze,input [5:0] hub_addr,link_addr,debug_addr,
 output [5:0] hub_height,link_height,debug_height,output reg committed);
 reg [5:0] index;reg [1:0] phase;reg initializing,pending,busy,initialized;
 wire [5:0] target,current,unused0,unused1,unused2;
 wire [5:0] step=STEP;
 wire [5:0] next_height=current<target ? (((current+step)<target)?current+step:target):
  current>target ? (((current-step)>target)?current-step:target):current;
 wire write_current=!rst && (initializing || (busy && phase==2 && !freeze));
 wire [5:0] value=initializing ? 6'd0:next_height;
 wire [5:0] h,l,d;
 spectrum_ram64 target_ram(clk,band_valid && !rst,band_addr,band_height,index,target,6'd0,unused0);
 spectrum_ram64 hub_ram(clk,write_current,index,value,index,current,hub_addr,h);
 spectrum_ram64 link_ram(clk,write_current,index,value,index,unused1,link_addr,l);
 spectrum_ram64 debug_ram(clk,write_current,index,value,index,unused2,debug_addr,d);
 assign hub_height=initialized ? h:6'd0;
 assign link_height=initialized ? l:6'd0;
 assign debug_height=initialized ? d:6'd0;
 always @(posedge clk or posedge rst)begin
  if(rst)begin index<=0;phase<=0;initializing<=1;pending<=0;busy<=0;initialized<=0;committed<=0;end
  else begin
   committed<=0;
   if(initializing)begin
    if(index==63)begin initializing<=0;initialized<=1;index<=0;end else index<=index+1'b1;
   end else if(!freeze)begin
    if(busy)begin
     if(phase==2)begin
      phase<=0;
      if(index==63)begin busy<=0;index<=0;committed<=1;end else index<=index+1'b1;
     end else phase<=phase+1'b1;
    end else if(pending)begin busy<=1;pending<=0;phase<=0;index<=0;end
   end
   if(update)pending<=1;
  end
 end
endmodule

// Only the target needs one read port. Four 16-entry LUT-RAM banks preserve
// the old synchronous port-A behavior, including read-before-write at WA.
// This frees one 9K BRAM without changing table size or smoothing arithmetic.
module spectrum_target_lut64(input wire clk,we,input wire [5:0] wa,wd,ra,
 output reg [5:0] qa);
 wire [5:0] read_address=we?wa:ra;
`ifdef PRIORITY12_SIM
 reg [5:0] mem[0:63];
 always @(posedge clk)begin
  if(we)mem[wa]<=wd;
  qa<=mem[read_address];
 end
`else
 wire [5:0] q[0:3];
 genvar bank;
 generate for(bank=0;bank<4;bank=bank+1)begin:g_bank
  wire [3:0] high;
  EG_LOGIC_DRAM16X4 low_ram(.di(wd[3:0]),.waddr(wa[3:0]),
   .we(we && wa[5:4]==bank),.wclk(clk),.raddr(read_address[3:0]),.do(q[bank][3:0]));
  EG_LOGIC_DRAM16X4 high_ram(.di({2'd0,wd[5:4]}),.waddr(wa[3:0]),
   .we(we && wa[5:4]==bank),.wclk(clk),.raddr(read_address[3:0]),.do(high));
  assign q[bank][5:4]=high[1:0];
 end endgenerate
 always @(posedge clk)qa<=q[read_address[5:4]];
`endif
endmodule

// Atomic spectrum snapshot for the serial board link. The smoother is held
// only while 64 entries are copied (< 6 us), never for the full wire transfer.
// Metadata frames do not advance the public band index.
module spectrum_link_snapshot(input clk,rst,start,input [5:0] index,
 input [5:0] live_height,output [5:0] live_addr,
 output freeze,output [5:0] height);
 reg copying;reg [5:0] cursor;reg [1:0] phase;
 reg [5:0] first;reg valid;
 wire [5:0] q,unused;
 assign live_addr=copying ? cursor:6'd0;
 assign freeze=copying || start;
 wire write=start || (copying && phase==2);
 wire [5:0] wa=start ? 6'd0:cursor;
 spectrum_ram64 snapshot(clk,write,wa,live_height,index,q,6'd0,unused);
 assign height=index==0 ? live_height:valid ? q:6'd0;
 always @(posedge clk or posedge rst)begin
  if(rst)begin copying<=0;cursor<=0;phase<=0;first<=0;valid<=0;end
  else begin
   if(start)begin copying<=1;cursor<=1;phase<=0;first<=live_height;valid<=0;end
   else if(copying)begin
    if(phase==2)begin
     phase<=0;
     if(cursor==63)begin copying<=0;valid<=1;end else cursor<=cursor+1'b1;
    end else phase<=phase+1'b1;
   end
  end
 end
endmodule
