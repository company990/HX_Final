// Parameterized payload followed by CRC-8/ATM (poly 0x07).
// SCK is an OUTPUT protocol strobe, never used as an FPGA internal clock.
module board_link_mailbox #(parameter W=48, parameter SOURCE_HOLDS=0)(input src_clk,dst_clk,rst,
 input valid,input[W-1:0] data,output reg out_valid,output reg[W-1:0] out_data);
 reg request,ack;wire[W-1:0] held;
 reg a0,a1,r0,r1;
 always @(negedge src_clk or posedge rst)
  if(rst)begin a0<=0;a1<=0;end else begin a0<=ack;a1<=a0;end
 always @(negedge dst_clk or posedge rst)
  if(rst)begin r0<=0;r1<=0;end else begin r0<=request;r1<=r0;end
 // SOURCE_HOLDS is only used for the master's completed receive shift word.
 // At 48kHz / 96 bits / 5MHz it stays stable >1us before next start;
 // destination capture/ack takes <0.3us. Keep all CDC synchronizers/capture FFs.
 generate if(SOURCE_HOLDS)begin:source_held
  assign held=data;
 end else begin:local_hold
  reg[W-1:0] stored;
  always @(posedge src_clk or posedge rst)
   if(rst)stored<=0;
   else if(valid && request==a1)stored<=data;
  assign held=stored;
 end endgenerate
 always @(posedge src_clk or posedge rst)
  if(rst)request<=0;
  else if(valid && request==a1)request<=~request;
 // Capture the held bus on the receiving falling edge. Both local clocks
 // are existing PLL-related clocks; this gives the 25->50 MHz payload a
 // half-cycle boundary instead of a zero-cycle hold requirement. The request
 // has still traversed two synchronizer stages before any bus capture.
 always @(negedge dst_clk or posedge rst)
  if(rst)begin ack<=0;out_valid<=0;out_data<=0;end
  else begin
   out_valid<=0;
   if(r1!=ack)begin out_data<=held;out_valid<=1;ack<=r1;end
  end
endmodule

module board_link_master #(parameter HALF=8, parameter W=48, parameter DIRECT_RX=0, parameter TX_PREFIX=0)(input clk,rst,start,input[W-1:0] payload,
 input miso,output reg cs_n,output reg sck,output reg mosi,output reg busy,
 output reg valid,output wire[W-1:0] received,output reg crc_error);
 reg[W-1:0] received_saved;
 localparam TX_W=W-TX_PREFIX;
 reg[TX_W+7:0] tx;reg[W+7:0] rx;reg[7:0] tx_crc,rx_crc;
 assign received=DIRECT_RX ? rx[W+7:8] : received_saved;
 reg[$clog2(W+8)-1:0] bit_index;reg[3:0] timer;
 reg mi0,mi1;
 // Every successful frame overwrites all W+8 bits before asserting valid.
 // The receive shift data needs no reset or start-clear mux; control/CRC do.
 always @(posedge clk)
  if(busy && sck && timer==1)rx<={rx[W+6:0],mi1};
 function[7:0] step;input[7:0] c;input d;begin step={c[6:0],1'b0}^((c[7]^d)?8'h07:0);end endfunction
 always @(posedge clk or posedge rst)
  if(rst)begin mi0<=0;mi1<=0;end else begin mi0<=miso;mi1<=mi0;end
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   cs_n<=1;sck<=0;mosi<=0;busy<=0;valid<=0;received_saved<=0;crc_error<=0;
   tx<=0;tx_crc<=0;rx_crc<=0;bit_index<=0;timer<=0;
  end else begin
   valid<=0;crc_error<=0;
   // Delay return sampling by two clk cycles after SCK rises. This covers
   // the slave edge synchronizer and the master's two-stage MISO path.
   // HALF must be >=5; SCK remains high throughout this sampling window.
   if(busy && sck && timer==1)begin
    if(bit_index<W)rx_crc<=step(rx_crc,mi1);
   end
   if(!busy)begin
    if(start)begin
     tx<={payload[TX_W-1:0],8'd0};tx_crc<=0;rx_crc<=0;bit_index<=0;timer<=0;
     cs_n<=0;sck<=0;mosi<=TX_PREFIX!=0 ? 1'b0 : payload[W-1];busy<=1;
    end
   end else if(timer==HALF-1)begin
    timer<=0;
    if(!sck)begin
     sck<=1;
     if(bit_index<W && bit_index>=TX_PREFIX)begin
      tx_crc<=step(tx_crc,tx[TX_W+7]);
      if(bit_index==W-1)tx[TX_W+6:TX_W-1]<=step(tx_crc,tx[TX_W+7]);
     end
    end else begin
     sck<=0;
     if(bit_index==W+7)begin
      cs_n<=1;busy<=0;mosi<=0;
      if(rx[7:0]==rx_crc)begin received_saved<=rx[W+7:8];valid<=1;end
      else crc_error<=1;
     end else begin
      bit_index<=bit_index+1'b1;
      // A's reserved 40-bit prefix is all zero. Emit it without shifting
      // the 48-bit payload through 40 redundant registers. CRC starts at 0.
      if(bit_index>=TX_PREFIX)begin tx<={tx[TX_W+6:0],1'b0};mosi<=tx[TX_W+6];end
      else mosi<=bit_index==TX_PREFIX-1 ? tx[TX_W+7] : 1'b0;
     end
    end
   end else timer<=timer+1'b1;
  end
 end
endmodule

module board_link_slave #(parameter W=48)(input clk,rst,cs_n,sck,mosi,input[W-1:0] payload,
 output wire miso,output reg valid,output reg[W-1:0] received,output reg crc_error);
 reg cs0,cs1,ck0,ck1,di0,di1,ck_d,active,finished;
 reg[W+7:0] tx,rx;reg[7:0] tx_crc,rx_crc;reg[$clog2(W+8)-1:0] bit_index;
 wire rising=ck1 && !ck_d,falling=!ck1 && ck_d;
 function[7:0] step;input[7:0] c;input d;begin step={c[6:0],1'b0}^((c[7]^d)?8'h07:0);end endfunction
 assign miso=active?tx[W+7]:1'b0;
 always @(posedge clk or posedge rst)
  if(rst)begin cs0<=1;cs1<=1;ck0<=0;ck1<=0;di0<=0;di1<=0;end
  else begin cs0<=cs_n;cs1<=cs0;ck0<=sck;ck1<=ck0;di0<=mosi;di1<=di0;end
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   ck_d<=0;active<=0;finished<=0;valid<=0;received<=0;crc_error<=0;
   tx<=0;rx<=0;tx_crc<=0;rx_crc<=0;bit_index<=0;
  end else begin
   ck_d<=ck1;valid<=0;crc_error<=0;
   if(cs1)begin active<=0;finished<=0;end
   else if(!active)begin
    active<=1;finished<=0;tx<={payload,8'd0};rx<=0;tx_crc<=0;rx_crc<=0;bit_index<=0;
   end else if(!finished)begin
    if(rising)begin
     rx<={rx[W+6:0],di1};
     if(bit_index<W)begin
      tx_crc<=step(tx_crc,tx[W+7]);rx_crc<=step(rx_crc,di1);
      if(bit_index==W-1)tx[W+6:W-1]<=step(tx_crc,tx[W+7]);
     end
     if(bit_index==W+7)begin
      finished<=1;
      if({rx[6:0],di1}==rx_crc)begin received<=rx[W+6:7];valid<=1;end
      else crc_error<=1;
     end
    end
    if(falling)begin bit_index<=bit_index+1'b1;tx<={tx[W+6:0],1'b0};end
   end
  end
 end
endmodule


// Single-clock stereo PCM queue. BRAM storage has no reset on its data array.
// Input transfers on in_valid && in_ready; output transfers on valid && take.
module board_pcm_queue #(parameter ADDR_BITS=11, parameter PREFILL=1024)(
 input wire clk,rst,input wire[31:0] in_data,input wire in_valid,
 output wire in_ready,input wire stream_end,
 output wire[31:0] data,output wire valid,input wire take,
 output wire[ADDR_BITS:0] level);
 localparam DEPTH=1<<ADDR_BITS;
 reg[31:0] mem[0:DEPTH-1];
 reg[ADDR_BITS-1:0] wp,rp;
 reg[ADDR_BITS:0] stored,count;
 reg[31:0] read_q,front;
 reg pending,front_valid,started;
 wire push=in_valid && in_ready;
 wire pop=valid && take;
 wire fetch=!pending && (!front_valid || pop) && stored!=0;
 assign in_ready=count<DEPTH;
 assign valid=started && front_valid;
 assign data=front;
 assign level=count;
 always @(posedge clk)begin
  if(push)mem[wp]<=in_data;
  if(fetch)read_q<=mem[rp];
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin wp<=0;rp<=0;stored<=0;count<=0;pending<=0;
   front<=0;front_valid<=0;started<=0;end
  else begin
   if(push)wp<=wp+1'b1;
   if(fetch)rp<=rp+1'b1;
   case({push,fetch})2'b10:stored<=stored+1'b1;2'b01:stored<=stored-1'b1;default:;endcase
   case({push,pop})2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:;endcase
   pending<=fetch;
   if(pop)front_valid<=0;
   if(pending)begin front<=read_q;front_valid<=1;end
   if(count>=PREFILL || (stream_end && count!=0))started<=1;
  end
 end
endmodule
