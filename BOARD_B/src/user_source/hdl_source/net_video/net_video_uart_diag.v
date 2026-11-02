// Optional production diagnostic. Count only cumulative, validated A ACKs;
// this is equivalent delivered media FPS, never HDMI presentation FPS.
// 12 pixels/group and 640*480 pixels/frame imply FPS*100 = groups/256.
module net_video_uart_diag #(parameter FAST_CLK_HZ=125000000)(
 input clk_fast,clk_app,rst,input [3:0] acknowledged_groups,
 input enabled,query,output tx_valid,input tx_ready,output reg [7:0] tx_data);
 localparam CW=FAST_CLK_HZ<2?1:$clog2(FAST_CLK_HZ);
 reg [CW-1:0] timer;reg [19:0] groups;reg [11:0] sample;
 reg sample_valid;wire sample_ready;
 wire [20:0] next_groups={1'b0,groups}+acknowledged_groups;
 always @(posedge clk_fast or posedge rst)begin
  if(rst)begin timer<=0;groups<=0;sample<=0;sample_valid<=0;end
  else begin
   sample_valid<=0;
   if(timer==FAST_CLK_HZ-1)begin
    timer<=0;groups<=0;
    sample<=next_groups[19:8];sample_valid<=1;
   end else begin timer<=timer+1'b1;groups<=next_groups[19:0];end
  end
 end
 wire [11:0] delivered;wire delivered_valid;
 net_video_mailbox #(.W(12)) meter_cdc(clk_fast,clk_app,rst,sample,sample_valid,
  sample_ready,delivered,delivered_valid,1'b1);
 reg [15:0] latest_bcd,working_bcd,adjusted,line_bcd;
 reg [11:0] binary_shift;reg [3:0] bits_left;
 reg converting,pending,sending;reg [4:0] index;
 integer d;
 always @*begin
  adjusted=working_bcd;
  for(d=0;d<4;d=d+1)if(working_bcd[d*4+:4]>=5)adjusted[d*4+:4]=working_bcd[d*4+:4]+3;
 end
 assign tx_valid=sending;
 always @*case(index)
  0:tx_data="B";1:tx_data="2";2:tx_data="A";3:tx_data="_";
  4:tx_data="E";5:tx_data="Q";6:tx_data="_";7:tx_data="F";
  8:tx_data="P";9:tx_data="S";10:tx_data="=";11,12:tx_data="0";
  13:tx_data={4'h3,line_bcd[15:12]};14:tx_data={4'h3,line_bcd[11:8]};
  15:tx_data=".";16:tx_data={4'h3,line_bcd[7:4]};17:tx_data={4'h3,line_bcd[3:0]};
  18:tx_data=13;default:tx_data=10;
 endcase
 always @(posedge clk_app or posedge rst)begin
  if(rst)begin latest_bcd<=0;working_bcd<=0;line_bcd<=0;binary_shift<=0;bits_left<=0;
   converting<=0;pending<=0;sending<=0;index<=0;end
  else begin
   if(query)pending<=1;
   if(delivered_valid)begin working_bcd<=0;binary_shift<=delivered;bits_left<=12;converting<=1;end
   else if(converting)begin
    working_bcd<={adjusted[14:0],binary_shift[11]};binary_shift<=binary_shift<<1;
    bits_left<=bits_left-1'b1;
    if(bits_left==1)begin
     converting<=0;latest_bcd<={adjusted[14:0],binary_shift[11]};
     if(enabled)pending<=1;
    end
   end
   if(!sending && pending && !converting && !delivered_valid)begin
    line_bcd<=latest_bcd;sending<=1;pending<=0;index<=0;
   end else if(sending && tx_ready)begin
    if(index==19)sending<=0;else index<=index+1'b1;
   end
  end
 end
endmodule

// Lock ownership through each complete ready/valid message. An FM query can
// wait for at most one 20-byte diagnostic line; bytes never interleave.
module net_video_uart_arbiter(input clk,rst,input fm_valid,input [7:0] fm_data,
 output fm_ready,input diagnostic_valid,input [7:0] diagnostic_data,output diagnostic_ready,
 output tx_valid,output [7:0] tx_data,input tx_ready);
 reg owned,diagnostic_owner;
 assign tx_valid=owned && (diagnostic_owner?diagnostic_valid:fm_valid);
 assign tx_data=diagnostic_owner?diagnostic_data:fm_data;
 assign fm_ready=owned && !diagnostic_owner && tx_ready;
 assign diagnostic_ready=owned && diagnostic_owner && tx_ready;
 always @(posedge clk or posedge rst)begin
  if(rst)begin owned<=0;diagnostic_owner<=0;end
  else if(!owned)begin
   if(fm_valid || diagnostic_valid)begin owned<=1;diagnostic_owner<=!fm_valid;end
  end else if(!(diagnostic_owner?diagnostic_valid:fm_valid))owned<=0;
 end
endmodule
