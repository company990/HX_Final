// Bundled command mailbox: data held until acknowledged; no pulse-width assumptions.
module touch_command_cdc(input src_clk,dst_clk,rst,src_valid,input [3:0] src_id,
 output reg dst_valid,output reg [3:0] dst_id);
 reg req,ack;reg [3:0] held;
 reg ack0,ack1,req0,req1;
 always @(negedge src_clk or posedge rst)
  if(rst)begin ack0<=0;ack1<=0;end else begin ack0<=ack;ack1<=ack0;end
 always @(posedge src_clk or posedge rst)
  if(rst)begin req<=0;held<=0;end
  else if(src_valid && req==ack1)begin held<=src_id;req<=~req;end
 always @(negedge dst_clk or posedge rst)
  if(rst)begin req0<=0;req1<=0;end else begin req0<=req;req1<=req0;end
 always @(posedge dst_clk or posedge rst)
  if(rst)begin ack<=0;dst_valid<=0;dst_id<=0;end
  else begin dst_valid<=0;if(req1!=ack)begin dst_id<=held;dst_valid<=1;ack<=req1;end end
endmodule
