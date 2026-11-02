`timescale 1ns/1ps
// Coherent mailbox: data is held until its synchronized request is acknowledged.
module osd_status_cdc(
 input src_clk,src_rst,src_auto_play,
 input [7:0] src_img_idx,src_img_count,
 input video_clk,video_rst,video_vs,
 output reg video_auto_play,
 output reg [7:0] video_img_idx,video_img_count
);
 wire [16:0] value={src_auto_play,src_img_idx,src_img_count};
 reg [16:0] held,pending;
 reg request,ack;
 // Half-cycle capture avoids a near-coincident launch/capture edge between
 // the video and SD clocks. Two rising-edge stages still synchronize the ack.
 reg ack_meta;
 always @(negedge src_clk or posedge src_rst) begin
  if(src_rst) ack_meta<=0; else ack_meta<=ack;
 end
 reg [1:0] ack_sync;
 reg [2:0] request_sync;
 reg vs_d;
 always @(posedge src_clk or posedge src_rst) begin
  if(src_rst) begin held<=0;request<=0;ack_sync<=0;end
  else begin
   ack_sync<={ack_sync[0],ack_meta};
   if(ack_sync[1]==request && held!=value) begin held<=value;request<=~request;end
  end
 end
 always @(posedge video_clk or posedge video_rst) begin
  if(video_rst)begin
   request_sync<=0;ack<=0;pending<=0;vs_d<=0;
   video_auto_play<=0;video_img_idx<=0;video_img_count<=0;
  end else begin
   request_sync<={request_sync[1:0],request};vs_d<=video_vs;
   if(request_sync[2]!=ack)begin pending<=held;ack<=request_sync[2];end
   if(video_vs && !vs_d)begin
    video_auto_play<=pending[16];video_img_idx<=pending[15:8];video_img_count<=pending[7:0];
   end
  end
 end
endmodule
