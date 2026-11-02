`timescale 1ns/1ps
module video_transition_fade_hold(
 input clk,rst,frame_tick,transition_start,image_ready,input fade_enable,
 input [23:0] rgb_in,output [23:0] rgb_out,
 output reg transition_busy,midpoint_pulse,transition_done,
 output [5:0] fade_level,output black_hold,
 input animation_busy,input [5:0] animation_counter
);
 reg ready_pending,active_fade;
 // Loading is transparent. Only the prepared-image animation fades RGB.
 assign fade_level=(!active_fade || !animation_busy) ? 6'd32 :
                   animation_counter<=6'd32 ? 6'd32-animation_counter : animation_counter-6'd32;
 (* dsp_style="yes" *) wire [12:0] red_scaled=rgb_in[23:16]*fade_level;
 (* dsp_style="yes" *) wire [12:0] green_scaled=rgb_in[15:8]*fade_level;
 (* dsp_style="yes" *) wire [12:0] blue_scaled=rgb_in[7:0]*fade_level;
 assign rgb_out=fade_level==32 ? rgb_in : {red_scaled[12:5],green_scaled[12:5],blue_scaled[12:5]};
 assign black_hold=fade_level==0;
 always @(posedge clk or posedge rst)begin
  if(rst)begin active_fade<=0;transition_busy<=0;midpoint_pulse<=0;transition_done<=0;ready_pending<=0;end
  else begin
   midpoint_pulse<=0;transition_done<=0;
   if(transition_start)begin active_fade<=fade_enable;transition_busy<=1;midpoint_pulse<=1;ready_pending<=0;end
   else if(transition_busy)begin
    if(image_ready)ready_pending<=1;
    if(frame_tick && (ready_pending || image_ready))begin transition_busy<=0;transition_done<=1;ready_pending<=0;end
   end
  end
 end
endmodule
