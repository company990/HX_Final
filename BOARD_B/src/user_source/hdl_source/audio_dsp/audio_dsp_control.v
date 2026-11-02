// UI events through the original numbered command/ACK mechanism.
module audio_dsp_control(input clk,rst,command_valid,input [15:0] command_value,
 output command_ready,output reg command_done,output reg [3:0] command_error,
 output reg enabled,agc_enabled,compress_enabled,limiter_enabled,gate_enabled,analysis_post,
 output reg [19:0] gains,output reg [1:0] threshold_select,ratio_select,speed_select,target_select,
 output [63:0] status);
 reg [2:0] preset,band;
 assign command_ready=1'b1;
 assign status={1'b1,enabled,analysis_post,agc_enabled,compress_enabled,limiter_enabled,gate_enabled,
 preset,band,gains,threshold_select,ratio_select,speed_select,target_select,23'd0};
 function [19:0] preset_gains;input [2:0] p;
 begin case(p)
  1:preset_gains={4'd4,4'd3,4'd4,4'd5,4'd7}; // bass +9,+3,0,-3,0 dB
  2:preset_gains={4'd4,4'd5,4'd6,4'd3,4'd2}; // voice -6,-3,+6,+3,0
  3:preset_gains={4'd7,4'd5,4'd4,4'd4,4'd3}; // treble -3,0,0,+3,+9
  4:preset_gains={4'd6,4'd5,4'd3,4'd4,4'd6}; // music +6,0,-3,+3,+6
  default:preset_gains=20'h44444;
 endcase end endfunction
 wire [3:0] selected=gains[band*4+:4];
 always @(posedge clk or posedge rst)begin
  if(rst)begin enabled<=0;agc_enabled<=0;compress_enabled<=0;limiter_enabled<=1;gate_enabled<=0;
   analysis_post<=1;gains<=20'h44444;preset<=0;band<=0;threshold_select<=1;ratio_select<=1;speed_select<=1;target_select<=1;command_done<=0;command_error<=0;end
  else begin
   command_done<=0;
   if(command_valid)begin
    command_done<=1;command_error<=0;
    if(command_value[15:4]!=12'h400)command_error<=6;
    else case(command_value[3:0])
     0:enabled<=!enabled;
     1:begin preset<=preset>=4?0:preset+1'b1;gains<=preset_gains(preset>=4?0:preset+1'b1);enabled<=1;end
     2:band<=band==4?0:band+1'b1;
     3:begin if(selected!=0)gains[band*4+:4]<=selected-1'b1;preset<=5;end
     4:begin if(selected<8)gains[band*4+:4]<=selected+1'b1;preset<=5;end
     5:agc_enabled<=!agc_enabled;
     6:compress_enabled<=!compress_enabled;
     7:limiter_enabled<=!limiter_enabled;
     8:gate_enabled<=!gate_enabled;
     9:threshold_select<=threshold_select==2?0:threshold_select+1'b1;
     10:ratio_select<=ratio_select==2?0:ratio_select+1'b1;
     11:speed_select<=speed_select==2?0:speed_select+1'b1;
     12:analysis_post<=!analysis_post;
     13:target_select<=target_select==2?0:target_select+1'b1;
     14:begin enabled<=0;agc_enabled<=0;compress_enabled<=0;limiter_enabled<=1;gate_enabled<=0;
      analysis_post<=1;gains<=20'h44444;preset<=0;band<=0;threshold_select<=1;ratio_select<=1;speed_select<=1;target_select<=1;end
     default:command_error<=6;
    endcase
   end
  end
 end
endmodule
