// Frame-boundary scheduler; 1024 us modular ticks match B's phase tracker.
// Quantization < 1.024 ms, below half of one 16.8 ms video frame.
module beat_visual_sync #(parameter FRAME_US=16800)(
 input clk,rst,enable,frame_tick,input [31:0] now_us,
 input snapshot_valid,source_match,locked,input [31:0] next_b_us,period_us,
 input signed [31:0] b_to_a_us,calibration_us,input observed_event,strong_event,
 input [7:0] fallback_energy,output reg frame_beat,frame_strong,fallback,
 output wire [7:0] pulse_level,ring_level,output wire [31:0] scheduled_us);
 // The existing compositor consumes only [7:3]. Keep envelopes in its native
 // five-bit range; fallback is selected by the existing AV controller outside.
 reg [4:0] pulse;
 assign pulse_level={pulse,3'b000};
 assign ring_level=8'd0; // Beat white-frame path removed.
 reg [21:0] received_at,target,last_fired;reg [10:0] period;
 reg valid,have_fired,strong_pending;
 wire [21:0] now_tick=now_us[31:10];
 wire [21:0] age=now_tick-received_at;
 wire signed [21:0] due_delta=now_tick+22'd8-target;
 wire signed [21:0] late_delta=now_tick-target;
 wire signed [21:0] gap=target-last_fired;
 wire usable=enable && valid && age<244;
 wire fresh_target=!have_fired || gap>$signed({12'd0,period[10:1]});
 assign scheduled_us={last_fired,10'd0};
 always @(posedge clk)begin
  frame_beat<=0;frame_strong<=0;
  if(rst || !enable)begin
   received_at<=0;target<=0;period<=488;last_fired<=0;valid<=0;
   have_fired<=0;strong_pending<=0;fallback<=1;pulse<=0;
  end else begin
   if(snapshot_valid)begin
    received_at<=now_tick;valid<=source_match && locked && period_us[31:10]>=243 && period_us[31:10]<=1173;
    target<=next_b_us[31:10]+b_to_a_us[31:10]+calibration_us[31:10];period<=period_us[20:10];
   end
   if(strong_event)strong_pending<=1;
   if(frame_tick)begin
    fallback<=!usable;
    if(pulse!=0)pulse<=pulse-((pulse>>3)+1'b1);
    if(usable && due_delta>=0 && fresh_target && late_delta<33)begin
     frame_beat<=1;pulse<=28;
     last_fired<=target;have_fired<=1;target<=target+period;
    end
    if(usable && strong_pending)begin frame_strong<=1;strong_pending<=0;end
    if(!usable)begin
     strong_pending<=0;
    end
   end
  end
 end
endmodule
