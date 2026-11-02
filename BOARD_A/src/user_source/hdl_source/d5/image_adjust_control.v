`timescale 1ns/1ps

// Fine, saturating adjustments. Commands are latched and committed only at a
// frame boundary, so no video frame contains two different settings.
module image_adjust_control (
    input wire touch_valid,input wire [3:0] touch_id,input wire frame_tick,
    input wire clk,rst,
    input wire brightness_next_pulse,contrast_next_pulse,reverse,
    output reg signed [8:0] brightness,
    output reg [9:0] contrast_gain
);
    localparam signed [8:0] BRIGHT_MIN=-9'sd64,BRIGHT_MAX=9'sd64;
    localparam [9:0] CONTRAST_MIN=10'd128,CONTRAST_MAX=10'd384;
    reg pending_touch;
    reg [3:0] saved_touch;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pending_touch<=0;saved_touch<=0;
        end else begin
            if (frame_tick) pending_touch<=0;
            if (touch_valid && (touch_id==6 || touch_id==7 ||
                                touch_id==9 || touch_id==11)) begin
                pending_touch<=1;
                saved_touch<=touch_id;
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            brightness<=0;
            contrast_gain<=10'd256;
        end else if (frame_tick) begin
            if (pending_touch && saved_touch==6) begin
                if (brightness>BRIGHT_MIN) brightness<=brightness-9'sd2;
            end else if (pending_touch && saved_touch==7) begin
                if (brightness<BRIGHT_MAX) brightness<=brightness+9'sd2;
            end else if (brightness_next_pulse) begin
                if (reverse && brightness>BRIGHT_MIN) brightness<=brightness-9'sd2;
                else if (!reverse && brightness<BRIGHT_MAX) brightness<=brightness+9'sd2;
            end

            if (pending_touch && saved_touch==9) begin
                if (contrast_gain>CONTRAST_MIN) contrast_gain<=contrast_gain-10'd4;
            end else if (pending_touch && saved_touch==11) begin
                if (contrast_gain<CONTRAST_MAX) contrast_gain<=contrast_gain+10'd4;
            end else if (contrast_next_pulse) begin
                if (reverse && contrast_gain>CONTRAST_MIN) contrast_gain<=contrast_gain-10'd4;
                else if (!reverse && contrast_gain<CONTRAST_MAX) contrast_gain<=contrast_gain+10'd4;
            end
        end
    end
endmodule

// In image mode, K4 by itself toggles auto-play on release. Holding K4 while
// pressing K3 marks a decrement chord and suppresses the auto-play click.
// Both inputs are already debounced by audio_panel_control. The auto-play
// output stays low across a full video frame for the SD-domain key sampler.
module image_key4_chord (
    input wire clk,rst,frame_tick,enabled,key4_n,key3_n,
    output wire auto_n
);
    localparam [1:0] IDLE=0,HOLD=1,AUTO_FIRST=2,AUTO_SECOND=3;
    reg [1:0] state;
    reg chord_used;
    assign auto_n=(state!=AUTO_FIRST && state!=AUTO_SECOND);
    always @(posedge clk or posedge rst) begin
        if (rst) begin state<=IDLE;chord_used<=0;end
        else if (!enabled) begin state<=IDLE;chord_used<=0;end
        else case (state)
            IDLE: if (!key4_n) begin state<=HOLD;chord_used<=!key3_n;end
            HOLD: begin
                if (!key3_n) chord_used<=1;
                if (key4_n) state<=chord_used || !key3_n ? IDLE : AUTO_FIRST;
            end
            AUTO_FIRST: if (frame_tick) state<=AUTO_SECOND;
            AUTO_SECOND: if (frame_tick) state<=IDLE;
            default: state<=IDLE;
        endcase
    end
endmodule

// K3 is already debounced in the video domain by audio_panel_control.
// A frame counter replaces the former 50 MHz wide hold counter and CDC path.
module adjust_key_frame_repeat #(
    parameter integer DELAY_FRAMES=18,
    parameter integer PERIOD_FRAMES=5
) (
    input wire clk,rst,frame_tick,button_n,adjust_select,reverse_n,
    output reg event_pulse,selected_contrast,selected_reverse
);
    localparam integer MAX_FRAMES=DELAY_FRAMES>PERIOD_FRAMES ?
                                  DELAY_FRAMES : PERIOD_FRAMES;
    localparam integer CW=MAX_FRAMES<2 ? 1 : $clog2(MAX_FRAMES+1);
    reg [CW-1:0] count;
    reg button_d;
    reg [1:0] select_sync;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            button_d<=1'b1;select_sync<=0;count<=0;
            selected_contrast<=0;selected_reverse<=0;event_pulse<=0;
        end else begin
            button_d<=button_n;
            select_sync<={select_sync[0],adjust_select};
            event_pulse<=0;
            if (button_d && !button_n) begin
                selected_contrast<=select_sync[1];
                selected_reverse<=!reverse_n;
                event_pulse<=1;
                count<=DELAY_FRAMES;
            end else if (button_n) begin
                count<=0;
            end else if (frame_tick && count==1) begin
                event_pulse<=1;
                count<=PERIOD_FRAMES;
            end else if (frame_tick && count!=0) count<=count-1'b1;
        end
    end
endmodule

// Two frame-rate PID channels share one 10-bit add/subtract datapath.
// P=1/8, I=1/8, D=1/2; derivative uses the previous applied step.
// Saturated integral and a monotonic two-code slew guard prevent overshoot.
// Both requests originate in vertical blank; updates finish within 16 clocks.
module brightness_pid_pair(
 input wire clk,rst,tick_manual,tick_gain,
 input wire signed [7:0] target_manual,
 input wire [6:0] target_gain,
 output reg signed [7:0] value_manual,
 output reg [6:0] value_gain
);
 reg [2:0] state;
 reg channel,pending_manual,pending_gain;
 reg signed [5:0] integral_manual,integral_gain,integral_next;
 reg signed [2:0] step_manual,step_gain;
 reg signed [8:0] error;
 reg signed [9:0] accumulator;
 wire signed [7:0] current_value=channel ? $signed({1'b0,value_gain}) : value_manual;
 wire signed [5:0] current_integral=channel ? integral_gain : integral_manual;
 wire signed [2:0] previous_step=channel ? step_gain : step_manual;
 reg signed [2:0] step;
 reg signed [9:0] operand_a,operand_b;
 reg subtract;
 wire signed [9:0] result=operand_a+(operand_b ^ {10{subtract}})+subtract;
 always @* begin
  step=0;
  if(error>0)begin
   if(accumulator>=2 && error>=2)step=2;else step=1;
  end else if(error<0)begin
   if(accumulator<= -2 && error<= -2)step=-2;else step=-1;
  end
  operand_a=0;operand_b=0;subtract=0;
  case(state)
   1:begin operand_a=accumulator;operand_b=current_value;subtract=1;end
   2:begin operand_a=error;operand_b=current_integral;end
   3:begin operand_a=$signed(error)>>>3;operand_b=$signed(integral_next)>>>3;end
   4:begin operand_a=accumulator;operand_b=$signed(previous_step)>>>1;subtract=1;end
   5:begin operand_a=current_value;operand_b=step;end
   default:begin end
  endcase
 end
 always @(posedge clk or posedge rst) begin
  if(rst)begin
   value_manual<=0;value_gain<=64;state<=0;channel<=0;
   pending_manual<=0;pending_gain<=0;integral_manual<=0;integral_gain<=0;
   step_manual<=0;step_gain<=0;integral_next<=0;error<=0;accumulator<=0;
  end else begin
   if(tick_manual)pending_manual<=1;
   if(tick_gain)pending_gain<=1;
   case(state)
    0:begin
     if(pending_manual)begin
      pending_manual<=0;channel<=0;accumulator<=target_manual;state<=1;
     end else if(pending_gain)begin
      pending_gain<=0;channel<=1;accumulator<=$signed({1'b0,target_gain});state<=1;
     end
    end
    1:begin error<=result[8:0];state<=2;end
    2:begin
     if(error==0 || (error>0 && current_integral<0) || (error<0 && current_integral>0))integral_next<=0;
     else if(result>16)integral_next<=16;
     else if(result< -16)integral_next<=-16;
     else integral_next<=result[5:0];
     state<=3;
    end
    3:begin accumulator<=result;state<=4;end
    4:begin accumulator<=result;state<=5;end
    5:begin
     if(channel)begin value_gain<=result[6:0];integral_gain<=integral_next;step_gain<=step;end
     else begin value_manual<=result[7:0];integral_manual<=integral_next;step_manual<=step;end
     state<=0;
    end
    default:state<=0;
   endcase
  end
 end
endmodule
