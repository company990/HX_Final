`timescale 1ns/1ps

module image_adjust_control (

    input  wire clk,
    input  wire rst,

    input  wire brightness_next_pulse,
    input  wire contrast_next_pulse,

    output reg signed [8:0] brightness,
    output reg        [9:0] contrast_gain,

    output reg [2:0] brightness_level,
    output reg [2:0] contrast_level

);


    // =========================================================
    // Brightness:
    //
    // level 0 = -64
    // level 1 = -32
    // level 2 =   0
    // level 3 = +32
    // level 4 = +64
    //
    // Default = level 2
    // =========================================================

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            brightness_level <= 3'd2;

        end
        else if (brightness_next_pulse) begin

            if (brightness_level == 3'd4)
                brightness_level <= 3'd0;
            else
                brightness_level <= brightness_level + 3'd1;

        end

    end


    always @(*) begin

        case (brightness_level)

            3'd0: brightness = -9'sd64;
            3'd1: brightness = -9'sd32;
            3'd2: brightness =  9'sd0;
            3'd3: brightness =  9'sd32;
            3'd4: brightness =  9'sd64;

            default:
                brightness = 9'sd0;

        endcase

    end


    // =========================================================
    // Contrast:
    //
    // level 0 = 0.50x = 128
    // level 1 = 0.75x = 192
    // level 2 = 1.00x = 256
    // level 3 = 1.25x = 320
    // level 4 = 1.50x = 384
    //
    // Default = level 2
    // =========================================================

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            contrast_level <= 3'd2;

        end
        else if (contrast_next_pulse) begin

            if (contrast_level == 3'd4)
                contrast_level <= 3'd0;
            else
                contrast_level <= contrast_level + 3'd1;

        end

    end


    always @(*) begin

        case (contrast_level)

            3'd0: contrast_gain = 10'd128;
            3'd1: contrast_gain = 10'd192;
            3'd2: contrast_gain = 10'd256;
            3'd3: contrast_gain = 10'd320;
            3'd4: contrast_gain = 10'd384;

            default:
                contrast_gain = 10'd256;

        endcase

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
