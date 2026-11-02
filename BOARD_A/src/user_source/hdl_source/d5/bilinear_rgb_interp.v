`timescale 1ns/1ps
// One signed multiplier and ALU shared by RGB. Exact original Q0.8 math.
// done is 27 clocks after accepting start, versus 8 in the parallel engine.
// Caller must wait for done (scaler_pixel_pipeline does); pixel_out is atomic.
module bilinear_rgb_interp #(parameter SOURCE_HOLDS=0)(input clk,rst,start,output reg busy,done,
 input [23:0] p00,p10,p01,p11,input [15:0] frac_x,frac_y,output reg [23:0] pixel_out);
localparam IDLE=0,PREP=1,TOP_MUL=2,TOP_ADD=3,BOT_MUL=4,BOT_ADD=5,
 VERT_PREP=6,VERT_MUL=7,VERT_ADD=8,OUT=9;
reg [3:0] state;
reg [1:0] channel;
reg [23:0] p00_q,p10_q,p01_q,p11_q;
reg [7:0] fx_q,fy_q,mul_b;
reg [15:0] top_q8,bot_q8,completed;
reg signed [16:0] mul_a;
(* dsp_style="yes" *) wire signed [25:0] product=$signed(mul_a)*$signed({1'b0,mul_b});
reg signed [25:0] product_q;
reg [23:0] final_q16;
reg [16:0] operand_a,operand_b;
wire subtract=state==PREP || state==TOP_MUL || state==VERT_PREP;
always @*begin
 operand_a={1'b0,top_q8};operand_b={1'b0,product_q[23:8]};
 case(state)
  PREP:begin operand_a={9'd0,p10_q[7:0]};operand_b={9'd0,p00_q[7:0]};end
  TOP_MUL:begin operand_a={9'd0,p11_q[7:0]};operand_b={9'd0,p01_q[7:0]};end
  TOP_ADD:operand_a={9'd0,p00_q[7:0]};
  BOT_ADD:operand_a={9'd0,p01_q[7:0]};
  VERT_PREP:begin operand_a={1'b0,bot_q8};operand_b={1'b0,top_q8};end
 endcase
end
wire [16:0] alu=operand_a+(subtract?~operand_b:operand_b)+subtract;
wire [23:0] rounded=final_q16+24'd32768;
always @(posedge clk or posedge rst)begin
 if(rst)begin state<=IDLE;busy<=0;done<=0;pixel_out<=0;channel<=0;
 p00_q<=0;p10_q<=0;p01_q<=0;p11_q<=0;fx_q<=0;fy_q<=0;mul_b<=0;
 top_q8<=0;bot_q8<=0;completed<=0;mul_a<=0;product_q<=0;final_q16<=0;end
 else begin
  done<=0;
  case(state)
   IDLE:begin busy<=0;if(start)begin
    p00_q<=SOURCE_HOLDS ? {16'd0,p00[7:0]}:p00;p10_q<=SOURCE_HOLDS ? {16'd0,p10[7:0]}:p10;p01_q<=SOURCE_HOLDS ? {16'd0,p01[7:0]}:p01;p11_q<=SOURCE_HOLDS ? {16'd0,p11[7:0]}:p11;fx_q<=frac_x[15:8];fy_q<=frac_y[15:8];
    channel<=0;busy<=1;state<=PREP;
   end end
   PREP:begin mul_a<=alu;mul_b<=fx_q;state<=TOP_MUL;end
   TOP_MUL:begin product_q<=product;mul_a<=alu;state<=TOP_ADD;end
   TOP_ADD:begin top_q8<={alu[7:0],product_q[7:0]};state<=BOT_MUL;end
   BOT_MUL:begin product_q<=product;state<=BOT_ADD;end
   BOT_ADD:begin bot_q8<={alu[7:0],product_q[7:0]};state<=VERT_PREP;end
   VERT_PREP:begin mul_a<=alu;mul_b<=fy_q;state<=VERT_MUL;end
   VERT_MUL:begin product_q<=product;state<=VERT_ADD;end
   VERT_ADD:begin final_q16<={alu[15:0],product_q[7:0]};state<=OUT;end
   OUT:begin
    if(channel==2)begin pixel_out<={rounded[23:16],completed};busy<=0;done<=1;state<=IDLE;end
    else begin
     completed<={rounded[23:16],completed[15:8]};
     p00_q<=SOURCE_HOLDS ? {16'd0,(channel==0 ? p00[15:8]:p00[23:16])}:p00_q>>8;p10_q<=SOURCE_HOLDS ? {16'd0,(channel==0 ? p10[15:8]:p10[23:16])}:p10_q>>8;p01_q<=SOURCE_HOLDS ? {16'd0,(channel==0 ? p01[15:8]:p01[23:16])}:p01_q>>8;p11_q<=SOURCE_HOLDS ? {16'd0,(channel==0 ? p11[15:8]:p11[23:16])}:p11_q>>8;
     channel<=channel+1'b1;state<=PREP;
    end
   end
   default:begin state<=IDLE;busy<=0;end
  endcase
 end
end
endmodule
