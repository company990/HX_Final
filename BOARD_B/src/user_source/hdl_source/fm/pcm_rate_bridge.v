// Related-clock mono ADC samples are rate matched to A's independent 48 kHz.
// Existing FIFO + Q8 linear interpolation; bounded occupancy correction avoids
// periodic abrupt whole-sample drops/repeats when crystal frequencies differ.
module pcm_rate_bridge(input clk,rst,input [23:0] pcm,input sample_valid,request,
 output reg [15:0] out_pcm,output reg ready,output reg overflow);
 wire fifo_ready,fifo_valid;
 wire [31:0] fifo_data;
 wire [7:0] level;
 reg take;
 board_pcm_queue #(.ADDR_BITS(7),.PREFILL(32)) fifo(clk,rst,
  {16'd0,pcm[23:8]},sample_valid,fifo_ready,1'b0,fifo_data,fifo_valid,take,level);
 reg signed [15:0] current,next;
 reg [7:0] phase;
 reg [3:0] state;
 reg double_advance;
 wire [8:0] step=level>48 ? 9'd257 : level<16 ? 9'd255 : 9'd256;
 wire [9:0] advanced={2'd0,phase}+{1'b0,step};
 wire signed [16:0] difference={next[15],next}-{current[15],current};
 wire signed [25:0] fraction=difference*$signed({1'b0,phase});
 wire signed [25:0] mixed=($signed(current)<<<8)+fraction;
 always @(posedge clk or posedge rst)begin
  if(rst)begin out_pcm<=0;ready<=0;overflow<=0;current<=0;next<=0;
   phase<=0;state<=0;take<=0;double_advance<=0;
  end else begin
   take<=0;
   if(sample_valid && !fifo_ready)overflow<=1;
   case(state)
    0:if(fifo_valid)begin current<=fifo_data[15:0];take<=1;state<=1;end
    1:state<=2;
    2:if(fifo_valid)begin next<=fifo_data[15:0];take<=1;state<=3;end
    3:begin ready<=1;state<=4;end
    4:if(request)begin
     out_pcm<=mixed>>>8;phase<=advanced[7:0];
     if(advanced>=256)begin current<=next;double_advance<=advanced>=512;state<=5;end
    end
    5:if(fifo_valid)begin
     if(double_advance)current<=fifo_data[15:0];else next<=fifo_data[15:0];
     take<=1;state<=6;
    end else begin ready<=0;out_pcm<=0;state<=0;end
    6:if(double_advance)begin double_advance<=0;state<=7;end else state<=4;
    7:state<=5;
    default:begin state<=0;ready<=0;end
   endcase
  end
 end
endmodule
