// Four-timestamp clock mapping. t1/t4 belong to A, t2/t3 belong to B.
// The shared transport owner supplies captured timestamps after CRC validation.
module beat_clock_sync #(parameter SAME_REMOTE_STAMP=0)(
 input wire clk,rst,exchange_valid,input wire [31:0] now_a_us,
 input wire [31:0] t1_a,t2_b,t3_b,t4_a,
 output reg valid,output reg signed [31:0] b_to_a_us,
 output reg [15:0] round_trip_us
);
 wire signed [31:0] ab=$signed(t1_a-t2_b),ba=$signed(t4_a-t3_b);
 wire signed [32:0] offset_sum={ab[31],ab}+{ba[31],ba};
 // This transport captures t2 and t3 on the very same B clock edge.
 // In that case B residence time is exactly zero; use A's midpoint directly.
 // Keep the general four-timestamp implementation for other transports/tests.
 wire signed [31:0] direct_rtt=$signed(t4_a-t1_a);
 wire signed [31:0] direct_estimate=$signed(t1_a)+(direct_rtt>>>1)-$signed(t3_b);
 wire signed [31:0] estimate=SAME_REMOTE_STAMP ? direct_estimate : offset_sum>>>1;
 // Algebraically identical modulo 2^32; reuse the two offset differences.
 wire signed [31:0] rtt=SAME_REMOTE_STAMP ? direct_rtt : ba-ab;
 wire signed [31:0] correction=estimate-b_to_a_us;
 reg [21:0] updated_at;
 wire [21:0] update_age=now_a_us[31:10]-updated_at;
 always @(posedge clk) begin
  if(rst)begin valid<=0;b_to_a_us<=0;round_trip_us<=0;updated_at<=0;end
  else begin
   if(valid && update_age>1954)valid<=0;
   if(exchange_valid && rtt>=0 && rtt<=2000)begin
    if(!valid)begin b_to_a_us<=estimate;valid<=1;updated_at<=now_a_us[31:10];round_trip_us<=rtt[15:0];end
    else if(correction<2000 && correction> -2000)begin
     b_to_a_us<=b_to_a_us+(correction>>>3);updated_at<=now_a_us[31:10];round_trip_us<=rtt[15:0];
    end
   end
  end
 end
endmodule
