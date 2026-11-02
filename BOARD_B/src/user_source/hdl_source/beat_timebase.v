module beat_timebase #(parameter CLOCK_MHZ=25)(input wire clk,rst,output reg [31:0] now_us);
 reg [7:0] divider;
 always @(posedge clk) begin
  if(rst) begin divider<=0; now_us<=0; end
  else if(divider==CLOCK_MHZ-1) begin divider<=0; now_us<=now_us+1'b1; end
  else divider<=divider+1'b1;
 end
endmodule
