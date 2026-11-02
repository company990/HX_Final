// Unsigned restoring divider: 32 cycles, no combinational '/' or DSP.
module beat_divider #(parameter WIDTH=32)(input wire clk,rst,start,input wire [WIDTH-1:0] numerator,denominator,
 output reg busy,done,output reg [WIDTH-1:0] quotient);
 reg [WIDTH-1:0] divisor, work_q;
 reg [WIDTH:0] remainder;
 reg [5:0] count;
 wire [WIDTH:0] trial={remainder[WIDTH-1:0],work_q[WIDTH-1]};
 wire ge=trial>={1'b0,divisor};
 always @(posedge clk) begin
  done<=0;
  if(rst) begin busy<=0; quotient<=0; divisor<=1; work_q<=0; remainder<=0; count<=0; end
  else if(start && !busy) begin
   divisor<=denominator==0 ? 32'd1:denominator; work_q<=numerator; remainder<=0; count<=0; busy<=1;
  end else if(busy) begin
   remainder<=ge ? trial-{1'b0,divisor}:trial;
   work_q<={work_q[WIDTH-2:0],ge}; count<=count+1'b1;
   if(count==WIDTH-1) begin busy<=0; done<=1; quotient<={work_q[WIDTH-2:0],ge}; end
  end
 end
endmodule
