// EG4 ADC digital control. 192k conversions/s, DC blocker, 4:1 boxcar decimation.
// ADC clock 12.5MHz (<16MHz); SOC interval 5.2us (<10us per TN304).
module mic_adc_capture #(
 parameter integer ADC_CLK_HZ=12500000,
 parameter integer ADC_RATE=192000,
 parameter integer GAIN_SHIFT=2
)(input clk,rst,input eoc,input [11:0] adc_data,
 output reg soc,output pd,output reg [23:0] pcm,output reg pcm_toggle,
 output reg fault,output reg clipped);
 function integer gcd;
  input integer a,b;integer t;
  begin while(b!=0) begin t=a%b;a=b;b=t;end gcd=a;end
 endfunction
 localparam integer DIVISOR=gcd(ADC_CLK_HZ,ADC_RATE);
 localparam integer PERIOD_UNITS=ADC_CLK_HZ/DIVISOR;
 localparam integer STEP_UNITS=ADC_RATE/DIVISOR;
 reg [$clog2(PERIOD_UNITS)-1:0] phase;
 reg [5:0] warm,age;
 reg pending,eoc_d,have_dc;
 reg [1:0] group_count;
 reg signed [24:0] dc;
 reg signed [26:0] sum;
 wire signed [24:0] raw=$signed({1'b0,adc_data,12'b0});
 wire signed [24:0] delta=raw-dc;
 wire signed [31:0] total={{5{sum[26]}},sum}+{{7{delta[24]}},delta};
 wire signed [31:0] scaled=(total <<< GAIN_SHIFT) >>> 2;
 assign pd=(warm<8);
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   phase<=0;warm<=0;age<=0;pending<=0;eoc_d<=0;have_dc<=0;
   group_count<=0;dc<=0;sum<=0;soc<=0;pcm<=0;pcm_toggle<=0;fault<=0;clipped<=0;
  end else begin
   soc<=0;eoc_d<=eoc;
   if(warm<32) warm<=warm+1'b1;
   else begin
    if(phase>=PERIOD_UNITS-STEP_UNITS) begin
     phase<=phase-(PERIOD_UNITS-STEP_UNITS);
     if(!pending) begin soc<=1;pending<=1;age<=0;end
    end else phase<=phase+STEP_UNITS;
    if(pending) begin
     age<=age+1'b1;
     if(age==48) begin
      pending<=0;warm<=0;phase<=0;fault<=1;have_dc<=0;group_count<=0;sum<=0;
     end
    end
    if(eoc && !eoc_d && pending) begin
     pending<=0;fault<=0;
     if(!have_dc) begin dc<=raw;have_dc<=1;end
     else begin
      dc<=dc+(delta >>> 11);
      if(group_count==3) begin
       group_count<=0;sum<=0;pcm_toggle<=~pcm_toggle;
       clipped<=0;
       if(scaled>8388607) begin pcm<=24'h7fffff;clipped<=1;end
       else if(scaled< -8388608) begin pcm<=24'h800000;clipped<=1;end
       else pcm<=scaled[23:0];
      end else begin sum<=sum+delta;group_count<=group_count+1'b1;end
     end
    end
   end
  end
 end
endmodule
