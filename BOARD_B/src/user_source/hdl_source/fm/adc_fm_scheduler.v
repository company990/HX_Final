// TN304: 16 ADC clocks/conversion. CH1 keeps the original 192 kHz schedule.
// CH3 is inserted after alternate CH1 results (96 kHz). CH2 after CH1 #1024
// uses a slot without CH3, so no interval contains more than two conversions.
module adc_fm_scheduler #(parameter LIGHT_DIV=1024)(
 input clk,rst,mic_soc,mic_pd,eoc,input [11:0] data,
 output reg adc_soc,adc_pd,output reg [2:0] adc_channel,
 output reg mic_eoc,output reg [11:0] mic_data,
 output reg fm_eoc,output reg [11:0] fm_data,
 output reg [11:0] light_data,output reg light_toggle,
 output reg recover,light_fault,fm_fault);
 localparam MIC=0,CAP_MIC=1,SETTLE=2,EXTRA=3,CAP_EXTRA=4;
 reg [2:0] state,channel;
 reg [$clog2(LIGHT_DIV)-1:0] count;
 reg [5:0] age;
 reg busy,eoc_d,extra_soc;
 always @(negedge clk or posedge rst)
  if(rst)begin adc_soc<=0;adc_pd<=1;adc_channel<=1;end
  else begin adc_pd<=mic_pd || recover;
   adc_soc<=!(mic_pd || recover) && ((state==MIC && mic_soc) || extra_soc);
   adc_channel<=channel;
  end
 always @(posedge clk or posedge rst)begin
  if(rst)begin state<=MIC;channel<=1;count<=0;age<=0;busy<=0;eoc_d<=0;extra_soc<=0;
   mic_eoc<=0;mic_data<=0;fm_eoc<=0;fm_data<=0;light_data<=0;light_toggle<=0;
   recover<=0;light_fault<=0;fm_fault<=0;
  end else begin
   eoc_d<=eoc;mic_eoc<=0;fm_eoc<=0;extra_soc<=0;recover<=0;
   if(mic_pd || recover)begin state<=MIC;channel<=1;busy<=0;age<=0;count<=0;end
   else begin
    if(mic_soc)begin
     if(state!=MIC)begin recover<=1;light_fault<=1;fm_fault<=1;state<=MIC;channel<=1;busy<=0;end
     else begin busy<=1;age<=0;end
    end
    case(state)
     MIC: if(eoc && !eoc_d && busy)state<=CAP_MIC;
     CAP_MIC:begin mic_data<=data;mic_eoc<=1;busy<=0;age<=0;
      count<=count+1'b1;
      if(count==LIGHT_DIV-1)begin count<=0;channel<=2;state<=SETTLE;end
      else if(!count[0])begin channel<=3;state<=SETTLE;end
      else state<=MIC;
     end
     SETTLE:if(age==3)begin extra_soc<=1;age<=0;state<=EXTRA;end else age<=age+1'b1;
     EXTRA:begin
      age<=age+1'b1;
      if(eoc && !eoc_d)state<=CAP_EXTRA;
      else if(age==27)begin recover<=1;fm_fault<=1;light_fault<=1;state<=MIC;channel<=1;end
     end
     CAP_EXTRA:begin
      if(channel==3)begin fm_data<=data;fm_eoc<=1;fm_fault<=0;end
      else begin light_data<=data;light_toggle<=~light_toggle;light_fault<=0;end
      channel<=1;state<=MIC;
     end
     default:begin recover<=1;state<=MIC;channel<=1;end
    endcase
   end
  end
 end
endmodule

// FM-only Q4 gain. One coefficient step per valid sample smooths changes.
// Target gains are 1/2/4/8/16; 24-bit saturation prevents wraparound.
module fm_pcm_gain(input clk,rst,input [23:0] in_pcm,input in_valid,
 input [2:0] gain_shift,output reg [23:0] out_pcm,
 output reg out_valid,output reg clipped);
 reg [8:0] gain_q4;
 wire [8:0] target=9'd16 << (gain_shift>4 ? 3'd4 : gain_shift);
 reg signed [33:0] product;
 reg product_valid;
 wire signed [33:0] scaled=(product+34'sd8)>>>4;
 always @(posedge clk or posedge rst)begin
  if(rst)begin gain_q4<=16;product<=0;product_valid<=0;out_pcm<=0;out_valid<=0;clipped<=0;end
  else begin
   product_valid<=in_valid;out_valid<=product_valid;
   if(in_valid)begin
    product<=$signed(in_pcm)*$signed({1'b0,gain_q4});
    if(gain_q4<target)gain_q4<=gain_q4+1'b1;
    else if(gain_q4>target)gain_q4<=gain_q4-1'b1;
   end
   if(product_valid)begin
    clipped<=0;
    if(scaled>8388607)begin out_pcm<=24'h7fffff;clipped<=1;end
    else if(scaled< -8388608)begin out_pcm<=24'h800000;clipped<=1;end
    else out_pcm<=scaled[23:0];
   end
  end
 end
endmodule

// 96 kHz unsigned ADC -> DC-blocked signed 24-bit 48 kHz mono PCM.
module fm_adc_decimator(input clk,rst,eoc,input [11:0] data,
 output reg [23:0] pcm,output reg toggle,output reg clipped);
 reg have_dc,half;
 reg signed [24:0] dc,first;
 wire signed [24:0] raw=$signed({1'b0,data,12'd0});
 wire signed [24:0] delta=raw-dc;
 wire signed [25:0] sum={first[24],first}+{delta[24],delta};
 wire signed [25:0] average=sum>>>1;
 always @(posedge clk or posedge rst)
  if(rst)begin pcm<=0;toggle<=0;clipped<=0;have_dc<=0;half<=0;dc<=0;first<=0;end
  else if(eoc)begin
   if(!have_dc)begin have_dc<=1;dc<=raw;end
   else begin
    dc<=dc+(delta>>>10);
    half<=~half;
    if(!half)first<=delta;
    else begin
     toggle<=~toggle;clipped<=data<16 || data>4079;
     if(average>8388607)begin pcm<=24'h7fffff;clipped<=1;end
     else if(average< -8388608)begin pcm<=24'h800000;clipped<=1;end
     else pcm<=average[23:0];
    end
   end
  end
endmodule
