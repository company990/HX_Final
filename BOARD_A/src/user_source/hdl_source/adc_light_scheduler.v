// Share the single ADC: retain every 192kHz microphone conversion, inserting
// one CH2 conversion after each 1024th CH1 result (187.5Hz). 12.5MHz ADC clock.
module adc_light_scheduler #(
 parameter integer LIGHT_DIV=1024
)(input clk,rst,input mic_soc,mic_pd,input eoc,input [11:0] data,
 output reg adc_soc,adc_pd,output reg [2:0] adc_channel,
 output reg mic_eoc,output reg [11:0] mic_data,
 output reg [11:0] light_data,output reg light_toggle,
 output reg recover,output reg light_fault);
 localparam CW=(LIGHT_DIV<2)?1:$clog2(LIGHT_DIV);
 localparam MIC=3'd0,SETTLE=3'd1,LIGHT=3'd2,MIC_CAPTURE=3'd3,LIGHT_CAPTURE=3'd4;
 reg [2:0] state;
 reg [CW-1:0] count;
 reg [5:0] age;
 reg busy,eoc_d,light_soc;
 reg [2:0] channel;
 // Controls settle on the falling edge, as in the original microphone path.
 always @(negedge clk or posedge rst) begin
  if(rst) begin adc_soc<=0;adc_pd<=1;adc_channel<=1;end
  else begin
   adc_pd<=mic_pd || recover;
   adc_soc<=!(mic_pd || recover) && (mic_soc || light_soc);
   adc_channel<=channel;
  end
 end
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   state<=MIC;count<=0;age<=0;busy<=0;eoc_d<=0;light_soc<=0;channel<=1;
   mic_eoc<=0;mic_data<=0;light_data<=0;light_toggle<=0;recover<=0;light_fault<=0;
  end else begin
   eoc_d<=eoc;mic_eoc<=0;light_soc<=0;recover<=0;
   if(mic_pd || recover) begin
    state<=MIC;count<=0;age<=0;busy<=0;channel<=1;
   end else begin
    if(mic_soc) begin
     // A late/missing CH2 conversion must never be mistaken for microphone data.
     if(state!=MIC) begin recover<=1;light_fault<=1;busy<=0;end
     else busy<=1;
    end
    case(state)
     // EOC marks completion; read the held result one further ADC clock later.
     // Keep the selected channel unchanged until the result has been captured.
     MIC: if(eoc && !eoc_d && busy) state<=MIC_CAPTURE;
     MIC_CAPTURE: begin
      state<=MIC;
      mic_eoc<=1;mic_data<=data;busy<=0;
      if(count==LIGHT_DIV-1) begin
       count<=0;channel<=2;age<=0;state<=SETTLE;
      end else count<=count+1'b1;
     end
     SETTLE: begin
      if(age==3) begin light_soc<=1;age<=0;state<=LIGHT;end
      else age<=age+1'b1;
     end
     LIGHT: begin
      age<=age+1'b1;
      if(eoc && !eoc_d) state<=LIGHT_CAPTURE;
      else if(age==31) begin
       recover<=1;light_fault<=1;state<=MIC;channel<=1;
      end
     end
     LIGHT_CAPTURE: begin
      light_data<=data;light_toggle<=~light_toggle;light_fault<=0;
      state<=MIC;channel<=1;age<=0;
     end
     default: begin state<=MIC;channel<=1;recover<=1;end
    endcase
   end
  end
 end
endmodule
