// M10 is reserved by EG_PHY_ADC CH1, not by a digital top-level input.
module mic_spectrum_source #(
 parameter integer DEBOUNCE_CYCLES=500000,
 parameter integer FM_ADC_ENABLE=0
)(input clk_adc,clk_video,rst,input select_sw,
 output reg selected,output reg [23:0] pcm,output reg valid,
 output wire [11:0] light_data,output wire light_toggle,
 output reg [23:0] fm_pcm,output reg fm_valid,
 output wire fm_fault,fm_clipped);
 wire soc,pd,eoc,toggle;wire [11:0] adc_data;wire [23:0] adc_pcm;
 wire adc_fault,adc_clipped;
 wire adc_soc,adc_pd,mic_eoc,recover,light_fault;
 wire [2:0] adc_channel;
 wire [11:0] mic_data;
 wire fm_eoc;wire [11:0] fm_data;wire [23:0] fm_adc_pcm;wire fm_toggle;
 generate if(FM_ADC_ENABLE)begin:three_channels
 adc_fm_scheduler scheduler(.clk(clk_adc),.rst(rst),.mic_soc(soc),.mic_pd(pd),
  .eoc(eoc),.data(adc_data),.adc_soc(adc_soc),.adc_pd(adc_pd),.adc_channel(adc_channel),
  .mic_eoc(mic_eoc),.mic_data(mic_data),.light_data(light_data),.light_toggle(light_toggle),
  .fm_eoc(fm_eoc),.fm_data(fm_data),.recover(recover),.light_fault(light_fault),.fm_fault(fm_fault));
 fm_adc_decimator decimator(.clk(clk_adc),.rst(rst | recover | pd),.eoc(fm_eoc),.data(fm_data),
  .pcm(fm_adc_pcm),.toggle(fm_toggle),.clipped(fm_clipped));
 end else begin:two_channels
 adc_light_scheduler scheduler(.clk(clk_adc),.rst(rst),.mic_soc(soc),.mic_pd(pd),
  .eoc(eoc),.data(adc_data),.adc_soc(adc_soc),.adc_pd(adc_pd),.adc_channel(adc_channel),
  .mic_eoc(mic_eoc),.mic_data(mic_data),.light_data(light_data),.light_toggle(light_toggle),
  .recover(recover),.light_fault(light_fault));
 assign fm_eoc=0;assign fm_data=0;assign fm_adc_pcm=0;assign fm_toggle=0;assign fm_fault=0;assign fm_clipped=0;
 end endgenerate
 // CH1=M10 microphone, CH2=L10 conditioned ambient light; one physical ADC.
 EG_PHY_ADC #(.CH1("ENABLE"),.CH2("ENABLE"),.CH3(FM_ADC_ENABLE ? "ENABLE" : "DISABLE"),.VREF("ENABLE")) adc(
  .clk(clk_adc),.pd(adc_pd),.s(adc_channel),.soc(adc_soc),.eoc(eoc),.dout(adc_data));
 mic_adc_capture capture(.clk(clk_adc),.rst(rst | recover),.eoc(mic_eoc),.adc_data(mic_data),
  .soc(soc),.pd(pd),.pcm(adc_pcm),.pcm_toggle(toggle),.fault(adc_fault),.clipped(adc_clipped));
 reg t0,t1,seen;
 reg f0,f1,fseen;
 reg sw0,sw1,candidate;
 localparam CW=(DEBOUNCE_CYCLES<2)?1:$clog2(DEBOUNCE_CYCLES);
 reg [CW-1:0] debounce;
 // Related PLL clocks; opposite-edge sampling keeps CDC hold paths timed.
 // PCM stays stable for >20us, while the toggle crosses two sampling stages.
 always @(negedge clk_video or posedge rst) begin
  if(rst) begin t0<=0;t1<=0;f0<=0;f1<=0;end
  else begin t0<=toggle;t1<=t0;f0<=fm_toggle;f1<=f0;end
 end
 always @(posedge clk_video or posedge rst) begin
  if(rst) begin
   seen<=0;pcm<=0;valid<=0;sw0<=0;sw1<=0;candidate<=0;debounce<=0;selected<=0;
   fseen<=0;fm_pcm<=0;fm_valid<=0;
  end else begin
   valid<=0;
   fm_valid<=0;
   if(f1!=fseen)begin fseen<=f1;fm_pcm<=fm_adc_pcm;fm_valid<=1;end
   if(t1!=seen) begin seen<=t1;pcm<=adc_pcm;valid<=1;end
   sw0<=select_sw;sw1<=sw0;
   if(sw1!=candidate) begin candidate<=sw1;debounce<=0;end
   else if(debounce<DEBOUNCE_CYCLES-1) debounce<=debounce+1'b1;
   else selected<=candidate;
  end
 end
endmodule
