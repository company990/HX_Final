// Relative FFT display energy (log amplitude), not calibrated acoustic power.
// Existing FFT bands 0..1 ~94..328Hz, 2..23 ~375..3094Hz, 24..63 ~3.1..8kHz.
module audio_visual_features #(
 parameter MIC_DISPLAY_FLOOR=8, MIC_MIN_PEAK=3,
 parameter MIC_BACKGROUND_MARGIN=1, MIC_RISE=1, MIC_COOLDOWN_FRAMES=96
)(input clk,rst,input band_valid,input[5:0] band_addr,band_height,
 input mode_mic,output reg[5:0] bass,mid,treble,energy,output reg event_toggle);
 reg[11:0] sum_all;reg[6:0] sum_low;
 reg [5:0] mid_quot,high_quot;reg [4:0] mid_rem;reg [5:0] high_rem;
 reg[5:0] expect_band;reg collecting,commit;
 reg[5:0] raw_b,raw_m,raw_t,raw_e;
 reg[5:0] mid_peak,high_peak,raw_mid_peak,raw_high_peak;
 reg[1:0] active_bands,raw_active_bands;
 reg[9:0] background;reg[5:0] previous;
 // Incremental exact quotient/remainder sums. Each band is only 0..63.
 // Invariant: sum_mid = 22*mid_quot+mid_rem; sum_high = 40*high_quot+high_rem.
 // One update per accepted FFT band; output commit cycle is unchanged.
 function [6:0] step22;input [4:0] remainder;input [5:0] height;
  reg [6:0] value,residue;reg [1:0] increment;
  begin value={2'd0,remainder}+{1'b0,height};
   if(value>=66)begin increment=3;residue=value-7'd66;end
   else if(value>=44)begin increment=2;residue=value-7'd44;end
   else if(value>=22)begin increment=1;residue=value-7'd22;end
   else begin increment=0;residue=value;end
   step22={increment,residue[4:0]};end
 endfunction
 function [7:0] step40;input [5:0] remainder;input [5:0] height;
  reg [6:0] value,residue;reg [1:0] increment;
  begin value={1'b0,remainder}+{1'b0,height};
   if(value>=80)begin increment=2;residue=value-7'd80;end
   else if(value>=40)begin increment=1;residue=value-7'd40;end
   else begin increment=0;residue=value;end
   step40={increment,residue[5:0]};end
 endfunction
 wire [6:0] next_mid=step22(mid_rem,band_height);
 wire [7:0] next_high=step40(high_rem,band_height);
 wire[6:0] mic_band_threshold={1'b0,background[9:4]}+MIC_BACKGROUND_MARGIN;
 // map_h(0)=8 in the current analyzer: this is a decorative floor, not sound.
 wire[5:0] mic_height=band_height>MIC_DISPLAY_FLOOR ? band_height-MIC_DISPLAY_FLOOR:0;
 wire[5:0] final_high_peak=mic_height>high_peak ? mic_height:high_peak;
 wire band_excursion={1'b0,mic_height}>mic_band_threshold;
 wire[1:0] final_active_bands=(band_excursion && active_bands!=3) ? active_bands+1'b1:active_bands;
 reg[$clog2(MIC_COOLDOWN_FRAMES+1)-1:0] cooldown;
 reg[5:0] warmup;reg armed;
 // 1024-point FFT, 256-sample hop: nominal updates every 5.33ms at 48kHz.
 function[5:0] envelope;input[5:0] oldv,newv;reg[6:0] d;begin
  if(newv>oldv)begin d={1'b0,newv}-{1'b0,oldv};envelope=oldv+((d+1)>>1);end
  else begin d={1'b0,oldv}-{1'b0,newv};envelope=oldv-((d+15)>>4);end
 end endfunction
 // Ordered bands finish low/mid sums before the full-frame commit.
 // Reuse their finished accumulators for pending envelope results, but
 // publish all four outputs on exactly the original commit clock.
 wire [5:0] treble_mean=high_quot+{4'd0,next_high[7:6]};
 wire [5:0] envelope_raw=commit?raw_e:band_addr==2?sum_low[6:1]:band_addr==24?mid_quot:treble_mean;
 wire [5:0] envelope_old=commit?energy:band_addr==2?bass:band_addr==24?mid:treble;
 wire [5:0] envelope_target=envelope_raw>4?envelope_raw-6'd4:6'd0;
 wire [5:0] next_envelope=envelope(envelope_old,envelope_target);
 // A brief weak clap can affect only a few bands: the 64-band mean hides it.
 // Use unsmoothed mid/high peaks for MIC only; reject bass-only/single-bin
 // interference by requiring both regions and at least three active bands.
 wire[5:0] mic_peak=raw_mid_peak>raw_high_peak ? raw_mid_peak:raw_high_peak;
 wire[5:0] detector=mode_mic ? mic_peak : raw_b;
 wire[6:0] threshold={1'b0,background[9:4]}+(mode_mic ? MIC_BACKGROUND_MARGIN:7'd5);
 wire onset=detector>=(mode_mic ? MIC_MIN_PEAK:12) && {1'b0,detector}>threshold &&
             {1'b0,detector}>{1'b0,previous}+(mode_mic ? MIC_RISE:7'd4);
 always @(posedge clk or posedge rst)begin
  if(rst)begin bass<=0;mid<=0;treble<=0;energy<=0;event_toggle<=0;
   sum_all<=0;mid_quot<=0;mid_rem<=0;high_quot<=0;high_rem<=0;sum_low<=0;expect_band<=0;collecting<=0;commit<=0;
   raw_b<=0;raw_m<=0;raw_t<=0;raw_e<=0;background<=0;previous<=0;cooldown<=0;warmup<=0;armed<=0;
   mid_peak<=0;high_peak<=0;raw_mid_peak<=0;raw_high_peak<=0;active_bands<=0;raw_active_bands<=0;
  end else begin
   commit<=0;
   if(band_valid)begin
    if(band_addr==0)begin sum_all<=band_height;sum_low<=band_height;mid_quot<=0;mid_rem<=0;high_quot<=0;high_rem<=0;collecting<=1;expect_band<=1;
     mid_peak<=0;high_peak<=0;active_bands<=0;end
    else if(collecting && band_addr==expect_band)begin
     sum_all<=sum_all+band_height;
     if(band_addr<2)sum_low<=sum_low+band_height;
     else if(band_addr<24)begin mid_quot<=mid_quot+{4'd0,next_mid[6:5]};mid_rem<=next_mid[4:0];end
     else begin high_quot<=high_quot+{4'd0,next_high[7:6]};high_rem<=next_high[5:0];end
     if(band_addr==2)begin raw_b<=sum_low[6:1];sum_low<={1'b0,next_envelope};end
     if(band_addr==24)raw_m<=next_envelope;
     if(band_addr>=2)begin
      if(band_excursion && active_bands!=3)active_bands<=active_bands+1'b1;
      if(band_addr<24)begin if(mic_height>mid_peak)mid_peak<=mic_height;end
      else if(mic_height>high_peak)high_peak<=mic_height;
     end
     expect_band<=expect_band+1'b1;
     if(band_addr==63)begin
      collecting<=0;commit<=1;
      raw_t<=next_envelope;raw_e<=(sum_all+band_height)>>6;
      raw_mid_peak<=mid_peak;raw_high_peak<=final_high_peak;raw_active_bands<=final_active_bands;
     end
    end else collecting<=0;
   end
   if(commit)begin
    bass<=sum_low[5:0];
    mid<=raw_m;
    treble<=raw_t;
    energy<=next_envelope;
    background<=background-background[9:4]+detector;
    previous<=detector;
    if(warmup<24)begin warmup<=warmup+1'b1;armed<=0;end
    else if(cooldown==0)armed<=1;
    if(cooldown!=0)cooldown<=cooldown-1'b1;
    if(warmup==24 && armed && cooldown==0 && onset &&
       (!mode_mic || ({1'b0,raw_mid_peak}>mic_band_threshold &&
                     {1'b0,raw_high_peak}>mic_band_threshold && raw_active_bands==3)))begin
     event_toggle<=~event_toggle;cooldown<=mode_mic?MIC_COOLDOWN_FRAMES:10;armed<=0;
    end
   end
  end
 end
endmodule
