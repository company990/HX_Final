// Serial autocorrelation of the positive-flux envelope, not a second FFT.
// 93.75 Hz novelty history; 512-sample window; lags 24..112 (50..234 BPM).
// One 8x8 multiplier, one dual-port 1024x8 history. About 5.6 ms/scan at 25 MHz.
module beat_tempo_acf(
 input wire clk,rst,enable,frame_valid,input wire [13:0] flux,
 input wire [31:0] frame_us,
 output reg tempo_valid,output reg [31:0] tempo_period_us,tempo_anchor_us,
 output reg [7:0] tempo_confidence,output reg harmonic_ambiguity
);
 reg [7:0] history [0:1023];
 reg [9:0] wr,fill,base,addr_a,addr_b;
 reg [7:0] qa,qb,first_value,write_value;
 reg half_frame,write_en;
 reg [17:0] mean_q4;
 reg [31:0] last_frame,snapshot_time;
 reg [5:0] cadence;
 reg [3:0] state;
 reg [8:0] sample_index;
 reg [6:0] lag,best_lag,candidate_lag,chosen_lag,phase_index,peak_index;
 reg [7:0] phase_peak;
 reg [3:0] consistency;
 reg [24:0] score,best_score,zero_score;
 reg [31:0] score_sum;
 wire [13:0] novelty=flux>mean_q4[17:4] ? flux-mean_q4[17:4]:14'd0;
 wire [7:0] scaled=novelty>2040 ? 8'd255:novelty[10:3];
 wire [15:0] product=qa*qb;
 wire [7:0] lag_difference=best_lag>candidate_lag ? best_lag-candidate_lag:candidate_lag-best_lag;
 wire [8:0] double_best={best_lag,1'b0};
 wire [8:0] double_candidate={candidate_lag,1'b0};
 wire half_relation=double_best+2>=candidate_lag && double_best<=candidate_lag+2;
 wire double_relation=double_candidate+2>=best_lag && double_candidate<=best_lag+2;
 // Reject an isolated pair of transients: correlation must exceed 3/8 energy.
 // Reject diffuse noise: peak must exceed ~1.56 times mean lag score.
 wire quality=best_score>256 && best_score>(zero_score>>2)+(zero_score>>3) &&
   {best_score,6'd0}>({1'b0,score_sum}+({1'b0,score_sum}>>3));
 wire [6:0] normalized_lag=(candidate_lag!=0 && half_relation && double_best<=112) ? double_best[6:0]:
   (candidate_lag!=0 && double_relation && (best_lag>>1)>=24) ? (best_lag>>1):best_lag;
 wire [7:0] candidate_error=normalized_lag>candidate_lag ? normalized_lag-candidate_lag:candidate_lag-normalized_lag;
 // A single port alternates between input writes and A reads. B is read-only.
 always @(posedge clk) begin
  if(write_en) history[wr]<=write_value; else qa<=history[addr_a];
  qb<=history[addr_b];
 end
 always @(posedge clk) begin
  tempo_valid<=0;write_en<=0;
  if(rst || !enable) begin
   wr<=0;fill<=0;base<=0;addr_a<=0;addr_b<=0;first_value<=0;write_value<=0;half_frame<=0;
   mean_q4<=0;last_frame<=0;snapshot_time<=0;cadence<=0;state<=0;sample_index<=0;lag<=0;
   best_lag<=0;candidate_lag<=0;chosen_lag<=0;phase_index<=0;peak_index<=0;phase_peak<=0;
   consistency<=0;score<=0;best_score<=0;zero_score<=0;score_sum<=0;
   tempo_period_us<=0;tempo_anchor_us<=0;tempo_confidence<=0;harmonic_ambiguity<=0;
  end else begin
   if(frame_valid) begin
    mean_q4<=mean_q4-(mean_q4>>4)+{4'd0,flux};
    last_frame<=frame_us;
    if(last_frame!=0 && (frame_us-last_frame<4800 || frame_us-last_frame>5900)) begin
     fill<=0;state<=0;half_frame<=0;consistency<=0;tempo_confidence<=0;
    end else begin
     half_frame<=!half_frame;
     if(!half_frame) first_value<=scaled;
     else begin write_value<=scaled>first_value ? scaled:first_value;write_en<=1;end
    end
   end
   if(write_en) begin
    wr<=wr+1'b1;if(fill<640)fill<=fill+1'b1;
    cadence<=cadence+1'b1;
    if(state==0 && fill>=639 && cadence==63) begin
     base<=wr+1'b1;snapshot_time<=last_frame;state<=1;sample_index<=0;lag<=0;
     score<=0;best_score<=0;zero_score<=0;score_sum<=0;best_lag<=24;
    end else if(state!=0) begin
     // Restart the synchronous read micro-step after a shared-port write.
     if(state<=3) state<=1;
     else if(state>=6 && state<=8)state<=6;
    end
   end else case(state)
    0:;
    1:begin addr_a<=base-1'b1-{1'b0,sample_index};addr_b<=base-1'b1-{1'b0,sample_index}-{3'd0,lag};state<=2;end
    2:state<=3;
    3:begin score<=score+{9'd0,product};if(sample_index==511)state<=4;else begin sample_index<=sample_index+1'b1;state<=1;end end
    4:begin
     if(lag==0)zero_score<=score;
     else begin score_sum<=score_sum+{7'd0,score};if(score>best_score)begin best_score<=score;best_lag<=lag;end end
     score<=0;sample_index<=0;
     if(lag==112)state<=5;else begin lag<=lag==0 ? 7'd24:lag+1'b1;state<=1;end
    end
    5:begin
     harmonic_ambiguity<=half_relation || double_relation;
     chosen_lag<=normalized_lag;candidate_lag<=normalized_lag;
     if(quality)begin
      if(candidate_lag!=0 && candidate_error<=2)begin if(consistency<12)consistency<=consistency+1'b1;end
      else consistency<=1;
     end else consistency<=0;
     phase_index<=0;phase_peak<=0;peak_index<=0;state<=6;
    end
    6:begin addr_a<=base-1'b1-{3'd0,phase_index};state<=7;end
    7:state<=8;
    8:begin
     if(qa>phase_peak)begin phase_peak<=qa;peak_index<=phase_index;end
     if(phase_index+1'b1>=chosen_lag)state<=9;
     else begin phase_index<=phase_index+1'b1;state<=6;end
    end
    9:begin
     tempo_valid<=1;tempo_period_us<=chosen_lag*32'd10667;
     tempo_anchor_us<=snapshot_time-peak_index*32'd10667;
     tempo_confidence<=consistency>=5 ? 8'd128+{consistency,3'd0}:8'd0;
     state<=0;
    end
    default:state<=0;
   endcase
   if(frame_valid && last_frame!=0 && (frame_us-last_frame<4800 || frame_us-last_frame>5900)) begin
    fill<=0;state<=0;half_frame<=0;consistency<=0;tempo_confidence<=0;tempo_valid<=0;write_en<=0;
   end
  end
 end
endmodule
