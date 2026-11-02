// IOI consensus + bounded phase correction. Timestamps are wrapping microseconds.
// 50..240 BPM. Half/double alternatives require repeated evidence.
module beat_tracker(
 input wire clk,rst,enable, input wire [31:0] now_us,
 input wire onset, input wire [31:0] onset_us, input wire [13:0] strength,
 input wire frame_valid,
 input wire tempo_valid,input wire [31:0] tempo_period_us,tempo_anchor_us,
 input wire [7:0] tempo_confidence,input wire tempo_ambiguity,
 output wire [31:0] period_us,next_beat_us,last_beat_us,
 output reg [7:0] bpm,confidence,phase,
 output reg [2:0] state,
 output wire locked, output reg predicted,strong_beat,ambiguity
);
 // Quantized time arithmetic: 22-bit modulo ticks of 1024 us.
 // This preserves the 71.6-minute wrap and bounds quantization below one Hop.
 wire [21:0] now_t=now_us[31:10],onset_t=onset_us[31:10];
 wire [10:0] tempo_t=tempo_period_us[20:10];wire [21:0] tempo_anchor_t=tempo_anchor_us[31:10];
 reg [10:0] period_t;
 reg [21:0] next_t,last_t;
 assign period_us={11'd0,period_t,10'd0};
 assign next_beat_us={next_t,10'd0};
 assign last_beat_us={last_t,10'd0};
 localparam SEARCH=3'd0, ACQUIRE=3'd1, LOCK=3'd2, LOST=3'd3, RECOVER=3'd4;
 reg have_anchor;
 reg acf_mode;reg [21:0] last_any_onset,acf_updated;
 reg [21:0] anchor, trusted_us;
 reg [10:0] candidate;
 reg [2:0] votes;
 reg [13:0] reference_strength;
 wire [21:0] interval=onset_t-anchor;
 wire signed [11:0] delta_period=$signed({1'b0,interval[10:0]})-$signed({1'b0,period_t});
 wire [11:0] error_period=delta_period[11] ? -delta_period:delta_period;
 wire [11:0] rounded_error=error_period+12'd2;
 // While locked, tempo-change votes use consecutive raw onsets, but
 // the phase anchor is moved only by trusted grid beats. Extra vocal onsets
 // therefore cannot move the next beat or immediately destroy confidence.
 wire [21:0] vote_interval=locked ? onset_t-last_any_onset:interval;
 wire signed [11:0] delta_candidate=$signed({1'b0,vote_interval[10:0]})-$signed({1'b0,candidate});
 wire [11:0] error_candidate=delta_candidate[11] ? -delta_candidate:delta_candidate;
 wire valid_interval=vote_interval>=244 && vote_interval<=1172;
 wire matches=interval[21:11]==0 && period_t!=0 && error_period<=(period_t>>4);
 wire [10:0] half_low=(period_t>>1)-(period_t>>4);
 wire [10:0] half_high=(period_t>>1)+(period_t>>4);
 wire half_match=period_t!=0 && interval[21:11]==0 && interval[10:0]>=half_low && interval[10:0]<=half_high;
 wire [12:0] twice_period={1'b0,period_t,1'b0};
 wire [12:0] double_low=twice_period-{4'd0,period_t[10:2]};
 wire [12:0] double_high=twice_period+{4'd0,period_t[10:2]};
 wire double_match=period_t!=0 && interval[21:13]==0 && interval[12:0]>=double_low && interval[12:0]<=double_high;
 wire signed [11:0] ioi_step=delta_period[11] ? -$signed({2'b0,rounded_error[11:2]}):$signed({2'b0,rounded_error[11:2]});
 wire [10:0] acf_difference=tempo_t>period_t ? tempo_t-period_t:period_t-tempo_t;
 wire signed [11:0] acf_step=tempo_t>period_t ? $signed({3'b0,acf_difference[10:2]}):-$signed({3'b0,acf_difference[10:2]});
 wire weak_subdivision={2'b0,strength} < ({2'b0,reference_strength}-({2'b0,reference_strength}>>2));
 wire signed [21:0] grid_error=$signed(onset_t-next_t);
 reg signed [21:0] correction_error;
 always @* begin
  correction_error=grid_error;
  if(grid_error>$signed({1'b0,period_t}>>1)) correction_error=grid_error-$signed({1'b0,period_t});
  else if(grid_error<-$signed({1'b0,period_t}>>1)) correction_error=grid_error+$signed({1'b0,period_t});
 end
 assign locked=(state==LOCK && confidence>=96);
 reg div_start,div_kind,measure_pending;
 reg [21:0] div_num,div_den;
 wire div_busy,div_done;
 wire [21:0] div_q;
 wire [21:0] phase_elapsed=now_t-last_t;
 beat_divider #(.WIDTH(22)) divider(clk,rst || !enable,div_start,div_num,div_den,div_busy,div_done,div_q);
 always @(posedge clk) begin
  predicted<=0; strong_beat<=0; div_start<=0;
  if(rst || !enable) begin
   period_t<=0; next_t<=0; last_t<=0; bpm<=0; confidence<=0; phase<=0; state<=SEARCH;
   have_anchor<=0; anchor<=0; candidate<=0; trusted_us<=0; votes<=0;
   reference_strength<=0; ambiguity<=0; div_kind<=0; measure_pending<=0; div_num<=0; div_den<=1;
   acf_mode<=0;last_any_onset<=0;acf_updated<=0;
  end else begin
   if(frame_valid) measure_pending<=1;
   if(div_done) begin
    if(!div_kind) begin
     bpm<=div_q>255 ? 8'd255:div_q[7:0];
     div_kind<=1; div_num<=phase_elapsed>period_t ? 22'h3fffff:{phase_elapsed[13:0],8'd0}; div_den<=period_t; div_start<=1;
    end else phase<=div_q>255 ? 8'd255:div_q[7:0];
   end else if(measure_pending && !div_busy && !div_start && period_t!=0) begin
    measure_pending<=0; div_kind<=0; div_num<=58594+(period_t>>1); div_den<=period_t; div_start<=1;
   end
   if(locked && $signed(now_t-next_t)>=0) begin
    predicted<=1; last_t<=next_t; next_t<=next_t+period_t;
   end
   if(have_anchor && period_t!=0 && now_t-trusted_us > period_t+(period_t<<1)) begin
    state<=LOST; confidence<=0; have_anchor<=0; votes<=0;
    acf_mode<=0;
   end
   if(onset) begin
    last_any_onset<=onset_t;
    if(acf_mode) begin
     if(correction_error<$signed({1'b0,period_t}>>3) && correction_error>-$signed({1'b0,period_t}>>3) && onset_t-anchor>(period_t>>1)) begin
      next_t<=next_t+$signed(correction_error>>>3)+(($signed(now_t-next_t)>=0)?period_t:22'd0);
      trusted_us<=onset_t;anchor<=onset_t;
      strong_beat<=({1'b0,strength}>({1'b0,reference_strength}+({1'b0,reference_strength}>>2)));
      reference_strength<=strength;
     end
    end else if(!have_anchor) begin
     anchor<=onset_t; trusted_us<=onset_t; have_anchor<=1; candidate<=0; votes<=0;
     state<=state==LOST ? RECOVER:ACQUIRE;
    end else if(half_match && weak_subdivision && locked) begin
     // Do not move the anchor to a weak offbeat.
     ambiguity<=1;
    end else begin
     if(!locked || matches || double_match) anchor<=onset_t;
     if(matches) begin
      // Symmetric rounding prevents a sub-four-tick deadband (240 -> 242 BPM).
      period_t<=period_t+ioi_step;
      trusted_us<=onset_t; votes<=0; ambiguity<=0;
      reference_strength<=strength;
      if(confidence<239) confidence<=confidence+16; else confidence<=255;
      if(confidence>=80) begin
       state<=LOCK;
       if(!locked) begin next_t<=onset_t+period_t; last_t<=onset_t; end
       else next_t<=next_t+$signed(correction_error>>>2)+(($signed(now_t-next_t)>=0)?period_t:22'd0);
      end
      strong_beat<=({1'b0,strength}>({1'b0,reference_strength}+({1'b0,reference_strength}>>2)));
     end else if(valid_interval && !locked) begin
      ambiguity<=half_match || double_match;
      // Repeated missing-grid beats may be an actual tempo halving. Keep the
      // candidate alive long enough for three independent interval votes.
      if(double_match) trusted_us<=onset_t;
      if(candidate!=0 && error_candidate<=(candidate>>3)) begin
       candidate<=vote_interval;
       if(votes>=2 || period_t==0) begin
        period_t<=vote_interval; next_t<=onset_t+vote_interval; last_t<=onset_t;
        confidence<=32; acf_updated<=now_t; state<=state==LOST ? RECOVER:ACQUIRE;
        trusted_us<=onset_t; anchor<=onset_t; votes<=0; reference_strength<=strength;
       end else votes<=votes+1'b1;
      end else begin candidate<=vote_interval; votes<=1; end
      if(!double_match && !locked) confidence<=32;
     end else if(double_match) begin
      // A skipped beat outside the allowed IOI range: coast, don't invent tempo.
      trusted_us<=onset_t;
      if(confidence>32) confidence<=confidence-16;
      ambiguity<=1;
     end else if(!locked) begin
      if(confidence>32) confidence<=confidence-32; else confidence<=0;
      if(locked) state<=RECOVER;
     end
    end
   end
   // Require three consecutive raw-IOI votes for a tempo change, including
   // onsets which also fall on the old grid (e.g. every second 240 BPM beat).
   // Otherwise old-grid matches would erase the evidence for a doubled tempo.
   if(onset && locked && !acf_mode) begin
    if(!valid_interval || (half_match && weak_subdivision)) begin votes<=0;candidate<=0;end
    else if(vote_interval+ (period_t>>4) < period_t ||
       vote_interval > period_t+(period_t>>4)) begin
     candidate<=vote_interval;
     if(candidate!=0 && error_candidate<=(candidate>>3)) begin
      if(votes>=2) begin
       period_t<=vote_interval; next_t<=onset_t+vote_interval;last_t<=onset_t;
       anchor<=onset_t;trusted_us<=onset_t;confidence<=32;state<=RECOVER;acf_updated<=now_t;
       votes<=0;reference_strength<=strength;
      end else votes<=votes+1'b1;
     end else votes<=1;
    end else begin votes<=0;candidate<=0;end
   end
   // ACF supplies tempo when extra vocal/percussion onsets defeat consecutive IOI.
   // Once in ACF mode, only near-grid onsets adjust phase; offbeats don't reset it.
   if(tempo_valid && tempo_confidence>=128 && now_t-last_any_onset<tempo_t && (acf_mode || (confidence<=32 && now_t-acf_updated>period_t+(period_t>>1)))) begin
    acf_mode<=1;acf_updated<=now_t;confidence<=tempo_confidence;state<=LOCK;have_anchor<=1;
    ambiguity<=tempo_ambiguity;
    if(!acf_mode || (tempo_t>period_t ? tempo_t-period_t:period_t-tempo_t)>(period_t>>3)) begin
     period_t<=tempo_t;next_t<=tempo_anchor_t+tempo_t;last_t<=tempo_anchor_t;
     anchor<=tempo_anchor_t;trusted_us<=last_any_onset;
    end else begin
     period_t<=period_t+acf_step;
    end
   end
   // No stale full progress bar while searching or recovering.
   if(!locked) phase<=0;
   if(acf_mode && now_t-acf_updated>22'd1953)begin acf_mode<=0;state<=LOST;confidence<=0;have_anchor<=0;end
  end
 end
endmodule

