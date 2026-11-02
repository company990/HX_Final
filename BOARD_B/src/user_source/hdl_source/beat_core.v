module beat_core(
 input wire clk,rst,enable,source_reset,
 input wire [31:0] now_us,frame_center_us,
 input wire band_valid,input wire [5:0] band_index,input wire [7:0] band_magnitude,
 input wire [1:0] sensitivity,
 output wire onset, predicted, strong_beat, locked, ambiguity,stream_error,
 output wire [31:0] onset_us,period_us,next_beat_us,
 output wire [7:0] bpm,confidence,phase,output wire [2:0] state,
 output wire [13:0] flux
);
 wire frame_valid; wire [13:0] strength; wire [31:0] last_beat_us,flux_frame_us;
 wire tempo_valid,tempo_ambiguity;wire [31:0] tempo_period,tempo_anchor;wire [7:0] tempo_confidence;
 beat_flux flux_unit(clk,rst||source_reset,enable,band_valid,band_index,band_magnitude,
  frame_center_us,sensitivity,frame_valid,onset,onset_us,strength,flux,stream_error,flux_frame_us);
 beat_tempo_acf acf(clk,rst||source_reset,enable,frame_valid,flux,flux_frame_us,
  tempo_valid,tempo_period,tempo_anchor,tempo_confidence,tempo_ambiguity);
 beat_tracker tracker(clk,rst||source_reset,enable,now_us,onset,onset_us,strength,frame_valid,
  tempo_valid,tempo_period,tempo_anchor,tempo_confidence,tempo_ambiguity,
  period_us,next_beat_us,last_beat_us,bpm,confidence,phase,state,locked,predicted,strong_beat,ambiguity);
endmodule
