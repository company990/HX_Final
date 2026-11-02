// Positive spectral flux, adaptive background and one-frame peak confirmation.
// Input is PRE display-map magnitude, ordered bins 0..63, not rendered height.
module beat_flux(
 input wire clk, rst, enable,
 input wire band_valid, input wire [5:0] band_index,
 input wire [7:0] band_magnitude,
 input wire [31:0] frame_center_us,
 input wire [1:0] sensitivity,
 output reg frame_valid, onset,
 output reg [31:0] onset_us,
 output reg [13:0] strength, flux,
 output reg stream_error,
 output wire [31:0] flux_frame_us
);
 reg [7:0] previous [0:63] /* fehdl force_ram=1, ram_style="bram" */;
 reg valid_d;reg [5:0] index_d;reg [7:0] magnitude_d,previous_q;reg [31:0] frame_d;
 // Synchronous read-before-write history: saves LUTs on the crowded B device.
 always @(posedge clk) begin
  if(band_valid) previous_q<=previous[band_index];
  // Write the preceding sample one cycle later: no same-address BRAM read/write
  // collision for the ordered 0..63 stream, including back-to-back valid bins.
  if(valid_d) previous[index_d]<=magnitude_d;
  valid_d<=band_valid && enable && !rst;
  index_d<=band_index;magnitude_d<=band_magnitude;frame_d<=frame_center_us;
 end
 reg [5:0] expected;
 reg primed, collecting, commit;
 reg [13:0] sum, completed, peak;
 reg [6:0] changed, completed_changed, peak_changed;
 reg [17:0] mean_q4;
 reg [31:0] stamp, peak_stamp, last_stamp, last_onset;
 // peak_stamp is updated with flux on commit; no additional timestamp register.
 assign flux_frame_us=peak_stamp;
 reg have_onset,peak_rising;
 reg [4:0] warm;
 wire [7:0] difference = magnitude_d > previous_q ? magnitude_d-previous_q : 8'd0;
 wire [13:0] floor_flux = sensitivity==2'd0 ? 14'd64 : sensitivity==2'd2 ? 14'd12 : 14'd24;
 wire [14:0] threshold = {1'b0,mean_q4[17:4]} + {2'b0,mean_q4[17:5]} + {1'b0,floor_flux};
 always @(posedge clk) begin
  frame_valid<=0; onset<=0; stream_error<=0;
  if(rst || !enable) begin
   expected<=0; primed<=0; collecting<=0; commit<=0; sum<=0;
   completed<=0; peak<=0; peak_rising<=0; changed<=0; completed_changed<=0;
   peak_changed<=0; mean_q4<=0; stamp<=0;
   peak_stamp<=0; last_stamp<=0; last_onset<=0; have_onset<=0;
   warm<=0; onset_us<=0; strength<=0; flux<=0;
  end else begin
   commit<=0;
   if(valid_d) begin
    if(index_d==0) begin
     collecting<=1; expected<=1; stamp<=frame_d;
     sum<=primed ? {6'd0,difference}:14'd0;
     changed<=(primed && difference>2) ? 7'd1:7'd0;
     if(primed && (frame_d-last_stamp>32'd50000)) begin
      primed<=0; warm<=0; peak<=0; peak_rising<=0; mean_q4<=0;
      sum<=0; changed<=0; stream_error<=1;
     end
    end else if(collecting && index_d==expected) begin
     sum<=sum+{6'd0,difference};
     changed<=changed+(difference>2 ? 7'd1:7'd0);
     expected<=expected+1'b1;
     if(index_d==63) begin
      collecting<=0; primed<=1; last_stamp<=stamp;
      if(primed) begin
       completed<=sum+{6'd0,difference};
       completed_changed<=changed+(difference>2 ? 7'd1:7'd0);
       commit<=1;
      end
     end
    end else begin collecting<=0; primed<=0; warm<=0; peak<=0; peak_rising<=0; stream_error<=1; end
   end
   if(commit) begin
    frame_valid<=1; flux<=completed;
    // Q4 EWMA alpha=1/16; dynamic threshold rises with nonstationary noise.
    mean_q4<=mean_q4-(mean_q4>>4)+{4'd0,completed};
    if(warm<16) warm<=warm+1'b1;
    if(warm>=16 && peak_rising && peak>=completed && {1'b0,peak}>threshold && peak_changed>=3 &&
       (!have_onset || peak_stamp-last_onset>=32'd120000)) begin
     onset<=1; onset_us<=peak_stamp; strength<=peak;
     last_onset<=peak_stamp; have_onset<=1;
    end
    peak_rising<=completed>peak; peak<=completed; peak_changed<=completed_changed; peak_stamp<=stamp;
   end
  end
 end
endmodule
