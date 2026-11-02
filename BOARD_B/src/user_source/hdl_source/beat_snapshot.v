// Proposed version-1 telemetry contract. Does NOT alter the shared wire protocol.
// ready/valid payload remains stable under backpressure; events use counters.
module beat_snapshot #(parameter SERIAL_OUTPUT=0,parameter COARSE_TELEMETRY=0)(
 input wire clk,rst,enable,source_reset,input wire [1:0] source_id,
 input wire [31:0] now_us,next_beat_us,period_us,onset_us,
 input wire onset,predicted,strong_beat,locked,ambiguity,stream_error,source_match,
 input wire [7:0] bpm,confidence,phase,input wire [2:0] state,
 input wire ready,output reg valid,output wire [255:0] payload, input wire advance, input wire [5:0] diagnostic
);
 wire [255:0] snapshot;
 reg initialized;
 reg [3:0] serial_word;
 // Full public payload still matches the former right-shifting snapshot.
 // Production SERIAL_BEAT consumes only the current low 32 bits, allowing
 // unused selector outputs to disappear without 256 per-advance FF muxes.
 assign payload=SERIAL_OUTPUT ? ((!initialized || serial_word>=8) ? 256'd0:snapshot>>(serial_word*32)):(initialized?snapshot:256'd0);
 reg [31:0] last_update,last_onset;
 reg [15:0] epoch,sequence_id,onset_count,predict_count,latency;
 reg [7:0] strong_count;reg error_seen;
 wire [31:0] age=now_us-onset_us;
 // Display telemetry may run at 30.5 Hz; onset/tempo/phase computation remains
 // at its original cadence. Counter comparison avoids a full microsecond adder.
 wire update_due=COARSE_TELEMETRY ? now_us[31:15]!=last_update[16:0] : now_us-last_update>=20000;
 wire snapshot_load=!rst && !source_reset && (!valid || ready) && update_due;
 wire [255:0] snapshot_data={8'd1,{enable,locked,ambiguity,error_seen,source_match,state},diagnostic,source_id,
      sequence_id,epoch,now_us,next_beat_us,period_us[23:0],last_onset,onset_count,predict_count,
      bpm,confidence,phase,latency,strong_count};
 net_video_capture32k #(.WIDTH(256)) snapshot_ram(clk,snapshot_load,snapshot_data,snapshot);
 always @(posedge clk)if(rst)initialized<=0;else if(snapshot_load)initialized<=1;
 always @(posedge clk) begin
  if(rst) begin
   valid<=0;serial_word<=0;last_update<=0;last_onset<=0;epoch<=0;sequence_id<=0;
   onset_count<=0;predict_count<=0;strong_count<=0;latency<=0;error_seen<=0;
  end else begin
   if(valid && ready) valid<=0;
   // Optional serial view: the link samples the current low word on this edge.
   // A new snapshot load below takes priority over shifting the final word.
   if(SERIAL_OUTPUT && valid && advance && serial_word<8)serial_word<=serial_word+1'b1;
   if(stream_error) error_seen<=1;
   if(onset) begin onset_count<=onset_count+1'b1;last_onset<=onset_us;latency<=age>65535 ? 16'hffff:age[15:0];end
   if(predicted) predict_count<=predict_count+1'b1;
   if(strong_beat) strong_count<=strong_count+1'b1;
   if(source_reset) begin
    epoch<=epoch+1'b1;onset_count<=0;predict_count<=0;strong_count<=0;error_seen<=0;
    // Invalidate any unsent snapshot belonging to the old source.
    valid<=0;last_update<=COARSE_TELEMETRY ? {15'd0,(now_us[31:15]-17'd1)} : now_us-32'd20000;
   end else if((!valid || ready) && update_due) begin
    valid<=1;last_update<=COARSE_TELEMETRY ? {15'd0,now_us[31:15]} : now_us;sequence_id<=sequence_id+1'b1;
    serial_word<=0;
   end
  end
 end
endmodule
