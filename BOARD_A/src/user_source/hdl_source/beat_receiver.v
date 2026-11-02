// A-domain version/epoch checked decoder. The transport must deliver one coherent
// 256-bit word after CRC/fragment checks. No per-bit two-FF synchronizers here.
module beat_receiver #(parameter SOURCE_HOLDS=0)(
 input wire clk,rst,enable,packet_valid,input wire [255:0] packet,input wire [31:0] now_us,
 output reg snapshot_valid,locked,source_match,observed_event,strong_event,
 output reg [31:0] next_b_us,period_us,b_timestamp_us,
 output reg [7:0] bpm,confidence,phase,output reg [2:0] state,
 output reg [1:0] source_id,output reg [15:0] source_epoch
);
 // Capture the completed packet before decoding it. The link mailbox launches
 // on the falling edge; this stage keeps decode/fanout on a full-cycle path.
 wire [255:0] packet_q;
 generate if(SOURCE_HOLDS)begin:completed_link_word
  // Production 8-fragment receiver holds the completed packet beyond both
  // decode edges. Keep the valid/time stage and all sequence/epoch checks.
  assign packet_q=packet;
 end else begin:private_capture
  reg [255:0] saved;
  always @(posedge clk)if(packet_valid)saved<=packet;
  assign packet_q=saved;
 end endgenerate
 reg packet_valid_q;
 reg [31:0] packet_time_q;
 always @(posedge clk) begin
  if(rst || !enable) packet_valid_q<=0;
  else packet_valid_q<=packet_valid;
  if(packet_valid)packet_time_q<=now_us;
 end
 reg have_packet;reg [15:0] sequence_id,onsets;reg [7:0] strong_count;reg [31:0] received_at;
 wire version_ok=packet_q[255:248]==1;
 wire epoch_change=packet_q[215:200]!=source_epoch;
 wire epoch_newer=$signed(packet_q[215:200]-source_epoch)>0;
 wire newer=$signed(packet_q[231:216]-sequence_id)>0;
 always @(posedge clk) begin
  snapshot_valid<=0;observed_event<=0;strong_event<=0;
  if(rst || !enable) begin
   locked<=0;source_match<=0;next_b_us<=0;period_us<=0;b_timestamp_us<=0;
   bpm<=0;confidence<=0;phase<=0;state<=0;source_id<=0;source_epoch<=0;
   have_packet<=0;sequence_id<=0;onsets<=0;strong_count<=0;received_at<=0;
  end else begin
   if(have_packet && now_us-received_at>=250000) begin locked<=0;confidence<=0;phase<=0;state<=3;have_packet<=0;end
   if(packet_valid_q && version_ok && (!have_packet || epoch_newer || (!epoch_change && newer))) begin
    snapshot_valid<=1;have_packet<=1;received_at<=packet_time_q;
    locked<=packet_q[247] && packet_q[246] && packet_q[39:32]>=96 && packet_q[242:240]==2;
    source_match<=packet_q[243];source_id<=packet_q[233:232];source_epoch<=packet_q[215:200];
    sequence_id<=packet_q[231:216];b_timestamp_us<=packet_q[199:168];next_b_us<=packet_q[167:136];period_us<={8'd0,packet_q[135:112]};
    bpm<=packet_q[47:40];confidence<=packet_q[39:32];phase<=(packet_q[247] && packet_q[246] && packet_q[39:32]>=96 && packet_q[242:240]==2) ? packet_q[31:24]:8'd0;state<=packet_q[242:240];
    observed_event<=have_packet && !epoch_change && packet_q[79:64]!=onsets;
    strong_event<=have_packet && !epoch_change && packet_q[7:0]!=strong_count;
    onsets<=packet_q[79:64];strong_count<=packet_q[7:0];
   end
  end
 end
endmodule
