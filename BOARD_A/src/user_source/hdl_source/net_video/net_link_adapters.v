`ifdef NET26_GAP_EQUIV
`timescale 1ns/1ps
`endif
// Preserve the existing 88-bit FM/DSP/time-shift/beat/PCM application verbatim.
// The new framing metadata lives outside it; media never borrows an old bit.
module net_link_a_adapter #(parameter PIXEL_STREAM=0,parameter STATUS_SOURCE_HELD=0,parameter MOSI_LANES=12)(
 input wire clk_fast,clk_app,rst,quiet,input wire [87:0] tx,input wire tx_valid,
 output wire [87:0] rx,output wire rx_valid,
 input wire [11:0] miso,output wire cs_n,sck,output wire [MOSI_LANES-1:0] mosi,
 input wire video_enable,video_play,output wire [95:0] pixels,
 output wire [2:0] pixel_flags,output wire pixel_valid,input wire pixel_ready,
 output reg remote_video_active,remote_video_done,remote_video_error);
 reg en_d;reg [3:0] epoch;
 reg launch;reg [4:0] gap;
 wire busy,valid,crc_error;wire [215:0] reply;
 wire accept_config=!busy && !launch;
 always @(posedge clk_app or posedge rst)
 if(rst)begin en_d<=0;epoch<=0;end
 else begin en_d<=video_enable;if(en_d!=video_enable)epoch<=epoch+1'b1;end
 wire [94:0] cfg;wire cfg_valid,cfg_ready;wire retire_config;
 reg cfg_pending,pending0,pending1;
 reg [94:0] cfg_snapshot;
 // WAV PCM is valid only on tx_valid (the 48 kHz sample enable).
 // The mailbox accepts cfg_pending one app clock later, so capture the
 // complete request at its valid edge instead of sampling the live bus then.
 // No reset is needed on payload: cfg_pending gates every mailbox write.
 always @(posedge clk_app)
  if(tx_valid)cfg_snapshot<={quiet,epoch,video_play,video_enable,tx};
 // A 48 kHz event first announces a new snapshot. It cannot overwrite the
 // source-held old word until the destination retires it at PHY idle.
 always @(posedge clk_app or posedge rst)
 if(rst)cfg_pending<=0;
 else if(tx_valid)cfg_pending<=1;
 else if(cfg_pending && cfg_ready)cfg_pending<=0;
 always @(posedge clk_fast or posedge rst)
 if(rst)begin pending0<=0;pending1<=0;end
 else begin pending0<=cfg_pending;pending1<=pending0;end

 net_video_mailbox #(.W(95)) config_cdc(clk_app,clk_fast,rst,
  cfg_snapshot,cfg_pending,cfg_ready,cfg,cfg_valid,retire_config);
 wire [87:0] request=cfg[87:0];
 reg config_latched,config_retired;
 reg enabled,playing,silent,remote_running;reg [3:0] session;
 assign retire_config=accept_config && config_latched && !config_retired && pending1;
 wire capture_config=cfg_valid && !config_latched && accept_config;
 reg [7:0] next_video,status_ack;reg status_seen;
 wire status_ready,video_ready;
 wire [119:0] wire_request={8'h4e,next_video,status_ack,
  status_seen,enabled,enabled,playing,session,request};
 // Media is speculative while enabled: ACK advances only after the local
 // mailbox accepts the exact next sequence. A stalled sink therefore causes
 // replay from B's eight retained slots. Advertising the instantaneous ready
 // bit at frame START inserted two empty transactions before it returned.
 wire request_present=request[47:40]==8'haa || request[47:40]==8'ha9 || request[47:40]==8'ha8;
 wire need_control=!status_seen || request[39:32]!=status_ack;
 wire rx_sink_shift;
 net_link_master_phy #(.APP_HELD(1),.SINK_SHIFT(1),.MOSI_LANES(MOSI_LANES)) phy(clk_fast,rst,launch,wire_request,miso,cs_n,sck,mosi,busy,valid,reply,crc_error,rx_sink_shift);
 wire status_packet=valid && reply[215:208]==8'h43;
 wire new_status=status_packet && (!status_seen || status_ack!=reply[39:32]);
 wire deliver_status=new_status && status_ready;
 net_video_mailbox #(.W(88),.SOURCE_HOLDS(STATUS_SOURCE_HELD)) status_cdc(clk_fast,clk_app,rst,reply[87:0],deliver_status,
  status_ready,rx,rx_valid,1'b1);
 wire media_rst=rst || !enabled;
 // The complete 216-bit RX word is stable during the final CRC beat,
 // before PHY valid rises. Predecode identity then; keep valid, enable and
 // mailbox ownership live at the ORIGINAL acceptance edge. This removes
 // header comparison from the high-fanout native RAM write-enable path.
 reg next_video_identity;
 always @(posedge clk_fast or posedge rst)
  if(rst)next_video_identity<=0;
  else next_video_identity<=reply[215:208]==8'h56 &&
       reply[199:196]==session && reply[207:200]==next_video;
 wire deliver_video=valid && enabled && next_video_identity && video_ready;
 // Serialize the completed PHY word into one narrow native RAM bank.
 // Explicit write-complete ownership gates further PHY transactions; only
 // destination consumption retires the RAM slot. Wire framing is unchanged.
 wire rx_source_captured;
 net_video_shift_rx #(.PIXEL_STREAM(PIXEL_STREAM)) video_cdc(clk_fast,clk_app,media_rst,reply[191:0],reply[194:192],
  deliver_video,video_ready,rx_source_captured,pixels,pixel_flags,pixel_valid,pixel_ready,rx_sink_shift);
 wire [2:0] remote_status;wire remote_status_valid,remote_status_ready;
 net_video_mailbox #(.W(3)) state_cdc(clk_fast,clk_app,rst,reply[194:192],
  status_packet && remote_status_ready,remote_status_ready,remote_status,remote_status_valid,1'b1);
 always @(posedge clk_app or posedge rst)
 if(rst)begin remote_video_active<=0;remote_video_done<=0;remote_video_error<=0;end
 else if(remote_status_valid)begin
  remote_video_active<=remote_status[0];remote_video_done<=remote_status[1];remote_video_error<=remote_status[2];
 end
 always @(posedge clk_fast or posedge rst)begin
  if(rst)begin config_latched<=0;config_retired<=0;enabled<=0;playing<=0;silent<=0;remote_running<=0;session<=0;next_video<=0;
   status_ack<=0;status_seen<=0;launch<=0;gap<=24;end
  else begin
   launch<=0;
   if(!cfg_valid)begin config_latched<=0;config_retired<=0;end
   if(retire_config)config_retired<=1;
   if(capture_config)begin
    config_latched<=1;
    {silent,session,playing,enabled}<=cfg[94:88];
    if(cfg[93:90]!=session || !cfg[88])next_video<=0;
   end
   if(deliver_status)begin status_ack<=reply[39:32];status_seen<=1;end
   if(status_packet)remote_running<=reply[192];
   if(deliver_video)next_video<=next_video+1'b1;
   // 24 existing 125MHz clocks let the next 16-word replay prefetch finish
   // before CS, avoiding an empty transaction. No new clock is introduced.
   // Reload from the registered launch pulse. The PHY is busy before the
   // next idle poll; completion reloads the same 24-cycle public interval.
   // Keep the acknowledgement comparator off the gap counter enable cone.
   if(valid || crc_error || launch)gap<=24;
   else if(!busy && gap!=0)gap<=gap-1'b1;
   if(!valid && !crc_error && !busy && !launch && gap==0)begin
    // Outside AV25 playback, poll only until the current 48kHz request has
    // a reply. This preserves PCM/control cadence without a continuously
    // toggling high-speed GPIO bus next to the FM tuner.
    if(config_latched && cfg_valid && !config_retired && !retire_config && rx_source_captured && (!STATUS_SOURCE_HELD || status_ready) && !silent && request_present && (need_control || (enabled && remote_running)))begin launch<=1;end
   end
  end
 end

`ifdef NET26_GAP_EQUIV
 wire [87:0] eq_rx;wire eq_rx_valid,eq_cs,eq_sck;
 wire [MOSI_LANES-1:0] eq_mosi;wire [95:0] eq_pixels;
 wire [2:0] eq_flags;wire eq_pixel_valid,eq_active,eq_done,eq_error;
 integer eq_cycles=0;
 net_link_a_adapter_reference #(.PIXEL_STREAM(PIXEL_STREAM),.STATUS_SOURCE_HELD(STATUS_SOURCE_HELD),.MOSI_LANES(MOSI_LANES)) equiv_ref(
  .clk_fast(clk_fast),.clk_app(clk_app),.rst(rst),.quiet(quiet),.tx(tx),.tx_valid(tx_valid),
  .rx(eq_rx),.rx_valid(eq_rx_valid),.miso(miso),.cs_n(eq_cs),.sck(eq_sck),.mosi(eq_mosi),
  .video_enable(video_enable),.video_play(video_play),.pixels(eq_pixels),.pixel_flags(eq_flags),
  .pixel_valid(eq_pixel_valid),.pixel_ready(pixel_ready),.remote_video_active(eq_active),
  .remote_video_done(eq_done),.remote_video_error(eq_error));
 initial $display("GAP_PUBLIC_EQUIV_ACTIVE: exact original adapter reference");
 always @(negedge clk_fast)begin
  #1;
  if(!rst)begin
   eq_cycles=eq_cycles+1;
   if({cs_n,sck,mosi,rx_valid,pixel_valid,remote_video_active,remote_video_done,remote_video_error} !==
      {eq_cs,eq_sck,eq_mosi,eq_rx_valid,eq_pixel_valid,eq_active,eq_done,eq_error})
    $fatal(1,"GAP public control/waveform mismatch cycle%0d",eq_cycles);
   if(rx_valid && rx!==eq_rx)$fatal(1,"GAP status payload mismatch");
   if(pixel_valid && {pixels,pixel_flags}!=={eq_pixels,eq_flags})$fatal(1,"GAP pixel payload mismatch");
   if(eq_cycles==65536 || eq_cycles==1048576)$display("GAP_PUBLIC_EQUIV_CHECKED: %0d cycles",eq_cycles);
  end
 end
`endif
endmodule

module net_link_b_adapter #(parameter MOSI_LANES=12)(
 input wire clk_fast,clk_app,rst,input wire cs_n,sck,input wire [MOSI_LANES-1:0] mosi,
 output wire [11:0] miso,input wire [87:0] tx,input wire tx_valid,
 output wire [87:0] rx,output wire rx_valid,
 input wire [95:0] pixels,input wire [2:0] pixel_flags,input wire pixel_valid,
 output wire pixel_ready,input wire video_active,video_done,video_error,
 output wire video_enable,video_play,output wire [3:0] video_epoch,output wire [3:0] acknowledged_groups);
 wire [119:0] request;wire incoming,crc_error,frame_active,frame_sent,frame_loaded;
 wire [23:0] loaded_header;wire payload_idle;
 reg enabled,playing;reg [3:0] epoch;reg flush;
 wire [5:0] peer_cfg;wire peer_cfg_valid,peer_cfg_ready;
 net_video_mailbox #(.W(6)) cfg_cdc(clk_fast,clk_app,rst,{epoch,playing,enabled},
  peer_cfg_ready,peer_cfg_ready,peer_cfg,peer_cfg_valid,1'b1);
 reg [5:0] local_cfg;
 always @(posedge clk_app or posedge rst)
 if(rst)local_cfg<=0;else if(peer_cfg_valid)local_cfg<=peer_cfg;
 assign {video_epoch,video_play,video_enable}=local_cfg;
 wire app_media_rst=rst || !video_enable || flush;
 reg half;reg [95:0] first_pixels;reg [2:0] first_flags;
 // Keep this CDC source in FFs: RAM WRITETHROUGH clk-to-out plus the
 // twelve-bit selector exceeded the related 125 MHz RAM-write budget.
 // It still holds the first half until the complete group is acknowledged.
 always @(posedge clk_app)
  if(!app_media_rst && pixel_valid && pixel_ready && !half)first_pixels<=pixels;
 wire packed_ready;
 wire group_valid,group_ready;wire [194:0] group;
 assign pixel_ready=video_enable && (!half || packed_ready);
 wire packed_valid=half && pixel_valid && video_enable;
 // B demux retains both halves until the returned destination ACK.
 net_direct_hold_mailbox #(.W(195),.RETIRE_VALID(1)) media_cdc(clk_app,clk_fast,app_media_rst,
  {{pixel_flags[2],pixel_flags[1],first_flags[0]},pixels,first_pixels},packed_valid,
  packed_ready,group,group_valid,group_ready);
 always @(posedge clk_app or posedge app_media_rst)
 if(app_media_rst)begin half<=0;first_flags<=0;end
 else if(pixel_valid && pixel_ready)begin
  half<=!half;if(!half)begin first_flags<=pixel_flags;end
 end
 wire [215:0] media_packet;wire media_packet_valid;wire [3:0] media_level;
 wire media_rst=rst || !enabled || flush;
 wire [11:0] replay_word;wire [3:0] replay_index;
 net_video_replay_stream #(.SOURCE_HOLDS(1)) replay(clk_fast,media_rst,epoch,group[191:0],group[194:192],
  group_valid,group_ready,incoming && request[119:112]==8'h4e,request[111:104],
  media_packet,media_packet_valid,frame_sent && loaded_header[23:16]==8'h56,media_level,acknowledged_groups,loaded_header[10:8],replay_index,replay_word,loaded_header[15:8]);
 wire reply_valid,reply_ready;wire [90:0] reply_data;
 // The CDC RAM already holds all response bits until out_ready. Retain
 // that same word through peer ACK instead of duplicating 91 output registers.
 reg response_retired;
 wire pending=reply_valid && !response_retired;
 wire [87:0] response=reply_data[87:0];
 wire [2:0] video_state=reply_data[90:88];
 net_video_mailbox #(.W(91),.RAM32K(1)) response_cdc(clk_app,clk_fast,rst,
  {video_error,video_done,video_active,tx},tx_valid,reply_ready,reply_data,reply_valid,
  payload_idle && response_retired);
 wire request_ready;reg request_seen;reg [7:0] request_seq;
 wire deliver_request=incoming && request[119:112]==8'h4e && request_ready &&
  (!request_seen || request_seq!=request[39:32]);
 net_video_mailbox #(.W(88),.DEST_CAPTURE(1)) request_cdc(clk_fast,clk_app,rst,request[87:0],
  deliver_request,request_ready,rx,rx_valid,1'b1);
 reg credit;
 wire [215:0] candidate_reply=pending ? {8'h43,response[39:32],epoch,1'b0,video_state,104'd0,response} :
  enabled && playing && credit && media_packet_valid ? media_packet:216'd0;
 // Fix the response kind at synchronized CS. In particular, a transaction
 // that starts empty stays empty if a replay prefetch finishes mid-frame.
 // The selected response/cache data remains held until frame_sent.
 wire [215:0] wire_reply=!frame_active ? candidate_reply:
  loaded_header[23:16]==8'h43 ? {loaded_header,104'd0,response}:
  loaded_header[23:16]==8'h56 ? {loaded_header,media_packet[191:0]}:216'd0;
 net_link_stream_slave_phy #(.APP_HELD(1),.MOSI_LANES(MOSI_LANES)) phy(clk_fast,rst,cs_n,sck,mosi,wire_reply,miso,incoming,
  request,crc_error,frame_active,frame_sent,frame_loaded,loaded_header,payload_idle,replay_word,replay_index);
 always @(posedge clk_fast or posedge rst)begin
  if(rst)begin enabled<=0;playing<=0;epoch<=0;flush<=0;response_retired<=0;
   request_seen<=0;request_seq<=0;credit<=0;end
  else begin
   flush<=0;
   if(incoming && request[119:112]==8'h4e)begin
    enabled<=request[93];playing<=request[92];credit<=request[94];
    epoch<=request[91:88];
    if(epoch!=request[91:88])flush<=1;
    if(pending && request[95] && request[103:96]==response[39:32])response_retired<=1;
   end
   if(deliver_request)begin request_seen<=1;request_seq<=request[39:32];end
   if(!reply_valid)response_retired<=0;
  end
 end
endmodule

module net_direct_hold_mailbox #(parameter W=195,parameter RETIRE_VALID=1)(
 input src_clk,dst_clk,rst,input [W-1:0] data,input valid,
 output ready,output [W-1:0] out_data,output reg out_valid,input out_ready);
 reg request,ack,a0,a1,r0,r1,pending;
 wire acknowledged=pending && request==a1;
 assign ready=RETIRE_VALID?acknowledged:!pending;
 assign out_data=data;
 always @(posedge src_clk or posedge rst)begin
  if(rst)begin request<=0;a0<=0;a1<=0;pending<=0;end
  else begin
   a0<=ack;a1<=a0;
   if(pending)begin if(acknowledged && (!RETIRE_VALID || valid))pending<=0;end
   else if(valid)begin pending<=1;request<=~request;end
  end
 end
 always @(posedge dst_clk or posedge rst)begin
  if(rst)begin r0<=0;r1<=0;ack<=0;out_valid<=0;end
  else begin
   r0<=request;r1<=r0;
   if(out_valid && out_ready)begin out_valid<=0;ack<=r1;end
   else if(!out_valid && r1!=ack)out_valid<=1;
  end
 end
endmodule
