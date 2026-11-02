module board_link_a #(parameter NET_VIDEO_EXT=0,parameter BEAT_EXT=0,parameter TIMEOUT=2500000,parameter INDEXED=0,parameter FM_EXT=0,parameter SCAN_EXT=0,parameter DSP_EXT=0,parameter TIME_EXT=0,parameter RF_QUIET_CYCLES=0)(input clk50,clk25,rst,sample_tick,
 input[23:0] pcm,input mode_mic,input boot_done,input miso,output cs_n,sck,mosi,
 output reg online,output reg remote_mic,output reg present,light_ready,light_fault,
 output reg[1:0] light_level,output reg[11:0] light_adc,
 output reg[383:0] heights,output reg heights_valid,output reg spectrum_ready,
 input[5:0] read_addr,input freeze,output wire[5:0] read_height,
 output reg net_mode,output reg net_valid,output reg[31:0] net_pcm,
 output reg[23:0] av_features,output reg av_event,
 input requested_legacy_mic,
 input fm_request_valid,input [7:0] fm_request_seq,input [2:0] fm_request_op,
 input [15:0] fm_request_value,output wire [127:0] fm_status,output reg fm_status_valid,
 output reg scan_point_valid,output reg [7:0] scan_generation,scan_index,
 output reg [15:0] scan_data,output reg [127:0] scan_status,output reg scan_status_valid,
 input [31:0] stereo_pcm,input stereo_valid,output wire [127:0] dsp_status,output reg dsp_status_valid,
 input history_mode,input [1:0] play_epoch,
 output reg capture_event,raw_valid,packet_gap,output reg [15:0] raw_pcm,
 input [31:0] beat_now_us,input beat_enable,input [1:0] beat_sensitivity,
 output wire beat_packet_valid,output wire [255:0] beat_packet,
 output reg beat_exchange_valid,output reg [31:0] beat_t1,beat_t2,beat_t3,beat_t4,
 input clk_fast,input [10:0] miso_extra,output [10:0] mosi_extra,
 input video_enable,video_play,output [95:0] video_pixels,output [2:0] video_flags,
 output video_valid,input video_ready,output remote_video_active,remote_video_done,remote_video_error);
 reg [127:0] fm_status_storage,dsp_status_storage;
 wire [127:0] fm_status_capture,dsp_status_capture;
 assign fm_status=TIME_EXT ? fm_status_capture:fm_status_storage;
 assign dsp_status=TIME_EXT ? dsp_status_capture:dsp_status_storage;
 reg[7:0] seq,last_seq;reg seen;
 // UART Z diagnostics are disabled in the integrated LCD-only build.
 // A nonzero parameter still preserves the legacy standalone diagnostic.
 localparam QW=RF_QUIET_CYCLES<1 ? 1:$clog2(RF_QUIET_CYCLES+1);
 reg [QW-1:0] quiet_left;reg quiet_seen;
 // Z:1 on B toggles reserved status bit92. Silence only link starts for
 // two seconds; an in-flight frame completes. All clocks/HDMI stay running.
 always @(posedge clk25 or posedge rst)begin
  if(rst)begin quiet_left<=0;quiet_seen<=0;end
  else begin
   if(quiet_left!=0)quiet_left<=quiet_left-1'b1;
   if(FM_EXT && fm_status_valid)begin
    quiet_seen<=fm_status[92];
    if(fm_status[78] && fm_status[92]!=quiet_seen)quiet_left<=RF_QUIET_CYCLES;
   end
  end
 end
 wire beat_clock_request=BEAT_EXT && seq[3:0]==4;
 wire [3:0] beat_flags={(BEAT_EXT!=0),beat_enable,beat_sensitivity};
 wire [3:0] time_flags=TIME_EXT ? {1'b1,history_mode,play_epoch} : 4'd0;
 wire [3:0] dsp_flags=DSP_EXT ? {1'b1,boot_done,requested_legacy_mic,stereo_valid} : 4'd0;
 wire [39:0] ordinary_extension={fm_request_seq,fm_request_op,fm_request_value,fm_request_valid,
   time_flags,BEAT_EXT ? beat_flags : 4'd0,dsp_flags};
 wire [39:0] extension=beat_clock_request ? {beat_now_us[31:4],time_flags,beat_now_us[3:0],dsp_flags} : ordinary_extension;
 reg [223:0] beat_assembling;reg [2:0] beat_expected,beat_epoch;
 reg [1:0] clock_expected,clock_epoch;
 wire[87:0] tx25={FM_EXT ? extension : 40'd0,DSP_EXT ? 8'haa : FM_EXT ? 8'ha9 : 8'ha8,seq,DSP_EXT ? stereo_pcm : {6'd0,boot_done,(FM_EXT ? requested_legacy_mic : mode_mic),pcm}};
 reg [127:0] dsp_assembling;reg [1:0] dsp_expected,dsp_epoch;
 reg [127:0] fm_assembling;
 reg [1:0] fm_expected,fm_epoch;
 reg [127:0] scan_assembling;reg [1:0] scan_expected,scan_epoch;
 wire txvalid;wire[87:0] tx50;
 wire rxvalid50,rxvalid25;wire[87:0] rx50,rx25;
 wire busy,crc_error;
 generate if(NET_VIDEO_EXT)begin:net_video_transport
  net_link_a_adapter #(.PIXEL_STREAM(1)) adapter(.clk_fast(clk_fast),.clk_app(clk25),.rst(rst),
   .quiet(quiet_left!=0),.tx(tx25),.tx_valid(sample_tick),.rx(rx25),.rx_valid(rxvalid25),
   .miso({miso_extra,miso}),.cs_n(cs_n),.sck(sck),.mosi({mosi_extra,mosi}),
   .video_enable(video_enable),.video_play(video_play),.pixels(video_pixels),
   .pixel_flags(video_flags),.pixel_valid(video_valid),.pixel_ready(video_ready),
   .remote_video_active(remote_video_active),.remote_video_done(remote_video_done),.remote_video_error(remote_video_error));
  defparam adapter.phy.CRC_PIPE=1;
  defparam adapter.status_cdc.DEST_CAPTURE=1;
  defparam adapter.STATUS_SOURCE_HELD=1;

 end else begin:legacy_transport
  board_link_mailbox #(.W(88)) mtx(clk25,clk50,rst,sample_tick && quiet_left==0,tx25,txvalid,tx50);
  board_link_master #(.W(88),.HALF(5),.DIRECT_RX(1),.TX_PREFIX(FM_EXT ? 0 : 40)) master(clk50,rst,txvalid,tx50,miso,cs_n,sck,mosi,busy,rxvalid50,rx50,crc_error);
  board_link_mailbox #(.W(88),.SOURCE_HOLDS(1)) mrx(clk50,clk25,rst,rxvalid50,rx50,rxvalid25,rx25);
  assign mosi_extra=0;assign video_pixels=0;assign video_flags=0;assign video_valid=0;
  assign remote_video_active=0;assign remote_video_done=0;assign remote_video_error=0;
 end endgenerate
 reg[$clog2(TIMEOUT+1)-1:0] age,band_age;
 reg[383:0] assembling;reg[5:0] expected;
 reg front,pending;
 wire[5:0] index=rx25[23:18],height=rx25[17:12];
 wire [7:0] reply_magic=TIME_EXT ? (rx25[47:40] | 8'h10) : rx25[47:40];
 wire fresh=rxvalid25 && (reply_magic==8'h7b || (FM_EXT && reply_magic==8'h7c) || (SCAN_EXT && (reply_magic==8'h7d || reply_magic==8'h7e)) || (DSP_EXT && reply_magic==8'h7f) || (BEAT_EXT && (reply_magic==8'h90 || reply_magic==8'h91))) && (!seen || rx25[39:32]!=last_seq);
 wire normal_reply=reply_magic==8'h7b;
 assign beat_packet={rx25[31:0],beat_assembling};
 assign beat_packet_valid=BEAT_EXT && fresh && reply_magic==8'h90 && beat_expected==7 && rx25[52:50]==7 && rx25[55:53]==beat_epoch;

 wire status_completed;wire [1:0] status_kind;wire [127:0] status_packet;
 // The complete assembler word is accepted on the same edge as the
 // old output FFs. Native captures hold it until the next same-kind snapshot.
 generate if(TIME_EXT)begin:native_confirmed_status
  genvar word;
  for(word=0;word<4;word=word+1)begin:capture
   endpoint_status_capture32 #(.NATIVE(0)) fm(clk25,rst,status_completed && status_kind==0,
     status_packet[word*32+:32],fm_status_capture[word*32+:32]);
   endpoint_status_capture32 #(.NATIVE(word!=0 && word!=2)) dsp(clk25,rst,status_completed && status_kind==2,
     status_packet[word*32+:32],dsp_status_capture[word*32+:32]);
  end
 end else begin:legacy_confirmed_status
  assign fm_status_capture=0;assign dsp_status_capture=0;
 end endgenerate
 generate if(TIME_EXT)begin:compact_status
 status_word_assembler receiver(.clk(clk25),.rst(rst),
  .incoming(fresh && (reply_magic==8'h7c || (SCAN_EXT && reply_magic==8'h7e) || (DSP_EXT && reply_magic==8'h7f))),
  .kind(reply_magic==8'h7c ? 2'd0 : reply_magic==8'h7e ? 2'd1 : 2'd2),
  .epoch(rx25[53:52]),.word_index(rx25[51:50]),.data(rx25[31:0]),
  .completed(status_completed),.completed_kind(status_kind),.packet(status_packet));
 end else begin assign status_completed=0;assign status_kind=0;assign status_packet=0;end endgenerate

 wire write_band=fresh && normal_reply && rx25[31] && rx25[30]==mode_mic && !pending &&
                 (index==0 || (index==expected && expected!=0));
 generate if(INDEXED)begin:ram_snapshot
  wire[5:0] q0,q1,unused0,unused1;
  reg read_bank;
  // Write only the back bank. Swap after all 64 bands arrive, and never
  // while the unchanged HDMI renderer is copying a video-frame snapshot.
  spectrum_ram64 bank0(clk25,write_band && front && !rst,index,height,
                      read_addr,q0,6'd0,unused0);
  spectrum_ram64 bank1(clk25,write_band && !front && !rst,index,height,
                      read_addr,q1,6'd0,unused1);
  always @(posedge clk25)read_bank<=front;
  assign read_height=read_bank?q1:q0;
 end else begin:packed_snapshot
  assign read_height=0;
 end endgenerate
 always @(posedge clk25 or posedge rst)begin
  if(rst)begin
   beat_exchange_valid<=0;beat_t1<=0;beat_t2<=0;beat_t3<=0;beat_t4<=0;
   beat_assembling<=0;beat_expected<=0;beat_epoch<=0;clock_expected<=0;clock_epoch<=0;
   capture_event<=0;raw_valid<=0;packet_gap<=0;raw_pcm<=0;dsp_status_storage<=0;dsp_status_valid<=0;dsp_assembling<=0;dsp_expected<=0;dsp_epoch<=0;
   scan_point_valid<=0;scan_generation<=0;scan_index<=0;scan_data<=0;scan_status<=0;scan_status_valid<=0;scan_assembling<=0;scan_expected<=0;scan_epoch<=0;
   fm_status_storage<=0;fm_status_valid<=0;fm_assembling<=0;fm_expected<=0;fm_epoch<=0;
   av_features<=0;av_event<=0;net_mode<=0;net_valid<=0;net_pcm<=0;seq<=0;last_seq<=0;seen<=0;age<=TIMEOUT;band_age<=TIMEOUT;online<=0;
   remote_mic<=1;present<=1;light_ready<=0;light_fault<=1;light_level<=1;light_adc<=0;
   heights<=0;heights_valid<=0;spectrum_ready<=0;assembling<=0;expected<=0;front<=0;pending<=0;
  end else begin
   beat_exchange_valid<=0;capture_event<=0;raw_valid<=0;packet_gap<=0;heights_valid<=0;fm_status_valid<=0;scan_point_valid<=0;scan_status_valid<=0;dsp_status_valid<=0;
   if(sample_tick)begin seq<=seq+1'b1;net_valid<=0;end
   if(age<TIMEOUT)age<=age+1'b1;
   else begin net_mode<=0;net_valid<=0;online<=0;seen<=0;present<=1;light_ready<=0;light_fault<=1;spectrum_ready<=0;expected<=0;pending<=0;beat_expected<=0;clock_expected<=0;end
   if(band_age<TIMEOUT)band_age<=band_age+1'b1;else spectrum_ready<=0;
   if(INDEXED && pending && !freeze && age<TIMEOUT)begin
    front<=~front;pending<=0;heights_valid<=1;spectrum_ready<=1;band_age<=0;
   end
   if(fresh)begin
    capture_event<=TIME_EXT;raw_valid<=TIME_EXT && !rx25[44];raw_pcm<=rx25[87:72];
    packet_gap<=TIME_EXT && seen && rx25[39:32]!=(last_seq+8'd1);
    online<=1;seen<=1;last_seq<=rx25[39:32];age<=0;
    net_mode<=rx25[48];net_valid<=rx25[49];net_pcm<=rx25[87:56];
    if(BEAT_EXT && reply_magic==8'h90)begin
     if(rx25[52:50]==0)begin beat_assembling<={rx25[31:0],beat_assembling[223:32]};beat_expected<=1;beat_epoch<=rx25[55:53];end
     else if(beat_expected!=0 && rx25[52:50]==beat_expected && rx25[55:53]==beat_epoch)begin
      if(beat_expected==7)begin beat_expected<=0;end
      else begin beat_assembling<={rx25[31:0],beat_assembling[223:32]};beat_expected<=beat_expected+1'b1;end
     end else beat_expected<=0;
    end else if(BEAT_EXT && reply_magic==8'h91)begin
     case(rx25[51:50])
      0:begin beat_t3<=rx25[31:0];beat_t4<=beat_now_us;clock_expected<=1;clock_epoch<=rx25[53:52];end
      1:if(clock_expected==1 && clock_epoch==rx25[53:52])begin beat_t1<=rx25[31:0];clock_expected<=2;end else clock_expected<=0;
      2:if(clock_expected==2 && clock_epoch==rx25[53:52])begin beat_t2<=rx25[31:0];beat_exchange_valid<=1;clock_expected<=0;end else clock_expected<=0;
      default:clock_expected<=0;
     endcase
    end else if(SCAN_EXT && reply_magic==8'h7d)begin
     scan_point_valid<=1;scan_generation<=rx25[31:24];scan_index<=rx25[23:16];scan_data<=rx25[15:0];
    end else if(SCAN_EXT && reply_magic==8'h7e)begin if(!TIME_EXT)begin
     if(rx25[51:50]==0)begin scan_assembling[31:0]<=rx25[31:0];scan_epoch<=rx25[53:52];scan_expected<=1;end
     else if(rx25[51:50]==scan_expected && rx25[53:52]==scan_epoch && scan_expected!=0)begin
      scan_assembling[rx25[51:50]*32+:32]<=rx25[31:0];
      if(scan_expected==3)begin scan_status<={rx25[31:0],scan_assembling[95:0]};scan_status_valid<=1;scan_expected<=0;end
      else scan_expected<=scan_expected+1'b1;
     end else scan_expected<=0;
    end end else if(DSP_EXT && reply_magic==8'h7f)begin if(!TIME_EXT)begin
     if(rx25[51:50]==0)begin dsp_assembling[31:0]<=rx25[31:0];dsp_epoch<=rx25[53:52];dsp_expected<=1;end
     else if(rx25[51:50]==dsp_expected && rx25[53:52]==dsp_epoch && dsp_expected!=0)begin
      dsp_assembling[rx25[51:50]*32+:32]<=rx25[31:0];
      if(dsp_expected==3)begin dsp_status_storage<={rx25[31:0],dsp_assembling[95:0]};dsp_status_valid<=1;dsp_expected<=0;end
      else dsp_expected<=dsp_expected+1'b1;
     end else dsp_expected<=0;
    end end else if(!normal_reply)begin if(!TIME_EXT)begin
     if(rx25[51:50]==0)begin fm_assembling[31:0]<=rx25[31:0];fm_epoch<=rx25[53:52];fm_expected<=1;end
     else if(rx25[51:50]==fm_expected && rx25[53:52]==fm_epoch && fm_expected!=0)begin
      fm_assembling[rx25[51:50]*32+:32]<=rx25[31:0];
      if(fm_expected==3)begin fm_status_storage<={rx25[31:0],fm_assembling[95:0]};fm_status_valid<=1;fm_expected<=0;
       if(rx25[8:7]!=fm_status[104:103])begin spectrum_ready<=0;expected<=0;pending<=0;end
      end
      else fm_expected<=fm_expected+1'b1;
     end else fm_expected<=0;
    end end else begin
    av_event<=rx25[26];
    case(rx25[33:32])
     0:av_features[23:18]<=rx25[55:50];1:av_features[17:12]<=rx25[55:50];
     2:av_features[11:6]<=rx25[55:50];3:av_features[5:0]<=rx25[55:50];
    endcase
    remote_mic<=rx25[30];present<=rx25[29];light_ready<=rx25[28];light_fault<=rx25[27];
    light_level<=rx25[25:24];light_adc<=rx25[11:0];
    if(rx25[31] && rx25[30]==mode_mic && !pending)begin
     if(index==0)begin if(!INDEXED)assembling[5:0]<=height;expected<=1;end
     else if(index==expected && expected!=0)begin
      if(!INDEXED)assembling[index*6+:6]<=height;
      if(index==63)begin
       if(INDEXED)pending<=1;
       else begin heights<={height,assembling[377:0]};heights_valid<=1;spectrum_ready<=1;band_age<=0;end
       expected<=0;
      end else expected<=expected+1'b1;
     end else expected<=0;
    end else if(!pending)begin expected<=0;spectrum_ready<=0;end
    end
   end
   if(status_completed)case(status_kind)
    0:begin fm_status_storage<=status_packet;fm_status_valid<=1;
     if(status_packet[104:103]!=fm_status[104:103])begin spectrum_ready<=0;expected<=0;pending<=0;end
    end
    1:begin scan_status<=status_packet;scan_status_valid<=1;end
    2:begin dsp_status_storage<=status_packet;dsp_status_valid<=1;end
   endcase
   if(remote_mic!=mode_mic)begin spectrum_ready<=0;expected<=0;pending<=0;end
  end
 end
endmodule

module board_link_b #(parameter BUFFER_BEAT=0,parameter SERIAL_BEAT=0,parameter BEAT_EXT=0,parameter TIMEOUT=2500000,parameter FM_EXT=0,parameter SCAN_EXT=0,parameter DSP_EXT=0,parameter TIME_EXT=0)(input clk50,clk25,rst,cs_n,sck,mosi,
 output miso,input[383:0] heights,input heights_valid,
 input present,light_ready,light_fault,input[1:0] light_level,input[11:0] light_adc,
 output reg online,output reg mode_mic,output reg source_change,
 output reg[23:0] pcm,output reg pcm_valid,output reg boot_done,
 input net_mode,input net_valid,input[31:0] net_pcm,output wire net_take,
 input[23:0] av_features,input av_event,
 input [127:0] fm_status,input analysis_mic,analysis_change,
 output reg requested_mode_mic,
 output reg fm_cmd_valid,input fm_cmd_ready,output reg [7:0] fm_cmd_seq,
 output reg [2:0] fm_cmd_op,output reg [15:0] fm_cmd_value,
 input [127:0] scan_status,input [15:0] scan_read_data,output reg [7:0] scan_read_index,
 input [127:0] dsp_status,output reg [31:0] stereo_pcm,output reg dsp_capable,dsp_pcm_valid,
 input raw_capture_valid,output reg time_capable,history_mode,output reg [1:0] play_epoch,
 input [31:0] beat_now_us,input [255:0] beat_packet,input beat_packet_valid,
 output wire beat_packet_advance,beat_packet_ready,output reg beat_enable,output reg [1:0] beat_sensitivity);
 wire v50,v25;wire[87:0] rx50,rx25;
 reg tx_valid;reg[87:0] tx25;wire tv50;wire[87:0] tp50;
 reg[87:0] reply;
 wire crc_error;
 board_link_slave #(.W(88)) slave(clk50,rst,cs_n,sck,mosi,reply,miso,v50,rx50,crc_error);
 board_link_mailbox #(.W(88)) mrx(clk50,clk25,rst,v50,rx50,v25,rx25);
 board_link_mailbox #(.W(88)) mtx(clk25,clk50,rst,tx_valid,tx25,tv50,tp50);
 always @(posedge clk50 or posedge rst)
  // A must not declare B online before B has received its first valid request.
  if(rst)reply<=88'd0;else if(tv50)reply<=tp50;
 reg[$clog2(TIMEOUT+1)-1:0] age;
 reg[7:0] last_seq;reg seen,fft_ready;
 reg[5:0] index;reg[383:0] snapshot;
 wire[5:0] av_value=rx25[33:32]==0 ? av_features[23:18] :
                           rx25[33:32]==1 ? av_features[17:12] :
                           rx25[33:32]==2 ? av_features[11:6] : av_features[5:0];
 wire stereo_frame=DSP_EXT && rx25[47:40]==8'haa;
 wire fresh=v25 && (rx25[47:40]==(FM_EXT ? 8'ha9 : 8'ha8) || stereo_frame) && (!seen || rx25[39:32]!=last_seq);
 reg [127:0] fm_snapshot,scan_snapshot,dsp_snapshot;
 reg [2:0] beat_word,beat_epoch;
 reg [31:0] clock_t1,clock_t2;
 reg beat_peer;
 wire beat_slot=BEAT_EXT && beat_peer && rx25[35:32]==5;
 wire clock_slot=BEAT_EXT && rx25[35:32]==4;
 reg [255:0] beat_shift;reg buffer_valid;
 wire beat_sending=BUFFER_BEAT ? buffer_valid:beat_packet_valid;
 assign beat_packet_advance=!BUFFER_BEAT && BEAT_EXT && fresh && beat_slot && beat_sending;
 assign beat_packet_ready=BEAT_EXT && (BUFFER_BEAT ? !buffer_valid:(fresh && beat_slot && beat_sending && beat_word==7));
 generate if(BUFFER_BEAT)begin: buffered_beat
 always @(posedge clk25 or posedge rst)begin
  if(rst)begin beat_shift<=0;buffer_valid<=0;beat_word<=0;beat_epoch<=0;end
  else begin
   if(beat_packet_valid && beat_packet_ready)begin
    beat_shift<=beat_packet;buffer_valid<=1;beat_word<=0;beat_epoch<=beat_epoch+1'b1;
   end
   if(fresh && beat_slot && buffer_valid)begin
    beat_shift<={32'd0,beat_shift[255:32]};beat_word<=beat_word+1'b1;
    if(beat_word==7)buffer_valid<=0;
   end
  end
 end
 end else begin: serial_beat
 always @(posedge clk25 or posedge rst)begin
  if(rst)begin beat_word<=0;beat_epoch<=0;end
  else if(!beat_packet_valid)beat_word<=0;
  else if(fresh && beat_slot)begin
   beat_word<=beat_word+1'b1;
   if(beat_word==7)beat_epoch<=beat_epoch+1'b1;
  end
 end
 end endgenerate

 reg command_seen;reg [7:0] command_last;
 wire requested_mic=FM_EXT ? analysis_mic : rx25[24];
 assign net_take=fresh && net_mode && net_valid;
 // Shift only on normal spectrum replies; metadata never advances a band.
 // Preserve the exact index-0 snapshot while removing a 64-to-1 pixel mux.
 wire[5:0] height=index==0?heights[5:0]:snapshot[5:0];
 always @(posedge clk25 or posedge rst)begin
  if(rst)begin
   beat_peer<=0;beat_enable<=1;beat_sensitivity<=1;clock_t1<=0;clock_t2<=0;
   time_capable<=0;history_mode<=0;play_epoch<=0;dsp_snapshot<=0;stereo_pcm<=0;dsp_capable<=0;dsp_pcm_valid<=0;scan_snapshot<=0;scan_read_index<=0;requested_mode_mic<=1;fm_cmd_valid<=0;fm_cmd_seq<=0;fm_cmd_op<=0;fm_cmd_value<=0;command_seen<=0;command_last<=0;fm_snapshot<=0;
   age<=TIMEOUT;last_seq<=0;seen<=0;online<=0;mode_mic<=1;source_change<=0;pcm<=0;pcm_valid<=0;
   tx_valid<=0;tx25<=0;index<=0;snapshot<=0;fft_ready<=0;boot_done<=0;
  end else begin
   pcm_valid<=0;source_change<=0;tx_valid<=0;
   if(fm_cmd_valid && fm_cmd_ready)begin fm_cmd_valid<=0;command_seen<=1;command_last<=fm_cmd_seq;end
   if(heights_valid)fft_ready<=1;
   if(age<TIMEOUT)age<=age+1'b1;
   else begin
    online<=0;seen<=0;time_capable<=0;history_mode<=0;requested_mode_mic<=1;dsp_capable<=0;dsp_pcm_valid<=0;
    if(!mode_mic)begin mode_mic<=1;source_change<=1;fft_ready<=0;index<=0;end
   end
   if(fresh)begin
    time_capable<=TIME_EXT && stereo_frame && rx25[59];
    history_mode<=TIME_EXT && stereo_frame && rx25[59] && rx25[58];play_epoch<=rx25[57:56];
    age<=0;online<=1;seen<=1;last_seq<=rx25[39:32];pcm<=stereo_frame ? {rx25[31:16],8'd0} : rx25[23:0];pcm_valid<=1;
    stereo_pcm<=rx25[31:0];dsp_capable<=stereo_frame && rx25[51];dsp_pcm_valid<=stereo_frame && rx25[48];
    boot_done<=stereo_frame ? rx25[50] : rx25[25];requested_mode_mic<=stereo_frame ? rx25[49] : rx25[24];
    // Reply carries previous established source; A hides mismatched mode until new FFT data.
    tx25<={net_pcm,av_value,(net_mode && net_valid),net_mode,8'h7b,rx25[39:32],fft_ready,mode_mic,present,light_ready,light_fault,av_event,light_level,index,height,light_adc};
    if(FM_EXT && rx25[35:32]==0)begin
     // One 32-bit status word each 16 frames. PCM still travels every frame.
     if(rx25[37:36]==0)fm_snapshot<=fm_status;
     tx25[47:40]<=8'h7c;
     tx25[55:50]<={2'd0,rx25[39:38],rx25[37:36]};
     tx25[31:0]<=rx25[37:36]==0 ? fm_status[31:0] : fm_snapshot[rx25[37:36]*32+:32];
    end
    if(SCAN_EXT && rx25[35:32]==1)begin
     tx25[47:40]<=8'h7d;tx25[31:0]<={scan_status[127:120],scan_read_index,scan_read_data};
     scan_read_index<=scan_read_index+1'b1;
    end
    if(SCAN_EXT && rx25[35:32]==2)begin
     if(rx25[37:36]==0)scan_snapshot<=scan_status;
     tx25[47:40]<=8'h7e;tx25[55:50]<={2'd0,rx25[39:38],rx25[37:36]};
     tx25[31:0]<=rx25[37:36]==0?scan_status[31:0]:scan_snapshot[rx25[37:36]*32+:32];
    end
    if(DSP_EXT && stereo_frame && rx25[51] && rx25[35:32]==3)begin
     if(rx25[37:36]==0)dsp_snapshot<=dsp_status;
     tx25[47:40]<=8'h7f;tx25[55:50]<={2'd0,rx25[39:38],rx25[37:36]};
     tx25[31:0]<=rx25[37:36]==0 ? dsp_status[31:0] : dsp_snapshot[rx25[37:36]*32+:32];
    end
    if(BEAT_EXT && !clock_slot)begin beat_peer<=rx25[55];beat_enable<=rx25[54];beat_sensitivity<=rx25[53:52];end
    if(beat_slot && beat_sending)begin
     tx25[47:40]<=8'h90;tx25[55:50]<={beat_epoch,beat_word};
     tx25[31:0]<=BUFFER_BEAT ? beat_shift[31:0] : SERIAL_BEAT ? beat_packet[31:0] : beat_packet[beat_word*32+:32];
    end
    if(clock_slot && beat_peer)begin
     tx25[47:40]<=8'h91;tx25[55:50]<={2'd0,rx25[39:38],rx25[37:36]};
     case(rx25[37:36])
      0:begin clock_t1<={rx25[87:60],rx25[55:52]};clock_t2<=beat_now_us;tx25[31:0]<=beat_now_us;end
      1:tx25[31:0]<=clock_t1;
      2:tx25[31:0]<=clock_t2;
      default:tx25[31:0]<=32'h42454101;
     endcase
    end
    if(FM_EXT && !clock_slot && rx25[60] && !fm_cmd_valid && (!command_seen || rx25[87:80]!=command_last))begin
     fm_cmd_valid<=1;fm_cmd_seq<=rx25[87:80];fm_cmd_op<=rx25[79:77];fm_cmd_value<=rx25[76:61];
    end
    if(TIME_EXT && stereo_frame && rx25[59] && raw_capture_valid)tx25[44]<=0;
    tx_valid<=1;
    if(!((beat_slot && beat_sending) || (clock_slot && beat_peer) || (FM_EXT && rx25[35:32]==0) || (SCAN_EXT && (rx25[35:32]==1 || rx25[35:32]==2)) || (DSP_EXT && stereo_frame && rx25[51] && rx25[35:32]==3)))begin
     if(index==0)snapshot<={6'd0,heights[383:6]};else snapshot<={6'd0,snapshot[383:6]};
     index<=index+1'b1;
    end
    if(mode_mic!=requested_mic)begin
     mode_mic<=requested_mic;source_change<=1;fft_ready<=0;index<=0;
    end
   end
   if(FM_EXT && analysis_change)begin fft_ready<=0;index<=0;source_change<=1;end
  end
 end
endmodule

module endpoint_status_capture32 #(parameter NATIVE=1)(input clk,rst,load,input [31:0] data,output [31:0] q);
 generate if(NATIVE)begin:native_storage
 wire [31:0] write_data=rst?32'd0:data;
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(1),.ADDR_WIDTH_B(1),
  .DATA_DEPTH_A(2),.DATA_DEPTH_B(2),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("WRITETHROUGH"),.WRITEMODE_B("WRITETHROUGH"),.RESETMODE("SYNC"),
  .IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
  .dia(write_data[15:0]),.addra(1'b0),.cea(load || rst),.ocea(1'b0),.clka(clk),.wea(load || rst),.rsta(1'b0),.bea(1'b0),.doa(q[15:0]),
  .dib(write_data[31:16]),.addrb(1'b1),.ceb(load || rst),.oceb(1'b0),.clkb(clk),.web(load || rst),.rstb(1'b0),.beb(1'b0),.dob(q[31:16]));
 end else begin:register_storage
  reg [31:0] held;
  always @(posedge clk)if(rst || load)held<=rst ? 32'd0:data;
  assign q=held;
 end endgenerate
endmodule
