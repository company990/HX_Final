// Shared datagram header/CRC and ACK serializer, with
// independent WAV (one 1024-byte slot) and VID (eight 1024-byte slots) state.
// No PHY/MAC changes and no loss of either original buffering capacity.
module udp_av_receiver(
 input wire clk,rst,input wire wav_enable,video_enable,
 input wire [7:0] rx_data,input wire rx_valid,input wire [15:0] rx_length,
 input wire tx_ready,tx_ack,output wire tx_request,tx_valid,
 output wire [7:0] tx_data,output wire [15:0] tx_length,
 output wire [7:0] file_byte,output wire file_valid,
 input wire file_ready,file_error,output reg [31:0] file_size,
 output reg file_end,output reg [31:0] next_offset,
 output reg debug_rx_seen,debug_wack_sent,
 output wire [3:0] wav_debug_state,output reg [3:0] wav_debug_checks,
 output wire [15:0] wav_debug_length,wav_debug_count,
 output wire [7:0] video_byte,output wire video_valid,
 input wire video_ready,video_error,output reg [31:0] video_size,
 output wire video_end,output reg video_active,output wire [3:0] video_slots,
 output wire video_obs_payload_byte,video_obs_commit_byte,
 output wire video_obs_drop_pkt,video_obs_commit_pkt
);
 reg [7:0] wram[0:1023] /* fehdl force_ram=1, ram_style="bram" */;
 reg [7:0] vram[0:8191] /* fehdl force_ram=1, ram_style="bram" */;
 reg [10:0] lengths[0:7];reg [2:0] wp,rp;reg [3:0] count;
 reg receiving,check,capture,wcap,vcap;reg [15:0] at_reg,size_reg,gap;
 // All four fields are consumed only after byte fifteen. One shift
 // enable avoids four independent byte-placement and clock-enable decoders.
 reg [127:0] header;
 wire [31:0] magic=header[127:96],p_session=header[95:64];
 wire [31:0] p_offset=header[63:32],p_size=header[31:0];
 reg [31:0] rx_crc,trailer;
 reg wactive;reg [31:0] wsession,vsession,vnext,vdrained;
 reg [3:0] wstate;reg [10:0] wlength;reg [9:0] wrd,vrd;
 reg [7:0] wq,vq;reg vq_valid;
 reg [31:0] wav_reply_session;reg [2:0] wav_status;
 reg vack_pending;reg [31:0] video_reply_session;reg [2:0] video_status;
 reg [2:0] ts;reg video_owner,prefer_video;reg [4:0] ai;
 reg [159:0] shift;reg [31:0] ack_crc;
 wire owned=ts!=0;
 wire wreq=wstate==7,vreq=vack_pending;
 wire wrst=rst || (!wav_enable && !(owned && !video_owner));
 wire vrst=rst || (!video_enable && !(owned && video_owner));
 wire [15:0] at=receiving?at_reg:16'd0;
 wire [15:0] length=receiving?size_reg:rx_length;
 wire eligible=!check && rx_length>=21 && rx_length<=1044;
 wire cap=receiving?capture:eligible;
 wire this_wcap=receiving?wcap:wav_enable && wstate==0;
 wire this_vcap=receiving?vcap:video_enable && count<8;
 wire payload_byte=rx_valid && cap && at>=16 && at<length-4;
 wire wwrite=payload_byte && wav_enable && this_wcap && magic==32'h57415631;
 wire vwrite=payload_byte && video_enable && this_vcap && magic==32'h56494431;
 wire [10:0] payload_length=size_reg-16'd20;
 // A 33-bit end offset simultaneously proves range and rejects unsigned
 // wraparound. Share this addition for commit, duplicate and debug checks.
 wire [32:0] payload_end={1'b0,p_offset}+payload_length;
 wire crc_ok=trailer==~rx_crc;
 wire payload_in_range=payload_end<={1'b0,p_size};
 wire common_good=check && crc_ok && payload_in_range;
 wire video_good=common_good && magic==32'h56494431 && vcap && p_size>=32 && !vrst;
 wire video_new=!video_active && p_offset==0;
 wire video_matched=video_active && p_session==vsession && p_size==video_size;
 wire vpush=video_good && !video_error && (video_new || (video_matched && p_offset==vnext));
 wire duplicate=video_matched && payload_end<={1'b0,vnext};
 wire vpop=video_enable && vq_valid && video_ready && ({1'b0,vrd}+11'd1==lengths[rp]);
 wire fetch=video_enable && (!vq_valid || video_ready) && count!=0 && (!vpop || count>1);
 wire [12:0] vaddr=vq_valid?(vpop?{rp+3'd1,10'd0}:{rp,vrd+10'd1}):{rp,vrd};
 wire reserve_video=receiving && vcap && capture;
 wire [7:0] credits=(count+reserve_video)>=8?0:8'd8-count-reserve_video;
 assign video_slots=count;
 assign file_byte=wq;assign file_valid=wstate==3 && wav_enable;
 assign video_byte=vq;assign video_valid=vq_valid && video_enable;
 assign video_end=video_active && vdrained==video_size;
 // Keep the standalone WAV receiver's diagnostic state numbering.
 assign wav_debug_state=(check && wcap && magic==32'h57415631)?4'd1:
   wstate==7?4'd6:wstate!=8?wstate:ts==1?4'd5:ts==2?4'd6:ts==3?4'd7:4'd8;
 assign wav_debug_length=size_reg;assign wav_debug_count=at_reg;
 assign video_obs_payload_byte=rx_valid && at>=16 && at<length-4;
 assign video_obs_commit_byte=vwrite;
 assign video_obs_drop_pkt=rx_valid && at==0 && !this_vcap;
 assign video_obs_commit_pkt=vpush;
 assign tx_request=ts==3;assign tx_valid=ts==4;
 assign tx_data=shift[159:152];assign tx_length=20;
 function [31:0] cb;input [31:0] c;input [7:0] b;integer i;reg [31:0] t;
 begin t=c^{24'd0,b};for(i=0;i<8;i=i+1)t=t[0]?(t>>1)^32'hedb88320:t>>1;cb=t;end endfunction
 wire rx_crc_cycle=rx_valid && cap && at<length-4;
 // Receive bytes have priority; ACK CRC uses idle cycles. Header creation
 // and transmission stay locked to one owner through all twenty bytes.
 wire [31:0] crc_next=cb(rx_crc_cycle?(at==0?32'hffffffff:rx_crc):ack_crc,
                             rx_crc_cycle?rx_data:shift[159:152]);
 wire choose_video=vreq && video_enable && (prefer_video || !wreq || !wav_enable);
 wire choose_wav=wreq && wav_enable && !choose_video;
 always @(posedge clk)begin
  if(wwrite)wram[at[9:0]-10'd16]<=rx_data;
  if(vwrite)vram[{wp,at[9:0]-10'd16}]<=rx_data;
  if(wstate==2)wq<=wram[wrd];
  if(fetch)vq<=vram[vaddr];
  if(vpush)lengths[wp]<=payload_length;
  if(ts==0 && tx_ready && (choose_video || choose_wav))begin
   if(choose_video)shift<={32'h5641434b,video_reply_session,vnext,16'd0,credits,5'd0,video_status,32'd0};
   else shift<={32'h5741434b,wav_reply_session,next_offset,29'd0,wav_status,32'd0};
  end else if(ts==1 && !rx_crc_cycle)begin
   shift<={shift[151:32],shift[159:152],ai==15?~crc_next:32'd0};
  end else if(ts==4)shift<={shift[151:0],8'd0};
 end
 // One shared packet boundary/header engine. A partial datagram times out
 // after 65536 idle cycles and never commits either stream.
 always @(posedge clk or posedge rst)begin
  if(rst)begin receiving<=0;check<=0;capture<=0;wcap<=0;vcap<=0;
   at_reg<=0;size_reg<=0;gap<=0;header<=0;rx_crc<=0;trailer<=0;end
  else begin
   check<=0;
   // Disable cancels capture for the remainder of this datagram, including
   // a disable/re-enable pulse before its last byte. The shared header engine
   // continues to track boundaries for the other independently enabled source.
   if(!wav_enable)wcap<=0;
   if(!video_enable)vcap<=0;
   if(rx_valid)begin
    gap<=0;
    if(!receiving)begin size_reg<=rx_length;capture<=cap;wcap<=this_wcap;vcap<=this_vcap;end
    if(length==0 || at==length-1)begin receiving<=0;at_reg<=0;check<=cap;end
    else begin receiving<=1;at_reg<=at+1'b1;end
    if(cap)begin
     if(at<16)header<={header[119:0],rx_data};
     if(rx_crc_cycle)rx_crc<=crc_next;
     if(at>=length-4)trailer<={trailer[23:0],rx_data};
    end
   end else if(receiving)begin
    gap<=gap+1'b1;if(&gap)begin receiving<=0;capture<=0;wcap<=0;vcap<=0;at_reg<=0;end
   end
  end
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin wstate<=0;wactive<=0;wsession<=0;file_size<=0;file_end<=0;next_offset<=0;
   wrd<=0;wlength<=0;wav_reply_session<=0;wav_status<=0;debug_rx_seen<=0;debug_wack_sent<=0;wav_debug_checks<=0;end
  else if(wrst)begin wstate<=0;wactive<=0;wsession<=0;file_size<=0;file_end<=0;next_offset<=0;
   wrd<=0;wlength<=0;wav_reply_session<=0;wav_status<=0;debug_rx_seen<=0;debug_wack_sent<=0;wav_debug_checks<=0;end
  else begin
   if(rx_valid && wav_enable)debug_rx_seen<=1;
   if(check && wcap && magic==32'h57415631)begin
    wav_debug_checks<={1'b1,crc_ok,p_size>=44 && payload_in_range,1'b1};
    if(common_good && p_size>=44 && wstate==0)begin
     wav_reply_session<=p_session;
     if(!wactive && p_offset==0)begin
      wactive<=1;wsession<=p_session;file_size<=p_size;next_offset<=0;file_end<=0;
      wrd<=0;wlength<=payload_length;wstate<=2;
     end else if(!wactive)begin wav_status<=2;wstate<=7;end
     else if(p_session!=wsession || p_size!=file_size)begin wav_status<=3;wstate<=7;end
     else if(p_offset==next_offset && !file_end)begin wrd<=0;wlength<=payload_length;wstate<=2;end
     else if(payload_end<={1'b0,next_offset})begin wav_status<=file_error?4:1;wstate<=7;end
     else begin wav_status<=2;wstate<=7;end
    end
   end
   if(wstate==2)wstate<=3;
   if(wstate==3 && file_ready && wav_enable)begin
    if({1'b0,wrd}+11'd1==wlength)begin
     next_offset<=next_offset+wlength;file_end<=next_offset+wlength==file_size;
     wav_status<=file_error?4:0;wstate<=7;
    end else begin wrd<=wrd+1'b1;wstate<=2;end
   end
   if(ts==0 && tx_ready && choose_wav)wstate<=8;
   if(ts==4 && ai==19 && !video_owner)begin wstate<=0;debug_wack_sent<=1;end
  end
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin wp<=0;rp<=0;count<=0;video_active<=0;video_size<=0;vsession<=0;vnext<=0;
   vdrained<=0;vrd<=0;vq_valid<=0;vack_pending<=0;video_reply_session<=0;video_status<=0;end
  else if(vrst)begin wp<=0;rp<=0;count<=0;video_active<=0;video_size<=0;vsession<=0;vnext<=0;
   vdrained<=0;vrd<=0;vq_valid<=0;vack_pending<=0;video_reply_session<=0;video_status<=0;end
  else begin
   case({vpush,vpop})2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:;endcase
   if(vpush)begin wp<=wp+1'b1;video_active<=1;vsession<=p_session;video_size<=p_size;vnext<=payload_end[31:0];end
   if(video_enable && (!vq_valid || video_ready))vq_valid<=fetch;
   if(vq_valid && video_ready && video_enable)begin
    vdrained<=vdrained+1'b1;
    if(vpop)begin rp<=rp+1'b1;vrd<=0;end else vrd<=vrd+1'b1;
   end
   if(ts==0 && tx_ready && choose_video)vack_pending<=0;
   if(vpop && video_active)begin vack_pending<=1;video_reply_session<=vsession;video_status<=video_error?4:0;end
   if(video_good)begin
    vack_pending<=1;video_reply_session<=p_session;
    video_status<=video_error?4:vpush?0:duplicate?1:(!video_active || video_matched)?2:3;
   end
  end
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin ts<=0;video_owner<=0;prefer_video<=0;ai<=0;ack_crc<=0;end
  else case(ts)
   0:if(tx_ready && (choose_video || choose_wav))begin
    video_owner<=choose_video;ts<=1;ai<=0;ack_crc<=32'hffffffff;
   end
   1:if(!rx_crc_cycle)begin ack_crc<=crc_next;if(ai==15)ts<=2;else ai<=ai+1'b1;end
   2:if(tx_ready)ts<=3;
   3:if(tx_ack)begin ts<=4;ai<=0;end
   4:if(ai==19)begin ts<=0;prefer_video<=!video_owner;end else ai<=ai+1'b1;
   default:ts<=0;
  endcase
 end
endmodule
