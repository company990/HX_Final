// Eight packet slots decouple UDP reception from the paced AV consumer.
// VID1/VACK use BE integers and reflected IEEE CRC32. Data is committed only
// after the complete datagram CRC passes. Credits include a reserved RX slot.
module udp_video_receiver(
 input wire clk,rst,input wire[7:0] rx_data,input wire rx_valid,
 input wire[15:0] rx_length,input wire tx_ready,tx_ack,
 output wire tx_request,tx_valid,output wire[7:0] tx_data,
 output wire[15:0] tx_length,output wire[7:0] data,
 output wire valid,input wire ready,input wire error,
 output reg active,output reg[31:0] file_size,output wire done,
 output wire[3:0] slot_count,
 output wire obs_payload_byte,obs_commit_byte,obs_drop_pkt,obs_commit_pkt);
 reg[7:0] ram[0:8191] /* fehdl force_ram=1, ram_style="bram" */;
 reg[10:0] length[0:7];
 reg[2:0] wp,rp;reg[3:0] count;
 reg receiving,capture,check;
 reg[15:0] index,total,gap;
 reg[31:0] magic,p_session,p_offset,p_size,crc,trailer;
 reg[31:0] session,next_offset,drained;
 reg[9:0] rd_index;reg[7:0] rd_data;reg rd_valid;
 wire[15:0] at=receiving?index:16'd0;
 wire[15:0] size=receiving?total:rx_length;
 wire cap=receiving?capture:(!check && count<8 && rx_length>=21 && rx_length<=1044);
 wire wr=rx_valid && cap && at>=16 && at<size-4;
 wire pop=rd_valid && ready && ({1'b0,rd_index}+1'b1==length[rp]);
 wire good=check && magic==32'h56494431 && trailer==~crc && p_size>=32 &&
           p_offset<=p_size && total-20<=p_size-p_offset;
 wire new_session=!active && p_offset==0;
 wire matched=active && p_session==session && p_size==file_size;
 wire push=good && !error && (new_session || (matched && p_offset==next_offset));
 wire duplicate=matched && p_offset<next_offset && total-20<=next_offset-p_offset;
 // Elastic synchronous RAM output: replace an accepted byte in the same cycle.
  // A slot is released only when its last byte is accepted, never on prefetch.
  wire fetch=(!rd_valid || ready) && count!=0 && (!pop || count>1);
  wire[12:0] fetch_addr=rd_valid ?
      (pop ? {rp+3'd1,10'd0} : {rp,rd_index+10'd1}) : {rp,rd_index};
 function[31:0] cb;input[31:0] c;input[7:0] b;reg[31:0] v;integer i;
 begin v=c^{24'd0,b};for(i=0;i<8;i=i+1)v=v[0]?(v>>1)^32'hedb88320:v>>1;cb=v;end endfunction
 always @(posedge clk)begin
  if(wr)ram[{wp,at[9:0]-10'd16}]<=rx_data;
  if(fetch)rd_data<=ram[fetch_addr];
  if(push)length[wp]<=total-20;
 end
 assign data=rd_data;assign valid=rd_valid;
 assign done=active && drained==file_size;
 assign slot_count=count;   // observation only
 assign obs_payload_byte = rx_valid && (at>=16) && (at<size-4);
 assign obs_commit_byte  = wr;
 assign obs_drop_pkt     = rx_valid && (at==0) && !cap;
 assign obs_commit_pkt   = push;
 reg ack_pending;reg[31:0] reply_session;reg[7:0] status;
 reg[2:0] ts;reg[4:0] ai;reg[159:0] shift;
 reg[31:0] ack_crc;
 wire[7:0] credit=(count+(receiving && capture))>=8?8'd0:8'd8-count-(receiving && capture);
 assign tx_request=ts==3;assign tx_valid=ts==4;
 assign tx_data=shift[159:152];assign tx_length=20;
 wire[7:0] crc_data=shift[159:152];
 // Reuse the 160-bit response buffer for header CRC rotation and transmit.
 // The sixteen-byte rotation restores the original header before sending.
 // No reset on packet storage: ts gates every observation until initialized.
 always @(posedge clk)begin
  if(ts==0 && ack_pending)shift<={32'h5641434b,reply_session,next_offset,16'd0,credit,status,32'd0};
  else if(ts==1)begin
   if(ai==15)shift<={shift[151:32],shift[159:152],~cb(ack_crc,crc_data)};
   else shift<={shift[151:32],shift[159:152],32'd0};
  end else if(ts==4)shift<={shift[151:0],8'd0};
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   wp<=0;rp<=0;count<=0;receiving<=0;capture<=0;check<=0;index<=0;total<=0;gap<=0;
   magic<=0;p_session<=0;p_offset<=0;p_size<=0;crc<=0;trailer<=0;
   active<=0;file_size<=0;session<=0;next_offset<=0;drained<=0;rd_index<=0;rd_valid<=0;
   ack_pending<=0;reply_session<=0;status<=0;ts<=0;ai<=0;ack_crc<=0;
  end else begin
   check<=0;
   if(rx_valid)begin
    gap<=0;
    if(!receiving)begin total<=rx_length;capture<=cap;end
    if(size==0 || at==size-1)begin receiving<=0;capture<=0;index<=0;check<=cap;end
    else begin receiving<=1;index<=at+1'b1;end
    if(cap)begin
     if(at==0)begin magic<={24'd0,rx_data};crc<=cb(32'hffffffff,rx_data);end
     else begin if(at<4)magic<={magic[23:0],rx_data};if(at<size-4)crc<=cb(crc,rx_data);end
     if(at>=4 && at<8)p_session<={p_session[23:0],rx_data};
     if(at>=8 && at<12)p_offset<={p_offset[23:0],rx_data};
     if(at>=12 && at<16)p_size<={p_size[23:0],rx_data};
     if(at>=size-4)trailer<={trailer[23:0],rx_data};
    end
   end else if(receiving)begin gap<=gap+1'b1;if(&gap)begin receiving<=0;capture<=0;index<=0;end end
   case({push,pop})2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:;endcase
   if(push)begin
    wp<=wp+1'b1;active<=1;session<=p_session;file_size<=p_size;next_offset<=p_offset+total-20;
   end
   if(!rd_valid || ready)rd_valid<=fetch;
   if(rd_valid && ready)begin
    drained<=drained+1'b1;
    if(pop)begin rp<=rp+1'b1;rd_index<=0;end else rd_index<=rd_index+1'b1;
   end
   // Snapshot an ACK independently of the RX and draining engines.
   case(ts)
    0:if(ack_pending)begin
      ack_crc<=32'hffffffff;ai<=0;ts<=1;ack_pending<=0;
    end
    1:begin
      ack_crc<=cb(ack_crc,crc_data);
      if(ai==15)ts<=2;else ai<=ai+1'b1;
    end
    2:if(tx_ready)ts<=3;
    3:if(tx_ack)begin ai<=0;ts<=4;end
    4:begin if(ai==19)ts<=0;else ai<=ai+1'b1;end
    default:ts<=0;
   endcase
   // Drain events replenish credits even when the sender has no packet in flight.
   if(pop && active)begin ack_pending<=1;reply_session<=session;status<=error?4:0;end
   if(good)begin
    ack_pending<=1;reply_session<=p_session;
    status<=error?4:push?0:duplicate?1:(!active||matched)?2:3;
   end
  end
 end
endmodule
