// Low-rate video DMA beside the existing D5 owner, ahead of the unchanged
// audio_sdram_access and SDRAM PHY. BL4 aligned, one burst in flight.
// HQ: two 128K-word slots at 0x1C0000, 76800 valid words each; legacy-only build: 2 x 19200 at 0x1B0000.
// RGB565 two-line cache scales HQ 320x240 by 2 or legacy 160x120 by 4.
module tf_video_frame_store #(parameter HQ_ENABLE=0, parameter FAST_ENABLE=0, parameter COMPACT_LAYOUT=0)(
 input sd_clk,mem_clk,pixel_clk,rst,input enable_sd,
 input [63:0] burst_data,input burst_valid,burst_last,output burst_ready,
 input legacy_busy,legacy_wr,legacy_rd,input audio_pause,
 input sdram_ready,sdram_refresh,sdram_busy,
 output reg owned,output wire pause_legacy,output wire busy,
 output wire wr_en,output wire [20:0] wr_addr,output wire [31:0] wr_data,
 output wire rd_en,output wire [20:0] rd_addr,input rd_valid,input [31:0] rd_data,
 input vs,de,input sample_transfer,play_enable,paused,
 output wire [23:0] rgb,output wire display_active,
 output reg underrun,output reg late_frame,output reg fault,
 output reg [7:0] displayed_frames,input profile_high_sd,input profile_fast_sd
);
 localparam PACK=HQ_ENABLE && FAST_ENABLE;
 localparam BANK_BITS=PACK?2:1;
 reg vs_d,de_d;reg [9:0] x;reg [8:0] y;
 localparam ROW_BITS=HQ_ENABLE?8:7, TAG_BITS=ROW_BITS+BANK_BITS;
 localparam ADDR_BITS=HQ_ENABLE?18:16, POS_BITS=HQ_ENABLE?(PACK?16:17):15;
 localparam CACHE_AW=HQ_ENABLE?(PACK?9:10):9, COLUMN_BITS=HQ_ENABLE?(PACK?8:9):8;
 localparam [ADDR_BITS-1:0] BANK_WORDS=HQ_ENABLE?131072:19200;
 localparam [ROW_BITS-1:0] ROW_ONE=1;
 reg [1:0] fast_pixel;
 reg profile_capture;reg [1:0] profile_mem,profile_pixel;
 wire high_mem=HQ_ENABLE && profile_mem[1];
 wire high_pixel=HQ_ENABLE && profile_pixel[1];
 wire [ROW_BITS-1:0] source_row=high_pixel?y[8:1]:{1'b0,y[8:2]};
 wire [8:0] source_column=high_pixel?x[9:1]:{1'b0,x[9:2]};
 // Mailbox payload is immutable from request until returned acknowledgement.
 reg req_sd;reg [63:0] payload_sd;reg last_sd;reg half_sd,launched_sd;
 reg ack_mem;reg [1:0] ack_sd;reg [1:0] req_mem,en_mem;
 assign burst_ready=enable_sd && (!PACK || !half_sd || launched_sd) && (req_sd==ack_sd[1]);
 always @(posedge sd_clk or posedge rst)begin
  if(rst)begin req_sd<=0;half_sd<=0;launched_sd<=0;payload_sd<=0;last_sd<=0;ack_sd<=0;end
  else begin ack_sd<={ack_sd[0],ack_mem};
   if(PACK)begin
    if(!enable_sd)begin half_sd<=0;launched_sd<=0;end
    else if(!half_sd && burst_valid && burst_ready)begin payload_sd<=burst_data;half_sd<=1;end
    else if(half_sd && !launched_sd && burst_valid)begin
     last_sd<=burst_last;req_sd<=~req_sd;launched_sd<=1;
    end else if(half_sd && launched_sd && burst_valid && burst_ready)begin half_sd<=0;launched_sd<=0;end
   end else if(burst_valid && burst_ready)begin payload_sd<=burst_data;last_sd<=burst_last;req_sd<=~req_sd;end
  end
 end
 reg [1:0] en_pixel;
 reg commit_req;reg commit_bank;reg commit_ack_pixel;
 reg [1:0] commit_ack_mem,commit_sync;
 reg [BANK_BITS-1:0] active_bank;reg have_display,due;
 // Completion sequence retains modulo-eight Gray CDC. Physical cursors
 // independently wrap at three to retain full FM history and WAV capacity.
 reg [1:0] write_slot,read_slot;
 reg [2:0] write_seq,read_seq,write_gray,read_gray;
 wire [1:0] display_slot=COMPACT_LAYOUT ? read_slot : read_seq[1:0];
 reg [2:0] write_gray_capture,write_gray_p1,write_gray_p2,read_gray_capture,read_gray_m1,read_gray_m2;
 wire [2:0] ready_seq={write_gray_p2[2],write_gray_p2[2]^write_gray_p2[1],^write_gray_p2};
 wire [2:0] released_seq={read_gray_m2[2],read_gray_m2[2]^read_gray_m2[1],^read_gray_m2};
 wire [2:0] queued_frames=write_seq-released_seq;
 wire frame_available=PACK ? read_seq!=ready_seq : commit_sync[1]!=commit_ack_pixel;
 reg [12:0] audio_position;

 wire frame_boundary=vs_d && !vs;
 reg row_req_pixel;reg [TAG_BITS-1:0] row_payload; // {display bank, source row}
 // Receive 25MHz-origin mailbox state on the falling 50MHz edge,
 // then use the rising-edge synchronizers/FSM. Every 25MHz edge coincides
 // with a 50MHz rising edge, so changing only the source edge is insufficient.
 // Payload is held until acknowledgement and captured with the request.
 reg commit_ack_capture,row_req_capture;reg [TAG_BITS-1:0] row_payload_capture;
 always @(negedge mem_clk or posedge rst)begin
  if(rst)begin commit_ack_capture<=0;row_req_capture<=0;row_payload_capture<=0;profile_capture<=0;read_gray_capture<=0;write_gray_capture<=0;end
  else begin commit_ack_capture<=commit_ack_pixel;row_req_capture<=row_req_pixel;row_payload_capture<=row_payload;profile_capture<=profile_high_sd;read_gray_capture<=read_gray;write_gray_capture<=write_gray;end
 end
 reg row_ack_mem;reg [1:0] row_ack_pixel,row_req_mem;
 reg [TAG_BITS-1:0] completed_row;reg [TAG_BITS-1:0] row_work;
 reg [TAG_BITS-1:0] tag0,tag1;reg valid0,valid1;
 reg [ROW_BITS-1:0] requested_row;
 reg [(PACK?32:16)-1:0] line_ram[0:(1<<CACHE_AW)-1];
 wire [15:0] cache_pixel;reg cache_valid;
 wire [CACHE_AW-1:0] cache_addr=PACK?{(high_pixel?y[1]:y[2]),source_column[8:1]}:HQ_ENABLE?{(high_pixel?y[1]:y[2]),source_column}:{y[2],x[9:2]};
 // RAM contents are never reset; tags prevent uninitialized reads being shown.
 generate if(PACK)begin:packed_cache
  reg [31:0] word_pixel;reg half_pixel;
  always @(posedge pixel_clk)begin word_pixel<=line_ram[cache_addr];half_pixel<=source_column[0];end
  assign cache_pixel=half_pixel?word_pixel[31:16]:word_pixel[15:0];
 end else begin:legacy_cache
  reg [15:0] value_pixel;always @(posedge pixel_clk)value_pixel<=line_ram[cache_addr];
  assign cache_pixel=value_pixel;
 end endgenerate
 assign rgb=cache_valid ? {cache_pixel[15:11],cache_pixel[15:13],
               cache_pixel[10:5],cache_pixel[10:9],cache_pixel[4:0],cache_pixel[4:2]} : 24'd0;
 assign display_active=have_display && en_pixel[1];
 wire [TAG_BITS-1:0] current_tag={active_bank,source_row};
 wire cache_bank=high_pixel?y[1]:y[2];
 always @(posedge pixel_clk or posedge rst)begin
  if(rst)begin read_seq<=0;read_gray<=0;write_gray_p1<=0;write_gray_p2<=0;fast_pixel<=0;profile_pixel<=0;en_pixel<=0;commit_sync<=0;commit_ack_pixel<=0;row_ack_pixel<=0;
   read_slot<=0;active_bank<=0;have_display<=0;due<=0;audio_position<=0;vs_d<=1;de_d<=0;
   x<=0;y<=0;row_req_pixel<=0;row_payload<=0;requested_row<=0;
   tag0<=0;tag1<=0;valid0<=0;valid1<=0;cache_valid<=0;
   underrun<=0;late_frame<=0;displayed_frames<=0;end
  else begin
   write_gray_p1<=write_gray_capture;write_gray_p2<=write_gray_p1;
   fast_pixel<={fast_pixel[0],FAST_ENABLE && profile_fast_sd};profile_pixel<={profile_pixel[0],profile_mem[1]};en_pixel<={en_pixel[0],enable_sd};commit_sync<={commit_sync[0],commit_req};
   row_ack_pixel<={row_ack_pixel[0],row_ack_mem};vs_d<=vs;de_d<=de;
   if(!vs)begin x<=0;y<=0;end
   else begin if(de)x<=x+1'b1;else x<=0;if(de_d && !de)y<=y+1'b1;end
   cache_valid<=display_active && de && (cache_bank ? valid1 && tag1==current_tag : valid0 && tag0==current_tag);
   if(display_active && de && !(cache_bank ? valid1 && tag1==current_tag : valid0 && tag0==current_tag))underrun<=1;
   if(row_ack_pixel[1]==row_req_pixel)begin
    if(row_payload==completed_row)begin
     if(completed_row[0])begin tag1<=completed_row;valid1<=1;end
     else begin tag0<=completed_row;valid0<=1;end
    end
   end
   if(!en_pixel[1])begin
    have_display<=0;due<=0;audio_position<=0;valid0<=0;valid1<=0;
    if(PACK)begin read_seq<=0;read_gray<=0;read_slot<=0;end
    commit_ack_pixel<=commit_sync[1];underrun<=0;late_frame<=0;displayed_frames<=0;
   end else begin
    // Begin the first HQ frame at VS before releasing its first PCM sample.
    if(FAST_ENABLE && high_pixel && !have_display && play_enable)due<=1;
    if(sample_transfer)begin
     if(!have_display)due<=1;
     if(audio_position==(high_pixel?(fast_pixel[1]?2399:4799):3199))begin audio_position<=0;due<=1;end
     else audio_position<=audio_position+1'b1;
    end
    if(frame_boundary)begin
     valid0<=0;valid1<=0;
     if(due && play_enable && !paused)begin
      if(frame_available)begin
       if(COMPACT_LAYOUT)read_slot<=read_slot==2 ? 0 : read_slot+1'b1;
       active_bank<=PACK?display_slot:commit_bank;have_display<=1;commit_ack_pixel<=commit_sync[1];
       if(PACK)begin read_seq<=read_seq+1'b1;read_gray<=((read_seq+3'd1)>>1)^(read_seq+3'd1);end
       due<=0;displayed_frames<=displayed_frames+1'b1;
      end else if(have_display)late_frame<=1;
     end
     // Back porch provides >1 ms for first-line prefetch.
     if(row_ack_pixel[1]==row_req_pixel && (have_display || (due && frame_available)))begin
      row_payload<={(due && play_enable && !paused && frame_available)?(PACK?display_slot:{{(BANK_BITS-1){1'b0}},commit_bank}):active_bank,{ROW_BITS{1'b0}}};
      requested_row<=0;row_req_pixel<=~row_req_pixel;
     end
    end else if(display_active && de && x==0 && (high_pixel?(y[0]==0 && y<478):(y[1:0]==0 && y<476)) && row_ack_pixel[1]==row_req_pixel)begin
     row_payload<={active_bank,(source_row+ROW_ONE)};requested_row<=source_row+ROW_ONE;row_req_pixel<=~row_req_pixel;
    end
   end
  end
 end

 localparam IDLE=0,WRITE=1,WTAIL=2,READ=3,RWAIT=4,RGAP=5,FAILED=6;
 reg [2:0] state;reg [4:0] tail; // shared by disjoint ownership tail and burst-tail states
 reg [1:0] beat,returned;reg [COLUMN_BITS-1:0] column;
 reg [ADDR_BITS-1:0] address;
 reg write_bank;reg [POS_BITS-1:0] write_position;
 wire [TAG_BITS-1:0] read_row=PACK?row_work:row_payload_capture;reg [15:0] timeout_count;
 reg enable_d;
 wire commit_pending=PACK ? queued_frames>=(COMPACT_LAYOUT?2:3) : commit_req!=commit_ack_mem[1];
 assign pause_legacy=en_mem[1] || owned;
 assign busy=owned && (state==WRITE || state==WTAIL || state==READ || state==RWAIT || state==RGAP);
 assign wr_en=owned && state==WRITE;
 // HQ banks are 128K-word aligned within the reserved final 1MiB.
 // Separate read and write pointers avoid a wide write-address mux/adder.
 wire [ADDR_BITS-1:0] write_address=PACK ? {(COMPACT_LAYOUT?write_slot:write_seq[1:0]),write_position} : HQ_ENABLE ? {write_bank,write_position} : (write_bank?BANK_WORDS:0)+write_position;
 wire [POS_BITS-1:0] row_offset=((({{(ADDR_BITS-ROW_BITS){1'b0}},read_row[ROW_BITS-1:0]}<<(PACK?6:7))+({{(ADDR_BITS-ROW_BITS){1'b0}},read_row[ROW_BITS-1:0]}<<(PACK?4:5)))<<(high_mem?1:0));
 function [20:0] slot_base;
  input [1:0] slot;
  begin case(slot)
   0:slot_base=21'h196000;
   1:slot_base=21'h1a8000;
   2:slot_base=21'h1b4000;
   default:slot_base=21'h196000;
  endcase end
 endfunction
 assign wr_addr=COMPACT_LAYOUT && PACK ? slot_base(write_slot)+{write_position[POS_BITS-1:2],beat} :
  (HQ_ENABLE?21'h1c0000:21'h1b0000) | {write_address[ADDR_BITS-1:2],beat};
 assign wr_data=PACK ? (beat[1]?burst_data[beat[0]*32+:32]:payload_sd[beat[0]*32+:32]) : {16'd0,payload_sd[beat*16+:16]};
 assign rd_en=owned && state==READ;
 assign rd_addr=COMPACT_LAYOUT && PACK ? slot_base(address[ADDR_BITS-1:POS_BITS])+{address[POS_BITS-1:2],beat} :
  (HQ_ENABLE?21'h1c0000:21'h1b0000) | {address[ADDR_BITS-1:2],beat};
 always @(posedge mem_clk)if(owned && rd_valid && (state==READ || state==RWAIT))
  line_ram[{read_row[0],column[COLUMN_BITS-1:2],returned}]<=(PACK?rd_data:{16'd0,rd_data[15:0]});
 always @(posedge mem_clk or posedge rst)begin
  if(rst)begin
   row_work<=0;write_seq<=0;write_gray<=0;read_gray_m1<=0;read_gray_m2<=0;profile_mem<=0;req_mem<=0;en_mem<=0;commit_ack_mem<=0;row_req_mem<=0;owned<=0;tail<=0;
   state<=IDLE;beat<=0;returned<=0;column<=0;address<=0;
   write_slot<=0;write_bank<=0;write_position<=0;
   ack_mem<=0;commit_req<=0;commit_bank<=0;row_ack_mem<=0;completed_row<=0;
   timeout_count<=0;enable_d<=0;fault<=0;
  end else begin
   row_work<=row_payload_capture;read_gray_m1<=read_gray_capture;read_gray_m2<=read_gray_m1;
   profile_mem<={profile_mem[0],profile_capture};req_mem<={req_mem[0],req_sd};en_mem<={en_mem[0],enable_sd};
   commit_ack_mem<={commit_ack_mem[0],commit_ack_capture};row_req_mem<={row_req_mem[0],row_req_capture};enable_d<=en_mem[1];
   if(en_mem[1] && !enable_d)begin write_position<=0;write_bank<=0;fault<=0;end
   if(!owned)begin
    if(!en_mem[1])begin tail<=0;ack_mem<=req_mem[1];row_ack_mem<=row_req_mem[1];end
    else if(legacy_busy || legacy_wr || legacy_rd || audio_pause || sdram_busy || sdram_refresh || !sdram_ready)tail<=0;
    else if(tail==16)begin owned<=1;tail<=0;end else tail<=tail+1'b1;
   end
   if(owned)case(state)
    IDLE:begin
     timeout_count<=0;
     if(!en_mem[1])begin
      ack_mem<=req_mem[1];row_ack_mem<=row_req_mem[1];commit_req<=commit_ack_mem[1];
      if(PACK)begin write_seq<=0;write_gray<=0;write_slot<=0;end
      if(audio_pause || sdram_busy || sdram_refresh)tail<=0;
      else if(tail==16)begin owned<=0;tail<=0;end else tail<=tail+1'b1;
     end else if(!audio_pause && !sdram_busy && !sdram_refresh && sdram_ready)begin
      tail<=0;
      if(row_req_mem[1]!=row_ack_mem)begin
       column<=0;beat<=0;returned<=0;
       address<=HQ_ENABLE ? {read_row[TAG_BITS-1:ROW_BITS],row_offset} : (row_payload_capture[ROW_BITS]?BANK_WORDS:0)+row_offset;
       state<=READ;
      end else if(req_mem[1]!=ack_mem && !commit_pending)begin
       beat<=0;tail<=0;
       state<=WRITE;
      end
     end
    end
    WRITE:begin
     if(beat==3)begin state<=WTAIL;tail<=0;end else beat<=beat+1'b1;
    end
    WTAIL:if(tail==(FAST_ENABLE?3:16))begin
     if(!FAST_ENABLE || (!sdram_busy && !sdram_refresh))begin
     ack_mem<=req_mem[1];state<=IDLE;
     if(last_sd)begin
      if(write_position!=(high_mem?(PACK?38396:76796):(PACK?9596:19196)))fault<=1;
      if(PACK)begin write_seq<=write_seq+1'b1;write_gray<=((write_seq+3'd1)>>1)^(write_seq+3'd1);end
      if(COMPACT_LAYOUT)write_slot<=write_slot==2 ? 0 : write_slot+1'b1;
      commit_bank<=write_bank;commit_req<=~commit_req;write_bank<=~write_bank;write_position<=0;
     end else write_position<=write_position+3'd4;
     end
    end else tail<=tail+1'b1;
    READ:begin
     if(beat==3)state<=RWAIT;else beat<=beat+1'b1;
     if(rd_valid)returned<=returned+1'b1;
    end
    RWAIT:begin
     timeout_count<=timeout_count+1'b1;
     if(rd_valid)begin
      returned<=returned+1'b1;
      if(returned==3)begin state<=RGAP;tail<=0;timeout_count<=0;end
     end else if(timeout_count==65535)begin fault<=1;state<=FAILED;end
    end
    RGAP:if(tail==3)begin
     if(column==(high_mem?(PACK?156:316):(PACK?76:156)))begin completed_row<=read_row;row_ack_mem<=row_req_mem[1];state<=IDLE;end
     // Wait at least four cycles after the final returned word and
     // also wait for actual controller busy/refresh to clear.
     // A row is one bounded display transaction. Keep v_busy asserted
     // throughout RGAP, so a pending audio CLAIM cannot preempt every BL4.
     // audio_pause includes CLAIM; waiting for it here would deadlock with
     // the bridge waiting for v_busy. SDRAM busy/refresh still take priority.
     else if(!sdram_busy && !sdram_refresh)begin
      column<=column+3'd4;address<=address+3'd4;beat<=0;returned<=0;state<=READ;
     end
    end else tail<=tail+1'b1;
    // Keep ownership on an incomplete SDRAM return rather than routing late
    // words to another client. A board reset is the explicit fault recovery.
    FAILED:;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
