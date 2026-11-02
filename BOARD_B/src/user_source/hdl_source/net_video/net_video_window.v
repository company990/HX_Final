// Explicit ready/valid CDC. Source storage stays stable until the destination
// accepts it; synchronizers and destination capture are retained in hardware.
module net_video_mailbox #(parameter W=192,parameter RAM32K=0,parameter DEST_CAPTURE=0,parameter RAM_DST_READ=0,parameter SOURCE_HOLDS=0)(
 input wire src_clk,dst_clk,rst,input wire [W-1:0] data,input wire valid,
 output wire ready,output wire [W-1:0] out_data,output reg out_valid,
 input wire out_ready);
 reg request,ack,a0,a1,r0,r1;wire [W-1:0] held;wire [W-1:0] stored;
 assign held=SOURCE_HOLDS ? data : stored;
 assign ready=request==a1;
 // held remains stable for the COMPLETE destination ready/valid transfer.
 // ACK is returned only after out_ready; an extra destination-wide payload
 // register duplicates the same storage without increasing CDC safety.
 // Optional destination capture breaks the related-clock path into the
 // 25 MHz application decode. Ownership and valid/ACK edges are unchanged.
 generate if(DEST_CAPTURE)begin:destination_capture
  reg [W-1:0] captured;
  always @(posedge dst_clk)if(!out_valid && r1!=ack)captured<=held;
  assign out_data=captured;
 end else begin:source_held
  assign out_data=held;
 end endgenerate
 generate if(RAM32K)begin:ram_payload
  if(RAM_DST_READ)begin:destination_read
   // The synchronized request arrives only after the entire source word
   // has been written. Read once on the edge that asserts out_valid; keep
   // that RAM output immutable through destination ready and returned ACK.
   net_video_cdc32k #(.WIDTH(W)) storage(src_clk,dst_clk,valid && ready,
      !out_valid && r1!=ack,data,stored);
  end else begin:source_read
   net_video_capture32k #(.WIDTH(W)) storage(src_clk,valid && ready,data,stored);
  end
 end else begin:register_payload
  reg [W-1:0] buffer;
  always @(posedge src_clk)if(valid && ready)buffer<=data;
  assign stored=buffer;
 end endgenerate
 always @(posedge src_clk or posedge rst)begin
  if(rst)begin request<=0;a0<=0;a1<=0;end
  else begin a0<=ack;a1<=a0;if(valid && ready)request<=~request;end
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

// Eight unacknowledged groups, twelve RGB565 pixels per group. A narrow
// 128x12 true dual-port memory fits one 9K BRAM instead of 192-wide replicated
// blocks. One port serializes writes; the other prefetches the next packet
// while the PHY is shifting the current one. No reset on the RAM array.
// ACK is cumulative NEXT sequence (modulo 256); only the eight live slots may
// be acknowledged. Cursor wraps to the oldest slot to recover lost packets.
module net_video_replay #(parameter SOURCE_HOLDS=0)(
 input wire clk,rst,input wire [3:0] epoch,
 input wire [191:0] data,input wire [2:0] flags,input wire valid,
 output wire ready,input wire ack_valid,input wire [7:0] ack_next,
 output wire [215:0] packet,output wire packet_valid,input wire packet_take,
 output wire [3:0] level,output wire [3:0] acknowledged_groups);
 reg [11:0] mem[0:127] /* fehdl force_ram=1, ram_style="bram" */;
 reg [2:0] flag_mem[0:7];
 reg [7:0] written,oldest,cursor,last_sent;reg sent_any;
 reg writing,reading,cached;
 reg [3:0] wi;reg [4:0] ri;
 reg [191:0] write_shift,read_shift;
 reg [7:0] read_seq,cache_seq;reg [2:0] write_flags,cache_flags;
 reg [11:0] q;
 reg [11:0] selected_word;
 always @*begin
  case(wi)
   0:selected_word=data[11:0];1:selected_word=data[23:12];2:selected_word=data[35:24];
   3:selected_word=data[47:36];4:selected_word=data[59:48];5:selected_word=data[71:60];
   6:selected_word=data[83:72];7:selected_word=data[95:84];8:selected_word=data[107:96];
   9:selected_word=data[119:108];10:selected_word=data[131:120];11:selected_word=data[143:132];
   12:selected_word=data[155:144];13:selected_word=data[167:156];14:selected_word=data[179:168];
   default:selected_word=data[191:180];
  endcase
 end
 wire [7:0] outstanding=written-oldest;
 wire [7:0] acknowledge_distance=ack_next-oldest;
 wire ack_ok=ack_valid && acknowledge_distance<=outstanding;
 // Count only newly retired groups; duplicates, stale ACKs and impossible
 // forward jumps never contribute to the delivered-media diagnostic.
 assign acknowledged_groups=(!rst && ack_ok)?acknowledge_distance[3:0]:4'd0;
 wire [7:0] effective_oldest=ack_ok?ack_next:oldest;
 wire [7:0] effective_level=written-effective_oldest;
 wire cache_live=(cache_seq-effective_oldest)<effective_level;
 wire cursor_live=(cursor-effective_oldest)<effective_level;
 // The request ACK precedes the concurrently shifted media response. When
 // it still points BEFORE that response sequence, a prior packet was lost.
 // Restart immediately at that missing sequence rather than spending the
 // rest of the window transmitting successors the receiver cannot accept.
 wire [7:0] sent_distance=last_sent-ack_next;
 wire missing=ack_ok && sent_any && sent_distance!=0 && sent_distance<8;
 assign level=outstanding[3:0];
 assign ready=SOURCE_HOLDS ? (writing && wi==15):(!writing && outstanding<8);
 assign packet={8'h56,cache_seq,epoch,1'b0,cache_flags,read_shift};
 assign packet_valid=cached && cache_live;
 always @(posedge clk)begin
  if(writing)mem[{written[2:0],wi}]<=SOURCE_HOLDS?selected_word:write_shift[11:0];
  if(reading && ri<16)q<=mem[{read_seq[2:0],ri[3:0]}];
  if(writing && wi==15)flag_mem[written[2:0]]<=SOURCE_HOLDS?flags:write_flags;
  if(!SOURCE_HOLDS)begin
   if(valid && ready)write_shift<=data;
   else if(writing)write_shift<={12'd0,write_shift[191:12]};
  end
  if(reading && ri!=0)read_shift<={q,read_shift[191:12]};
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin written<=0;oldest<=0;cursor<=0;last_sent<=0;sent_any<=0;writing<=0;reading<=0;cached<=0;
   wi<=0;ri<=0;read_seq<=0;
   cache_seq<=0;write_flags<=0;cache_flags<=0;end
  else begin
   if(ack_ok)oldest<=ack_next;
   if(valid && !writing && outstanding<8)begin writing<=1;wi<=0;write_flags<=flags;end
   if(writing)begin
    if(wi==15)begin written<=written+1'b1;writing<=0;end else wi<=wi+1'b1;
   end
   if(cached && !cache_live)begin cached<=0;cursor<=effective_oldest;end
   if(packet_take && packet_valid)begin cached<=0;cursor<=cache_seq+1'b1;last_sent<=cache_seq;sent_any<=1;end
   if(!cached && !reading && effective_level!=0)begin
    read_seq<=cursor_live?cursor:effective_oldest;
    reading<=1;ri<=0;
   end
   if(reading)begin
    if(ri==16)begin
     reading<=0;
     if((read_seq-effective_oldest)<effective_level)begin
      cache_seq<=read_seq;
      cache_flags<=flag_mem[read_seq[2:0]];cached<=1;
     end else cursor<=effective_oldest;
    end else ri<=ri+1'b1;
   end
   if(missing)begin
    cached<=0;reading<=0;cursor<=ack_next;sent_any<=0;
   end
  end
 end
endmodule

// Standard six-pixel interface used by the imported frame writer. Flags 0/1
// mark only the first/last half respectively; last-frame applies to both.
module net_video_unpack #(parameter SOURCE_HOLDS=0)(
 input wire clk,rst,input wire [194:0] in_data,input wire in_valid,
 output wire in_ready,output wire [95:0] pixels,output wire [2:0] flags,
 output wire valid,input wire ready);
 reg full,second;reg [191:0] held;reg [2:0] held_flags;
 wire [191:0] words=SOURCE_HOLDS?in_data[191:0]:held;
 wire [2:0] group_flags=SOURCE_HOLDS?in_data[194:192]:held_flags;
 // For a ready/valid mailbox, keep its ACK pending through both halves.
 // The source-held payload is then the only 192-bit group storage.
 assign in_ready=SOURCE_HOLDS ? (second && ready):!full;
 assign pixels=second?words[191:96]:words[95:0];
 assign flags={group_flags[2],second && group_flags[1],!second && group_flags[0]};
 assign valid=SOURCE_HOLDS?in_valid:full;
 always @(posedge clk or posedge rst)begin
  if(rst)begin full<=0;second<=0;held<=0;held_flags<=0;end
  else begin
   if(!SOURCE_HOLDS && in_valid && in_ready)begin held<=in_data[191:0];held_flags<=in_data[194:192];full<=1;second<=0;end
   if(valid && ready)begin if(second)begin full<=0;second<=0;end else second<=1;end
  end
 end
endmodule

// Source-held mailbox storage. No reset on data, exactly like the former
// registers; ready/valid handshake prevents uninitialized data consumption.
module net_video_capture32k #(parameter WIDTH=128)(input clk,load,input [WIDTH-1:0] data,output [WIDTH-1:0] q);
 localparam BANKS=(WIDTH+31)/32;
 wire [BANKS*32-1:0] padded={{(BANKS*32-WIDTH){1'b0}},data};wire [BANKS*32-1:0] stored;
 assign q=stored[WIDTH-1:0];
`ifdef PRIORITY12_SIM
 reg [WIDTH-1:0] held;
 always @(posedge clk)if(load)held<=data;
 assign stored={{(BANKS*32-WIDTH){1'b0}},held};
`else
 genvar bank;
 generate for(bank=0;bank<BANKS;bank=bank+1)begin:g_bank
  EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(1),.ADDR_WIDTH_B(1),
   .DATA_DEPTH_A(2),.DATA_DEPTH_B(2),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
   .WRITEMODE_A("WRITETHROUGH"),.WRITEMODE_B("WRITETHROUGH"),.RESETMODE("SYNC"),
   .IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
   .dia(padded[bank*32+:16]),.addra(1'b0),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(load),.rsta(1'b0),.bea(1'b0),.doa(stored[bank*32+:16]),
   .dib(padded[bank*32+16+:16]),.addrb(1'b1),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(load),.rstb(1'b0),.beb(1'b0),.dob(stored[bank*32+16+:16]));
 end endgenerate
`endif
endmodule

// Candidate-only direct word read; original replay API remains available.
module net_video_replay_stream #(parameter SOURCE_HOLDS=0,parameter ACK_PIPE=0)(
 input wire clk,rst,input wire [3:0] epoch,
 input wire [191:0] data,input wire [2:0] flags,input wire valid,
 output wire ready,input wire ack_valid,input wire [7:0] ack_next,
 output wire [215:0] packet,output wire packet_valid,input wire packet_take,
 output wire [3:0] level,output wire [3:0] acknowledged_groups,
 input wire [2:0] serial_slot,input wire [3:0] serial_index,output reg [11:0] serial_word,
 input wire [7:0] sent_sequence);
 // Optional one-fast-clock ACK stage separates the physical RX registers
 // from the replay-window control. It adds 8ns internally at 125MHz; wire
 // framing, data retention and the existing inter-frame gap are unchanged.
 reg ack_valid_q;reg [7:0] ack_next_q;
 always @(posedge clk or posedge rst)
  if(rst)begin ack_valid_q<=0;ack_next_q<=0;end
  else begin ack_valid_q<=ack_valid;if(ack_valid)ack_next_q<=ack_next;end
 wire commit_ack_valid=ACK_PIPE?ack_valid_q:ack_valid;
 wire [7:0] commit_ack_next=ACK_PIPE?ack_next_q:ack_next;
 reg [11:0] mem[0:127] /* fehdl force_ram=1, ram_style="bram" */;
 reg [2:0] flag_mem[0:7];
 reg [7:0] written,oldest,cursor,last_sent;reg sent_any;
 reg writing,cached;
 reg [3:0] wi;
 reg [191:0] write_shift;
 reg [7:0] cache_seq;reg [2:0] write_flags,cache_flags;
 reg [11:0] selected_word;
 always @*begin
  case(wi)
   0:selected_word=data[11:0];1:selected_word=data[23:12];2:selected_word=data[35:24];
   3:selected_word=data[47:36];4:selected_word=data[59:48];5:selected_word=data[71:60];
   6:selected_word=data[83:72];7:selected_word=data[95:84];8:selected_word=data[107:96];
   9:selected_word=data[119:108];10:selected_word=data[131:120];11:selected_word=data[143:132];
   12:selected_word=data[155:144];13:selected_word=data[167:156];14:selected_word=data[179:168];
   default:selected_word=data[191:180];
  endcase
 end
 wire [7:0] outstanding=written-oldest;
 wire [7:0] acknowledge_distance=commit_ack_next-oldest;
 wire ack_ok=commit_ack_valid && acknowledge_distance<=outstanding;
 // Count only newly retired groups; duplicates, stale ACKs and impossible
 // forward jumps never contribute to the delivered-media diagnostic.
 assign acknowledged_groups=(!rst && ack_ok)?acknowledge_distance[3:0]:4'd0;
 wire [7:0] effective_oldest=ack_ok?commit_ack_next:oldest;
 wire [7:0] effective_level=written-effective_oldest;
 // Exact circular [lo, hi) membership, including 255->0 wrap and
 // the empty lo==hi interval. Parallel unsigned compares replace the
 // subtract-then-compare chain on the 125MHz cache/cursor enable path.
 function in_window;input [7:0] x,lo,hi;
 begin in_window=(lo<=hi)?(x>=lo && x<hi):(x>=lo || x<hi);end endfunction
 wire cache_live=in_window(cache_seq,oldest,written) &&
                  (!ack_ok || !in_window(cache_seq,oldest,commit_ack_next));
 wire cursor_live=in_window(cursor,oldest,written) &&
                   (!ack_ok || !in_window(cursor,oldest,commit_ack_next));
 wire have_data=ack_ok ? (written!=commit_ack_next):(written!=oldest);
 // The request ACK precedes the concurrently shifted media response. When
 // it still points BEFORE that response sequence, a prior packet was lost.
 // Restart immediately at that missing sequence rather than spending the
 // rest of the window transmitting successors the receiver cannot accept.
 wire [7:0] sent_distance=last_sent-commit_ack_next;
 wire missing=ack_ok && sent_any && sent_distance!=0 && sent_distance<8;
 assign level=outstanding[3:0];
 assign ready=SOURCE_HOLDS ? (writing && wi==15):(!writing && outstanding<8);
 assign packet={8'h56,cache_seq,epoch,1'b0,cache_flags,192'd0};
 assign packet_valid=cached && cache_live;
 always @(posedge clk)begin
  if(writing)mem[{written[2:0],wi}]<=SOURCE_HOLDS?selected_word:write_shift[11:0];
  // CS-latched sequence selects the slot for the complete transaction.
  // The PHY allows several fast clocks between changing index and sampling.
  serial_word<=mem[{serial_slot,serial_index}];
  if(writing && wi==15)flag_mem[written[2:0]]<=SOURCE_HOLDS?flags:write_flags;
  if(!SOURCE_HOLDS)begin
   if(valid && ready)write_shift<=data;
   else if(writing)write_shift<={12'd0,write_shift[191:12]};
  end

 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin written<=0;oldest<=0;cursor<=0;last_sent<=0;sent_any<=0;writing<=0;cached<=0;
   wi<=0;
   cache_seq<=0;write_flags<=0;cache_flags<=0;end
  else begin
   if(ack_ok)oldest<=commit_ack_next;
   if(valid && !writing && outstanding<8)begin writing<=1;wi<=0;write_flags<=flags;end
   if(writing)begin
    if(wi==15)begin written<=written+1'b1;writing<=0;end else wi<=wi+1'b1;
   end
   if(cached && !cache_live)begin cached<=0;cursor<=effective_oldest;end
   // An early cumulative ACK may have invalidated the outgoing cache and
   // prefetched a missing sequence BEFORE this transaction finishes. Retire
   // only the sequence latched at CS; never consume a different new cache.
   if(packet_take)begin
    last_sent<=sent_sequence;sent_any<=1;
    if(cached && cache_seq==sent_sequence)begin
     cached<=0;cursor<=sent_sequence+1'b1;
    end
   end
   if(!cached && have_data)begin
    cache_seq<=cursor_live?cursor:effective_oldest;
    cache_flags<=flag_mem[cursor_live?cursor[2:0]:effective_oldest[2:0]];
    cached<=1;
   end
   if(missing)begin
    cached<=0;cursor<=commit_ack_next;sent_any<=0;
   end
  end
 end
endmodule

// A single source write and destination-clocked synchronous read per mailbox
// transaction. Sixteen bits per native DP bank preserve both independent
// clocks without introducing an external W-bit destination register bank.
module net_video_cdc32k #(parameter WIDTH=195)(
 input src_clk,dst_clk,write_enable,read_enable,
 input [WIDTH-1:0] data,output [WIDTH-1:0] q);
 localparam BANKS=(WIDTH+15)/16;
 wire [BANKS*16-1:0] padded={{(BANKS*16-WIDTH){1'b0}},data};
 wire [BANKS*16-1:0] stored;
 assign q=stored[WIDTH-1:0];
`ifdef PRIORITY12_SIM
 reg [WIDTH-1:0] memory,captured;
 always @(posedge src_clk)if(write_enable)memory<=data;
 always @(posedge dst_clk)if(read_enable)captured<=memory;
 assign stored={{(BANKS*16-WIDTH){1'b0}},captured};
`else
 genvar bank;
 generate for(bank=0;bank<BANKS;bank=bank+1)begin:g_bank
  EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(1),.ADDR_WIDTH_B(1),
   .DATA_DEPTH_A(2),.DATA_DEPTH_B(2),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
   .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),
   .IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
   .dia(padded[bank*16+:16]),.addra(1'b0),.cea(write_enable),.ocea(1'b0),
   .clka(src_clk),.wea(write_enable),.rsta(1'b0),.bea(1'b0),.doa(),
   .dib(16'd0),.addrb(1'b0),.ceb(read_enable),.oceb(1'b0),
   .clkb(dst_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(stored[bank*16+:16]));
 end endgenerate
`endif
endmodule

// Source is the complete PHY RX register. It must not be overwritten before
// source_captured, which the adapter explicitly gates into transaction launch.
// The separate consumption ACK holds ready low until both pixel halves retire.
module net_video_rx_capture #(parameter W=195)(
 input src_clk,dst_clk,rst,input [W-1:0] data,input valid,
 output ready,output [W-1:0] out_data,output reg out_valid,input out_ready,
 output source_captured);
 reg request,ack,a0,a1,r0,r1,capture_ack,c0,c1;
 reg [W-1:0] captured;
 assign ready=request==a1;
 assign source_captured=request==c1;
 assign out_data=captured;
 always @(posedge src_clk or posedge rst)begin
  if(rst)begin request<=0;a0<=0;a1<=0;c0<=0;c1<=0;end
  else begin
   a0<=ack;a1<=a0;c0<=capture_ack;c1<=c0;
   if(valid && ready)request<=~request;
  end
 end
 always @(posedge dst_clk)if(!out_valid && r1!=ack)captured<=data;
 always @(posedge dst_clk or posedge rst)begin
  if(rst)begin r0<=0;r1<=0;ack<=0;capture_ack<=0;out_valid<=0;end
  else begin
   r0<=request;r1<=r0;
   if(out_valid && out_ready)begin out_valid<=0;ack<=r1;end
   else if(!out_valid && r1!=ack)begin
    out_valid<=1;capture_ack<=r1;
   end
  end
 end
endmodule

// One CRC-accepted twelve-pixel slot, written as sixteen 12-bit words.
// The PHY shifts only on source_shift until source_captured. Request crosses only
// AFTER the final RAM write; ACK returns only after BOTH six-pixel halves.
module net_video_shift_rx #(parameter PIXEL_STREAM=0)(
 input src_clk,dst_clk,rst,input [191:0] data,input [2:0] flags,input valid,
 output ready,source_captured,output [95:0] pixels,output [2:0] pixel_flags,
 output wire pixel_valid,input pixel_ready,output source_shift);
 reg request,ack,a0,a1,r0,r1,writing,write_pending;
 reg [3:0] write_index,staged_address;
 reg [11:0] staged_data;
 reg [2:0] source_flags,dest_flags;
 wire [23:0] read_data;
 // The PHY shifts toward bit215 after each staged word. Its payload's
 // high twelve bits therefore expose the next descending word directly.
 assign source_shift=writing && !rst;
 assign source_captured=!writing && !write_pending;
 assign ready=request==a1 && source_captured;
 always @(posedge src_clk)if(writing)begin
  staged_data<=data[191:180];staged_address<=4'd15-write_index;
 end
 always @(posedge src_clk or posedge rst)begin
  if(rst)begin request<=0;a0<=0;a1<=0;writing<=0;write_pending<=0;
   write_index<=0;source_flags<=0;end
  else begin
   a0<=ack;a1<=a0;write_pending<=writing;
   if(valid && ready)begin writing<=1;write_index<=0;source_flags<=flags;end
   if(writing)begin
    if(write_index==15)writing<=0;else write_index<=write_index+1'b1;
   end
   if(write_pending && staged_address==0)request<=~request;
  end
 end
 wire read_enable;wire [2:0] read_address;
 generate if(PIXEL_STREAM)begin:stream_pixels
  // Two 24-bit RAM reads produce three RGB565 pixels. Hold only the
  // crossing eight bits and the current output, avoiding the 96-bit group
  // register and the consumer's six-to-one pixel selector.
  localparam IDLE=0,FETCH0=1,WAIT0=2,OUT0=3,FETCH1=4,WAIT1=5,OUT1=6,OUT2=7;
  reg [2:0] state,index;
  reg [7:0] carry;
  reg [15:0] sample;
  assign read_enable=(state==FETCH0 || state==FETCH1) && !rst;
  assign read_address=index;
  assign pixels={80'd0,sample};
  assign pixel_valid=!rst && (state==OUT0 || state==OUT1 || state==OUT2);
  assign pixel_flags={dest_flags[2],index==7 && state==OUT2 && dest_flags[1],
                      index==0 && state==OUT0 && dest_flags[0]};
  always @(posedge dst_clk)begin
   if(state==WAIT0)begin sample<=read_data[15:0];carry<=read_data[23:16];end
   if(state==WAIT1)sample<={read_data[7:0],carry};
   if(state==OUT1 && pixel_ready)sample<=read_data[23:8];
  end
  always @(posedge dst_clk or posedge rst)begin
   if(rst)begin r0<=0;r1<=0;ack<=0;state<=IDLE;index<=0;dest_flags<=0;end
   else begin
    r0<=request;r1<=r0;
    case(state)
     IDLE:if(r1!=ack)begin index<=0;dest_flags<=source_flags;state<=FETCH0;end
     FETCH0:state<=WAIT0;
     WAIT0:state<=OUT0;
     OUT0:if(pixel_ready)begin index<=index+1'b1;state<=FETCH1;end
     FETCH1:state<=WAIT1;
     WAIT1:state<=OUT1;
     OUT1:if(pixel_ready)state<=OUT2;
     OUT2:if(pixel_ready)begin
      if(index==7)begin ack<=r1;state<=IDLE;end
      else begin index<=index+1'b1;state<=FETCH0;end
     end
     default:state<=IDLE;
    endcase
   end
  end
 end else begin:group_pixels
 localparam IDLE=0,FETCH=1,OUTPUT=2;
 reg [1:0] state;
 reg second,read_pending;
 reg [2:0] read_index;
 reg [95:0] assembled;
 reg group_valid;
 assign pixel_valid=group_valid;
 assign read_enable=state==FETCH && read_index<4 && !rst;
 assign read_address={second,read_index[1:0]};
 assign pixels=assembled;
 assign pixel_flags={dest_flags[2],second && dest_flags[1],!second && dest_flags[0]};
 always @(posedge dst_clk)if(read_pending)assembled<={read_data,assembled[95:24]};
 always @(posedge dst_clk or posedge rst)begin
  if(rst)begin r0<=0;r1<=0;ack<=0;state<=IDLE;second<=0;
   read_index<=0;read_pending<=0;group_valid<=0;dest_flags<=0;end
  else begin
   r0<=request;r1<=r0;read_pending<=read_enable;
   case(state)
    IDLE:if(r1!=ack)begin
     second<=0;read_index<=0;dest_flags<=source_flags;state<=FETCH;
    end
    FETCH:begin
     if(read_enable)read_index<=read_index+1'b1;
     if(read_pending && read_index==4)begin group_valid<=1;state<=OUTPUT;end
    end
    OUTPUT:if(pixel_valid && pixel_ready)begin
     group_valid<=0;
     if(!second)begin second<=1;read_index<=0;state<=FETCH;end
     else begin ack<=r1;state<=IDLE;end
    end
    default:state<=IDLE;
   endcase
  end
 end
 end endgenerate
`ifdef PRIORITY12_SIM
 reg [11:0] memory[0:15];reg [23:0] q;
 always @(posedge src_clk)if(write_pending && !rst)memory[staged_address]<=staged_data;
 always @(posedge dst_clk)if(read_enable)q<={memory[{read_address,1'b1}],memory[{read_address,1'b0}]};
 assign read_data=q;
`else
 // Pad each 12-bit source word to16. The mixed-width destination reads
 // both adjacent16-bit words atomically; discard each padding nibble.
 // Ownership, write order, destination read edge and latency stay identical.
 wire [31:0] mixed_q;
 assign read_data={mixed_q[27:16],mixed_q[11:0]};
 wire lane_write=write_pending && !rst;
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(32),.ADDR_WIDTH_A(4),.ADDR_WIDTH_B(3),
   .DATA_DEPTH_A(16),.DATA_DEPTH_B(8),.MODE("PDPW"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
   .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),
   .IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) storage(
   .dia({4'd0,staged_data}),.addra(staged_address),.cea(lane_write),.ocea(1'b0),
   .clka(src_clk),.wea(lane_write),.rsta(1'b0),.bea(1'b0),.doa(),
   .dib(32'd0),.addrb(read_address),.ceb(read_enable),.oceb(1'b0),
   .clkb(dst_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(mixed_q));

`endif
endmodule
