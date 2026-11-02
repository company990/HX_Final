// Raw mono signed16 history. One block = 8 samples = four SDRAM words.
// Default 65536 blocks: 1 MiB, 524288 samples, 10.922666 seconds @48k.
// Sequence arithmetic uses an extra wrap bit; retained history is <= one ring.
module fm_timeshift_buffer #(
 parameter BW=16, parameter BLOCKS=(1<<BW), parameter BLOCKS_PER_SECOND=6000
)(input clk,rst,input capture_valid,input [17:0] capture_packet,
 output capture_ready,input fifo_fault,input tick,input command_valid,input [3:0] command,
 output wr_req,output [BW-1:0] wr_block, input [1:0] wr_word,
 input wr_active,output reg [31:0] wr_data,input wr_ack,
 output reg rd_req,output reg [BW-1:0] rd_block,
 input rd_valid,input [1:0] rd_word,input [31:0] rd_data,input mem_fault,
 output reg reply_valid,output reg [20:0] reply,output [23:0] status);
 localparam STOP=0,LIVE=1,PAUSE=2,HISTORY=3;
 reg [1:0] mode,epoch,record_epoch,write_epoch,read_epoch;
 reg [BW:0] write_sequence,read_sequence;
 reg history_full;
 wire [BW:0] valid_blocks=history_full ? BLOCKS : write_sequence;
 reg [2:0] pack_index,read_offset;
 reg [15:0] first_sample;
 reg burst_ready,recording,station_tag,cache_valid,read_busy;
 reg [BW-1:0] cache_block;
 reg [1:0] play_state;
 reg overwritten,overflow_error,memory_error,underflow_error;
 wire capture_accept=capture_valid && !capture_packet[17] &&
  !(recording && capture_packet[16]!=station_tag) && mode!=STOP && !burst_ready && !read_busy;
 wire pack_write=capture_accept && pack_index[0];
 wire [1:0] pack_word=pack_index[2:1];
 wire [31:0] pack_data={capture_packet[15:0],first_sample};
 wire [15:0] ram_low,ram_high;
 wire [31:0] read_q={ram_high,ram_low};
 wire [1:0] prefetch_word=wr_active ? (wr_word==3 ? 2'd3 : wr_word+2'd1) : 2'd0;
 wire cache_write=rd_valid && read_busy && read_epoch==epoch;
 wire ram_for_write=wr_active || (burst_ready && !read_busy);
 wire ram_history_cycle=!ram_for_write && !pack_write && !cache_write;
 reg ram_history_q;
 always @(posedge clk or posedge rst)if(rst)ram_history_q<=0;else ram_history_q<=ram_history_cycle;
 wire [2:0] ram_pair=cache_write ? {1'b1,rd_word} : pack_write ? {1'b0,pack_word} : ram_for_write ? {1'b0,prefetch_word} : {1'b1,read_offset[2:1]};
 wire [31:0] ram_din=cache_write ? rd_data : pack_data;
 wire ram_we=cache_write || pack_write;
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(4),.ADDR_WIDTH_B(4),
  .DATA_DEPTH_A(16),.DATA_DEPTH_B(16),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) scratch(
  .dia(ram_din[15:0]),.addra({ram_pair,1'b0}),.cea(1'b1),.ocea(1'b0),
  .clka(clk),.wea(ram_we),.rsta(1'b0),.bea(1'b0),.doa(ram_low),
  .dib(ram_din[31:16]),.addrb({ram_pair,1'b1}),.ceb(1'b1),.oceb(1'b0),
  .clkb(clk),.web(ram_we),.rstb(1'b0),.beb(1'b0),.dob(ram_high));
 always @*wr_data={ram_high,ram_low};
 assign capture_ready=!burst_ready && !read_busy;
 // synthesis translate_off
 always @(posedge clk)if(!rst && cache_write && pack_write)$fatal(1,"Timeshift scratch port collision");
 // synthesis translate_on
 assign wr_req=burst_ready;
 assign wr_block=write_sequence[BW-1:0];
 wire full=history_full;
 wire [BW:0] lag=write_sequence-read_sequence;
 // Before full, all committed blocks start at sequence zero. Once full,
 // capacity is a power of two: subtraction only flips the wrap bit.
 wire [BW:0] oldest=full ? {~write_sequence[BW],write_sequence[BW-1:0]} : 0;

 wire segment_break=capture_valid && (capture_packet[17] || (recording && capture_packet[16]!=station_tag));
 wire guard_jump=mode[1] && recording && full && lag>=BLOCKS-2;
 wire playback_safe=!segment_break && !guard_jump && !mem_fault && !fifo_fault && !command_valid;
 wire [BW:0] rewind_blocks=command==4 ? BLOCKS_PER_SECOND*10 : BLOCKS_PER_SECOND*5;
 wire [BW:0] rewind_lag=(mode==LIVE ? {{(BW-1){1'b0}},2'b10} : lag)+rewind_blocks;
 wire [BW:0] rewind_target=(mode==LIVE ? write_sequence : read_sequence)-
  (mode==LIVE ? rewind_blocks+2 : rewind_blocks);
 // Bounded serial division for display only. Timeline itself is sample exact.
 reg [BW:0] display_remainder;reg [3:0] display_digit;
 reg [1:0] display_field;reg [3:0] buffer_seconds,lag_seconds,position_seconds;
 assign status={recording,full,overwritten,overflow_error,memory_error,underflow_error,
  mode,buffer_seconds,lag_seconds,position_seconds,epoch,(mode!=STOP),(valid_blocks>2)};

 // One cursor ALU; priority exactly matches original sequential assignments.
 reg cursor_load;reg [BW:0] cursor_base,cursor_step;
 always @*begin
  cursor_load=0;cursor_base=read_sequence;cursor_step=0;
  if(segment_break && recording)begin cursor_load=1;cursor_base=0;end
  if(guard_jump)begin cursor_load=1;cursor_base=oldest;cursor_step=3;end
  if(playback_safe && play_state==2 && mode==HISTORY && ram_history_cycle && ram_history_q && read_offset==7)begin
   cursor_load=1;cursor_base=read_sequence;cursor_step=1;
  end
  if(mem_fault || fifo_fault)begin cursor_load=1;cursor_base=0;cursor_step=0;end
  if(command_valid)case(command)
   1:if(mode==LIVE)begin
    cursor_load=1;cursor_base=valid_blocks>2 ? write_sequence : oldest;
    cursor_step=valid_blocks>2 ? -2 : 0;
   end
   3,4:cursor_load=1;
   6:begin cursor_load=1;cursor_base=0;cursor_step=0;end
  endcase
 end
 // Select the completed rewind target AFTER its arithmetic. Modular
 // W - ((W-R)+rewind) equals R-rewind; avoid a dependent lag/negate/add chain.
 wire rewind_command=command_valid && (command==3 || command==4);
 wire [BW:0] cursor_next=rewind_command ?
  (rewind_lag>=valid_blocks ? oldest+{{(BW-1){1'b0}},2'b10} : rewind_target) :
  cursor_base+cursor_step;
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   mode<=LIVE;epoch<=0;record_epoch<=0;write_epoch<=0;read_epoch<=0;
   write_sequence<=0;read_sequence<=0;history_full<=0;pack_index<=0;read_offset<=0;first_sample<=0;
   burst_ready<=0;recording<=0;station_tag<=0;cache_valid<=0;read_busy<=0;cache_block<=0;
   play_state<=0;overwritten<=0;overflow_error<=0;memory_error<=0;underflow_error<=0;
   rd_req<=0;rd_block<=0;reply_valid<=0;reply<=0;
   display_remainder<=0;display_digit<=0;display_field<=3;buffer_seconds<=0;lag_seconds<=0;position_seconds<=0;
  end else begin
   rd_req<=0;reply_valid<=0;
   if(cursor_load)read_sequence<=cursor_next;
   if(fifo_fault)overflow_error<=1;
   if(wr_ack)begin
    burst_ready<=0;
    if(write_epoch==record_epoch && recording)begin
     write_sequence<=write_sequence+1'b1;
     if(write_sequence==BLOCKS-1)history_full<=1;
    end
   end
   if(rd_valid && read_busy && rd_word==3)begin
    read_busy<=0;
    if(read_epoch==epoch)cache_valid<=1;
   end
   if(capture_valid)begin
    if(capture_packet[17] || (recording && capture_packet[16]!=station_tag))begin
     if(recording)begin
      epoch<=epoch+1'b1;record_epoch<=record_epoch+1'b1;history_full<=0;write_sequence<=0;
      read_offset<=0;cache_valid<=0;play_state<=0;pack_index<=0;
      if(mode!=STOP)mode<=LIVE;
     end
     recording<=0;station_tag<=capture_packet[16];
    end else if(capture_accept)begin
     recording<=1;station_tag<=capture_packet[16];
     if(!pack_index[0])first_sample<=capture_packet[15:0];
     pack_index<=pack_index+1'b1;
     if(pack_index==7)begin burst_ready<=1;write_epoch<=record_epoch;end
    end
   end
   // Protect a two-block guard BEFORE the oldest SDRAM block is overwritten.
   if(mode[1] && recording && full && lag>=BLOCKS-2)begin
    read_offset<=0;cache_valid<=0;play_state<=0;
    epoch<=epoch+1'b1;overwritten<=1;
   end
   if(tick && playback_safe)begin
    if(display_field==3)begin display_field<=0;display_digit<=0;display_remainder<=valid_blocks;end
    if(mode==HISTORY && recording && !memory_error && !overflow_error)begin
     if(lag<=2)begin mode<=LIVE;epoch<=epoch+1'b1;cache_valid<=0;play_state<=0;
      reply_valid<=1;reply<={2'd1,epoch+2'd1,1'b0,16'd0};end
     else if(play_state==0)play_state<=1;
     else begin underflow_error<=1;play_state<=0;reply_valid<=1;reply<={mode,epoch,1'b0,16'd0};end
    end else begin reply_valid<=1;reply<={mode,epoch,1'b0,16'd0};end
   end
   if(playback_safe && !tick && play_state==1 && mode==HISTORY)begin
    if(cache_valid && cache_block==read_sequence[BW-1:0])begin if(ram_history_cycle)play_state<=2;end
    else if(!read_busy)begin
     rd_req<=1;rd_block<=read_sequence[BW-1:0];cache_block<=read_sequence[BW-1:0];
     read_epoch<=epoch;read_busy<=1;
    end
   end
   if(playback_safe && play_state==2 && mode==HISTORY && ram_history_cycle && ram_history_q)begin
    reply_valid<=1;reply<={mode,epoch,1'b1,(read_offset[0]?read_q[31:16]:read_q[15:0])};
    play_state<=0;read_offset<=read_offset+1'b1;
    if(read_offset==7)begin cache_valid<=0;end
   end
   if(display_field<3)begin
    if(display_remainder>=BLOCKS_PER_SECOND)begin
     display_remainder<=display_remainder-BLOCKS_PER_SECOND;display_digit<=display_digit+1'b1;
    end else begin
     case(display_field)
      0:begin buffer_seconds<=display_digit;display_remainder<=mode==LIVE || mode==STOP ? 0 : lag;end
      1:begin lag_seconds<=display_digit;display_remainder<=mode==LIVE || mode==STOP ? valid_blocks : valid_blocks>lag ? valid_blocks-lag : 0;end
      2:position_seconds<=display_digit;
     endcase
     display_digit<=0;display_field<=display_field+1'b1;
    end
   end
   if(mem_fault || fifo_fault)begin
    if(recording)begin epoch<=epoch+1'b1;record_epoch<=record_epoch+1'b1;end
    mode<=LIVE;recording<=0;cache_valid<=0;play_state<=0;pack_index<=0;
    history_full<=0;write_sequence<=0;
    if(mem_fault)begin memory_error<=1;read_busy<=0;burst_ready<=0;end
   end
   if(command_valid)begin
    epoch<=epoch+1'b1;cache_valid<=0;play_state<=0;
    case(command)
     0,5:mode<=LIVE;
     1:begin
      mode<=PAUSE;
      if(mode==LIVE)begin read_offset<=0;end
     end
     2:mode<=valid_blocks>2 ? HISTORY : LIVE;
     3,4:begin
      mode<=valid_blocks>2 ? HISTORY : LIVE;
      
      if(mode==LIVE || rewind_lag>=valid_blocks)read_offset<=0;
     end
     6:begin record_epoch<=record_epoch+1'b1;mode<=STOP;recording<=0;history_full<=0;pack_index<=0;write_sequence<=0;end
    endcase
    if(command==0)begin memory_error<=0;overflow_error<=0;underflow_error<=0;overwritten<=0;end
    // The next tick publishes the acknowledged mode/epoch; old read returns
    // are drained by read_busy but cannot validate a new-generation cache.
   end
  end
 end
endmodule
