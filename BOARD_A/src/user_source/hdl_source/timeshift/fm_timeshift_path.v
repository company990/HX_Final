// Three clock domains. ADC has already been rate matched to A's frame clock.
// Ingress uses the existing Gray-pointer FIFO; playback never repeats stale data.
module fm_timeshift_path(input video_clk,mem_clk,lcd_clk,rst,
 input capture_event,input [15:0] raw_pcm,input raw_valid,packet_gap,
 input station_change,input sample_tick,input command_valid,input [3:0] command,
 output history_mode,output [1:0] play_epoch,output [15:0] history_pcm,output history_valid,
 output [23:0] lcd_status,
 output wr_req,output [15:0] wr_block,input [1:0] wr_word,input wr_active,
 output [31:0] wr_data,input wr_ack,output rd_req,output [15:0] rd_block,
 input rd_valid,input [1:0] rd_word,input [31:0] rd_data,input mem_fault);
 reg station_tag,overflow_latched;
 wire full,empty,read_valid;wire [17:0] capture_data;wire core_ready;
 reg pop_busy,capture_held;reg [17:0] held_packet;
 wire pop=!empty && !pop_busy && !capture_held && core_ready;
 // A registered FIFO read cannot be withdrawn when core_ready falls.
 // Retain that one in-flight packet until the memory-side core accepts it.
 wire capture_fire=core_ready && (capture_held || read_valid);
 wire [17:0] core_packet=capture_held ? held_packet : capture_data;
 wire overflow;
 always @(posedge video_clk or posedge rst)begin
  if(rst)begin station_tag<=0;overflow_latched<=0;end
  else begin
   if(station_change)station_tag<=~station_tag;
   if(overflow)overflow_latched<=1;
   if(command_valid && command==0)overflow_latched<=0;
  end
 end
 fm_timeshift_ingress_fifo #(.DATA_W(18),.DEPTH(512),.ADDR_W(9)) ingress(
  .clk_w(video_clk),.rst_w_n(!rst),.i_wr(capture_event),
  .i_data({(!raw_valid || packet_gap || station_change),station_tag,raw_pcm}),
  .o_wr_acked(),.o_full(full),.o_almost_full(),.o_overflow(overflow),
  .clk_r(mem_clk),.rst_r_n(!rst),.i_rd(pop),.o_data(capture_data),
  .o_rd_valid(read_valid),.o_empty(empty),.o_almost_empty(),.o_underflow());
 reg overflow_meta,overflow_sync;
 // The sticky video-domain alarm uses the same half-cycle CDC boundary
 // as the ingress Gray pointer. The second stage stays on rising mem_clk.
 always @(negedge mem_clk or posedge rst)
  if(rst)overflow_meta<=0;else overflow_meta<=overflow_latched;
 always @(posedge mem_clk or posedge rst)begin
  if(rst)begin pop_busy<=0;overflow_sync<=0;capture_held<=0;held_packet<=0;end
  else begin
   overflow_sync<=overflow_meta;
   if(pop)pop_busy<=1;
   if(read_valid)pop_busy<=0;
   if(read_valid && !core_ready)begin capture_held<=1;held_packet<=capture_data;end
   if(capture_held && core_ready)capture_held<=0;
  end
 end
 wire mem_tick,mem_command;wire unused_tick;wire [3:0] mem_id;
 board_link_mailbox #(.W(1)) ticks(video_clk,mem_clk,rst,sample_tick,1'b0,mem_tick,unused_tick);
 board_link_mailbox #(.W(4)) commands(video_clk,mem_clk,rst,command_valid,command,mem_command,mem_id);
 // Mailbox outputs arrive on falling mem_clk. Register them before the
 // multi-bit rewind arithmetic, giving that arithmetic a full 20ns cycle.
 reg core_command;reg [3:0] core_id;
 always @(posedge mem_clk or posedge rst)begin
  if(rst)begin core_command<=0;core_id<=0;end
  else begin core_command<=mem_command;if(mem_command)core_id<=mem_id;end
 end
 wire reply_event;wire [20:0] response;
 wire [23:0] mem_status;
 fm_timeshift_buffer buffer(.clk(mem_clk),.rst(rst),
  .capture_valid(capture_fire),.capture_packet(core_packet),.capture_ready(core_ready),
  .fifo_fault(overflow_sync),.tick(mem_tick),.command_valid(core_command),.command(core_id),
  .wr_req(wr_req),.wr_block(wr_block),.wr_word(wr_word),.wr_active(wr_active),
  .wr_data(wr_data),.wr_ack(wr_ack),.rd_req(rd_req),.rd_block(rd_block),
  .rd_valid(rd_valid),.rd_word(rd_word),.rd_data(rd_data),.mem_fault(mem_fault),
  .reply_valid(reply_event),.reply(response),.status(mem_status));
 wire received;wire [20:0] received_data;
 board_link_mailbox #(.W(21)) samples(mem_clk,video_clk,rst,reply_event,response,received,received_data);
 wire lcd_update;
 board_link_mailbox #(.W(24)) state_to_lcd(mem_clk,lcd_clk,rst,mem_tick,mem_status,lcd_update,lcd_status);
 reg [1:0] mode,epoch;reg [15:0] sample;reg fresh_sample,waiting_command;
 reg [1:0] command_epoch;
 assign history_mode=(mode!=1);
 assign play_epoch=epoch;
 assign history_pcm=sample;
 assign history_valid=(mode==3) && fresh_sample && !waiting_command;
 always @(posedge video_clk or posedge rst)begin
  if(rst)begin mode<=1;epoch<=0;sample<=0;fresh_sample<=0;waiting_command<=0;command_epoch<=0;end
  else begin
   if(sample_tick)fresh_sample<=0;
   if(received)begin
    if(!waiting_command || received_data[18:17]!=command_epoch)begin
     mode<=received_data[20:19];epoch<=received_data[18:17];
     sample<=received_data[15:0];fresh_sample<=received_data[16];
     waiting_command<=0;
    end
   end
   if(command_valid)begin waiting_command<=1;command_epoch<=epoch;fresh_sample<=0;end
   if(packet_gap || station_change)fresh_sample<=0;
  end
 end
endmodule
