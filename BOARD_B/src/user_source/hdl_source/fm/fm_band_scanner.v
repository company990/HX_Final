// Swept-channel RSSI measurement, not a wideband RF FFT.
// All frequency values use 100 kHz units. One owner of the existing tuner.
module fm_band_scanner #(
 parameter CLK_HZ=25000000, SETTLE_MS=80, SAMPLE_MS=20,
 parameter COMMAND_MS=1500, MAX_SPREAD=8, MIN_RSSI=28
)(input clk,rst,input command_valid,output command_ready,
 input [15:0] command_value,output reg command_done,output reg [3:0] command_error,
 input [79:0] chip_status,input chip_ready,input [3:0] chip_error,
 output reg chip_valid,output reg [2:0] chip_op,output reg [15:0] chip_value,
 output busy,output [127:0] status,
 input [7:0] read_index,output reg [15:0] read_data);
 localparam IDLE=0,CALC=1,CLEAR=2,MUTE=3,TUNE=4,SETTLE=5,
 READ=6,SAMPLE=7,STORE=8,NEXT=9,HIST=10,ANALYZE=11,
 LOAD=12,PEAK=13,MARK=14,RESTORE=15,UNMUTE=16,FINISH=17,
 ISSUE=18,WAIT_DONE=19,PAUSED=20,SELECT=21,SELECT_LOAD=22,SELECT_CHECK=23,
 REPLACE=24,ADVANCE=25,LISTEN_MUTE=26,LOAD_WAIT=27;
 localparam MS_DIV=CLK_HZ/1000, MW=MS_DIV<2?1:$clog2(MS_DIV);
 reg [MW-1:0] ms_count; wire ms_tick=ms_count==MS_DIV-1;
 reg [4:0] state,successor;
 reg [3:0] view_state,error;
 reg [7:0] generation,index,selected_index,station_count,clear_index;
 reg [10:0] start_freq,end_freq,frequency,saved_frequency,selected_frequency,calc_freq;
 reg [3:0] step;
 reg [8:0] total,completed,calc_total,valid_count;
 reg saved_mute,owned,done,stop_pending,pause_pending,listen_pending;
 reg [11:0] timer; reg [1:0] retry,sample_count;
 reg [6:0] rssi0,rssi1,rssi2,min_sample,max_sample,selected_rssi,noise,threshold;
 reg [1:0] ready_count,true_count,stereo_count;
 reg [3:0] selected_flags;
 reg [9:0] elapsed_ms;reg [5:0] elapsed_seconds;
 reg [3:0] bin; reg [8:0] cumulative;
 reg [15:0] points[0:255]; reg wr; reg [7:0] wr_index,internal_index;
 reg [15:0] wr_data,internal_data,previous_point,current_point;
 reg [7:0] last_peak; reg [6:0] last_peak_rssi; reg have_peak;
 reg [15:0] last_peak_word;
 reg [1:0] select_direction; reg [8:0] select_left;
 always @(posedge clk)begin
  if(wr)points[wr_index]<=wr_data;
  read_data<=points[read_index]; internal_data<=points[internal_index];
 end
 wire [6:0] median=(rssi0>rssi1) ? ((rssi1>rssi2)?rssi1:((rssi0>rssi2)?rssi2:rssi0)) :
                                                  ((rssi0>rssi2)?rssi0:((rssi1>rssi2)?rssi2:rssi1));
 wire stable=max_sample-min_sample<=MAX_SPREAD;
 wire nearby=step==1 ? (index-last_peak<=2) : step==2 ? (index-last_peak<=1) : 1'b0;
 wire [6:0] relaxed=threshold>=4?threshold-7'd4:0;
 // MARK and SELECT_CHECK never execute together. Share one tiny product
 // instead of keeping two separate shift/add networks for frequency labels.
 wire [7:0] frequency_index=state==SELECT_CHECK ? selected_index:index;
 wire [11:0] frequency_product=frequency_index*step;
 wire [10:0] index_offset=frequency_product[10:0];
 wire [10:0] selected_offset=frequency_product[10:0];
 wire histogram_clear=state==CLEAR && clear_index<16;
 wire histogram_increment=state==STORE && ready_count>=2 && stable;
 wire [3:0] histogram_address=histogram_clear?clear_index[3:0]:state==HIST?bin:median[6:3];
 wire [8:0] histogram_value;
 fm_scan_histogram histogram(.clk(clk),.we(!rst && (histogram_clear || histogram_increment)),
  .addr(histogram_address),.data(histogram_clear?9'd0:histogram_value+9'd1),.q(histogram_value));
 wire peak_candidate=current_point[7] && current_point[9] &&
  current_point[6:0]>=(current_point[11]?relaxed:threshold) &&
  (index==0 || current_point[6:0]>previous_point[6:0]) &&
  ({1'b0,index}+9'd1>=total || current_point[6:0]>=internal_data[6:0]);
 assign command_ready=1'b1;
 assign busy=owned;
 assign status={generation,view_state,owned,done,completed,total,index,station_count,error,
 start_freq,end_freq,step,selected_index,selected_frequency,selected_rssi,selected_flags,noise,threshold,elapsed_seconds};
 task launch;
  input [2:0] op;input [15:0] value;input [4:0] next_state;
  begin chip_op<=op;chip_value<=value;chip_valid<=1;successor<=next_state;timer<=COMMAND_MS;state<=ISSUE;end
 endtask
 task restore;
  begin chip_valid<=0;view_state<=5;state<=RESTORE;stop_pending<=0;pause_pending<=0;retry<=0;timer<=COMMAND_MS;end
 endtask
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   ms_count<=0;state<=CLEAR;successor<=IDLE;view_state<=0;error<=0;
   generation<=0;index<=0;selected_index<=0;station_count<=0;clear_index<=0;
   start_freq<=875;end_freq<=1080;step<=1;frequency<=875;calc_freq<=875;
   saved_frequency<=968;selected_frequency<=0;saved_mute<=0;owned<=0;done<=0;
   stop_pending<=0;pause_pending<=0;listen_pending<=0;total<=206;completed<=0;calc_total<=0;valid_count<=0;
   timer<=0;retry<=0;sample_count<=0;rssi0<=0;rssi1<=0;rssi2<=0;min_sample<=127;max_sample<=0;
   ready_count<=0;true_count<=0;stereo_count<=0;selected_rssi<=0;selected_flags<=0;noise<=0;threshold<=MIN_RSSI;
   bin<=0;cumulative<=0;wr<=0;wr_index<=0;wr_data<=0;internal_index<=0;elapsed_ms<=0;elapsed_seconds<=0;
   previous_point<=0;current_point<=0;last_peak<=0;last_peak_rssi<=0;last_peak_word<=0;have_peak<=0;
   select_direction<=0;select_left<=0;command_done<=0;command_error<=0;chip_valid<=0;chip_op<=0;chip_value<=0;
  end else begin
   command_done<=0;wr<=0;
   if(ms_tick)begin ms_count<=0;if(timer!=0)timer<=timer-1'b1;end else ms_count<=ms_count+1'b1;
   if(ms_tick && owned && view_state!=2 && elapsed_seconds!=63)begin
    if(elapsed_ms==999)begin elapsed_ms<=0;elapsed_seconds<=elapsed_seconds+1'b1;end else elapsed_ms<=elapsed_ms+1'b1;
   end
   case(state)
    IDLE:begin chip_valid<=0;end
    CALC:begin
     if(calc_total==256 || calc_freq>end_freq)begin
      if(calc_total==0 || (calc_total==256 && calc_freq<=end_freq))begin error<=6;restore;end
      else begin total<=calc_total;state<=CLEAR;clear_index<=0;end
     end else begin calc_freq<=calc_freq+step;calc_total<=calc_total+1'b1;end
    end
    CLEAR:begin
     wr<=1;wr_index<=clear_index;wr_data<=0;clear_index<=clear_index+1'b1;
     if(clear_index==255)begin
      if(owned)state<=MUTE;else begin state<=IDLE;view_state<=0;end
     end
    end
    MUTE:if(chip_ready)launch(2,1,TUNE);else if(timer==0)begin error<=10;restore;end
    TUNE:begin
     if(stop_pending)restore;
     else if(pause_pending)begin state<=PAUSED;view_state<=2;end
     else if(chip_ready)launch(0,{5'd0,frequency},SETTLE);
     else if(timer==0)begin error<=10;restore;end
    end
    SETTLE:begin
     if(timer==0)begin sample_count<=0;min_sample<=127;max_sample<=0;ready_count<=0;true_count<=0;stereo_count<=0;state<=READ;timer<=COMMAND_MS;end
    end
    READ:if(chip_ready)launch(7,0,SAMPLE);else if(timer==0)begin error<=10;restore;end
    SAMPLE:begin
     if(chip_status[15:9]<min_sample)min_sample<=chip_status[15:9];
     if(chip_status[15:9]>max_sample)max_sample<=chip_status[15:9];
     if(chip_status[7] && chip_status[25:16]+11'd870==frequency)ready_count<=ready_count+1'b1;
     if(chip_status[8])true_count<=true_count+1'b1;
     if(chip_status[26])stereo_count<=stereo_count+1'b1;
     case(sample_count)0:rssi0<=chip_status[15:9];1:rssi1<=chip_status[15:9];2:rssi2<=chip_status[15:9];endcase
     if(sample_count==2)state<=STORE;
     else begin sample_count<=sample_count+1'b1;timer<=SAMPLE_MS;state<=NEXT;end
    end
    NEXT:if(timer==0)begin state<=READ;timer<=COMMAND_MS;end
    STORE:begin
     wr<=1;wr_index<=index;wr_data<={4'd0,(true_count>=2),(stereo_count>=2),stable,1'b0,(ready_count>=2),median};
     completed<=completed+1'b1;
     if(ready_count>=2 && stable)valid_count<=valid_count+1'b1;
     retry<=0;
     if(stop_pending)restore;
     else if(completed+1'b1>=total)begin bin<=0;cumulative<=0;state<=HIST;view_state<=3;end
     else begin index<=index+1'b1;frequency<=frequency+step;state<=TUNE;timer<=COMMAND_MS;end
    end
    HIST:begin
     if(cumulative+histogram_value>=((valid_count+3)>>2) || bin==15)begin
      noise<={bin,3'd0};threshold<=bin>=14?127:(({bin,3'd0}+7'd12)<MIN_RSSI?MIN_RSSI:({bin,3'd0}+7'd12));
      index<=0;internal_index<=0;previous_point<=0;have_peak<=0;station_count<=0;state<=ANALYZE;
     end else begin cumulative<=cumulative+histogram_value;bin<=bin+1'b1;end
    end
    ANALYZE:begin internal_index<=index;state<=LOAD_WAIT;end
    LOAD_WAIT:state<=LOAD;
    LOAD:begin current_point<=internal_data;internal_index<=index+1'b1;state<=PEAK;end
    PEAK:begin state<=MARK;end // Wait for synchronous next-neighbour read.
    MARK:begin
     if(peak_candidate && (!have_peak || !nearby || current_point[6:0]>last_peak_rssi))begin
      if(have_peak && nearby)begin
       wr<=1;wr_index<=last_peak;wr_data<=last_peak_word & 16'hfeff;state<=REPLACE;
      end else begin
       wr<=1;wr_index<=index;wr_data<=current_point|16'h0100;station_count<=station_count+1'b1;state<=ADVANCE;
      end
      last_peak<=index;last_peak_rssi<=current_point[6:0];last_peak_word<=current_point|16'h0100;have_peak<=1;
      if(!have_peak || (nearby && selected_index==last_peak))begin selected_index<=index;selected_frequency<=start_freq+index_offset;selected_rssi<=current_point[6:0];selected_flags<={current_point[10:9],1'b1,current_point[7]};end
     end else state<=ADVANCE;
    end
    REPLACE:begin wr<=1;wr_index<=index;wr_data<=current_point|16'h0100;state<=ADVANCE;end
    ADVANCE:begin
     previous_point<=current_point;
     if({1'b0,index}+9'd1>=total)begin done<=1;restore;end
     else begin index<=index+1'b1;state<=ANALYZE;end
    end
    RESTORE:if(chip_ready)launch(0,{5'd0,listen_pending?selected_frequency:saved_frequency},UNMUTE);else if(timer==0)begin error<=10;state<=FINISH;end
    UNMUTE:if(chip_ready)launch(2,{15'd0,saved_mute},FINISH);else if(timer==0)begin error<=10;state<=FINISH;end
    LISTEN_MUTE:if(chip_ready)launch(2,1,RESTORE);else if(timer==0)begin error<=10;restore;end
    FINISH:begin owned<=0;state<=IDLE;view_state<=error!=0?7:listen_pending?6:done?4:0;listen_pending<=0;end
    ISSUE:begin
     if(chip_ready && chip_valid)begin chip_valid<=0;state<=WAIT_DONE;end
     else if(timer==0)begin error<=10;chip_valid<=0;restore;end
    end
    WAIT_DONE:begin
     if(chip_ready)begin
      if(chip_error==0)begin
       state<=successor;
       if(successor==SETTLE)timer<=SETTLE_MS;
      end else if(successor==UNMUTE || successor==FINISH)begin
       error<=chip_error;
       if(successor==UNMUTE)state<=UNMUTE;else state<=FINISH;
      end else if(retry<2 && !stop_pending)begin retry<=retry+1'b1;state<=TUNE;timer<=COMMAND_MS;end
      else begin error<=chip_error;restore;end
     end else if(timer==0)begin
      error<=10;
      // The underlying controller has its own finite transaction/tune timeout.
      // Do not issue restore until it releases ownership.
      state<=successor==FINISH?FINISH:RESTORE;
     end
    end
    PAUSED:if(stop_pending)restore;else if(!pause_pending)begin state<=TUNE;view_state<=1;timer<=COMMAND_MS;end
    SELECT:begin internal_index<=selected_index;state<=SELECT_LOAD;end
    SELECT_LOAD:state<=SELECT_CHECK;
    SELECT_CHECK:begin
     if(internal_data[8] || select_direction==0)begin selected_rssi<=internal_data[6:0];selected_flags<=internal_data[10:7];selected_frequency<=start_freq+selected_offset;state<=IDLE;end
     else if(select_left==0)begin command_error<=11;state<=IDLE;end
     else begin
      selected_index<=select_direction==1 ? (selected_index==0?total-1'b1:selected_index-1'b1) : ({1'b0,selected_index}+1'b1>=total?0:selected_index+1'b1);
      select_left<=select_left-1'b1;state<=SELECT;
     end
    end
    default:begin error<=10;restore;end
   endcase
   if(owned && stop_pending && state!=ISSUE && state!=WAIT_DONE && state!=RESTORE && state!=UNMUTE && state!=FINISH)restore;
   if(command_valid)begin
    command_done<=1;command_error<=0;
    case(command_value[14:12])
     0:case(command_value[11:0])
      0:if(!owned && state==IDLE && chip_status[78] && chip_status[32])begin
       saved_frequency<=chip_status[58:48];saved_mute<=chip_status[40];generation<=generation+1'b1;
       owned<=1;done<=0;error<=0;completed<=0;valid_count<=0;station_count<=0;selected_flags<=0;selected_frequency<=0;selected_rssi<=0;elapsed_ms<=0;elapsed_seconds<=0;
       index<=0;frequency<=start_freq;calc_freq<=start_freq;calc_total<=0;state<=CALC;view_state<=1;retry<=0;stop_pending<=0;pause_pending<=0;listen_pending<=0;timer<=COMMAND_MS;
      end else command_error<=9;
      1:if(owned)stop_pending<=1;
      2:if(owned && view_state==1)pause_pending<=1;else command_error<=9;
      3:if(state==PAUSED)begin pause_pending<=0;view_state<=1;end else command_error<=9;
      4:if(!owned && state==IDLE)begin generation<=generation+1'b1;done<=0;completed<=0;station_count<=0;selected_flags<=0;selected_frequency<=0;selected_rssi<=0;state<=CLEAR;clear_index<=0;end else command_error<=9;
      default:command_error<=6;
     endcase
     1,2:if(!owned && state==IDLE && station_count!=0)begin
      select_direction<=command_value[14:12]==1?1:2;select_left<=total;
      selected_index<=command_value[14:12]==1?(selected_index==0?total-1'b1:selected_index-1'b1):({1'b0,selected_index}+1'b1>=total?0:selected_index+1'b1);
      state<=SELECT;
     end else command_error<=9;
     3:if(!owned && state==IDLE && selected_flags[1])begin
      saved_mute<=chip_status[40];saved_frequency<=chip_status[58:48];owned<=1;listen_pending<=1;error<=0;view_state<=5;state<=LISTEN_MUTE;timer<=COMMAND_MS;
     end else command_error<=9;
     4:if(!owned && state==IDLE && command_value[11:0]>=870 && command_value[11:0]<=end_freq)start_freq<=command_value[10:0];else command_error<=6;
     5:if(!owned && state==IDLE && command_value[11:0]>=start_freq && command_value[11:0]<=1080)end_freq<=command_value[10:0];else command_error<=6;
     6:if(!owned && state==IDLE && command_value[11:0]>=1 && command_value[11:0]<=15)step<=command_value[3:0];else command_error<=6;
     7:if(!owned && state==IDLE && command_value[11:0]<total)begin selected_index<=command_value[7:0];select_direction<=0;select_left<=0;state<=SELECT;end else command_error<=6;
    endcase
   end
  end
 end
endmodule

// The sixteen-bin, nine-bit histogram retains all counts. The existing
// 256-cycle point-table CLEAR also clears these sixteen bins, so no RAM
// reset, extra scan state or timing change is required. Async LUT read is
// equivalent to the old register-array mux, including increment collisions.
module fm_scan_histogram(input clk,we,input [3:0] addr,input [8:0] data,
 output [8:0] q);
`ifdef PRIORITY12_SIM
 reg [8:0] mem[0:15];
 always @(posedge clk)if(we)mem[addr]<=data;
 assign q=mem[addr];
`else
 wire [11:0] bits;
 genvar nibble;
 generate for(nibble=0;nibble<3;nibble=nibble+1)begin:g_histogram
  wire [11:0] padded={3'd0,data};
  EG_LOGIC_DRAM16X4 storage(.di(padded[nibble*4+:4]),.waddr(addr),
   .we(we),.wclk(clk),.raddr(addr),.do(bits[nibble*4+:4]));
 end endgenerate
 assign q=bits[8:0];
`endif
endmodule
