// LCD events become numbered requests, held until execution feedback returns.
// No optimistic frequency/volume updates: the page shows B's confirmed state.
module fm_console_a #(parameter SCAN_ENABLE=0,parameter DSP_ENABLE=0)(input clk,rst,event_valid,input [3:0] event_id,
 input legacy_valid,input [3:0] legacy_id,
 input online,status_valid,input [127:0] status,
 output reg request_valid,output reg [7:0] request_seq,
 output reg [2:0] request_op,output reg [15:0] request_value,
 output [47:0] ui_status,output reg local_error,input scan_event_valid,input [3:0] scan_event_id,input dsp_event_valid,input [3:0] dsp_event_id);
 wire [15:0] frequency=status[63:48];
 wire frequency_in_band=!(|frequency[15:11]) && frequency[10:0]>=11'd870 && frequency[10:0]<=11'd1080;
 wire [10:0] tune_frequency=frequency_in_band ? frequency[10:0] : 11'd968;
 wire [3:0] volume=status[47:44];
 wire volume_down=event_id==8;
 wire[3:0] volume_sense=volume^{4{volume_down}};
 wire[3:0] volume_toggle={(&volume_sense[2:0]),(&volume_sense[1:0]),volume_sense[0],1'b1};
 wire[3:0] requested_volume=volume ^ (volume_toggle & {4{!( &volume_sense)}});
 wire mute=status[40],powered=status[101],monitor=status[102];
 wire [1:0] source=status[106:105],analysis=status[104:103];
 reg status_seen;
 reg [2:0] op;
 reg [15:0] value;
 wire dsp_event=DSP_ENABLE && dsp_event_valid;
 wire scan_event=SCAN_ENABLE && scan_event_valid;
 wire tune_down=scan_event ? scan_event_id==8 : event_id==4;
 wire[10:0] tune_sum;
 assign tune_sum[0]=~tune_frequency[0];
 genvar tune_bit;
 generate for(tune_bit=1;tune_bit<11;tune_bit=tune_bit+1)begin:frequency_prefix
  assign tune_sum[tune_bit]=tune_frequency[tune_bit] ^
   (&(tune_frequency[tune_bit-1:0] ^ {tune_bit{tune_down}}));
 end endgenerate
 wire [10:0] requested_frequency=(tune_down && tune_frequency==870) ? 11'd1080 :
   (!tune_down && tune_frequency==1080) ? 11'd870 : tune_sum;
 reg [2:0] scan_op;reg [15:0] scan_value;
 always @*begin
 scan_op=7;scan_value=16'h8000;
 case(scan_event_id)
 0,14:scan_value=16'h8000;1:scan_value=16'h8001;2:scan_value=16'h8002;3:scan_value=16'h8003;
 4:scan_value=16'h9000;5:scan_value=16'ha000;6:scan_value=16'hb000;7:scan_value=16'h8004;
 8:begin scan_op=0;scan_value=requested_frequency;end
 9:begin scan_op=0;scan_value=requested_frequency;end
 10:begin scan_op=2;scan_value=!mute;end
 11:scan_value={11'd0,monitor,2'd2,2'd2};
 12:scan_value={11'd0,monitor,2'd0,2'd0};13:scan_value={11'd0,monitor,2'd3,2'd3};
 endcase end
 wire [1:0] match_source=source==1 && !monitor ? 0 : source;
 always @*begin
  op=7;value={11'd0,monitor,analysis,source};
  case(event_id)
   0:begin op=6;value=!powered;end
   1:value={11'd0,monitor,2'd2,2'd2};
   2:value={11'd0,monitor,2'd0,2'd0};
   3:value={11'd0,monitor,2'd3,2'd3};
   4:begin op=0;value=requested_frequency;end
   5:begin op=0;value=requested_frequency;end
   6:begin op=5;value=0;end
   7:begin op=4;value=0;end
   8:begin op=1;value=requested_volume;end
   9:begin op=1;value=requested_volume;end
   10:begin op=2;value=!mute;end
   11:begin op=3;value=0;end
   12:value={11'd0,monitor,2'd1,2'd1};
   13:value={11'd0,!monitor,2'd1,2'd1};
   14:value={11'd0,monitor,(analysis+2'd1),source};
   15:value={11'd0,monitor,match_source,source};
  endcase
 end
 // 48 bits: reserved8, freq11,RSSI7,volume4,error4,source2,FFT2,
 // monitor,power,mute,stereo,init,chip_busy,ADCfault,clip,online,pending.
 wire [3:0] visible_error=local_error ? 9 : status[110:107]!=0 ? status[110:107] : status[75:72];
 assign ui_status={status[8],status[7],status[33],status[95:93],status[99],status[34],frequency[10:0],status[15:9],volume,visible_error,
                   source,analysis,monitor,powered,mute,status[26],status[78],
                   status[79],status[100],status[99],online,request_valid};
 always @(posedge clk or posedge rst)begin
  if(rst)begin status_seen<=0;request_valid<=0;request_seq<=0;request_op<=0;request_value<=0;local_error<=0;end
  else begin
   if(status_valid && !status_seen)begin status_seen<=1;request_seq<=status[127:120];end
   if(status_valid && status[127:120]==request_seq && status[119:112]==request_seq && !status[111])request_valid<=0;
   if(dsp_event || scan_event || event_valid || (legacy_valid && (legacy_id==12 || legacy_id==14)))begin
    if(!request_valid && online && status_seen)begin
     request_seq<=request_seq+1'b1;request_valid<=1;local_error<=0;
     request_op<=dsp_event ? 7 : scan_event?scan_op:event_valid ? op : 7;
     // Original media buttons keep MIC as analysis-only unless MON enabled.
     request_value<=dsp_event ? {12'h400,dsp_event_id} : scan_event?scan_value:event_valid ? value : legacy_id==14 ? {11'd0,monitor,2'd1,2'd1} : {11'd0,monitor,2'd0,2'd0};
    end else local_error<=1;
   end
  end
 end
endmodule

// Brief ramp around routing/validity changes; no second audio processing chain.
module audio_source_guard #(parameter KEY_W=6)(input clk,rst,tick,input [KEY_W-1:0] key,
 input signed [23:0] left,right,output signed [23:0] out_left,out_right);
 reg [KEY_W-1:0] previous_key;
 reg [8:0] gain;
 reg [1:0] state;
 reg signed [23:0] held_left,held_right,last_left,last_right;
 wire signed [23:0] l=state==1 ? held_left : key!=previous_key ? last_left : left;
 wire signed [23:0] r=state==1 ? held_right : key!=previous_key ? last_right : right;
 wire signed [33:0] scaled_l=l*$signed({1'b0,gain});
 wire signed [33:0] scaled_r=r*$signed({1'b0,gain});
 assign out_left=scaled_l>>>8;
 assign out_right=scaled_r>>>8;
 always @(posedge clk or posedge rst)begin
  if(rst)begin previous_key<=0;gain<=0;state<=2;held_left<=0;held_right<=0;last_left<=0;last_right<=0;end
  else if(tick)begin
   last_left<=left;last_right<=right;
   if(key!=previous_key)begin
    previous_key<=key;
    // Hold the last waveform sample through ramp-down; it may disappear
    // immediately on link failure or when B changes its physical source.
    held_left<=state==1 ? held_left : last_left;
    held_right<=state==1 ? held_right : last_right;
    state<=1;
   end else if(state==1)begin
    if(gain<=2)begin gain<=0;state<=2;end else gain<=gain-2;
   end else if(state==2)begin
    if(gain>=254)begin gain<=256;state<=0;end else gain<=gain+2;
   end
  end
 end
endmodule
