// Stereo 48 kHz engine. One shared 32x18 multiplier and serial divider.
// Samples: signed 24-bit. IIR states: signed 32-bit, sample << 4 (16x headroom).
// Coefficients: signed Q2.16. Five DF-I biquads per channel, linked dynamics.
module audio_dsp_engine(input clk,rst,in_valid,flush,
 input signed [23:0] in_left,in_right,output in_ready,
 input enabled,agc_enabled,compress_enabled,limiter_enabled,gate_enabled,
 input [19:0] gains,input [1:0] threshold_select,ratio_select,speed_select,target_select,
 output reg out_valid,output reg signed [23:0] out_left,out_right,
 output [63:0] status,output reg [9:0] max_cycles);
 localparam INIT=0,IDLE=1,CLEAR=2,COEFF_REQ=3,COEFF_APPLY=4,COEFF_SAFE=5,
 FETCH=6,TAP_SETUP=7,TAP_ACC=8,WRITE=9,ENVELOPE=10,AGC_DONE=11,
 LEVEL_SET=12,LEVEL_DONE=13,COMP_DONE=14,GAIN_SET=15,GAIN_DONE=16,
 GATE_SET=17,GATE_DONE=18,PRED_SET=19,PRED_DONE=20,LIMIT_DONE=21,
 WET_L_SET=22,WET_L_DONE=23,WET_R_SET=24,WET_R_DONE=25,
 MIX_L_D_SET=26,MIX_L_D_DONE=27,MIX_L_W_SET=28,MIX_L_W_DONE=29,
 MIX_R_D_SET=30,MIX_R_D_DONE=31,MIX_R_W_SET=32,MIX_R_W_DONE=33,
 FINISH=34,DIVIDE=35,MAC_WAIT=36;
 localparam signed [23:0] CEILING=24'sd7476355;
 reg [5:0] state,successor;
 reg initialized,pending_flush,overrun,clipped;
 reg [5:0] clear_index;reg [2:0] band,tap;reg channel;reg [2:0] fetch_index;reg [1:0] write_index;
 reg [19:0] frame_gains;reg fa,fc,fl,fg;reg [1:0] ft,fr,fs,fp;
 reg signed [23:0] dry_l,dry_r,wet_l,wet_r;
 reg signed [31:0] eq_l,eq_r,filtered;
 reg signed [27:0] dc_l,dc_r;
 // Keep a 32-bit subtraction before the divide-by-256 shift: opposite
 // extrema need 29 signed bits even though the stored mean fits 28.
 wire signed [31:0] dc_input_l={{4{in_left[23]}},in_left,4'b0};
 wire signed [31:0] dc_input_r={{4{in_right[23]}},in_right,4'b0};
 wire signed [31:0] dc_step_l=(dc_input_l-$signed(dc_l))>>>8;
 wire signed [31:0] dc_step_r=(dc_input_r-$signed(dc_r))>>>8;
 reg signed [31:0] delay0,delay2;
 reg signed [17:0] coefficients[0:4];
 reg [17:0] active_coefficients[0:31] /* fehdl force_ram=1, ram_style="bram" */;
 reg signed [17:0] active_q;

 wire signed [31:0] delay_q;
 reg coeff_wr,delay_wr;reg [4:0] coeff_waddr;reg signed [17:0] coeff_wdata;
 reg [5:0] delay_waddr;reg signed [31:0] delay_wdata;
 wire [4:0] coeff_addr=({2'd0,band}<<2)+{2'd0,band}+{2'd0,tap};
 // Prefetch the following tap's coefficient during the two MAC clocks.
 // FETCH primes tap zero; stability feedback still uses registers 3/4.
 wire [4:0] coeff_base=({2'd0,band}<<2)+{2'd0,band};
 wire [4:0] coeff_read_addr=state==FETCH ? coeff_base :
  (state==TAP_SETUP || state==TAP_ACC) ? coeff_addr+5'd1 : coeff_addr;
 wire [5:0] delay_addr={band,channel,fetch_index[1:0]};
 reg [3:0] gain_index;
 always @*case(band)
  0:gain_index=frame_gains[3:0];1:gain_index=frame_gains[7:4];
  2:gain_index=frame_gains[11:8];3:gain_index=frame_gains[15:12];
  4:gain_index=frame_gains[19:16];default:gain_index=0;
 endcase
 wire signed [17:0] target_q;
 audio_dsp_coeff_rom rom(clk,{band,gain_index,tap},target_q);
 always @(posedge clk)begin
  if(coeff_wr)active_coefficients[coeff_waddr]<=coeff_wdata;
  active_q<=active_coefficients[coeff_read_addr];

 end
 // Each history word uses both ports of one native 16-bit dual-port RAM.
 // A writes/reads its low half at an even address; B its high half at odd.
 // WRITE does not consume q; FETCH primes both read ports before use.
 wire history_write=!rst && (state==INIT || state==CLEAR || state==WRITE);
 wire [5:0] history_waddr=state==WRITE?{band,channel,write_index}:clear_index;
 reg [31:0] history_wdata;
 always @*begin
  history_wdata=0;
  if(state==WRITE)case(write_index)
   0:history_wdata=channel?eq_r:eq_l;1:history_wdata=delay0;2:history_wdata=filtered;3:history_wdata=delay2;
  endcase
 end
 // Commit the current WRITE beat directly, one edge before the former
 // registered write-enable. No history read is consumed during WRITE, and
 // both ports are free for the very first FETCH after the fourth write.
 audio_dsp_delay_ram delay_storage(clk,history_write,history_waddr,history_wdata,delay_addr,delay_q);
 reg signed [31:0] mul_a;reg signed [17:0] mul_b;
 wire signed [49:0] product=mul_a*mul_b;
 reg signed [55:0] accumulator;
 wire signed [55:0] sum=accumulator+product;
 wire signed [55:0] rounded=(sum+56'sd32768)>>>16;
 wire signed [31:0] sample=channel?eq_r:eq_l;
 reg [39:0] div_quot;reg [31:0] div_den;reg [32:0] div_rem;
 reg [5:0] div_count;
 wire [32:0] shifted_rem={div_rem[31:0],div_quot[39]};
 wire [33:0] div_difference={1'b0,shifted_rem}-{2'b0,div_den};
 wire subtract=!div_difference[33];
 wire [39:0] quotient_next={div_quot[38:0],subtract};
 reg [31:0] envelope;reg [8:0] noise;reg [15:0] agc_gain,comp_gain,gate_gain,final_gain,lim_gain;
 reg gate_open,compress_active,limit_active;
 reg [8:0] mix_gain;reg [7:0] raw_peak,output_peak;
 reg [31:0] level_units;reg [9:0] cycle_count;
 function [31:0] magnitude;input signed [31:0] x;begin magnitude=x[31]?(~x)+1'b1:x;end endfunction
 function [7:0] meter;input [31:0] x;begin meter=|x[31:27]?8'd255:x[26:19];end endfunction
 wire [31:0] peak_l=magnitude(eq_l),peak_r=magnitude(eq_r);
 wire [31:0] peak_max=peak_l>peak_r?peak_l:peak_r;
 wire [31:0] peak=peak_max[31]?32'h7fffffff:peak_max;
 wire [31:0] decay=(envelope>>10)+1'b1;
 wire [31:0] decayed=envelope>decay?envelope-decay:0;
 wire [31:0] envelope_next=peak>decayed?peak:decayed;
 wire [31:0] units=envelope_next>>12;
 wire [15:0] noise_threshold=noise>=256?16'd512:noise<<1;
 wire [15:0] gate_threshold=noise_threshold>128?noise_threshold:16'd128;
 wire [15:0] threshold=ft==0?16'd16384:ft==1?16'd8192:16'd4096;
 wire [15:0] agc_target=fp==0?16'd8192:fp==1?16'd4096:16'd2048;
 wire [31:0] comp_level=level_units>threshold?
  {16'd0,threshold}+((level_units-{16'd0,threshold})>>(fr==0?1:fr==1?2:3)):level_units;
 function signed [31:0] sat32;input signed [55:0] x;
 begin if(x>56'sd2147483647)sat32=32'sh7fffffff;
 else if(x< -56'sd2147483648)sat32=32'sh80000000;else sat32=x[31:0];end endfunction
 function signed [23:0] sat24;input signed [55:0] x;
 begin if(x>56'sd8388607)sat24=24'sh7fffff;
 else if(x< -56'sd8388608)sat24=24'sh800000;else sat24=x[23:0];end endfunction
 function signed [23:0] wet_sample;input signed [49:0] p;input limit_on;
 reg signed [55:0] x;begin x=$signed(p)>>>16;
 if(limit_on && x>CEILING)wet_sample=CEILING;
 else if(limit_on && x< -CEILING)wet_sample=-CEILING;else wet_sample=sat24(x);end endfunction
 function signed [17:0] interpolate;input signed [17:0] a,b;
 reg signed [18:0] difference;reg [18:0] distance,step_size,correction;
 begin difference=$signed(b)-$signed(a);
 distance=difference[18]?-difference:difference;step_size=distance>>7;
 if(step_size==0 && distance!=0)step_size=1;
 correction=(step_size^{19{difference[18]}})+difference[18];
 interpolate=$signed(a)+$signed(correction);
 end endfunction
 wire signed [17:0] candidate=interpolate(active_q,target_q);
 wire signed [17:0] feedback4=candidate>18'sd65532 ? 18'sd65532 : candidate< -18'sd65532 ? -18'sd65532 : candidate;
 wire signed [19:0] feedback_bound=20'sd65536-$signed(feedback4)-20'sd4;
 wire signed [17:0] feedback3=$signed(coefficients[3])>feedback_bound?feedback_bound[17:0]:
  $signed(coefficients[3])< -feedback_bound?(-feedback_bound):coefficients[3];
 function [15:0] slew;input [15:0] current,target;input [1:0] speed;
 reg [16:0] difference,distance,shifted,step_size,correction;reg falling;
 begin
 difference={1'b0,target}-{1'b0,current};falling=difference[16];
 distance=falling?-difference:difference;
 shifted=falling?distance>>4:distance>>8;
 case(speed)0:step_size=shifted;1:step_size=shifted>>2;default:step_size=shifted>>4;endcase
 if(step_size==0 && distance!=0)step_size=1;
 correction=(step_size ^ {17{falling}})+falling;
 slew={1'b0,current}+correction;
 end endfunction
 wire [15:0] requested_agc=!fa?16'd4096:div_quot>16384?16'd16384:div_quot<1024?16'd1024:div_quot[15:0];
 wire [15:0] requested_comp=!fc?16'd16384:div_quot>16384?16'd16384:div_quot[15:0];
 wire [15:0] requested_limit=fl && div_quot<final_gain?div_quot[15:0]:final_gain;
 // These three updates occupy distinct FSM states; one combinational slew
 // datapath preserves the exact update values and the original cycle count.
 wire [15:0] slew_current=state==AGC_DONE ? agc_gain:state==GAIN_SET ? comp_gain:lim_gain;
 wire [15:0] slew_target=state==AGC_DONE ? requested_agc:state==GAIN_SET ? requested_comp:requested_limit;
 wire [15:0] slewed_gain=slew(slew_current,slew_target,fs);

 // Raw/post metering occurs in distinct states; share the magnitude circuit.
 wire signed [23:0] meter_l=state==FINISH?out_left:dry_l;
 wire signed [23:0] meter_r=state==FINISH?out_right:dry_r;
 wire [23:0] meter_abs_l=meter_l[23]?-meter_l:meter_l;
 wire [23:0] meter_abs_r=meter_r[23]?-meter_r:meter_r;
 wire [23:0] meter_max=meter_abs_l>meter_abs_r?meter_abs_l:meter_abs_r;
 wire [7:0] shared_meter=meter_max[23]?8'd255:meter_max[22:15];
 assign in_ready=state==IDLE;
 assign status={initialized,state!=IDLE,overrun,clipped,gate_open,compress_active,limit_active,
  (enabled?mix_gain!=256:mix_gain!=0),raw_peak,output_peak,agc_gain,final_gain,
  (gate_gain>=16320?8'd255:gate_gain[13:6])};
 task multiply;input signed [31:0] a;input signed [17:0] b;input [5:0] next_state;
 begin mul_a<=a;mul_b<=b;successor<=next_state;state<=MAC_WAIT;end endtask
 task divide;input [39:0] numerator;input [31:0] denominator;input [5:0] next_state;
 begin div_quot<=numerator;div_den<=denominator==0?32'd1:denominator;div_rem<=0;div_count<=39;successor<=next_state;state<=DIVIDE;end endtask
 integer i;
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   state<=INIT;successor<=IDLE;initialized<=0;pending_flush<=0;overrun<=0;clipped<=0;clear_index<=0;
   band<=0;tap<=0;channel<=0;fetch_index<=0;write_index<=0;frame_gains<=20'h44444;
   fa<=0;fc<=0;fl<=1;fg<=0;ft<=1;fr<=1;fs<=1;fp<=1;
   dry_l<=0;dry_r<=0;wet_l<=0;wet_r<=0;eq_l<=0;eq_r<=0;dc_l<=0;dc_r<=0;filtered<=0;
   delay0<=0;delay2<=0;coeff_wr<=0;delay_wr<=0;coeff_waddr<=0;coeff_wdata<=0;delay_waddr<=0;delay_wdata<=0;
   for(i=0;i<5;i=i+1)coefficients[i]<=0;
   mul_a<=0;mul_b<=0;accumulator<=0;div_quot<=0;div_den<=1;div_rem<=0;div_count<=0;
   envelope<=0;noise<=0;agc_gain<=4096;comp_gain<=16384;gate_gain<=16384;final_gain<=4096;lim_gain<=4096;
   gate_open<=0;compress_active<=0;limit_active<=0;mix_gain<=0;raw_peak<=0;output_peak<=0;level_units<=0;
   cycle_count<=0;max_cycles<=0;out_valid<=0;out_left<=0;out_right<=0;
  end else begin
   out_valid<=0;coeff_wr<=0;delay_wr<=0;
   if(flush)pending_flush<=1;
   if(in_valid && !in_ready && initialized)overrun<=1;
   if(state!=IDLE && state!=INIT)cycle_count<=cycle_count+1'b1;
   case(state)
    INIT:begin
     delay_wr<=1;delay_waddr<=clear_index;delay_wdata<=0;
     if(clear_index<25)begin coeff_wr<=1;coeff_waddr<=clear_index[4:0];coeff_wdata<=clear_index%5==0?18'sd65536:18'sd0;end
     clear_index<=clear_index+1'b1;
     if(clear_index==63)begin initialized<=1;state<=IDLE;end
    end
    IDLE:if(in_valid)begin
     dry_l<=in_left;dry_r<=in_right;
     eq_l<=gate_enabled?($signed(in_left)<<<4)-dc_l:($signed(in_left)<<<4);
     eq_r<=gate_enabled?($signed(in_right)<<<4)-dc_r:($signed(in_right)<<<4);
     dc_l<=dc_l+dc_step_l;
     dc_r<=dc_r+dc_step_r;
     frame_gains<=gains;fa<=agc_enabled;fc<=compress_enabled;fl<=limiter_enabled;fg<=gate_enabled;
     ft<=threshold_select;fr<=ratio_select;fs<=speed_select;fp<=target_select;
     if(enabled && mix_gain<256)mix_gain<=mix_gain+1'b1;
     else if(!enabled && mix_gain!=0)mix_gain<=mix_gain-1'b1;
     band<=0;tap<=0;channel<=0;cycle_count<=0;
     if(pending_flush || flush)begin
      pending_flush<=0;clear_index<=0;state<=CLEAR;envelope<=0;noise<=0;clipped<=0;overrun<=0;dc_l<=0;dc_r<=0;
      // The first new-source sample must not subtract the old DC estimate.
      eq_l<=($signed(in_left)<<<4);eq_r<=($signed(in_right)<<<4);
     end else state<=COEFF_REQ;
    end
    CLEAR:begin delay_wr<=1;delay_waddr<=clear_index;delay_wdata<=0;clear_index<=clear_index+1'b1;
     if(clear_index==63)state<=COEFF_REQ;end
    COEFF_REQ:state<=COEFF_APPLY;
    COEFF_APPLY:begin
     coeff_wr<=1;coeff_waddr<=coeff_addr;coeff_wdata<=tap==4?feedback4:candidate;
     coefficients[tap]<=tap==4?feedback4:candidate;
     if(tap==4)begin coefficients[3]<=feedback3;state<=COEFF_SAFE;end
     else begin tap<=tap+1'b1;state<=COEFF_REQ;end
    end
    COEFF_SAFE:begin coeff_wr<=1;coeff_waddr<=({2'd0,band}<<2)+{2'd0,band}+5'd3;coeff_wdata<=coefficients[3];
     fetch_index<=0;channel<=0;state<=FETCH;end
    FETCH:begin
     case(fetch_index)1:delay0<=delay_q;3:delay2<=delay_q;endcase
     if(fetch_index==4)begin tap<=0;accumulator<=0;state<=TAP_SETUP;end else fetch_index<=fetch_index+1'b1;
    end
    TAP_SETUP:begin
     // x2/y2 come directly from the synchronous state RAM. x1/y1
     // remain latched for the later state-history shift writes.
     case(tap)0:mul_a<=sample;1:mul_a<=delay0;2:mul_a<=delay_q;3:mul_a<=delay2;4:mul_a<=delay_q;endcase
     if(tap==0)fetch_index<=1;
     mul_b<=active_q;state<=TAP_ACC;
    end
    TAP_ACC:begin accumulator<=sum;
     if(tap==1)fetch_index<=3;
     if(tap==4)begin filtered<=sat32(rounded);if(rounded>56'sd2147483647 || rounded< -56'sd2147483648)clipped<=1;write_index<=0;state<=WRITE;end
     else begin tap<=tap+1'b1;state<=TAP_SETUP;end
    end
    WRITE:begin
     delay_wr<=1;delay_waddr<={band,channel,write_index};
     case(write_index)0:delay_wdata<=sample;1:delay_wdata<=delay0;2:delay_wdata<=filtered;3:delay_wdata<=delay2;endcase
     if(write_index==3)begin
      if(channel)eq_r<=filtered;else eq_l<=filtered;
      if(!channel)begin channel<=1;fetch_index<=0;state<=FETCH;end
      else if(band==4)state<=ENVELOPE;
      else begin band<=band+1'b1;tap<=0;state<=COEFF_REQ;end
     end else write_index<=write_index+1'b1;
    end
    ENVELOPE:begin
     envelope<=envelope_next;
     raw_peak<=shared_meter;
     if(!gate_open && units<512)begin
      // units<512 here. From reset, noise stays in [0,511], so every
      // original distance>>10 is exactly zero. Keep the same +/-1 update.
      if(units[8:0]>noise)noise<=noise+1'b1;
      else if(units[8:0]<noise)noise<=noise-1'b1;
     end
     if(!fg)begin gate_open<=1;gate_gain<=16384;end
     else begin
      if(units>gate_threshold+(gate_threshold>>1))gate_open<=1;
      else if(units<gate_threshold)gate_open<=0;
      if(gate_open)gate_gain<=gate_gain>=15872?16'd16384:gate_gain+16'd512;
      else gate_gain<=gate_gain<=128?16'd0:gate_gain-16'd128;
     end
     divide({12'd0,agc_target,12'd0},units,AGC_DONE);
    end
    DIVIDE:begin div_rem<=subtract?div_difference[32:0]:shifted_rem;div_quot<=quotient_next;
     if(div_count==0)state<=successor;else div_count<=div_count-1'b1;end
    AGC_DONE:begin agc_gain<=slewed_gain;state<=LEVEL_SET;end
    LEVEL_SET:multiply($signed(envelope),$signed({2'd0,agc_gain}),LEVEL_DONE);
    MAC_WAIT:state<=successor;
    LEVEL_DONE:begin level_units<=product>>>24;state<=COMP_DONE;end
    COMP_DONE:divide({comp_level[25:0],14'd0},level_units,GAIN_SET);
    GAIN_SET:begin comp_gain<=slewed_gain;state<=GAIN_DONE;end
    GAIN_DONE:multiply($signed({16'd0,agc_gain}),$signed({2'd0,comp_gain}),GATE_SET);
    GATE_SET:begin final_gain<=product>>>14;state<=GATE_DONE;end
    GATE_DONE:multiply($signed({16'd0,final_gain}),$signed({2'd0,gate_gain}),PRED_SET);
    PRED_SET:begin final_gain<=product>>>14;state<=PRED_DONE;end
    PRED_DONE:begin
     compress_active<=fc && comp_gain<16300;
     divide({CEILING,4'd0,12'd0},peak,LIMIT_DONE);
    end
    LIMIT_DONE:begin
     if(requested_limit<lim_gain)lim_gain<=requested_limit;else lim_gain<=slewed_gain;
     limit_active<=fl && requested_limit<final_gain;state<=WET_L_SET;
    end
    WET_L_SET:multiply(eq_l,$signed({2'd0,lim_gain}),WET_L_DONE);
    WET_L_DONE:begin wet_l<=wet_sample(product,fl);if(!fl && ($signed(product)>>>16>8388607 || $signed(product)>>>16< -8388608))clipped<=1;state<=WET_R_SET;end
    WET_R_SET:multiply(eq_r,$signed({2'd0,lim_gain}),WET_R_DONE);
    WET_R_DONE:begin wet_r<=wet_sample(product,fl);if(!fl && ($signed(product)>>>16>8388607 || $signed(product)>>>16< -8388608))clipped<=1;state<=MIX_L_D_SET;end
    MIX_L_D_SET:multiply($signed(dry_l),$signed(18'd256-{9'd0,mix_gain}),MIX_L_D_DONE);
    MIX_L_D_DONE:begin accumulator<=product;state<=MIX_L_W_SET;end
    MIX_L_W_SET:multiply($signed(wet_l),$signed({9'd0,mix_gain}),MIX_L_W_DONE);
    // dry/wet are saturated signed24 and mix_gain is in [0,256].
    // Their convex combination cannot exceed signed24; no extra clamp needed.
    MIX_L_W_DONE:begin out_left<=sum[31:8];state<=MIX_R_D_SET;end
    MIX_R_D_SET:multiply($signed(dry_r),$signed(18'd256-{9'd0,mix_gain}),MIX_R_D_DONE);
    MIX_R_D_DONE:begin accumulator<=product;state<=MIX_R_W_SET;end
    MIX_R_W_SET:multiply($signed(wet_r),$signed({9'd0,mix_gain}),MIX_R_W_DONE);
    MIX_R_W_DONE:begin out_right<=sum[31:8];state<=FINISH;end
    FINISH:begin
     out_valid<=1;state<=IDLE;
     output_peak<=shared_meter;
     if(cycle_count>max_cycles)max_cycles<=cycle_count;
    end
    default:begin overrun<=1;state<=IDLE;end
   endcase
  end
 end
endmodule

// Shared history address/control with split 16-bit halves. EG4 maps two 9K blocks.
// Preserve synchronous 32-bit reads and commit writes during the WRITE state.
module audio_dsp_delay_ram(input clk,we,input [5:0] wa,input [31:0] data,
 input [5:0] ra,output [31:0] q);
 wire [5:0] address=we?wa:ra;
`ifdef PRIORITY12_SIM
 reg [15:0] mem[0:127];reg [31:0] read_q;
 always @(posedge clk)begin
  if(we)begin mem[{wa,1'b0}]<=data[15:0];mem[{wa,1'b1}]<=data[31:16];end
  read_q<={mem[{address,1'b1}],mem[{address,1'b0}]};
 end
 assign q=read_q;
`else
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(7),.ADDR_WIDTH_B(7),
  .DATA_DEPTH_A(128),.DATA_DEPTH_B(128),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),
  .INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
  .dia(data[15:0]),.addra({address,1'b0}),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(we),.rsta(1'b0),.bea(1'b0),.doa(q[15:0]),
  .dib(data[31:16]),.addrb({address,1'b1}),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(we),.rstb(1'b0),.beb(1'b0),.dob(q[31:16]));
`endif
endmodule
