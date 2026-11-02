// Inverse light sensor: high ADC code means dark environment.
// RGB gain only; does not command the monitor's physical backlight.
module ambient_brightness #(
 parameter integer PIR_MIN_GAIN=1, // 1/16 RGB gain; preserve visible content
 parameter integer AVG_LOG2=5,
 parameter integer CONFIRM_BLOCKS=3,
 parameter integer RAMP_FRAMES=2,
 parameter integer STALE_FRAMES=60,
 parameter integer BRIGHT_ENTER=621, // <0.50V => bright (VREF=3.3V)
 parameter integer BRIGHT_LEAVE=745, // >0.60V => room
 parameter integer DARK_ENTER=1985, // >1.60V => dark
 parameter integer DARK_LEAVE=1737  // <1.40V => room
)(input clk,rst,input [11:0] light_data,input light_toggle,
 input wire presence_run,
 input wire [6:0] gain_applied,
 output wire [6:0] gain_target,output wire gain_tick,
 input vs,de,input [23:0] rgb,
 output reg out_vs,out_de,output wire [23:0] out_rgb,
 output reg [11:0] filtered_adc,output reg [1:0] level,
 output wire [4:0] gain,output reg ready,output wire fault,
 output reg [11:0] raw_adc,output reg [7:0] sample_count,output reg [6:0] frame_count,
 output reg [7:0] rejected_count,output reg [7:0] average_count);
 reg t0,t1,seen,vs_d;
 reg [11+AVG_LOG2:0] sum;
 reg [AVG_LOG2-1:0] samples;
 reg [1:0] candidate;
 localparam CC=(CONFIRM_BLOCKS<2)?1:$clog2(CONFIRM_BLOCKS+1);
 localparam RC=(RAMP_FRAMES<2)?1:$clog2(RAMP_FRAMES);
 localparam WC=(STALE_FRAMES<2)?1:$clog2(STALE_FRAMES+1);
 reg [CC-1:0] confirm;
 reg [RC-1:0] ramp;
 reg [WC-1:0] stale,block_age;
 wire event_sample=(t1!=seen);
 wire plausible=(light_data>=128 && light_data<=3968);
 wire [11+AVG_LOG2:0] total=sum+light_data;
 wire [11:0] average=total>>AVG_LOG2;
 wire frame_start=vs && !vs_d;
 assign fault=(stale>=STALE_FRAMES);
 reg [1:0] proposed;
 always @* begin
  proposed=level;
  case(level)
   0: if(average>DARK_ENTER) proposed=2; else if(average>BRIGHT_LEAVE) proposed=1;
   1: if(average<BRIGHT_ENTER) proposed=0; else if(average>DARK_ENTER) proposed=2;
   2: if(average<BRIGHT_ENTER) proposed=0; else if(average<DARK_LEAVE) proposed=1;
   default: proposed=1;
  endcase
 end
 wire [4:0] ambient_target=(!ready) ? (fault?5'd12:5'd16) :
                       level==0 ? 5'd16 : level==2 ? 5'd6 : 5'd12;
 // Applied gain uses six fractional bits; debug gain retains its old /16 scale.
 assign gain_target=presence_run ? {ambient_target,2'b00} : (PIR_MIN_GAIN*4);
 assign gain_tick=frame_start && (ramp==RAMP_FRAMES-1);
 assign gain=gain_applied[6:2];
 // Bundled data stays stable for ~5.3ms; two falling-edge toggle synchronizers.
 always @(negedge clk or posedge rst) begin
  if(rst) begin t0<=0;t1<=0;end
  else begin t0<=light_toggle;t1<=t0;end
 end
 always @(posedge clk or posedge rst) begin
  if(rst) begin
   seen<=0;vs_d<=0;sum<=0;samples<=0;filtered_adc<=0;level<=1;
   candidate<=1;confirm<=0;ready<=0;ramp<=0;stale<=0;block_age<=0;
   raw_adc<=0;sample_count<=0;frame_count<=0;rejected_count<=0;average_count<=0;
  end else begin
   vs_d<=vs;
   if(frame_start) begin
    frame_count<=frame_count+1'b1;
    if(stale<STALE_FRAMES) stale<=stale+1'b1;
    if(block_age<STALE_FRAMES-1) block_age<=block_age+1'b1;
    else block_age<=0;
    if(ramp==RAMP_FRAMES-1) begin
     ramp<=0;
    end else ramp<=ramp+1'b1;
   end
   // Expire an unfinished block once per watchdog interval, not every
   // cycle while fault is high; otherwise recovery could never accumulate.
   if(frame_start && block_age==STALE_FRAMES-1) begin
    ready<=0;confirm<=0;sum<=0;samples<=0;
   end
   if(event_sample) begin
    seen<=t1;raw_adc<=light_data;sample_count<=sample_count+1'b1;
    if(plausible) begin
     if(frame_start && block_age==STALE_FRAMES-1) begin
      samples<=1;sum<=light_data;
     end else if(samples==((1<<AVG_LOG2)-1)) begin
      filtered_adc<=average;samples<=0;sum<=0;stale<=0;block_age<=0;average_count<=average_count+1'b1;
      if(proposed==level && ready) begin candidate<=proposed;confirm<=0;end
      else if(proposed!=candidate) begin candidate<=proposed;confirm<=1;end
      else if(confirm>=CONFIRM_BLOCKS-1) begin level<=proposed;ready<=1;confirm<=0;end
      else confirm<=confirm+1'b1;
     end else begin samples<=samples+1'b1;sum<=total;end
    end else begin
     // Reject only this sample. Isolated rail values must not restart all
     // 32 accepted samples or erase the level confirmation progress.
     rejected_count<=rejected_count+1'b1;
    end
   end
  end
 end
// BOARD_B: RGB executor moved to A; original preserved.
//  // Two aligned pixel stages keep added arithmetic off the spectrum path.
//  reg [23:0] rgb0;
//  reg vs0,de0;
//  reg [6:0] gain0;
//  reg [14:0] red,green,blue;
//  always @(posedge clk or posedge rst) begin
//   if(rst) begin
//    rgb0<=0;vs0<=0;de0<=0;gain0<=64;red<=0;green<=0;blue<=0;out_vs<=0;out_de<=0;
//   end else begin
//    rgb0<=rgb;vs0<=vs;de0<=de;gain0<=gain_applied;
//    red<=rgb0[23:16]*gain0;green<=rgb0[15:8]*gain0;blue<=rgb0[7:0]*gain0;
//    out_vs<=vs0;out_de<=de0;
//   end
//  end
//  assign out_rgb={red[13:6],green[13:6],blue[13:6]};
 always @* begin out_vs=0;out_de=0;end
 assign out_rgb=24'd0;
endmodule
