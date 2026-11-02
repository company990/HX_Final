// LCD-only controls. Commit effect parameters at blanking frame boundaries.
module audio_visual_control(input clk,rst,frame_tick,input cmd_valid,input[3:0] cmd,
 input online,mode_mic,presentation_mode,input[23:0] features,input event_toggle,
 output reg enabled,output reg[1:0] strength,output reg clap_enabled,output reg demo,
 output reg[23:0] levels,output reg[4:0] pulse,output reg[4:0] ring,
 output reg clap_step);
 reg pending_event,last_event,primed,last_mode;reg[4:0] pending_settings;
 // The border is independent of the star enable. Commit only in blanking.
 reg border_pending,border_event;
 always @(posedge clk or posedge rst)begin
  if(rst)begin border_pending<=0;border_event<=0;ring<=0;end
  else begin
   if(cmd_valid && cmd==13)border_pending<=1;
   if(cmd_valid && cmd==14)border_pending<=0;
   if(!online || last_mode!=mode_mic)border_event<=0;
   else if(primed && event_toggle!=last_event)border_event<=1;
   if(frame_tick)begin
    if(!border_pending || !online || last_mode!=mode_mic)begin ring<=0;border_event<=0;end
    else begin
     if(ring!=0)ring<=ring-1'b1;
     if(border_event || (primed && event_toggle!=last_event))begin ring<=24;border_event<=0;end
    end
   end
  end
 end
 wire usable=online && enabled;
 // The analyzer floor is 8; B publishes max(mean-4,0), leaving 4 at silence.
 // Remove that remainder only for the star channel, before strength scaling.
 wire[5:0] star_treble=features[11:6]>4 ? features[11:6]-6'd4 : 6'd0;
 function[5:0] scaled;input[5:0] v;input[1:0] s;begin
  case(s)0:scaled=v>>2;1:scaled=v>>1;default:scaled=v;endcase
 end endfunction
 always @(posedge clk or posedge rst)begin
  if(rst)begin enabled<=1;strength<=1;clap_enabled<=0;demo<=0;pending_settings<=5'b00011;
   levels<=0;pulse<=0;clap_step<=0;pending_event<=0;last_event<=0;primed<=0;last_mode<=0;
  end else begin
   clap_step<=0;
   if(cmd_valid)case(cmd)
    7:pending_settings[0]<=~pending_settings[0];
    8:pending_settings[2:1]<=pending_settings[2:1]==2 ? 0:pending_settings[2:1]+1'b1;
    9:pending_settings[3]<=~pending_settings[3];
    10:pending_settings[4]<=~pending_settings[4];
    default:;
   endcase
   last_mode<=mode_mic;last_event<=event_toggle;
   if(!online || last_mode!=mode_mic)begin primed<=0;pending_event<=0;end
   else if(!primed)begin primed<=1;pending_event<=0;end
   else if(event_toggle!=last_event && usable)pending_event<=1;
   if(frame_tick)begin
    {demo,clap_enabled,strength,enabled}<=pending_settings;
    if(!usable || !pending_settings[0] || last_mode!=mode_mic)begin levels<=0;pulse<=0;pending_event<=0;end
    else begin
     levels<={scaled(features[23:18],strength),scaled(features[17:12],strength),
              scaled(star_treble,strength),scaled(features[5:0],strength)};
     if(pulse!=0)pulse<=pulse-1'b1;
     if(pending_event)begin
      pulse<=strength==0?8:strength==1?16:24;pending_event<=0;
      if(mode_mic && clap_enabled)clap_step<=1;
     end
    end
   end
  end
 end
endmodule

// Classify at the event source, then transfer the ordinary-image action to SD.
// Existing LCD commands take priority if both arrive in the same SD cycle.
module audio_visual_clap_route(input video_clk,sd_clk,rst,clap_step,presentation_mode,
 input touch_valid,input[3:0] touch_id,output image_valid,output[3:0] image_id);
 reg request,sync0,sync1,seen;
 always @(posedge video_clk or posedge rst)
  if(rst)request<=0;else if(clap_step && !presentation_mode)request<=~request;
 always @(posedge sd_clk or posedge rst)
  if(rst)begin sync0<=0;sync1<=0;seen<=0;end
  else begin sync0<=request;sync1<=sync0;seen<=sync1;end
 assign image_valid=touch_valid || (sync1!=seen);
 assign image_id=touch_valid ? touch_id : 4'd5;
endmodule

// Treble sparkles plus an independently gated, observed-beat white border.
// Output masks register once, matching OSD's BRAM/font pixel latency.
module audio_visual_mask(input clk,rst,de,input[9:0] x,y,input enabled,mode_mic,
 input[23:0] levels,input[4:0] pulse,ring,output reg paint,output reg[23:0] color,
 input frame_tick,output reg[4:0] opacity);
 wire[5:0] treble=levels[11:6];
 reg active,falling,half_rate;
 reg[4:0] alpha;
 reg[3:0] density;
 reg[7:0] seed;
 // One finite batch: eight frames in, thirty-two frames out. New seed only
 // when invisible, so stars never jump while visible or tear within a frame.
 always @(posedge clk or posedge rst)
  if(rst)begin active<=0;falling<=0;half_rate<=0;alpha<=0;density<=0;seed<=8'h5a;end
  else if(frame_tick)begin
   half_rate<=~half_rate;
   if(!enabled)begin active<=0;falling<=0;alpha<=0;density<=0;end
   else if(!active)begin
    alpha<=0;
    if(treble!=0)begin
     active<=1;falling<=0;
     density<=treble>=8 ? 4'd8 : {1'b0,treble[2:0]};
     seed<={seed[6:0],seed[7]^seed[5]^seed[4]^seed[3]};
    end
   end else if(!falling && treble!=0)begin
    alpha<=alpha+5'd2;
    if(alpha>=14)falling<=1;
   end else begin
    falling<=1;
    if(half_rate)begin
     if(alpha<=1)begin alpha<=0;active<=0;end
     else alpha<=alpha-1'b1;
    end
   end
  end
 // 13x13 four-point sparkle, instead of a single pixel. Compare the full
 // smoothed level: the old /8 threshold hid normal low-level high frequencies.
 wire[5:0] sx=x[5:0]-{1'b0,(seed[4:3]^y[7:6]),3'b000};
 wire[5:0] sy=y[5:0]-{1'b0,(seed[6:5]^x[7:6]),3'b000};
 wire star_shape=(sx==8 && sy>=2 && sy<=14) ||
                 (sy==8 && sx>=2 && sx<=14) ||
                 (sx>=7 && sx<=9 && sy>=6 && sy<=10) ||
                 (sy>=7 && sy<=9 && sx>=6 && sx<=10);
 wire dots=star_shape && active && alpha!=0 && y>=128 && y<416 &&
            ({1'b0,(x[8:6]^y[8:6]^seed[2:0])}<density);
 wire[8:0] xc=x[9:1];wire[7:0] yc=y[8:1];
 wire[3:0] inset=ring[4:1];
 wire[8:0] xl={5'd0,inset},xr={5'd19,~inset};
 wire[7:0] yt={4'd0,inset},yb={4'd14,~inset};
 wire border=ring!=0 && (((xc==xl || xc==xr) && yc>=yt && yc<=yb) ||
                        ((yc==yt || yc==yb) && xc>=xl && xc<=xr));
 reg detail_paint;
 // Register a compact color index instead of 24 independently selected bits.
 // The palette is decoded after the same one-clock pixel delay.
 reg[2:0] color_id;
 always @* paint=detail_paint;
 always @(posedge clk or posedge rst)
  if(rst)begin detail_paint<=0;color_id<=0;opacity<=0;end
  else begin
   detail_paint<=de && ((enabled && dots) || border);
   color_id<=border ? 3'd4:{1'b0,(x[7:6]^seed[1:0])};
   opacity<=border ? (ring>=16 ? 5'd16:ring):alpha;
  end
 always @* case(color_id)
  0:color=24'h00aaff;
  1:color=24'hff55aa;
  2:color=24'hffaa00;
  3:color=24'haa55ff;
  default:color=24'hffffff;
 endcase
endmodule

// Exact six-bit saturated tone lift, retaining each channel's low two bits.
// Replaces three read ports of an immutable arithmetic table, one-clock latency.
// Exact old saturated tone lift, now using independent 32K RAM resources.
// Two dual-port ROM copies supply three synchronous channel reads.
(* keep_hierarchy = "true" *)
module av_tone_rom(input clk,input [2:0] amount,input [23:0] rgb,output [23:0] result);
`ifdef FM_FONT_SIM
 localparam ROM_FILE="av_tone_32k.dat";
`else
 localparam ROM_FILE="av_tone_32k.mif";
`endif
 wire [9:0] ar={amount[2:1],rgb[23:16]},ag={amount[2:1],rgb[15:8]},ab={amount[2:1],rgb[7:0]};
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(8),.DATA_WIDTH_B(8),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
  .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),.INIT_FILE(ROM_FILE),.FILL_ALL("NONE")) rg(
  .dia(8'd0),.addra(ar),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(result[23:16]),
  .dib(8'd0),.addrb(ag),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(result[15:8]));
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(8),.DATA_WIDTH_B(8),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
  .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),.INIT_FILE(ROM_FILE),.FILL_ALL("NONE")) blue(
  .dia(8'd0),.addra(ab),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(result[7:0]),
  .dib(8'd0),.addrb(10'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob());
endmodule

