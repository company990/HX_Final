// FVP1: 160x120 RGB565 / 3200 PCM16 samples, 15fps.
// FVP2: 320x240 palette512 + indices76800 / 4800 PCM16 samples, 10fps.
// FVP3: 2400 PCM16 samples BEFORE palette512 + indices76800, 20fps.
// FVP4: FVP3 timing/palette/PCM, lossless literal/repeat PAL8 packets, flags=7.
// All have a strict 32-byte header and 48kHz mono PCM. Every output is a
// real ready/valid transfer. No frame or audio data are discarded on stalls.
module tf_video_demux #(parameter HQ_ENABLE=1, parameter FAST_ENABLE=0, parameter RLE_ENABLE=1)(
 input clk,rst,start,enable,input [7:0] byte_data,input byte_valid,stream_end,
 output wire byte_ready,input pcm_ready,output reg pcm_valid,
 output reg signed [15:0] pcm_sample,
 output reg [63:0] burst_data,output reg burst_valid,output reg burst_last,
 input burst_ready,output reg found,done,error,output reg [3:0] error_code,
 output wire [3:0] debug_state,output reg profile_high,output reg profile_fast
);
 localparam IDLE=0,HEADER=1,VIDEO=2,AUDIO=3,FINISH=4,FAILED=5,PALETTE=6;
 reg profile_rle;
 reg [2:0] state;reg [4:0] header_index;
 reg [15:0] frames_left;reg [16:0] count;
 reg [2:0] pack_index;reg [7:0] audio_low;
 reg [15:0] palette [0:255];reg [15:0] palette_color;
 reg [7:0] palette_low;reg palette_pending,palette_last_pixel;
 wire transfer=byte_valid && byte_ready;
 wire rle_byte_ready,rle_index_valid,rle_packet_last;
 wire [7:0] rle_index;
 wire palette_ready=enable && state==VIDEO && !burst_valid && !palette_pending;
 // Stop lookahead at the last decoded pixel: the following input byte may
 // already belong to the next frame's PCM, rather than another RLE token.
 wire rle_enabled=profile_rle && state==VIDEO && !(palette_pending && palette_last_pixel);
 tf_video_rle_decode rle(.clk(clk),.rst(rst),.clear(start || state!=VIDEO),.enable(rle_enabled),
  .byte_data(byte_data),.byte_valid(byte_valid && enable),.byte_ready(rle_byte_ready),
  .index_data(rle_index),.index_valid(rle_index_valid),.index_ready(palette_ready),.packet_last(rle_packet_last));
 wire pixel_transfer=profile_rle ? (rle_index_valid && palette_ready):transfer;
 wire [7:0] pixel_index=profile_rle ? rle_index:byte_data;
 always @(posedge clk)begin
  if(HQ_ENABLE && enable && state==PALETTE && transfer && count[0])
   palette[count[8:1]]<={byte_data,palette_low};
  if(HQ_ENABLE && enable && state==VIDEO && profile_high && pixel_transfer)
   palette_color<=palette[pixel_index];
 end
 wire header_variable=(header_index==3 || header_index==16 || header_index==17);
 reg [7:0] expected;
 always @*case(header_index)
  0:expected="F";1:expected="V";2:expected="P";3:expected="1";
  4:expected=profile_high?8'h40:8'ha0;5:expected=profile_high?1:0;
  6:expected=profile_high?240:120;8:expected=profile_high?(profile_fast?20:10):15;10:expected=1;
  12:expected=8'h80;13:expected=8'hbb;
  20:expected=8'h00;21:expected=profile_high?8'h2e:8'h96;22:expected=profile_high?1:0; // FVP1 38400 / FVP2 77312 bytes
  24:expected=profile_high?(profile_fast?8'h60:8'hc0):8'h80;25:expected=profile_high?(profile_fast?8'h09:8'h12):8'h0c;28:expected=profile_high?(profile_fast?(profile_rle?7:3):1):0; // FVP1 3200 / FVP2 4800 samples
  default:expected=0;
 endcase
 assign debug_state={1'b0,state};
 assign byte_ready=enable && ((state==HEADER) ||
       (state==VIDEO && (profile_rle ? rle_byte_ready:(!burst_valid && !palette_pending))) || (state==PALETTE) ||
       (state==AUDIO && (!count[0] || pcm_ready)));
 always @(posedge clk or posedge rst)begin
  if(rst)begin profile_rle<=0;profile_fast<=0;profile_high<=0;palette_low<=0;palette_pending<=0;palette_last_pixel<=0;state<=IDLE;header_index<=0;frames_left<=0;count<=0;
   pack_index<=0;audio_low<=0;burst_data<=0;burst_valid<=0;
   burst_last<=0;pcm_valid<=0;pcm_sample<=0;found<=0;done<=0;error<=0;error_code<=0;end
  else begin
   pcm_valid<=0;palette_pending<=0;
   if(burst_valid && burst_ready)burst_valid<=0;
   if(start && enable)begin profile_rle<=0;profile_fast<=0;profile_high<=0;palette_pending<=0;state<=HEADER;header_index<=0;frames_left<=0;count<=0;
    pack_index<=0;burst_valid<=0;found<=0;done<=0;error<=0;error_code<=0;end
   else if(enable)begin
    if(HQ_ENABLE && palette_pending && state!=FAILED)begin
     burst_data<={palette_color,burst_data[63:16]};
     if(pack_index==3)begin pack_index<=0;burst_valid<=1;burst_last<=palette_last_pixel;end
     else pack_index<=pack_index+1'b1;
     if(palette_last_pixel)begin
      count<=0;state<=AUDIO;
      if(profile_fast)begin
       frames_left<=frames_left-1'b1;
       if(frames_left==1)begin state<=FINISH;done<=1;end
      end
     end
    end
    if(state==VIDEO && profile_high && pixel_transfer)begin
     if(profile_rle && count==76799 && !rle_packet_last)begin
      state<=FAILED;error<=1;error_code<=4;palette_pending<=0;burst_valid<=0;
     end else begin
      palette_pending<=1;palette_last_pixel<=(count==76799);
      if(count!=76799)count<=count+1'b1;
     end
    end
    if(byte_valid && byte_ready)case(state)
     HEADER:begin
      if((!header_variable && byte_data!=expected) || (header_index==3 && byte_data!="1" && !(HQ_ENABLE && (byte_data=="2" || (FAST_ENABLE && (byte_data=="3" || (RLE_ENABLE && byte_data=="4")))))))begin state<=FAILED;error<=1;error_code<=1;end
      else begin
       if(header_index==3)begin
        profile_high<=HQ_ENABLE && (byte_data=="2" || byte_data=="3" || (RLE_ENABLE && byte_data=="4"));
        profile_fast<=HQ_ENABLE && FAST_ENABLE && (byte_data=="3" || (RLE_ENABLE && byte_data=="4"));
        profile_rle<=HQ_ENABLE && FAST_ENABLE && RLE_ENABLE && byte_data=="4";
       end
       if(header_index==16)frames_left[7:0]<=byte_data;
       if(header_index==17)frames_left[15:8]<=byte_data;
       if(header_index==31)begin
        if(frames_left==0)begin state<=FAILED;error<=1;error_code<=2;end
        else begin state<=profile_fast?AUDIO:profile_high?PALETTE:VIDEO;count<=0;pack_index<=0;found<=1;end
       end else header_index<=header_index+1'b1;
      end
     end
     PALETTE:begin
      if(!count[0])palette_low<=byte_data;
      if(count==511)begin count<=0;pack_index<=0;state<=VIDEO;end else count<=count+1'b1;
     end
     VIDEO:if(!profile_high)begin
      burst_data<={byte_data,burst_data[63:8]};pack_index<=pack_index+1'b1;
      if(pack_index==7)begin
       burst_valid<=1;
       burst_last<=(count==38399);
      end
      if(count==38399)begin count<=0;state<=AUDIO;end else count<=count+1'b1;
     end
     AUDIO:begin
      if(!count[0])audio_low<=byte_data;
      else begin pcm_sample<={byte_data,audio_low};pcm_valid<=1;end
      if(count==(profile_high?(profile_fast?4799:9599):6399))begin
       count<=0;pack_index<=0;
       if(profile_fast)state<=PALETTE;
       else begin frames_left<=frames_left-1'b1;
        if(frames_left==1)begin state<=FINISH;done<=1;end else state<=profile_high?PALETTE:VIDEO;
       end
      end else count<=count+1'b1;
     end
     default:;
    endcase
    // EOF can be held high while the last valid byte is backpressured.
    if(stream_end && !byte_valid && !palette_pending && !(profile_rle && state==VIDEO && rle_index_valid) && state!=FINISH && state!=IDLE && state!=FAILED)begin
     state<=FAILED;error<=1;error_code<=3;
    end
   end
  end
 end
endmodule

// At most one run descriptor is held. Literal bytes remain under the normal
// ready/valid contract; a repeat run uses an 8-bit value and an 8-bit count.
// No pixel/line/frame RAM, DSP, PLL or second SDRAM controller is added.
module tf_video_rle_decode(input clk,rst,clear,enable,input [7:0] byte_data,input byte_valid,
 output wire byte_ready,output wire [7:0] index_data,output wire index_valid,
 input index_ready,output wire packet_last);
 localparam TOKEN=0,VALUE=1,LITERAL=2,REPEAT=3;
 reg [1:0] state;reg [7:0] remaining,value;
 assign byte_ready=enable && (state==TOKEN || state==VALUE || (state==LITERAL && index_ready));
 assign index_valid=enable && (state==REPEAT || (state==LITERAL && byte_valid));
 assign index_data=state==REPEAT ? value:byte_data;
 assign packet_last=remaining==1;
 always @(posedge clk or posedge rst)begin
  if(rst)begin state<=TOKEN;remaining<=0;value<=0;end
  else if(clear)begin state<=TOKEN;remaining<=0;end
  else if(enable)case(state)
   TOKEN:if(byte_valid)begin remaining<={1'b0,byte_data[6:0]}+8'd1;state<=byte_data[7]?VALUE:LITERAL;end
   VALUE:if(byte_valid)begin value<=byte_data;state<=REPEAT;end
   LITERAL:if(byte_valid && index_ready)begin
    if(remaining==1)state<=TOKEN;else remaining<=remaining-1'b1;
   end
   REPEAT:if(index_ready)begin
    if(remaining==1)state<=TOKEN;else remaining<=remaining-1'b1;
   end
   default:state<=TOKEN;
  endcase
 end
endmodule
