// AV25 header followed by video-first 50 ms records. Pixel/PCM bytes are LE.
module media25_demux(input wire clk,rst,input wire[7:0] data,input wire valid,
 output wire ready,input wire[31:0] file_size,
 output reg error,output reg active,
 output reg[95:0] pixels,output reg pixels_valid,input wire pixels_ready,
 output reg[2:0] pixels_flags,
 output reg[31:0] pcm,output reg pcm_valid,input wire pcm_ready,
 output wire ended,
 // Observation-only counters. They never influence ready/active/error.
 // Each is a free-running 32-bit counter in this module's clock domain.
 // [191:160] active cycles, [159:128] active && !valid cycles,
 // [127:96] audio-phase backpressure cycles, [95:64] video-phase backpressure cycles,
 // [63:32] error rising edges, [31:0] complete rising edges.
 input wire [3:0] slot_count,
 output wire [31:0] idle_ne,
 output wire [191:0] diag,
 output wire        diag_error,     // sticky: parser rejected the stream
 output wire        diag_complete); // sticky: last frame of the session reached

 reg[5:0] hi;reg[31:0] word,frames,frame;
 reg[19:0] pos;reg[3:0] byte_count;reg[95:0] pack;
 reg header_done,complete;
 wire audio=pos>=614400;
 assign ready=!error && !complete && (!header_done ||
   (audio?(!pcm_valid||pcm_ready):(!pixels_valid||pixels_ready)));
 assign ended=complete;
 wire[31:0] hw={word[23:0],data};
 always @(posedge clk or posedge rst)begin
  if(rst)begin hi<=0;word<=0;frames<=0;frame<=0;pos<=0;byte_count<=0;pack<=0;
   header_done<=0;complete<=0;error<=0;active<=0;pixels<=0;pixels_valid<=0;pixels_flags<=0;pcm<=0;pcm_valid<=0;end
  else begin
   if(pixels_valid&&pixels_ready)pixels_valid<=0;
   if(pcm_valid&&pcm_ready)pcm_valid<=0;
   if(valid&&ready)begin
    if(!header_done)begin
     word<=hw;hi<=hi+1'b1;
     case(hi)
      3:if(hw!=32'h41563235)error<=1;
      7:if(hw!=1)error<=1;
      11:if(hw!=640)error<=1;
      15:if(hw!=480)error<=1;
      19:if(hw!=20)error<=1;
      23:if(hw!=48000)error<=1;
      27:begin frames<=hw;if(hw==0||hw>6882)error<=1;end
      31:begin header_done<=1;active<=1;if(hw!=1)error<=1;end
      default:;
     endcase
    end else begin
     pack<={data,pack[95:8]};byte_count<=(!audio && byte_count==11)?0:((audio && byte_count==3)?0:byte_count+1'b1);
     if(!audio && byte_count==11)begin
      pixels<={data,pack[95:8]};pixels_valid<=1;
      pixels_flags<={frame==frames-1,pos==614399,pos==11};
     end
     if(audio && byte_count[1:0]==3)begin
      // Last four arriving LE bytes are Llo,Lhi,Rlo,Rhi.
      pcm<={pack[87:80],pack[79:72],data,pack[95:88]};pcm_valid<=1;
     end
     if(pos==623999)begin
      pos<=0;frame<=frame+1'b1;
      if(frame+1'b1==frames)complete<=1;
     end else pos<=pos+1'b1;
    end
   end
  end
 end
 // ---- observation-only counters (no effect on ready/active/error) ----
 // valid && !ready is counted per CLOCK CYCLE, never as received data.
 // The audio/video split reuses the same 'audio' term as the ready mux, so the
 // two backpressure counters are mutually exclusive by construction.
 reg [31:0] d_act,d_idle,d_bp_pcm,d_bp_pix,d_err,d_done;
 reg        err_d,done_d;
 always @(posedge clk or posedge rst)begin
  if(rst)begin d_act<=0;d_idle<=0;d_bp_pcm<=0;d_bp_pix<=0;d_err<=0;d_done<=0;err_d<=0;done_d<=0;end
  else begin
   if(active)              d_act    <= d_act  + 1'b1;
   if(active && !valid)    d_idle   <= d_idle + 1'b1;
   if(valid && !ready && audio  && pcm_valid    && !pcm_ready   ) d_bp_pcm <= d_bp_pcm + 1'b1;
   if(valid && !ready && !audio && pixels_valid && !pixels_ready) d_bp_pix <= d_bp_pix + 1'b1;
   err_d<=error;    if(error    && !err_d ) d_err  <= d_err  + 1'b1;
   done_d<=complete;if(complete && !done_d) d_done <= d_done + 1'b1;
  end
 end
 assign diag={d_act,d_idle,d_bp_pcm,d_bp_pix,d_err,d_done};
 // 25 MHz: idle cycles while a committed packet slot is still occupied.
 reg [31:0] d_idle_ne;
 always @(posedge clk or posedge rst)begin
  if(rst)d_idle_ne<=0;
  else if(active && !valid && slot_count!=4'd0)d_idle_ne<=d_idle_ne+1'b1;
 end
 assign idle_ne=d_idle_ne;
 assign diag_error=error;
 assign diag_complete=complete;
endmodule
