// Tag complete stereo pairs so a source change/reset cannot publish an old
// in-flight result into the new source. No FIFO pop or sample-rate generation.
module audio_dsp_return_cache(input clk,rst,in_accept,in_available,flush,
 input [1:0] source,input source_live,out_valid,
 input signed [23:0] left,right,output reg [31:0] pcm,
 output valid,output pair_current);
 reg [1:0] epoch,accepted_epoch,cached_epoch,accepted_source,cached_source;
 reg accepted_available,cached_valid;
 assign pair_current=accepted_available && accepted_source==source && accepted_epoch==epoch && source_live && !flush;
 assign valid=cached_valid && cached_source==source && cached_epoch==epoch && source_live && !flush;
 function [15:0] round16;input signed [23:0] value;reg signed [24:0] rounded;
 begin rounded=$signed(value)+25'sd128;
 if(rounded>25'sd8388352)round16=16'h7fff;
 else if(rounded< -25'sd8388608)round16=16'h8000;
 else round16=rounded[23:8];end endfunction
 always @(posedge clk or posedge rst)
  if(rst)begin epoch<=0;accepted_epoch<=0;cached_epoch<=0;accepted_source<=0;cached_source<=0;
   accepted_available<=0;cached_valid<=0;pcm<=0;end
  else begin
   if(in_accept)begin accepted_source<=source;accepted_available<=in_available;accepted_epoch<=flush ? epoch+1'b1 : epoch;end
   if(flush)begin epoch<=epoch+1'b1;cached_valid<=0;pcm<=0;end
   else if(out_valid)begin
    pcm<={round16(left),round16(right)};cached_valid<=pair_current;
    cached_source<=accepted_source;cached_epoch<=accepted_epoch;
   end
  end
endmodule
