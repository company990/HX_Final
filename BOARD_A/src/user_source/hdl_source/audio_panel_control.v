// SW4=1: KEY2 next / KEY4 previous. SW4=0: image owns both keys.
// Debounce all inputs; require release after every mode change.
module audio_panel_control #(
 parameter integer ENUMERATE_WAV = 0,
 parameter integer DEBOUNCE_CYCLES = 500000,
 parameter integer PAUSE_KEY_ENABLE = 0,
 parameter integer AUTO_NEXT_ENABLE = 0,
 parameter [87:0] NAME_0 = "MUSIC   WAV",
 parameter [87:0] NAME_1 = "TEST    WAV",
 parameter [87:0] NAME_2 = "TEST2   WAV",
 parameter [87:0] NAME_3 = "SONG4   WAV"
)(
 input clk, rst,
 input touch_valid,input [3:0] touch_id,
 input mode_audio, key_next_n, key_prev_n, key_pause_n,
 input play_finished, session_busy, external_play,
 input [87:0] current_name,
 input catalog_valid,
 input [15:0] catalog_count, catalog_index,
 output reg image_next_n, image_auto_n,
 output reg play_cmd, pause_cmd,
 output reg image_pause_n,
 output reg [87:0] target_name
);
 localparam integer CW = (DEBOUNCE_CYCLES < 2) ? 1 : $clog2(DEBOUNCE_CYCLES);
 reg [3:0] meta, sync, candidate;
 reg [CW-1:0] count;
 reg mode, initialized, ready;
 reg [15:0] slot, song_count;
 reg done_seen;
 wire [15:0] current_slot = ENUMERATE_WAV ? slot : !AUTO_NEXT_ENABLE ? slot :
     current_name==NAME_0 ? 16'd0 : current_name==NAME_1 ? 16'd1 :
     current_name==NAME_2 ? 16'd2 : 16'd3;
 wire [15:0] last_slot = song_count==0 ? 16'd0 : song_count-1'b1;
 wire [15:0] next_slot = ENUMERATE_WAV ?
     (current_slot>=last_slot ? 16'd0 : current_slot+1'b1) : {14'd0,(current_slot[1:0]+2'd1)};
 wire [15:0] prev_slot = ENUMERATE_WAV ?
     (current_slot==0 ? last_slot : current_slot-1'b1) : {14'd0,(current_slot[1:0]-2'd1)};
 wire [15:0] new_slot = candidate[1] ? prev_slot : next_slot;
 function [87:0] slot_name;
  input [15:0] s;
  begin
   if(ENUMERATE_WAV) slot_name={72'd0,s};
   else case(s[1:0]) 0:slot_name=NAME_0;1:slot_name=NAME_1;
    2:slot_name=NAME_2;3:slot_name=NAME_3;endcase
  end
 endfunction
 always @(posedge clk or posedge rst) begin
  if (rst) begin meta<=4'b0111; sync<=4'b0111; end
  else begin meta<={mode_audio,(PAUSE_KEY_ENABLE ? key_pause_n : 1'b1),key_next_n,key_prev_n}; sync<=meta; end
 end
 always @(posedge clk or posedge rst) begin
  if (rst) begin
   candidate<=4'b0111; count<=0; mode<=0; initialized<=0; ready<=0;
   slot<=0; song_count<=0; target_name<=ENUMERATE_WAV ? 88'd0 : NAME_0; play_cmd<=0; pause_cmd<=0;
   done_seen<=0;
   image_next_n<=1; image_auto_n<=1; image_pause_n<=1;
  end else begin
   play_cmd<=0; pause_cmd<=0;
   done_seen<=play_finished;
   if(ENUMERATE_WAV && catalog_valid) begin
    song_count<=catalog_count; slot<=catalog_index;
   end
   // Independent of SW4: image ownership does not stop the playlist.
   // Edge qualification prevents a sticky EOF from skipping several songs.
   if(AUTO_NEXT_ENABLE && play_finished && !done_seen && !session_busy && !external_play) begin
    slot<=next_slot;target_name<=slot_name(next_slot);play_cmd<=1;
   end
   if (sync[3] != mode) begin
    ready<=0; image_next_n<=1; image_auto_n<=1; image_pause_n<=1;
   end
   if (sync != candidate) begin candidate<=sync; count<=0; end
   else if (count < DEBOUNCE_CYCLES-1) count<=count+1'b1;
   else if (!initialized || candidate[3] != mode) begin
    initialized<=1; mode<=candidate[3]; ready<=0;
    image_next_n<=1; image_auto_n<=1; image_pause_n<=1;
   end else if (candidate[2:0] == 3'b111) begin
    ready<=1; image_next_n<=1; image_auto_n<=1; image_pause_n<=1;
   end else if (ready) begin
    if (mode) begin
     ready<=0;
     if (candidate[2:0] == 3'b011) begin pause_cmd<=1;play_cmd<=0;end
     if (candidate[2] && (candidate[1:0] == 2'b01 || candidate[1:0] == 2'b10)) begin
      slot<=new_slot; play_cmd<=1;
      target_name<=slot_name(new_slot);
     end
    end else begin
     image_next_n<=candidate[1]; image_auto_n<=candidate[0]; image_pause_n<=candidate[2];
    end
   end

   // Touch takes priority over automatic EOF / physical command this cycle.
   if(touch_valid && !session_busy) begin
    if(touch_id==1) begin pause_cmd<=1;play_cmd<=0;end
    if((touch_id==0 || touch_id==2) && song_count!=0) begin
     slot<=(touch_id==0)?prev_slot:next_slot;
     target_name<=slot_name((touch_id==0)?prev_slot:next_slot);play_cmd<=1;
    end
   end
  end
 end
endmodule
