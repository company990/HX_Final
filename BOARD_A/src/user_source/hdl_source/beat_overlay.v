// Independent paint layer, not connected to the shared HDMI compositor.
// 3-digit BPM; confidence and phase bars; observed (cyan), predicted (gold).
module beat_overlay # (parameter X=16,Y=360)(
 input wire clk,rst,frame_tick,enable,
 input wire [10:0] x,y,input wire de,
 input wire [7:0] bpm,confidence,phase,input wire [2:0] state,
 input wire observed_event,predicted_event,
 output reg paint,output reg [23:0] rgb
);
 reg [7:0] bpm_frame,conf_frame,phase_frame;
 reg [2:0] state_frame; reg [3:0] observed_hold,predicted_hold;
 reg observed_pending,predicted_pending;
 reg [3:0] hundreds,tens,ones;
 integer b;
 always @(posedge clk) begin
  if(rst) begin
   bpm_frame<=0; conf_frame<=0; phase_frame<=0; state_frame<=0;
   observed_hold<=0; predicted_hold<=0; observed_pending<=0; predicted_pending<=0;
  end else begin
   if(observed_event) observed_pending<=1;
   if(predicted_event) predicted_pending<=1;
   if(frame_tick) begin
    bpm_frame<=bpm; conf_frame<=confidence; phase_frame<=phase; state_frame<=state;
    observed_pending<=observed_event; predicted_pending<=predicted_event;
    if(observed_pending || observed_event) observed_hold<=6; else if(observed_hold!=0) observed_hold<=observed_hold-1'b1;
    if(predicted_pending || predicted_event) predicted_hold<=6; else if(predicted_hold!=0) predicted_hold<=predicted_hold-1'b1;
   end
  end
 end
 always @* begin
  // Bounded 8-bit binary to BCD, no divider.
  hundreds=0; tens=0; ones=0;
  for(b=7;b>=0;b=b-1) begin
   if(hundreds>=5) hundreds=hundreds+3;
   if(tens>=5) tens=tens+3;
   if(ones>=5) ones=ones+3;
   {hundreds,tens,ones}={hundreds[2:0],tens,ones,bpm_frame[b]};
  end
 end
 function [6:0] segments;
  input [3:0] digit;
  begin case(digit)
   0:segments=7'b1111110;1:segments=7'b0110000;2:segments=7'b1101101;
   3:segments=7'b1111001;4:segments=7'b0110011;5:segments=7'b1011011;
   6:segments=7'b1011111;7:segments=7'b1110000;8:segments=7'b1111111;
   9:segments=7'b1111011;default:segments=0;
  endcase end
 endfunction
 function [34:0] glyph;
  input [7:0] c;
  begin case(c)
   "B":glyph=35'b11110_10001_10001_11110_10001_10001_11110;
   "P":glyph=35'b11110_10001_10001_11110_10000_10000_10000;
   "M":glyph=35'b10001_11011_10101_10101_10001_10001_10001;
   "S":glyph=35'b01111_10000_10000_01110_00001_00001_11110;
   "E":glyph=35'b11111_10000_10000_11110_10000_10000_11111;
   "K":glyph=35'b10001_10010_10100_11000_10100_10010_10001;
   "Y":glyph=35'b10001_10001_01010_00100_00100_00100_00100;
   "N":glyph=35'b10001_11001_10101_10011_10001_10001_10001;
   "C":glyph=35'b01111_10000_10000_10000_10000_10000_01111;
   "L":glyph=35'b10000_10000_10000_10000_10000_10000_11111;
   "O":glyph=35'b01110_10001_10001_10001_10001_10001_01110;
   "T":glyph=35'b11111_00100_00100_00100_00100_00100_00100;
   "R":glyph=35'b11110_10001_10001_11110_10100_10010_10001;
   "V":glyph=35'b10001_10001_10001_10001_10001_01010_00100;
   default:glyph=0;
  endcase end
 endfunction
 wire inside=de && enable && x>=X && x<X+256 && y>=Y && y<Y+76;
 wire [10:0] dx=x-X,dy=y-Y;
 reg [6:0] seg; reg [5:0] digit_x; reg stroke;
 reg [31:0] status_word;reg [7:0] letter;reg [5:0] tx,ty;reg text_pixel;
 reg [34:0] letter_bits;
 always @* begin
  case(state_frame)
   0:status_word="SEEK";1:status_word="SYNC";2:status_word="LOCK";3:status_word="LOST";default:status_word="RECV";
  endcase
  letter=0;tx=0;ty=0;text_pixel=0;
  if(dx>=80 && dx<116 && dy<14)begin
   if(dx<92)begin letter="B";tx=dx-80;end
   else if(dx<104)begin letter="P";tx=dx-92;end
   else begin letter="M";tx=dx-104;end
   ty=dy;
  end
  if(dx>=168 && dx<216 && dy>=20 && dy<34)begin
   ty=dy-20;
   if(dx<180)begin letter=status_word[31:24];tx=dx-168;end
   else if(dx<192)begin letter=status_word[23:16];tx=dx-180;end
   else if(dx<204)begin letter=status_word[15:8];tx=dx-192;end
   else begin letter=status_word[7:0];tx=dx-204;end
  end
  letter_bits=glyph(letter);
  if(tx<10 && ty<14) text_pixel=letter_bits[34-((ty>>1)*5+(tx>>1))];
  if(dx<24) begin seg=segments(hundreds); digit_x=dx[5:0]; end
  else if(dx<48) begin seg=segments(tens); digit_x=dx-24; end
  else begin seg=segments(ones); digit_x=dx-48; end
  if(state_frame!=2) seg=7'b0000001; // Do not present unlocked BPM as reliable.
  stroke=(seg[6] && dy<3 && digit_x>=3 && digit_x<18) ||
   (seg[5] && digit_x>=18 && digit_x<21 && dy>=3 && dy<16) ||
   (seg[4] && digit_x>=18 && digit_x<21 && dy>=19 && dy<32) ||
   (seg[3] && dy>=32 && dy<35 && digit_x>=3 && digit_x<18) ||
   (seg[2] && digit_x<3 && dy>=19 && dy<32) ||
   (seg[1] && digit_x<3 && dy>=3 && dy<16) ||
   (seg[0] && dy>=16 && dy<19 && digit_x>=3 && digit_x<18);
  paint=0; rgb=0;
  if(inside) begin
   if(dx<72 && dy<35 && stroke) begin paint=1; rgb=24'h8de7ff; end
   if(text_pixel)begin paint=1;rgb=24'h8de7ff;end
   if(dy>=43 && dy<49 && dx<conf_frame) begin paint=1; rgb=24'h37c896; end
   if(dy>=56 && dy<62 && dx<phase_frame) begin paint=1; rgb=24'heba64c; end
   // State markers: search / acquiring / locked / lost / recovering, left to right.
   if(dy>=68 && dy<73 && dx<100 && dx[3:0]<12 && (dx>>4)==state_frame) begin paint=1; rgb=state_frame==2 ? 24'h46ff90:24'he8a45b; end
   if(dx>=128 && dx<140 && dy<14 && observed_hold!=0) begin paint=1; rgb=24'h40dcff; end
   if(dx>=148 && dx<160 && dy<14 && predicted_hold!=0) begin paint=1; rgb=24'hffd040; end
  end
 end
endmodule
