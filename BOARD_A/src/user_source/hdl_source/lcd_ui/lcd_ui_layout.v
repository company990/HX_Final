// Shared full-width media geometry; five navigation tabs live at the bottom.
module lcd_ui_layout #(parameter integer WIDTH=480,HEIGHT=800)(
 input wire [15:0] x,y,output wire in_panel,is_button,
 output wire [2:0] row,output wire [1:0] col,output reg [3:0] button_id,
 output wire [15:0] left,cell_width,output reg [15:0] top,output wire [4:0] label_id);
 localparam RH=HEIGHT/10,CW=WIDTH/3;
 assign in_panel=x<WIDTH && y>=RH && y<6*RH;
 assign is_button=in_panel;
 assign row=y<2*RH ? 0:y<3*RH ? 1:y<4*RH ? 2:y<5*RH ? 3:4;
 assign col=x<CW ? 0:x<2*CW ? 1:2;
 reg [4:0] label;
 always @*begin
  case({row,col})
   {3'd0,2'd0}:begin button_id=0;label=5;end
   {3'd0,2'd1}:begin button_id=1;label=6;end
   {3'd0,2'd2}:begin button_id=2;label=7;end
   {3'd1,2'd0}:begin button_id=3;label=8;end
   {3'd1,2'd1}:begin button_id=4;label=9;end
   {3'd1,2'd2}:begin button_id=5;label=10;end
   {3'd2,2'd0}:begin button_id=6;label=11;end
   {3'd2,2'd1}:begin button_id=8;label=12;end
   {3'd2,2'd2}:begin button_id=7;label=13;end
   {3'd3,2'd0}:begin button_id=9;label=14;end
   {3'd3,2'd1}:begin button_id=10;label=15;end
   {3'd3,2'd2}:begin button_id=11;label=16;end
   {3'd4,2'd0}:begin button_id=12;label=17;end
   {3'd4,2'd1}:begin button_id=13;label=18;end
   default:begin button_id=14;label=19;end
  endcase
  case(row)0:top=RH;1:top=2*RH;2:top=3*RH;3:top=4*RH;default:top=5*RH;endcase
 end
 assign left=col==0 ? 0:col==1 ? CW:2*CW;
 assign cell_width=col==2 ? WIDTH-2*CW:CW;
 assign label_id=label;
endmodule

// Native 24px body glyphs; same 480x800 touch regions and action IDs.
module lcd_ui_pixels #(parameter integer WIDTH=480,HEIGHT=800)(
 input clk,input [15:0] x,y,input pressed,input [3:0] pressed_id,
 input [2:0] beat_settings,input [4:0] effect_settings,
 input [2:0] transition_mode,output reg [15:0] pixel);
 wire panel,button;wire [2:0] row;wire [1:0] col;wire [3:0] id;wire [4:0] label_id;
 wire [15:0] left,top,cell_width;
 lcd_ui_layout #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) layout(x,y,panel,button,row,col,id,left,cell_width,top,label_id);
 wire nav=y>=704 && y<800 && x<480;
 wire [2:0] nav_col=x<96?0:x<192?1:x<288?2:x<384?3:4;
 wire [8:0] nav_left=nav_col==0?0:nav_col==1?96:nav_col==2?192:nav_col==3?288:384;
 wire [39:0] next_codes;wire [2:0] next_length;reg [39:0] codes;reg [2:0] length;
 lcd_ui_label labels(nav?{2'd0,nav_col}:label_id,next_codes,next_length);
 always @(posedge clk)begin codes<=next_codes;length<=next_length;end
 wire [7:0] label_width={length,4'd0}+{1'b0,length,3'd0};
 wire [8:0] text_left=nav?nav_left+24:left[8:0]+((9'd160-{1'b0,label_width})>>1);
 wire [9:0] text_top=nav?10'd740:top[9:0]+28;
 wire [8:0] dx=x[8:0]-text_left;wire [9:0] dy=y[9:0]-text_top;
 wire [8:0] bx=x[8:0]-left[8:0];wire [9:0] by=y[9:0]-top[9:0];
 reg [7:0] glyph;reg [8:0] char_start;
 always @*begin
  if(dx<24)begin glyph=codes[39:32];char_start=0;end
  else if(dx<48)begin glyph=codes[31:24];char_start=24;end
  else if(dx<72)begin glyph=codes[23:16];char_start=48;end
  else if(dx<96)begin glyph=codes[15:8];char_start=72;end
  else begin glyph=codes[7:0];char_start=96;end
 end
 wire compare_area=y>=480 && y<552 && x<480;
 wire transition_area=y>=552 && y<624 && x<480;
 wire [2:0] aux_col=x<96?0:x<192?1:x<288?2:x<384?3:4;
 wire [8:0] aux_left=aux_col==0?0:aux_col==1?96:aux_col==2?192:aux_col==3?288:384;
 wire [8:0] aux_x=x[8:0]-aux_left;
 wire frame_cell=transition_area && (aux_col==1 || aux_col==3);
 wire effects=y>=624 && y<704 && x<480;
 wire [1:0] effect_col=x<120?0:x<240?1:x<360?2:3;
 wire [8:0] effect_left=effect_col==0?0:effect_col==1?120:effect_col==2?240:360;
 wire [8:0] ex=x[8:0]-effect_left;
 wire effect_on=effect_col==0?effect_settings[0]:effect_col==2?effect_settings[3]:effect_col==3?effect_settings[4]:1'b1;
 reg [5:0] font_glyph;reg [8:0] font_x;reg [9:0] font_y;reg text_paint;
 always @*begin
  font_glyph=glyph[5:0];font_x=dx-char_start;font_y=dy;text_paint=(panel || nav) && dx<label_width && dy<24;
  if(compare_area)begin
   font_x=aux_x-(aux_x<48?24:48);font_y=y-504;text_paint=aux_x>=24 && aux_x<72 && font_y<24;
   case(aux_col)
    0:font_glyph=aux_x<48?6'd28:6'd29;
    1:font_glyph=aux_x<48?6'd41:6'd9;
    2:font_glyph=aux_x<48?6'd42:6'd63;
    3:font_glyph=aux_x<48?6'd43:6'd9;
    default:font_glyph=aux_x<48?6'd44:6'd52;
   endcase
  end
  if(transition_area)begin
   font_y=y-576;text_paint=font_y<24;
   if(frame_cell)begin
    font_x=aux_x-(aux_x<48?24:48);text_paint=text_paint && aux_x>=24 && aux_x<72;
    font_glyph=aux_x<48?(aux_col==1?6'd38:6'd61):6'd53;
   end else if(aux_col==2)begin
    text_paint=text_paint && aux_x>=12 && aux_x<84;
    if(aux_x<36)begin font_x=aux_x-12;font_glyph=55;end
    else if(aux_x<60)begin font_x=aux_x-36;font_glyph=57;end
    else begin font_x=aux_x-60;font_glyph=45+transition_mode;end
   end else begin font_x=aux_x-36;font_glyph=aux_col==0?40:60;text_paint=text_paint && aux_x>=36 && aux_x<60;end
  end
  if(effects)begin
   font_x=ex-(ex<60?36:60);font_y=y-652;text_paint=ex>=36 && ex<84 && font_y<24;
   case(effect_col)
    0:font_glyph=62;1:font_glyph=ex<60?54:26;
    2:font_glyph=ex<60?56:21;default:font_glyph=ex<60?58:59;
   endcase
  end
 end
 wire [23:0] font_bits;
 lcd_ui_font font(clk,{font_glyph,font_y[4:0]},font_bits);
 wire title_bit;wire [8:0] title_x=x[8:0]-112;wire [5:0] title_y=y[5:0]-16;
 lcd_home_title title(clk,{title_y[4:0],title_x[7:0]},title_bit);
 localparam [15:0] BG=16'hf7bf,
  NAV=16'hf7bf,
  CARD=16'hffff,
  CARD_TOP=16'hffff,
  LINE=16'hcedc,
  ACCENT=16'h23f1,
  TEXT=16'h2188,
  MUTED=16'h63b1,
  SELECTED=16'hdf7e,
  WARN=16'h9344;
 wire body=bx>=8 && bx<152 && by>=7 && by<73;
 wire cut=(bx<12 || bx>=148) && (by<11 || by>=69);
 wire card_outline=bx==8 || bx==151 || by==7 || by==72;
 wire aux_box=aux_x>=6 && aux_x<90;
 always @*begin
  pixel=BG;
  if(y<76)begin
   if(y>=16 && y<48 && x>=112 && x<368 && title_bit)pixel=TEXT;
   if(y>=74 && y<76 && x>=24 && x<456)pixel=ACCENT;
  end
  if(panel && body && !cut)begin
   pixel=(pressed && id==pressed_id)?SELECTED:CARD;
   if(card_outline)pixel=(pressed && id==pressed_id)?ACCENT:LINE;
  end
  if(compare_area || transition_area)begin
   if(aux_box && ((compare_area && y>=487 && y<545) || (transition_area && y>=559 && y<616)))begin
    pixel=frame_cell && (beat_settings[0]==(aux_col==1))?SELECTED:CARD;
    if(aux_x==6 || aux_x==89 || y==487 || y==544 || y==559 || y==615)pixel=LINE;
   end
  end
  if(effects && ex>=6 && ex<114 && y>=631 && y<696)begin
   pixel=effect_on?SELECTED:CARD;
   if(ex==6 || ex==113 || y==631 || y==695)pixel=effect_on?ACCENT:LINE;
   if(y>=685 && y<687 && ex>=44 && ex<(effect_col==1?48+{effect_settings[2:1],3'd0}:76))pixel=ACCENT;
  end
  if(nav)begin
   if(y==704)pixel=LINE;
   if(nav_col==0 && y>=709 && x>=4 && x<92)pixel=SELECTED;
  end
  if(text_paint && font_x<24 && font_bits[23-font_x])
   pixel=(effects && effect_on) || nav_col==0 && nav ? ACCENT:TEXT;
 end
endmodule

// 1-bit pre-rasterized branding: 8192 bits, no framebuffer or font decoder.
module lcd_home_title(input clk,input [12:0] addr,output pixel);
`ifdef FM_FONT_SIM
 localparam TITLE_FILE="fm_lcd_home_title.dat";
`else
 localparam TITLE_FILE="fm_lcd_home_title.mif";
`endif
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(1),.DATA_WIDTH_B(1),.ADDR_WIDTH_A(13),.ADDR_WIDTH_B(13),
  .DATA_DEPTH_A(8192),.DATA_DEPTH_B(8192),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE(TITLE_FILE),.FILL_ALL("NONE")) branding(
  .dia(1'b0),.addra(addr),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(pixel),
  .dib(1'b0),.addrb(13'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob());
endmodule
