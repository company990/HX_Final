// Shared portrait card theme. Pure RGB565 constants and narrow comparisons;
// font ports, caption depth and pixel latency stay unchanged.
module fm_lcd_pixels #(parameter SCAN_ENABLE=0,SHARED_FONT=0,VIDEO_ENABLE=0,DSP_ENABLE=0,TIME_ENABLE=0,PIPELINE_FONT=0)(input clk,input rst_n,input [8:0] x,input [9:0] y,
 input [47:0] status,input pressed,input [3:0] pressed_id,output reg [15:0] pixel,input scan_page,input scan_graph,input scan_point_valid,input [31:0] scan_point,input [127:0] scan_status,output [10:0] shared_font_addr,input [15:0] shared_font_bits,input video_page,input [15:0] video_status,input dsp_page,input [127:0] dsp_status,input time_page,input [23:0] time_status);
 wire time_view=TIME_ENABLE && time_page;reg [23:0] time_snapshot;
 always @(posedge clk or negedge rst_n)if(!rst_n)time_snapshot<=0;else if(x==0 && y==0)time_snapshot<=time_status;
 wire dsp_view=DSP_ENABLE && dsp_page;
 wire scanning=SCAN_ENABLE && scan_page && !dsp_view;
 wire graph_view=scanning && scan_graph;
 reg [127:0] dsp_snapshot;
 wire [20:0] agc_percent_product={5'd0,dsp_snapshot[39:24]}*21'd25;
 reg [3:0] selected_gain;
 always @*case(dsp_snapshot[117:115])
 0:selected_gain=dsp_snapshot[98:95];1:selected_gain=dsp_snapshot[102:99];2:selected_gain=dsp_snapshot[106:103];
 3:selected_gain=dsp_snapshot[110:107];default:selected_gain=dsp_snapshot[114:111];endcase
 wire [3:0] gain_db=selected_gain>=4 ? (selected_gain-4)*3 : (4-selected_gain)*3;
 reg [127:0] scan_snapshot;
 reg [11:0] progress_bcd,total_bcd,station_bcd;
 reg [47:0] snapshot;
 reg [15:0] freq_bcd,start_bcd,end_bcd;
 reg [11:0] rssi_bcd;
 reg [15:0] number_work,number_adjust;
 reg [10:0] number_shift;
 reg [3:0] number_count;
 reg [2:0] number_field;
 reg number_frame_seen;
 integer m;
 always @*begin
  number_adjust=number_work;
  for(m=0;m<4;m=m+1)
   if(number_work[m*4+:4]>=5)number_adjust[m*4+:4]=number_work[m*4+:4]+3;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   dsp_snapshot<=0;scan_snapshot<=0;snapshot<=0;
   freq_bcd<=0;start_bcd<=0;end_bcd<=0;rssi_bcd<=0;progress_bcd<=0;total_bcd<=0;station_bcd<=0;
   number_work<=0;number_shift<=0;number_count<=0;number_field<=7;number_frame_seen<=0;
  end else begin
   if(x!=0 || y!=0)number_frame_seen<=0;
   if(x==0 && y==0 && !number_frame_seen)begin
    dsp_snapshot<=dsp_status;scan_snapshot<=scan_status;snapshot<=status;
    number_field<=0;number_count<=0;number_frame_seen<=1;
   end else if(number_field<7)begin
    if(number_count==0)begin
     number_work<=0;number_count<=11;
     case(number_field)
      0:number_shift<=scanning ? scan_snapshot[41:31] : snapshot[39:29];
      1:number_shift<={4'd0,scanning ? scan_snapshot[30:24] : snapshot[28:22]};
      2:number_shift<=dsp_view ? {3'd0,dsp_snapshot[55:48]} : {2'd0,scan_snapshot[113:105]};
      3:number_shift<=dsp_view ? {3'd0,dsp_snapshot[47:40]} : {2'd0,scan_snapshot[104:96]};
      4:number_shift<=dsp_view ? agc_percent_product[20:10] : {3'd0,scan_snapshot[87:80]};
      5:number_shift<=scan_snapshot[75:65];
      6:number_shift<=scan_snapshot[64:54];
     endcase
    end else begin
     number_work<={number_adjust[14:0],number_shift[10]};
     number_shift<=number_shift<<1;number_count<=number_count-1'b1;
     if(number_count==1)begin
      case(number_field)
       0:freq_bcd<={number_adjust[14:0],number_shift[10]};
       1:rssi_bcd<={number_adjust[10:0],number_shift[10]};
       2:progress_bcd<={number_adjust[10:0],number_shift[10]};
       3:total_bcd<={number_adjust[10:0],number_shift[10]};
       4:station_bcd<={number_adjust[10:0],number_shift[10]};
       5:start_bcd<={number_adjust[14:0],number_shift[10]};
       6:end_bcd<={number_adjust[14:0],number_shift[10]};
      endcase
      number_field<=number_field+1'b1;
     end
    end
   end
  end
 end
 wire [1:0] col=x<120 ? 0 : x<240 ? 1 : x<360 ? 2 : 3;
 wire [8:0] left=col==0 ? 0 : col==1 ? 120 : col==2 ? 240 : 360;
 wire buttons=graph_view ? (y>=500 && y<700) : (y>=200 && y<600 && !(VIDEO_ENABLE && video_page && y>=400));
 wire [1:0] row=graph_view ? (y<600 ? 0 : 1) : y<300 ? 0 : y<400 ? 1 : y<500 ? 2 : 3;
 wire [3:0] id={row,col};
 wire [9:0] top=graph_view ? (row==0 ? 500 : 600) : row==0 ? 200 : row==1 ? 300 : row==2 ? 400 : 500;
 wire [8:0] bx=x-left;
 wire [9:0] by=y-top;
 wire video_screen=VIDEO_ENABLE && video_page;
 function [2:0] button_length;input [6:0] index;begin case(index)
 7'd0:button_length=3'd5;
 7'd1:button_length=3'd2;
 7'd2:button_length=3'd3;
 7'd3:button_length=3'd3;
 7'd4:button_length=3'd3;
 7'd5:button_length=3'd3;
 7'd6:button_length=3'd5;
 7'd7:button_length=3'd5;
 7'd12:button_length=3'd3;
 7'd13:button_length=3'd3;
 7'd14:button_length=3'd3;
 7'd15:button_length=3'd5;
 7'd16:button_length=3'd5;
 7'd18:button_length=3'd5;
 7'd19:button_length=3'd6;
 7'd22:button_length=3'd6;
 7'd23:button_length=3'd5;
 7'd24:button_length=3'd3;
 7'd25:button_length=3'd3;
 7'd27:button_length=3'd2;
 7'd28:button_length=3'd3;
 7'd29:button_length=3'd3;
 7'd30:button_length=3'd6;
 7'd31:button_length=3'd6;
 7'd33:button_length=3'd5;
 7'd35:button_length=3'd6;
 7'd36:button_length=3'd2;
 7'd37:button_length=3'd3;
 7'd38:button_length=3'd6;
 7'd39:button_length=3'd5;
 7'd48:button_length=3'd6;
 7'd49:button_length=3'd6;
 7'd53:button_length=3'd3;
 7'd55:button_length=3'd5;
 7'd57:button_length=3'd6;
 7'd58:button_length=3'd5;
 7'd59:button_length=3'd5;
 7'd60:button_length=3'd3;
 7'd61:button_length=3'd6;
 7'd62:button_length=3'd5;
 7'd65:button_length=3'd5;
 7'd66:button_length=3'd6;
 7'd67:button_length=3'd6;
 7'd68:button_length=3'd6;
 7'd69:button_length=3'd6;
 7'd72:button_length=3'd0;
 7'd73:button_length=3'd0;
 7'd74:button_length=3'd0;
 7'd75:button_length=3'd0;
 7'd76:button_length=3'd0;
 7'd77:button_length=3'd0;
 7'd78:button_length=3'd0;
 7'd79:button_length=3'd0;
 default:button_length=3'd4;endcase end endfunction
 wire [2:0] button_page=time_view?3'd4:dsp_view?3'd3:video_screen?3'd2:scanning?3'd1:3'd0;
 wire [2:0] button_chars=button_length({button_page,id});
 wire [8:0] button_text_left=9'd60-({3'd0,button_chars,3'd0}-{6'd0,button_chars});
 wire [8:0] button_text_width={2'd0,button_chars,4'd0}-{5'd0,button_chars,1'b0};
 reg [8:0] dx;reg [10:0] text_addr;
 reg [9:0] dy;
 reg text;
 reg [2:0] section;
 wire hero_text=section==1 && !dsp_view;
 // Exact /14 in two short registered stages; no divider or DSP.
 reg [1:0] column_high;reg [6:0] column_remainder;
 reg hero_column_pending;reg [4:0] hero_column;reg [4:0] char_index;
 reg [8:0] column_work;reg [1:0] column_prefix;
 always @*begin
  column_work=dx;column_prefix=0;
  if(column_work>=224)begin column_work=column_work-224;column_prefix[1]=1;end
  if(column_work>=112)begin column_work=column_work-112;column_prefix[0]=1;end
 end
 function [2:0] column_suffix;input [6:0] value;reg [6:0] v;reg [2:0] q;begin
  v=value;q=0;
  if(v>=56)begin v=v-56;q[2]=1;end
  if(v>=28)begin v=v-28;q[1]=1;end
  if(v>=14)q[0]=1;
  column_suffix=q;
 end endfunction
 always @(posedge clk)begin
  column_high<=column_prefix;column_remainder<=column_work[6:0];
  hero_column_pending<=hero_text;hero_column<={1'b0,dx[8:5]};
  char_index<=hero_column_pending?hero_column:{column_high,column_suffix(column_remainder)};
 end
 wire [8:0] cell_start={char_index,4'd0}-{3'd0,char_index,1'b0};
 wire [8:0] native_column=dx-cell_start;
 wire [10:0] caption_addr=section==7 ? text_addr : video_screen ? (buttons ? 10'd832+text_addr :
  section==1 ? (video_status[15] ? 10'd960:10'd416)+{5'd0,char_index} : section==2 ? (video_status[15] ? 10'd992:10'd448)+{5'd0,char_index} :
  section==4 ? 10'd480+{5'd0,char_index} : text_addr>=128 && text_addr<160 ? 10'd384+text_addr-10'd128 : 10'd959) :
  text_addr+(time_view ? 11'd1536 : dsp_view ? 11'd1024 : scanning && section!=6 ? 11'd512 : 11'd0);
 always @*begin
  dx=0;dy=0;text=0;text_addr=0;section=0;
  if(buttons)begin
   dx=bx-button_text_left;dy=by-38;text=dx<button_text_width && dy<24;
   text_addr={2'd0,id,3'd0}+{6'd0,char_index[2:0]};
  end else if(y>=29 && y<53)begin
   if(x>=388 && x<444)begin dx=x-388;dy=y-29;text=1;text_addr=144+char_index;end
   else if(x>=24 && x<248)begin dx=x-24;dy=y-29;text=1;text_addr=128+char_index;end
  end else if(scanning && y>=86 && y<110)begin
   dx=x-56;dy=y-86;text=x>=56 && x<312;text_addr=1888+char_index;section=7;
  end else if(graph_view && y>=184 && y<208)begin
   dx=x-40;dy=y-184;text=x>=40 && x<440;text_addr=1920+char_index;section=7;
  end else if(graph_view && y>=472 && y<496)begin
   dx=x-32;dy=y-472;text=x>=32 && x<448;text_addr=1952+char_index;section=7;
  end else if(graph_view && y>=704 && y<728)begin
   dx=x-56;dy=y-704;text=x>=56 && x<424;text_addr=1984+char_index;section=7;
  end else if(graph_view && y>=736 && y<760)begin
   dx=x-32;dy=y-736;text=x>=32 && x<448;text_addr=704+char_index;section=7;
  end else if(graph_view && y>=768 && y<792)begin
   dx=x-32;dy=y-768;text=x>=32 && x<448;text_addr=736+char_index;section=7;
  end else if(time_view && y>=86 && y<110)begin
   dx=x-32;dy=y-86;text=x>=32 && x<448;text_addr=320+char_index;section=6;
  end else if(dsp_view && y>=86 && y<110)begin
   dx=x-32;dy=y-86;text=x>=32 && x<336;text_addr=320+char_index;section=6;
  end else if(SCAN_ENABLE && !scanning && !dsp_view && y>=86 && y<110)begin
   dx=x-112;dy=y-86;text=x>=112 && x<256;text_addr=320+char_index;
  end else if(!dsp_view && y>=120 && y<168)begin
   dx=x-16;dy=y-120;text=x>=16 && x<464;text_addr=10'd160+char_index;section=1;
  end else if(dsp_view && y>=140 && y<164)begin
   dx=x-32;dy=y-140;text=x>=32 && x<288;text_addr=10'd352+{2'd0,dsp_snapshot[117:115],5'd0}+char_index;section=1;
  end else if(video_screen && y>=460 && y<484)begin
   dx=x-128;dy=y-460;text=x>=128 && x<352;text_addr=(video_status[15]?11'd928:11'd896)+char_index;section=7;
  end else if(video_screen && y>=508 && y<532)begin
   dx=x-128;dy=y-508;text=x>=128 && x<352;text_addr=(video_status[15]?11'd944:11'd912)+char_index;section=7;
  end else if(!graph_view && y>=604 && y<628)begin
   dx=x-32;dy=y-604;text=x>=32 && x<((dsp_view || time_view) ? 448 : 384);text_addr=192+char_index;section=(scanning || dsp_view || time_view)?5:0;
  end else if(!graph_view && y>=640 && y<664)begin
   dx=x-32;dy=y-640;text=x>=32 && x<448;text_addr=224+char_index;section=2;
  end else if(!dsp_view && !time_view && !video_screen && !graph_view && y>=672 && y<696)begin
   dx=x-32;dy=y-672;text=x>=32 && x<448;text_addr=352+char_index;section=6;
  end else if(!graph_view && y>=704 && y<728)begin
   dx=x-32;dy=y-704;text=x>=32 && x<448;text_addr=256+char_index;section=3;
  end else if(!graph_view && y>=768 && y<792)begin
   dx=x-32;dy=y-768;text=x>=32 && x<448;text_addr=288+char_index;section=4;
  end
 end
 wire [8:0] program_word;
 wire [10:0] caption_lookup;
 generate if(PIPELINE_FONT)begin:caption_stage
  reg [10:0] address_q;
  always @(posedge clk)address_q<=caption_addr;
  assign caption_lookup=address_q;
 end else begin:caption_direct
  assign caption_lookup=caption_addr;
 end endgenerate
 fm_lcd_program captions(clk,caption_lookup,program_word);
 reg [31:0] time_mode;
 reg [31:0] scan_mode_word;
 always @*case(scan_snapshot[119:116])
  0:scan_mode_word="IDLE";1:scan_mode_word="SCAN";2:scan_mode_word="PAUS";3:scan_mode_word="DETC";
  4:scan_mode_word="DONE";5:scan_mode_word="REST";6:scan_mode_word="PLAY";default:scan_mode_word="ERR ";
 endcase
 always @*case(time_snapshot[17:16])0:time_mode="STOP";1:time_mode="LIVE";2:time_mode="PAUS";3:time_mode="HIST";endcase

 wire [31:0] field_flags={(video_status[15]?video_status[13]:video_status[8]),(video_status[15]?video_status[12]:video_status[3]),(video_status[15]?video_status[14]:video_status[2]),video_status[1],(video_status[15]?video_status[11]:video_status[0]),time_snapshot[18],time_snapshot[19],time_snapshot[20],time_snapshot[21],time_snapshot[22],time_snapshot[23],dsp_snapshot[62],dsp_snapshot[60],dsp_snapshot[61],dsp_snapshot[121],dsp_snapshot[122],dsp_snapshot[123],dsp_snapshot[124],dsp_snapshot[125],dsp_snapshot[126],scan_snapshot[23],snapshot[40],snapshot[45],snapshot[41],snapshot[46],snapshot[47],snapshot[1],snapshot[3],snapshot[9],snapshot[8],snapshot[6],snapshot[7]};
 wire [63:0] field_digits={station_bcd[3:0],station_bcd[7:4],station_bcd[11:8],total_bcd[3:0],total_bcd[7:4],total_bcd[11:8],progress_bcd[3:0],progress_bcd[7:4],progress_bcd[11:8],rssi_bcd[3:0],rssi_bcd[7:4],rssi_bcd[11:8],freq_bcd[3:0],freq_bcd[7:4],freq_bcd[11:8],freq_bcd[15:12]};
 reg [7:0] glyph_character;
 always @*begin
  glyph_character=program_word[7:0];
  if(program_word[8])begin
   if(program_word[7])glyph_character={7'b0011000,field_flags[program_word[4:0]]};
   else if(program_word[6])glyph_character={4'h3,field_digits[{program_word[3:0],2'b0}+:4]};
   else case(program_word[5:0])
    6'd0:glyph_character=8'd48+(snapshot[13:12]);
    6'd1:glyph_character=8'd48+(snapshot[11:10]);
    6'd2:glyph_character=(snapshot[21:18]>=10 ? "1" : "0");
    6'd3:glyph_character=8'd48+(snapshot[21:18]>=10 ? snapshot[21:18]-10 : snapshot[21:18]);
    6'd4:glyph_character=(snapshot[17:14]<10 ? 8'd48+snapshot[17:14] : 8'd55+snapshot[17:14]);
    6'd5:glyph_character=8'd48+(snapshot[0] || snapshot[4]);
    6'd6:glyph_character=(snapshot[44:42]==4 ? "1" : "0");
    6'd7:glyph_character=(snapshot[44:42]==0 ? "1" : snapshot[44:42]==1 ? "2" : snapshot[44:42]==2 ? "4" : snapshot[44:42]==3 ? "8" : snapshot[44:42]==4 ? "6" : "?");
    6'd8:glyph_character=scan_mode_word[31:24];
    6'd9:glyph_character=scan_mode_word[23:16];
    6'd10:glyph_character=scan_mode_word[15:8];
    6'd11:glyph_character=scan_mode_word[7:0];
    6'd12:glyph_character=(scan_snapshot[53:50]>=10 ? "1" : "0");
    6'd13:glyph_character=8'd48+(scan_snapshot[53:50]>=10 ? scan_snapshot[53:50]-10 : scan_snapshot[53:50]);
    6'd14:glyph_character="0";
    6'd15:glyph_character=(scan_snapshot[79:76]<10 ? 8'd48+scan_snapshot[79:76] : 8'd55+scan_snapshot[79:76]);
    6'd16:glyph_character=(dsp_snapshot[94:93]==0 ? "0" : "1");
    6'd17:glyph_character=(dsp_snapshot[94:93]==0 ? "6" : dsp_snapshot[94:93]==1 ? "2" : "8");
    6'd18:glyph_character=(dsp_snapshot[92:91]==0 ? "2" : dsp_snapshot[92:91]==1 ? "4" : "8");
    6'd19:glyph_character=8'd48+(dsp_snapshot[90:89]);
    6'd20:glyph_character=(dsp_snapshot[88:87]==2 ? "2" : "1");
    6'd21:glyph_character=(dsp_snapshot[88:87]==0 ? "2" : dsp_snapshot[88:87]==1 ? "8" : "4");
    6'd22:glyph_character=(selected_gain<4 ? "-" : "+");
    6'd23:glyph_character=(gain_db>=10 ? "1" : "0");
    6'd24:glyph_character=8'd48+(gain_db>=10 ? gain_db-10 : gain_db);
    6'd25:glyph_character=8'd48+(dsp_snapshot[120:118]);
    6'd26:glyph_character=time_mode[31:24];
    6'd27:glyph_character=time_mode[23:16];
    6'd28:glyph_character=time_mode[15:8];
    6'd29:glyph_character=time_mode[7:0];
    6'd30:glyph_character=(time_snapshot[15:12]>=10 ? "1" : "0");
    6'd31:glyph_character=8'd48+(time_snapshot[15:12]>=10 ? time_snapshot[15:12]-10 : time_snapshot[15:12]);
    6'd32:glyph_character=(time_snapshot[11:8]>=10 ? "1" : "0");
    6'd33:glyph_character=8'd48+(time_snapshot[11:8]>=10 ? time_snapshot[11:8]-10 : time_snapshot[11:8]);
    6'd34:glyph_character=(time_snapshot[7:4]>=10 ? "1" : "0");
    6'd35:glyph_character=8'd48+(time_snapshot[7:4]>=10 ? time_snapshot[7:4]-10 : time_snapshot[7:4]);
    6'd36:glyph_character=(video_status[9] ? "3" : 8'h31);
    6'd37:glyph_character=(video_status[9] ? "2" : 8'h36);
    6'd38:glyph_character=(video_status[9] ? "2" : 8'h31);
    6'd39:glyph_character=(video_status[9] ? "4" : 8'h32);
    6'd40:glyph_character=(video_status[9] ? (video_status[10] ? "2" : "1") : 8'h31);
    6'd41:glyph_character=(video_status[9] ? "0" : 8'h35);
    6'd42:glyph_character=(video_status[7:4]<10 ? 8'd48+video_status[7:4] : 8'd55+video_status[7:4]);
    6'd43:glyph_character=48+start_bcd[15:12];
    6'd44:glyph_character=48+start_bcd[11:8];
    6'd45:glyph_character=48+start_bcd[7:4];
    6'd46:glyph_character=48+start_bcd[3:0];
    6'd47:glyph_character=48+end_bcd[15:12];
    6'd48:glyph_character=48+end_bcd[11:8];
    6'd49:glyph_character=48+end_bcd[7:4];
    6'd50:glyph_character=48+end_bcd[3:0];
    default:glyph_character="?";
   endcase
  end
  // Preserve the original defensive rendering for reserved band codes.
  if(dsp_view && section==1 && dsp_snapshot[117:115]>=5)case(char_index)
   9:glyph_character=selected_gain<4 ? "-" : "+";
   10:glyph_character=gain_db>=10 ? "1" : "0";
   11:glyph_character=48+(gain_db>=10 ? gain_db-10 : gain_db);
  endcase
 end
 wire [15:0] bits;wire [31:0] hero_bits;
 reg [11:0] native_address;reg [12:0] hero_address;
 always @(posedge clk)begin native_address<={glyph_character[6:0],dy[4:0]};hero_address<={glyph_character[6:0],dy[5:0]};end
 generate if(PIPELINE_FONT)begin:glyph_stage
  reg [10:0] address_q;
  always @(posedge clk)address_q<={glyph_character[6:0],dy[4:1]};
  assign shared_font_addr=address_q;
 end else begin:glyph_direct
  assign shared_font_addr={glyph_character[6:0],dy[4:1]};
 end endgenerate
 // A dedicated tiny LCD font keeps the HDMI analyzer typography untouched.
 lcd_ui_ascii_font lcd_glyphs(clk,native_address,bits);
 lcd_ui_hero_font hero_glyphs(clk,hero_address,hero_bits);
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
 wire card_body=bx>=5 && bx<115 && by>=5 && by<95;
 wire card_cut=(bx<10 || bx>=110) && (by<10 || by>=90);
 wire card_edge=bx<7 || bx>=113 || by<7 || by>=93;
 wire used_button=!(time_view || video_screen) || !id[3];
 reg selected;
 always @*begin
  selected=0;
  if(dsp_view)case(id)
   0:selected=!dsp_snapshot[126];5:selected=dsp_snapshot[124];
   6:selected=dsp_snapshot[123];7:selected=dsp_snapshot[122];8:selected=dsp_snapshot[121];
   default:selected=0;
  endcase
  else if(time_view)case(id)
   0:selected=time_snapshot[17:16]==1;1:selected=time_snapshot[17:16]==2;
   2:selected=time_snapshot[17:16]==3;6:selected=time_snapshot[17:16]==0;
   default:selected=0;
  endcase
  else if(video_screen)begin
   selected=(id==4 && !video_status[15]) || (id==5 && video_status[15]);
  end else if(scanning)begin
   selected=(id==0 && scan_snapshot[119:116]==1) || (id==2 && scan_snapshot[119:116]==2);
  end else case(id)
   0:selected=snapshot[8];1:selected=snapshot[13:12]==2;
   2:selected=snapshot[13:12]==0;3:selected=snapshot[13:12]==3;
   10:selected=snapshot[7];12:selected=snapshot[13:12]==1;13:selected=snapshot[9];
   default:selected=0;
  endcase
 end
 wire back_pill=x>=368 && x<464 && y>=22 && y<60;
 wire hero=x>=16 && x<464 && y>=120 && y<184;
 wire status_card=x>=16 && x<464 && y>=600 && y<792;
 wire scan_link=SCAN_ENABLE && !scanning && !video_screen && !dsp_view && !time_view &&
                x>=24 && x<344 && y>=80 && y<116;
 wire [6:0] rssi_level=scanning ? scan_snapshot[30:24] : snapshot[28:22];
 wire [8:0] meter_value=dsp_view ? ({1'b0,dsp_snapshot[47:40]}+{2'd0,dsp_snapshot[47:41]}) :
                          ({2'd0,rssi_level}+{1'b0,rssi_level,1'b0});
 wire [8:0] meter_x=x-32;
 wire meter=y>=176 && y<180 && x>=32 && x<448;
 wire [8:0] time_buffer_pixels={1'b0,time_snapshot[15:12],4'd0}+{2'd0,time_snapshot[15:12],3'd0};
 wire [8:0] time_position_pixels={1'b0,time_snapshot[7:4],4'd0}+{2'd0,time_snapshot[7:4],3'd0};
 wire [8:0] timeline_x=x-48;
 wire plot_paint;wire [15:0] plot_pixel;
 generate if(SCAN_ENABLE)begin:band_plot
  fm_lcd_scan_plot plot(.clk(clk),.rst_n(rst_n),.online(status[1]),.x(x),.y(y),
   .scan_status(scan_status),.point_valid(scan_point_valid),.point(scan_point),
   .paint(plot_paint),.pixel(plot_pixel));
 end else begin:no_band_plot assign plot_paint=0;assign plot_pixel=0;end endgenerate
 always @*begin
  pixel=BG;
  if(y<192)pixel=NAV;
  if(y==75 && x>=24 && x<456)pixel=LINE;
  if(back_pill)pixel=CARD_TOP;
  if(scan_link || (scanning && x>=24 && x<344 && y>=80 && y<116))pixel=SELECTED;
  if(hero)pixel=CARD;
  if(status_card && !graph_view)begin
   pixel=NAV;
   if(x==16 || x==463 || y==600 || y==791)pixel=LINE;
   if((y==632 || y==696 || y==760) && x>=32 && x<448)pixel=LINE;
  end
  if(buttons)begin
   pixel=BG;
   if(used_button && card_body && !card_cut)begin
    pixel=(selected || (pressed && id==pressed_id)) ? SELECTED : CARD;
    if(card_edge)pixel=(selected || (pressed && id==pressed_id)) ? ACCENT:LINE;
   end
   // PAUSE keeps recording; this track is seconds of valid history, not FFT.
   // 24 pixels/second, 16 second display scale, existing status nibbles only.
   if(time_view && x>=48 && x<432 && y>=450 && y<466)begin
    pixel=timeline_x<time_buffer_pixels ? SELECTED:LINE;
    if(timeline_x<time_buffer_pixels && y>=456 && y<460)pixel=ACCENT;
    if(timeline_x==time_position_pixels || timeline_x==time_position_pixels+1'b1)pixel=WARN;
   end
  end
  if(video_screen && x>=24 && x<456 && y>=438 && y<559)begin
   pixel=CARD;if(x==24 || x==455 || y==438 || y==558)pixel=LINE;
  end
  if(graph_view && plot_paint)pixel=plot_pixel;
  if(meter && !video_screen)pixel=meter_x<meter_value ? ACCENT:LINE;
  if(text && (hero_text ? hero_bits[31-dx[4:0]]:bits[15-native_column[3:0]]))pixel=(section==1 || scan_link) ? ACCENT : (section==4 && snapshot[17:14]!=0 && !dsp_view && !time_view && !video_screen && !scanning) ? WARN:TEXT;
 end
endmodule

// The existing 8080 raster holds each coordinate for many 50MHz clocks.
// A fractional accumulator maps 416 LCD columns onto 1..256 bins exactly:
// floor((x-32)*total/416), with no multiplier or combinational divider.
// One 256x10 simple-dual-port memory holds RSSI + ready/peak/stable flags.
module fm_lcd_scan_plot(input clk,rst_n,online,input [8:0] x,input [9:0] y,
 input [127:0] scan_status,input point_valid,input [31:0] point,
 output wire paint,output reg [15:0] pixel);
 // The existing mailbox updates on falling edges. Capture its complete,
 // already coherent status before drawing: negative->positive is a short
 // register path, and the graph comparisons now have a full 20ns cycle.
 reg [127:0] scan_config;reg link_online;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin scan_config<=0;link_online<=0;end
  else begin scan_config<=scan_status;link_online<=online;end
 end
 reg [9:0] points[0:255];reg [9:0] point_q;
 reg [7:0] generation,clear_index,pending_index;
 reg clearing,have_generation,pending_valid;reg [9:0] pending_data;
 wire [7:0] status_delta=scan_config[127:120]-generation;
 wire [7:0] point_delta=point[31:24]-generation;
 wire status_new=link_online && (!have_generation || (status_delta!=0 && !status_delta[7]));
 wire point_new=link_online && point_valid && (!have_generation || (point_delta!=0 && !point_delta[7]));
 wire new_generation=status_new || point_new;
 wire [7:0] next_generation=point_new ? point[31:24]:scan_config[127:120];
 wire point_matches=link_online && have_generation && point_valid && point[31:24]==generation && !new_generation;
 reg [8:0] last_x,fraction;reg [7:0] bin;
 wire [9:0] fraction_sum={1'b0,fraction}+{1'b0,scan_config[104:96]};
 wire [9:0] fraction_sub=fraction_sum-10'd416;
 always @(posedge clk)begin
  if(clearing)points[clear_index]<=0;
  else if(pending_valid)points[pending_index]<=pending_data;
  else if(point_matches)points[point[23:16]]<=point[9:0];
  point_q<=points[bin];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   generation<=0;clear_index<=0;clearing<=1;have_generation<=0;
   pending_valid<=0;pending_index<=0;pending_data<=0;last_x<=0;fraction<=0;bin<=0;
  end else begin
   last_x<=x;
   if(x==0 || x==32)begin fraction<=0;bin<=0;end
   else if(x!=last_x && x>32 && x<448)begin
    if(fraction_sum>=416)begin fraction<=fraction_sub[8:0];bin<=bin+1'b1;end
    else fraction<=fraction_sum[8:0];
   end
   if(clearing)begin clear_index<=clear_index+1'b1;if(clear_index==255)clearing<=0;end
   if(!link_online)begin have_generation<=0;pending_valid<=0;end
   else if(new_generation)begin
    generation<=next_generation;have_generation<=1;clearing<=1;clear_index<=0;
    pending_valid<=point_valid && point[31:24]==next_generation;
    pending_index<=point[23:16];pending_data<=point[9:0];
   end else if(clearing)begin
    // Clearing takes 5.12us. Existing point packets are >20us apart, so one
    // held first-point slot covers a new-generation/clear collision losslessly.
    if(point_matches)begin pending_valid<=1;pending_index<=point[23:16];pending_data<=point[9:0];end
   end else if(pending_valid)begin
    pending_valid<=point_matches;
    if(point_matches)begin pending_index<=point[23:16];pending_data<=point[9:0];end
   end
  end
 end
 wire graph=x>=32 && x<448 && y>=208 && y<=464;
 wire measured={1'b0,bin}<scan_config[113:105] && {1'b0,bin}<scan_config[104:96];
 wire [9:0] bar_top=10'd464-{2'd0,point_q[6:0],1'b0};
 assign paint=graph;
 always @*begin
  pixel=16'hffff;
  if(y==208 || y==272 || y==336 || y==400 || y==464)
   if(!x[3])pixel=16'hcedc;
  if(x==32 || x==447 || y==464)pixel=16'hcedc;
  if(link_online && have_generation && !clearing && measured && y>=bar_top && y<464)
   pixel=point_q[8] ? 16'h3cce : (point_q[7] && point_q[9]) ? 16'h4d9b:16'hadf9;
  if(link_online && scan_config[115] && bin==scan_config[95:88] && y[3:2]!=0)pixel=16'ha307;
  if(link_online && scan_config[41:31]!=0 && bin==scan_config[49:42] && y[3:0]<2)pixel=16'h23f1;
 end
endmodule
