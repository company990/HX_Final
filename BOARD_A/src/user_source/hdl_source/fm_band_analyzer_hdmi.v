// Three pixel clocks of latency, matched RGB/VS/DE on the bypass path.
module fm_band_analyzer_hdmi #(parameter SHARED_FONT=0,BG_LATENCY=0)(input clk,rst,enable,online,input vs,de,input [23:0] rgb,
 input point_valid,input [7:0] point_generation,point_index,input [15:0] point_data,
 input [127:0] status,output out_vs,out_de,output [23:0] out_rgb,output [10:0] shared_font_addr,input [15:0] shared_font_bits);
 reg [11:0] points[0:255];reg [11:0] point_q;reg [127:0] snapshot;
 reg [7:0] generation,clear_index;reg clearing,have_generation;
 wire [7:0] generation_delta=point_generation-generation;
 reg [9:0] x,y;reg vs_d,de_d,seen;
 wire frame=vs && !vs_d;
 reg active_enable;
 always @(posedge clk or posedge rst)if(rst)active_enable<=0;else if(frame)active_enable<=enable;
 wire [9:0] py=y+((de && !de_d && seen)?10'd1:10'd0);
 wire [9:0] off=x-10'd64;
 wire [17:0] scaled_bin=off[8:0]*snapshot[104:96];
 wire [7:0] bin=scaled_bin[16:9];
 wire point_write=point_valid && have_generation && point_generation==generation && !clearing;
 always @(posedge clk)begin
  if(clearing)points[clear_index]<=0;else if(point_write)points[point_index]<=point_data[11:0];
  point_q<=points[bin];
 end
 reg [10:0] peak0,peak1,peak2;reg [1:0] peak_count;
 wire [3:0] step=status[53:50];
 // Same 11-bit modular product as the original shift/add expression.
 (* dsp_style="yes" *) wire [11:0] offset_product=point_index*step;
 wire [10:0] offset=offset_product[10:0];
 always @(posedge clk or posedge rst)begin
  if(rst)begin generation<=0;have_generation<=0;clearing<=1;clear_index<=0;x<=0;y<=0;vs_d<=0;de_d<=0;seen<=0;snapshot<=0;peak0<=0;peak1<=0;peak2<=0;peak_count<=0;end
  else begin
   vs_d<=vs;de_d<=de;
   if(frame)begin x<=0;y<=0;seen<=0;snapshot<=status;end
   else if(de)begin x<=x+1'b1;if(!de_d)begin seen<=1;if(seen)y<=y+1'b1;end end else x<=0;
   if(!online)have_generation<=0;
   if(clearing)begin clear_index<=clear_index+1'b1;if(clear_index==255)clearing<=0;end
   if(point_valid && online && (!have_generation || (generation_delta!=0 && !generation_delta[7])))begin
    generation<=point_generation;have_generation<=1;clearing<=1;clear_index<=0;peak_count<=0;peak0<=0;peak1<=0;peak2<=0;
   end
   if(point_write)begin
    if(point_index==0)peak_count<=0;
    if(point_data[8] && {1'b0,point_index}<status[104:96])begin
     case(point_index==0?2'd0:peak_count)0:peak0<=status[75:65]+offset;1:peak1<=status[75:65]+offset;2:peak2<=status[75:65]+offset;endcase
     if(point_index==0)peak_count<=1;else if(peak_count<3)peak_count<=peak_count+1'b1;
    end
   end
  end
 end
 // Sparse 64-column character map occupies one BRAM, including dynamic digits.
 reg [7:0] characters[0:1023];reg [7:0] character_q;
 integer n; initial begin
for(n=0;n<1024;n=n+1)characters[n]=8'h20;
characters[0]=8'h20;
characters[1]=8'h20;
characters[2]=8'h20;
characters[3]=8'h20;
characters[4]=8'h46;
characters[5]=8'h4d;
characters[6]=8'h20;
characters[7]=8'h42;
characters[8]=8'h41;
characters[9]=8'h4e;
characters[10]=8'h44;
characters[11]=8'h20;
characters[12]=8'h41;
characters[13]=8'h4e;
characters[14]=8'h41;
characters[15]=8'h4c;
characters[16]=8'h59;
characters[17]=8'h5a;
characters[18]=8'h45;
characters[19]=8'h52;
characters[64]=8'h20;
characters[65]=8'h20;
characters[66]=8'h20;
characters[67]=8'h20;
characters[68]=8'h53;
characters[69]=8'h43;
characters[70]=8'h41;
characters[71]=8'h4e;
characters[72]=8'h20;
characters[73]=8'h30;
characters[74]=8'h30;
characters[75]=8'h30;
characters[76]=8'h2f;
characters[77]=8'h30;
characters[78]=8'h30;
characters[79]=8'h30;
characters[80]=8'h20;
characters[81]=8'h20;
characters[82]=8'h20;
characters[83]=8'h20;
characters[84]=8'h53;
characters[85]=8'h54;
characters[86]=8'h41;
characters[87]=8'h54;
characters[88]=8'h49;
characters[89]=8'h4f;
characters[90]=8'h4e;
characters[91]=8'h53;
characters[92]=8'h20;
characters[93]=8'h30;
characters[94]=8'h30;
characters[95]=8'h30;
characters[128]=8'h20;
characters[129]=8'h20;
characters[130]=8'h20;
characters[131]=8'h20;
characters[132]=8'h53;
characters[133]=8'h45;
characters[134]=8'h4c;
characters[135]=8'h45;
characters[136]=8'h43;
characters[137]=8'h54;
characters[138]=8'h20;
characters[139]=8'h30;
characters[140]=8'h30;
characters[141]=8'h30;
characters[142]=8'h2e;
characters[143]=8'h30;
characters[144]=8'h20;
characters[145]=8'h4d;
characters[146]=8'h48;
characters[147]=8'h5a;
characters[148]=8'h20;
characters[149]=8'h20;
characters[150]=8'h20;
characters[151]=8'h20;
characters[152]=8'h52;
characters[153]=8'h53;
characters[154]=8'h53;
characters[155]=8'h49;
characters[156]=8'h20;
characters[157]=8'h30;
characters[158]=8'h30;
characters[192]=8'h52;
characters[193]=8'h53;
characters[194]=8'h53;
characters[195]=8'h49;
characters[196]=8'h20;
characters[197]=8'h20;
characters[198]=8'h53;
characters[199]=8'h54;
characters[200]=8'h41;
characters[201]=8'h54;
characters[202]=8'h45;
characters[203]=8'h20;
characters[204]=8'h49;
characters[205]=8'h44;
characters[206]=8'h4c;
characters[207]=8'h20;
characters[208]=8'h20;
characters[209]=8'h20;
characters[210]=8'h20;
characters[211]=8'h20;
characters[212]=8'h45;
characters[213]=8'h52;
characters[214]=8'h52;
characters[215]=8'h20;
characters[216]=8'h30;
characters[217]=8'h30;
characters[218]=8'h30;
characters[768]=8'h20;
characters[769]=8'h20;
characters[770]=8'h20;
characters[771]=8'h20;
characters[772]=8'h30;
characters[773]=8'h30;
characters[774]=8'h30;
characters[775]=8'h2e;
characters[776]=8'h30;
characters[777]=8'h20;
characters[778]=8'h20;
characters[779]=8'h20;
characters[780]=8'h20;
characters[781]=8'h20;
characters[782]=8'h20;
characters[783]=8'h20;
characters[784]=8'h20;
characters[785]=8'h20;
characters[786]=8'h30;
characters[787]=8'h30;
characters[788]=8'h30;
characters[789]=8'h2e;
characters[790]=8'h30;
characters[791]=8'h20;
characters[792]=8'h20;
characters[793]=8'h20;
characters[794]=8'h20;
characters[795]=8'h20;
characters[796]=8'h30;
characters[797]=8'h30;
characters[798]=8'h30;
characters[799]=8'h2e;
characters[800]=8'h30;
characters[801]=8'h20;
characters[802]=8'h4d;
characters[832]=8'h20;
characters[833]=8'h20;
characters[834]=8'h20;
characters[835]=8'h20;
characters[836]=8'h50;
characters[837]=8'h45;
characters[838]=8'h41;
characters[839]=8'h4b;
characters[840]=8'h53;
characters[841]=8'h20;
characters[842]=8'h30;
characters[843]=8'h30;
characters[844]=8'h30;
characters[845]=8'h2e;
characters[846]=8'h30;
characters[847]=8'h20;
characters[848]=8'h20;
characters[849]=8'h20;
characters[850]=8'h30;
characters[851]=8'h30;
characters[852]=8'h30;
characters[853]=8'h2e;
characters[854]=8'h30;
characters[855]=8'h20;
characters[856]=8'h20;
characters[857]=8'h20;
characters[858]=8'h30;
characters[859]=8'h30;
characters[860]=8'h30;
characters[861]=8'h2e;
characters[862]=8'h30;
characters[863]=8'h20;
characters[864]=8'h4d;
characters[865]=8'h48;
characters[866]=8'h7a;
characters[896]=8'h20;
characters[897]=8'h20;
characters[898]=8'h20;
characters[899]=8'h20;
characters[900]=8'h53;
characters[901]=8'h57;
characters[902]=8'h45;
characters[903]=8'h50;
characters[904]=8'h54;
characters[905]=8'h20;
characters[906]=8'h52;
characters[907]=8'h53;
characters[908]=8'h53;
characters[909]=8'h49;
characters[910]=8'h20;
characters[911]=8'h2f;
characters[912]=8'h20;
characters[913]=8'h4e;
characters[914]=8'h4f;
characters[915]=8'h54;
characters[916]=8'h20;
characters[917]=8'h57;
characters[918]=8'h49;
characters[919]=8'h44;
characters[920]=8'h45;
characters[921]=8'h42;
characters[922]=8'h41;
characters[923]=8'h4e;
characters[924]=8'h44;
characters[925]=8'h20;
characters[926]=8'h52;
characters[927]=8'h46;
characters[928]=8'h20;
characters[929]=8'h46;
characters[930]=8'h46;
characters[931]=8'h54;
characters[257]=8'h31;
characters[258]=8'h32;
characters[259]=8'h30;
characters[385]=8'h30;
characters[386]=8'h38;
characters[387]=8'h38;
characters[513]=8'h30;
characters[514]=8'h35;
characters[515]=8'h36;
characters[641]=8'h30;
characters[642]=8'h32;
characters[643]=8'h34;
characters[705]=8'h30;
characters[706]=8'h30;
characters[707]=8'h38;
characters[803]=8'h48;
characters[804]=8'h7a;
characters[805]=8'h20;
characters[806]=8'h20;
characters[807]=8'h20;
characters[867]=8'h20;
characters[868]=8'h20;
characters[869]=8'h20;
characters[870]=8'h20;
characters[871]=8'h20;
characters[96]=8'h20;
characters[97]=8'h20;
characters[98]=8'h20;
characters[99]=8'h20;
characters[100]=8'h20;
characters[101]=8'h20;
characters[102]=8'h20;
characters[103]=8'h20;
characters[159]=8'h30;
characters[160]=8'h20;
characters[161]=8'h20;
characters[162]=8'h20;
characters[163]=8'h20;
characters[164]=8'h20;
characters[165]=8'h20;
characters[166]=8'h20;
characters[167]=8'h20;
characters[219]=8'h20;
characters[220]=8'h20;
characters[221]=8'h20;
characters[222]=8'h20;
characters[223]=8'h20;
characters[224]=8'h20;
characters[225]=8'h20;
characters[226]=8'h20;
characters[227]=8'h20;
characters[228]=8'h20;
characters[229]=8'h20;
characters[230]=8'h20;
characters[231]=8'h20;
end
 reg [3:0] field;reg [4:0] count;reg [10:0] shift;reg [15:0] work_bcd,bcd,adjust;
 reg [1:0] digit;reg writing;reg [9:0] base_addr;reg freq_field;
 reg [10:0] number;reg [9:0] number_addr;reg is_frequency;
 integer j;
 always @*begin
  adjust=work_bcd;for(j=0;j<4;j=j+1)if(work_bcd[j*4+:4]>=5)adjust[j*4+:4]=work_bcd[j*4+:4]+3;
  number=0;number_addr=0;is_frequency=0;
  case(field)
   0:begin number=snapshot[113:105];number_addr=64+9;end
   1:begin number=snapshot[104:96];number_addr=64+13;end
   2:begin number=snapshot[87:80];number_addr=64+29;end
   3:begin number=snapshot[41:31];number_addr=128+11;is_frequency=1;end
   4:begin number=snapshot[30:24];number_addr=128+29;end
   5:begin number=snapshot[119:116];number_addr=192+12;end
   6:begin number=snapshot[79:76];number_addr=192+24;end
   7:begin number=snapshot[75:65];number_addr=768+4;is_frequency=1;end
   8:begin number=snapshot[64:54];number_addr=768+28;is_frequency=1;end
   9:begin number=peak0;number_addr=832+10;is_frequency=1;end
   10:begin number=peak1;number_addr=832+18;is_frequency=1;end
   11:begin number=peak2;number_addr=832+26;is_frequency=1;end
   12:begin number=({1'b0,snapshot[75:65]}+{1'b0,snapshot[64:54]})>>1;number_addr=768+18;is_frequency=1;end
  endcase
 end
 wire [9:0] digit_addr=base_addr+(freq_field?{8'd0,digit}+(digit==3?10'd1:10'd0):({8'd0,digit}-10'd1));
 reg [3:0] digit_value;
 always @*case(digit)
  0:digit_value=bcd[15:12];1:digit_value=bcd[11:8];
  2:digit_value=bcd[7:4];3:digit_value=bcd[3:0];
 endcase
 reg [23:0] state_word;
 always @*case(snapshot[119:116])
  0:state_word="IDL";1:state_word="RUN";2:state_word="PAU";3:state_word="DET";
  4:state_word="END";5:state_word="RET";6:state_word="FM ";default:state_word="ERR";
 endcase
 wire [7:0] digit_char=field==5?(digit==1?state_word[23:16]:digit==2?state_word[15:8]:state_word[7:0]):((freq_field && number==0)?8'h2d:8'd48+digit_value);
 always @(posedge clk)begin
  if(writing)characters[digit_addr]<=digit_char;
  character_q<=characters[{py[8:5],x[9:4]}];
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin field<=13;count<=0;shift<=0;work_bcd<=0;bcd<=0;digit<=0;writing<=0;base_addr<=0;freq_field<=0;end
  else if(frame)begin field<=0;count<=0;writing<=0;end
  else if(field<13)begin
   if(writing)begin if(digit==3)begin writing<=0;field<=field+1'b1;count<=0;end else digit<=digit+1'b1;end
   else if(count==0)begin shift<=number;work_bcd<=0;count<=11;base_addr<=number_addr;freq_field<=is_frequency;end
   else begin
    shift<=shift<<1;work_bcd<={adjust[14:0],shift[10]};count<=count-1'b1;
    if(count==1)begin bcd<={adjust[14:0],shift[10]};writing<=1;digit<=freq_field?0:1;end
   end
  end
 end
 reg [9:0] x1,y1,x2,y2;reg [7:0] bin1;reg [23:0] bg1,bg2,pixel;
 localparam LATENCY=3+BG_LATENCY;
 reg [LATENCY-1:0] vp,dp,ep,lp;
 wire [15:0] font_bits;
 reg [2:0] graph_color;reg graph_text;
 reg [23:0] passthrough;
 assign shared_font_addr={character_q[6:0],y1[3:0]};
 generate if(SHARED_FONT)begin:shared_glyphs
  assign font_bits=shared_font_bits;
 end else begin:local_glyphs
  fm_lcd_font font(clk,shared_font_addr,font_bits);
 end endgenerate
 wire graph=x1>=64 && x1<576 && y1>=112 && y1<=368;
 wire measured={1'b0,bin1}<snapshot[113:105];
 // Compute each point in the original bin domain; registered x/y match RAM.
 reg [2:0] drawing;
 function [23:0] palette;
 input [2:0] color;
 begin case(color)
  0:palette=24'h0d1626;1:palette=24'h172338;2:palette=24'h303e54;
  3:palette=24'h7de2b2;4:palette=24'h61b1ee;5:palette=24'h8a708b;
  6:palette=24'hf4cc70;7:palette=24'hffffff;
 endcase end endfunction
 always @*begin
  drawing=0;
  if(y1>=400)drawing=1;
  if(graph)begin
   if(y1==112 || y1==176 || y1==240 || y1==304 || y1==368)begin if(!x1[3])drawing=2;end
   if(!clearing && point_q[7] && measured && y1>=10'd368-{2'd0,point_q[6:0],1'b0} && y1<368)
    drawing=point_q[8]?3:point_q[9]?4:5;
   if(snapshot[115] && bin1==snapshot[95:88] && y1[3:2]!=0)drawing=6;
   if(bin1==snapshot[49:42] && y1>=112 && y1<368 && y1[3:0]==0)drawing=7;
  end
  if(y1>=90 && y1<94 && x1>=64 && x1<576)drawing=measured?4:2;
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin x1<=0;y1<=0;x2<=0;y2<=0;bin1<=0;bg1<=0;bg2<=0;pixel<=0;graph_color<=0;graph_text<=0;passthrough<=0;vp<=0;dp<=0;ep<=0;lp<=0;end
  else begin
   x1<=x;y1<=py;bin1<=bin;bg1<=rgb;x2<=x1;y2<=y1;bg2<=bg1;
   vp<={vp[LATENCY-2:0],vs};dp<={dp[LATENCY-2:0],de};ep<={ep[LATENCY-2:0],active_enable};lp<={lp[LATENCY-2:0],online};
   graph_color<=drawing;
   graph_text<=!y2[4] && font_bits[15-x2[3:0]];
   pixel<=(!y2[4] && font_bits[15-x2[3:0]])?24'heaf2ff:palette(graph_color);
   passthrough<=bg2;
  end
 end
 // Background already has BG_LATENCY external pixel stages. Keep all
 // RAM, point writes, status snapshots, counters and font addresses on
 // their original clocks. Delay only chart output to match that background.
 wire [23:0] chart_pixel;
 generate if(BG_LATENCY==1)begin:chart_delay
  reg [23:0] held;
  always @(posedge clk or posedge rst)if(rst)held<=0;else held<=pixel;
  assign chart_pixel=held;
 end else begin:no_chart_delay
  assign chart_pixel=pixel;
 end endgenerate
 assign out_vs=vp[LATENCY-1];assign out_de=dp[LATENCY-1];assign out_rgb=ep[LATENCY-1]?chart_pixel:passthrough;
endmodule
