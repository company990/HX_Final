`timescale 1ns/1ps
// The existing OSD provides x/y and the synchronized RGB pipeline. This
// module adds one-cycle text and hotspot masks from three 9K bitmap ROMs.
module presentation_overlay #(
    parameter [4:0] TITLE_COL=5'd6, TITLE_ROW=5'd4,
    parameter [5:0] BODY_COL=6'd10, BODY1_ROW=6'd14, BODY2_ROW=6'd17,
    parameter [4:0] ICON_COL=5'd15, ICON_ROW=5'd7,
    parameter [2:0] ICON_ID=3'd0,
    // {left 32-pixel column, top 32-pixel row, guide/arrow style}.
    parameter [11:0] HOTSPOT1={5'd1,5'd4,2'b00},
    parameter [11:0] HOTSPOT2={5'd1,5'd7,2'b01},
    parameter [11:0] HOTSPOT3={5'd13,5'd8,2'b11},
    parameter [11:0] HOTSPOT4={5'd15,5'd7,2'b11}
) (
    input clk,rst,mode,key_next_n,vs,de,
    input av_step,av_zoom,input[5:0] av_mid,
    input [9:0] x,y,
    output wire paint,hotspot_paint,focus_active,focus_area,
    output wire[1:0] text_coverage,
    output wire [2:0] focus_step
);
    reg [2:0] step;
    reg armed,vs_d;
    reg [4:0] title_alpha;
    always @(posedge clk or posedge rst) begin
        if (rst) begin step<=0; armed<=0; vs_d<=0; title_alpha<=0; end
        else begin
            vs_d<=vs;
            if (!mode) armed<=0;
            else if (key_next_n) armed<=1;
            else if (armed) armed<=0;
            if (mode && ((armed && !key_next_n) || av_step)) begin
                step<=step==4 ? 3'd0 : step+1'b1;
                if (step==0) title_alpha<=0;
            end else if (vs && !vs_d && step!=0 && title_alpha<16)
                title_alpha<=title_alpha+1'b1;
        end
    end

    wire [4:0] tx=x[9:5],ty=y[9:5];
    wire [5:0] bx=x[9:4],by=y[9:4];
    wire title_area=mode && de && step>=1 &&
        tx>=TITLE_COL && tx<TITLE_COL+5'd7 && ty==TITLE_ROW;
    wire body1_area=mode && de && step>=2 &&
        bx>=BODY_COL && bx<BODY_COL+6'd11 && by==BODY1_ROW;
    wire body2_area=mode && de && step>=3 &&
        bx>=BODY_COL && bx<BODY_COL+6'd9 && by==BODY2_ROW;
    wire icon_area=mode && de && step>=4 && tx==ICON_COL && ty==ICON_ROW;
    wire [4:0] title_cell=tx-TITLE_COL;
    wire [5:0] body_cell=bx-BODY_COL;

    wire[1:0] title_pixel,body_pixel; wire icon_pixel;
    // Independent addresses/ports preserve title/body overlap behavior.
    presentation_title_body_rom u_text_rom(
        .clk(clk),.title_addr({y[4:0],title_cell[2:0],x[4:0]}),
        .body_addr({body2_area,y[3:0],body_cell[3:0],x[3:0]}),
        .title_pixel(title_pixel),.body_pixel(body_pixel));
    wire arrow_pixel;
    // Repeat the two central sprite columns/rows: a subtle ~6% local expansion.
    wire[4:0] icon_x=av_zoom ? (x[4:0]<16 ? x[4:0]+1'b1 : x[4:0]-1'b1) : x[4:0];
    wire[4:0] icon_y=av_zoom ? (y[4:0]<16 ? y[4:0]+1'b1 : y[4:0]-1'b1) : y[4:0];
    presentation_bitrom #(.INIT_FILE("presentation_icons.mif")) u_icon_rom (
        .clk(clk),.addr({ICON_ID,icon_y,icon_x}),.pixel(icon_pixel),
        .addr_b({3'd5,y[4:0],x[4:0]}),.pixel_b(arrow_pixel));

    reg title_d,body_d,icon_d;
    reg fade_d;
    wire [3:0] threshold={y[0],x[0],y[1],x[1]};
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            title_d<=0; body_d<=0; icon_d<=0; fade_d<=0;
        end else begin
            title_d<=title_area;
            body_d<=body1_area || body2_area;
            icon_d<=icon_area;
            fade_d<=title_alpha>{1'b0,threshold};
        end
    end
    assign text_coverage=title_d && fade_d ? title_pixel : body_d ? body_pixel : 2'd0;
    assign paint=icon_d && icon_pixel;

    // A shared two-pixel border and optional leader are placed at each step.
    wire [11:0] hot_cfg=step==1 ? HOTSPOT1 : step==2 ? HOTSPOT2 :
                        step==3 ? HOTSPOT3 : HOTSPOT4;
    wire [4:0] hx=hot_cfg[11:7],hy=hot_cfg[6:2];
    wire [1:0] hot_style=hot_cfg[1:0];
    // The same 32x32 cell as the visible hotspot stays at full brightness.
    // This mask is combinational so the OSD can register it with the RGB pixel.
    assign focus_active=mode && step!=0;
    assign focus_area=focus_active && de && tx==hx && ty==hy;
    assign focus_step=step;
    wire box=tx==hx && ty==hy &&
        (x[4:0]<(2+av_mid[5:4]) || x[4:0]>=(30-av_mid[5:4]) ||
         y[4:0]<(2+av_mid[5:4]) || y[4:0]>=(30-av_mid[5:4]));
    wire leader_cell=tx==hx+5'd1 && ty==hy;
    wire guide=hot_style[0] && leader_cell && y[4:0]==16 && x[4:0]<=24;
    reg hotspot_d,arrow_d;
    always @(posedge clk or posedge rst)
        if (rst) begin hotspot_d<=0; arrow_d<=0; end
        else begin
            hotspot_d<=mode && de && step!=0 && (box || guide);
            arrow_d<=mode && de && step!=0 && hot_style[1] && leader_cell;
        end
    assign hotspot_paint=hotspot_d || (arrow_d && arrow_pixel);
endmodule

// The four hotspot cells all occur above this fixed lower-right window.
// Each source pixel is written before its enlarged copy is displayed, so a
// single 32x32 frame buffer is enough and the underlying image stays visible.
module presentation_magnifier #(
    parameter [9:0] WINDOW_X=10'd544,
    parameter [9:0] WINDOW_Y=10'd352
) (
    input clk,rst,vs,de,
    input focus_active,focus_area,
    input [2:0] focus_step,
    input [9:0] x,y,
    input [23:0] rgb_in,
    output wire [23:0] pixel,
    output reg inside_d,border_d
);
    wire [9:0] dx=x-WINDOW_X;
    wire [9:0] dy=y-WINDOW_Y;
    wire inside=x>=WINDOW_X && x<WINDOW_X+10'd64 &&
                y>=WINDOW_Y && y<WINDOW_Y+10'd64;
    wire border=x>=WINDOW_X-10'd2 && x<WINDOW_X+10'd66 &&
                y>=WINDOW_Y-10'd2 && y<WINDOW_Y+10'd66 && !inside;

    // Adjacent destination pixels share the same source address (2x nearest
    // neighbor). The dual-port RAM has the same one-clock read latency as OSD.
    wire [9:0] read_addr={dy[5:1],dx[5:1]};
    EG_LOGIC_BRAM #(.DATA_WIDTH_A(24),.DATA_WIDTH_B(24),
        .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
        .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
        .RESETMODE("SYNC"),.IMPLEMENT("9K"),
        .INIT_FILE("NONE"),.FILL_ALL("NONE")) pixels (
        .dia(rgb_in),.addra({y[4:0],x[4:0]}),.cea(focus_area),
        .ocea(1'b0),.clka(clk),.wea(focus_area),.rsta(1'b0),
        .bea(1'b0),.doa(),
        .dib(24'd0),.addrb(read_addr),.ceb(1'b1),.oceb(1'b0),
        .clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(pixel));

    reg vs_d,complete,capturing;
    reg [2:0] captured_step,observed_step;
    wire source_first=focus_area && x[4:0]==5'd0 && y[4:0]==5'd0;
    wire source_last=focus_area && x[4:0]==5'd31 && y[4:0]==5'd31;
    wire window_ready=focus_active && complete && captured_step==focus_step;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vs_d<=0;complete<=0;capturing<=0;
            captured_step<=0;observed_step<=0;
            inside_d<=0;border_d<=0;
        end else begin
            vs_d<=vs;
            observed_step<=focus_step;
            if ((vs && !vs_d) || !focus_active || focus_step!=observed_step) begin
                complete<=0;
                capturing<=0;
            end else begin
                if (source_first) begin
                    capturing<=1;
                    captured_step<=focus_step;
                end
                if (source_last && capturing && captured_step==focus_step)
                    complete<=1;
            end
            inside_d<=de && window_ready && inside;
            border_d<=de && window_ready && border;
        end
    end
endmodule

module presentation_bitrom #(parameter WIDTH=1, parameter INIT_FILE="presentation_title.mif") (
    input clk,input [12:0] addr,addr_b,output [WIDTH-1:0] pixel,pixel_b
);
    generate if(0)begin:compressed
     presentation_aa_dictionary #(.INIT_FILE(INIT_FILE)) packed_font(clk,addr,addr_b,pixel,pixel_b);
    end else begin:direct
    EG_LOGIC_BRAM #(.DATA_WIDTH_A(WIDTH),.DATA_WIDTH_B(WIDTH),
        .ADDR_WIDTH_A(13),.ADDR_WIDTH_B(13),
        .DATA_DEPTH_A(8192),.DATA_DEPTH_B(8192),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
        .RESETMODE("SYNC"),.IMPLEMENT("9K"),
        .INIT_FILE(INIT_FILE),.FILL_ALL("NONE")) u_rom (
        .dia({WIDTH{1'b0}}),.addra(addr),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),
        .rsta(1'b0),.bea(1'b0),.doa(pixel),
        .dib({WIDTH{1'b0}}),.addrb(addr_b),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(1'b0),
        .rstb(1'b0),.beb(1'b0),.dob(pixel_b));
    end endgenerate
endmodule

// Lossless dictionary ROM: same pixels and one-clock latency, one 9K block.
module presentation_aa_dictionary #(parameter INIT_FILE="presentation_title.mif")(
 input clk,input [12:0] addr,addr_b,output [1:0] pixel,pixel_b);
 localparam INDEX_FILE=INIT_FILE=="presentation_title.mif" ? "presentation_title_index.mif":"presentation_body_index.mif";
`ifdef FM_FONT_SIM
 localparam MEMORY_FILE=INIT_FILE=="presentation_title.mif" ? "presentation_title_index.dat":"presentation_body_index.dat";
`else
 localparam MEMORY_FILE=INDEX_FILE;
`endif
 wire [8:0] qa,qb;reg [2:0] lo_a,lo_b;
 always @(posedge clk)begin lo_a<=addr[2:0];lo_b<=addr_b[2:0];end
function [15:0] decode_presentation_title;input [8:0] code;begin case(code)
 9'd0:decode_presentation_title=16'h0;
 9'd1:decode_presentation_title=16'h1;
 9'd2:decode_presentation_title=16'h2;
 9'd3:decode_presentation_title=16'h3;
 9'd4:decode_presentation_title=16'h4;
 9'd5:decode_presentation_title=16'h5;
 9'd6:decode_presentation_title=16'h6;
 9'd7:decode_presentation_title=16'h7;
 9'd8:decode_presentation_title=16'ha;
 9'd9:decode_presentation_title=16'hb;
 9'd10:decode_presentation_title=16'hf;
 9'd11:decode_presentation_title=16'h10;
 9'd12:decode_presentation_title=16'h14;
 9'd13:decode_presentation_title=16'h15;
 9'd14:decode_presentation_title=16'h1a;
 9'd15:decode_presentation_title=16'h1e;
 9'd16:decode_presentation_title=16'h1f;
 9'd17:decode_presentation_title=16'h28;
 9'd18:decode_presentation_title=16'h2d;
 9'd19:decode_presentation_title=16'h2e;
 9'd20:decode_presentation_title=16'h2f;
 9'd21:decode_presentation_title=16'h3d;
 9'd22:decode_presentation_title=16'h3e;
 9'd23:decode_presentation_title=16'h3f;
 9'd24:decode_presentation_title=16'h40;
 9'd25:decode_presentation_title=16'h45;
 9'd26:decode_presentation_title=16'h50;
 9'd27:decode_presentation_title=16'h54;
 9'd28:decode_presentation_title=16'h56;
 9'd29:decode_presentation_title=16'h59;
 9'd30:decode_presentation_title=16'h60;
 9'd31:decode_presentation_title=16'h70;
 9'd32:decode_presentation_title=16'h74;
 9'd33:decode_presentation_title=16'h78;
 9'd34:decode_presentation_title=16'h7c;
 9'd35:decode_presentation_title=16'h7d;
 9'd36:decode_presentation_title=16'h7f;
 9'd37:decode_presentation_title=16'h90;
 9'd38:decode_presentation_title=16'hb4;
 9'd39:decode_presentation_title=16'hb8;
 9'd40:decode_presentation_title=16'hbd;
 9'd41:decode_presentation_title=16'hbe;
 9'd42:decode_presentation_title=16'hbf;
 9'd43:decode_presentation_title=16'hd0;
 9'd44:decode_presentation_title=16'he0;
 9'd45:decode_presentation_title=16'hf8;
 9'd46:decode_presentation_title=16'h100;
 9'd47:decode_presentation_title=16'h10b;
 9'd48:decode_presentation_title=16'h14b;
 9'd49:decode_presentation_title=16'h16f;
 9'd50:decode_presentation_title=16'h182;
 9'd51:decode_presentation_title=16'h1a0;
 9'd52:decode_presentation_title=16'h1af;
 9'd53:decode_presentation_title=16'h1bf;
 9'd54:decode_presentation_title=16'h1e0;
 9'd55:decode_presentation_title=16'h1e1;
 9'd56:decode_presentation_title=16'h1e6;
 9'd57:decode_presentation_title=16'h1f0;
 9'd58:decode_presentation_title=16'h1f1;
 9'd59:decode_presentation_title=16'h1f4;
 9'd60:decode_presentation_title=16'h1f8;
 9'd61:decode_presentation_title=16'h1fa;
 9'd62:decode_presentation_title=16'h1ff;
 9'd63:decode_presentation_title=16'h2c0;
 9'd64:decode_presentation_title=16'h2d0;
 9'd65:decode_presentation_title=16'h2e0;
 9'd66:decode_presentation_title=16'h2f0;
 9'd67:decode_presentation_title=16'h2f4;
 9'd68:decode_presentation_title=16'h2fa;
 9'd69:decode_presentation_title=16'h2fe;
 9'd70:decode_presentation_title=16'h3d0;
 9'd71:decode_presentation_title=16'h3f4;
 9'd72:decode_presentation_title=16'h3ff;
 9'd73:decode_presentation_title=16'h500;
 9'd74:decode_presentation_title=16'h56a;
 9'd75:decode_presentation_title=16'h5be;
 9'd76:decode_presentation_title=16'h6af;
 9'd77:decode_presentation_title=16'h6d0;
 9'd78:decode_presentation_title=16'h6e0;
 9'd79:decode_presentation_title=16'h6fd;
 9'd80:decode_presentation_title=16'h6ff;
 9'd81:decode_presentation_title=16'h782;
 9'd82:decode_presentation_title=16'h787;
 9'd83:decode_presentation_title=16'h7f4;
 9'd84:decode_presentation_title=16'h7f8;
 9'd85:decode_presentation_title=16'h7fa;
 9'd86:decode_presentation_title=16'h7ff;
 9'd87:decode_presentation_title=16'h900;
 9'd88:decode_presentation_title=16'hb4f;
 9'd89:decode_presentation_title=16'hb5f;
 9'd90:decode_presentation_title=16'hb80;
 9'd91:decode_presentation_title=16'hbae;
 9'd92:decode_presentation_title=16'hbd0;
 9'd93:decode_presentation_title=16'hbfd;
 9'd94:decode_presentation_title=16'hbff;
 9'd95:decode_presentation_title=16'hf95;
 9'd96:decode_presentation_title=16'hfc0;
 9'd97:decode_presentation_title=16'hfdb;
 9'd98:decode_presentation_title=16'h1000;
 9'd99:decode_presentation_title=16'h107f;
 9'd100:decode_presentation_title=16'h1400;
 9'd101:decode_presentation_title=16'h16ff;
 9'd102:decode_presentation_title=16'h1800;
 9'd103:decode_presentation_title=16'h1ae0;
 9'd104:decode_presentation_title=16'h1b40;
 9'd105:decode_presentation_title=16'h1b80;
 9'd106:decode_presentation_title=16'h1b90;
 9'd107:decode_presentation_title=16'h1bc0;
 9'd108:decode_presentation_title=16'h1bf9;
 9'd109:decode_presentation_title=16'h1bff;
 9'd110:decode_presentation_title=16'h1c00;
 9'd111:decode_presentation_title=16'h1d3c;
 9'd112:decode_presentation_title=16'h1e3e;
 9'd113:decode_presentation_title=16'h1ebf;
 9'd114:decode_presentation_title=16'h1f0f;
 9'd115:decode_presentation_title=16'h1faa;
 9'd116:decode_presentation_title=16'h1fbd;
 9'd117:decode_presentation_title=16'h1fbf;
 9'd118:decode_presentation_title=16'h1fd0;
 9'd119:decode_presentation_title=16'h1fe0;
 9'd120:decode_presentation_title=16'h1ffe;
 9'd121:decode_presentation_title=16'h2400;
 9'd122:decode_presentation_title=16'h26e9;
 9'd123:decode_presentation_title=16'h2c00;
 9'd124:decode_presentation_title=16'h2d15;
 9'd125:decode_presentation_title=16'h2d3c;
 9'd126:decode_presentation_title=16'h2d3d;
 9'd127:decode_presentation_title=16'h2d7b;
 9'd128:decode_presentation_title=16'h2e00;
 9'd129:decode_presentation_title=16'h2e04;
 9'd130:decode_presentation_title=16'h2e0b;
 9'd131:decode_presentation_title=16'h2fd0;
 9'd132:decode_presentation_title=16'h2fe0;
 9'd133:decode_presentation_title=16'h2fff;
 9'd134:decode_presentation_title=16'h3800;
 9'd135:decode_presentation_title=16'h3900;
 9'd136:decode_presentation_title=16'h3c01;
 9'd137:decode_presentation_title=16'h3d07;
 9'd138:decode_presentation_title=16'h3d40;
 9'd139:decode_presentation_title=16'h3f40;
 9'd140:decode_presentation_title=16'h3f42;
 9'd141:decode_presentation_title=16'h3f56;
 9'd142:decode_presentation_title=16'h3fff;
 9'd143:decode_presentation_title=16'h4000;
 9'd144:decode_presentation_title=16'h4002;
 9'd145:decode_presentation_title=16'h400b;
 9'd146:decode_presentation_title=16'h401f;
 9'd147:decode_presentation_title=16'h406e;
 9'd148:decode_presentation_title=16'h40b1;
 9'd149:decode_presentation_title=16'h41e0;
 9'd150:decode_presentation_title=16'h438f;
 9'd151:decode_presentation_title=16'h47c0;
 9'd152:decode_presentation_title=16'h5000;
 9'd153:decode_presentation_title=16'h500b;
 9'd154:decode_presentation_title=16'h5078;
 9'd155:decode_presentation_title=16'h51f4;
 9'd156:decode_presentation_title=16'h5400;
 9'd157:decode_presentation_title=16'h5478;
 9'd158:decode_presentation_title=16'h54b4;
 9'd159:decode_presentation_title=16'h56e0;
 9'd160:decode_presentation_title=16'h56e2;
 9'd161:decode_presentation_title=16'h56ff;
 9'd162:decode_presentation_title=16'h57ff;
 9'd163:decode_presentation_title=16'h5ab4;
 9'd164:decode_presentation_title=16'h5ae5;
 9'd165:decode_presentation_title=16'h5aff;
 9'd166:decode_presentation_title=16'h5bd0;
 9'd167:decode_presentation_title=16'h5bfb;
 9'd168:decode_presentation_title=16'h5bff;
 9'd169:decode_presentation_title=16'h5f00;
 9'd170:decode_presentation_title=16'h5f4b;
 9'd171:decode_presentation_title=16'h6000;
 9'd172:decode_presentation_title=16'h6400;
 9'd173:decode_presentation_title=16'h6aff;
 9'd174:decode_presentation_title=16'h6be0;
 9'd175:decode_presentation_title=16'h6d00;
 9'd176:decode_presentation_title=16'h6ff4;
 9'd177:decode_presentation_title=16'h7000;
 9'd178:decode_presentation_title=16'h7402;
 9'd179:decode_presentation_title=16'h7800;
 9'd180:decode_presentation_title=16'h7801;
 9'd181:decode_presentation_title=16'h7815;
 9'd182:decode_presentation_title=16'h781b;
 9'd183:decode_presentation_title=16'h7850;
 9'd184:decode_presentation_title=16'h7875;
 9'd185:decode_presentation_title=16'h79bf;
 9'd186:decode_presentation_title=16'h7aab;
 9'd187:decode_presentation_title=16'h7aff;
 9'd188:decode_presentation_title=16'h7c00;
 9'd189:decode_presentation_title=16'h7c0b;
 9'd190:decode_presentation_title=16'h7cbf;
 9'd191:decode_presentation_title=16'h7d00;
 9'd192:decode_presentation_title=16'h7d1b;
 9'd193:decode_presentation_title=16'h7d2f;
 9'd194:decode_presentation_title=16'h7dff;
 9'd195:decode_presentation_title=16'h7e00;
 9'd196:decode_presentation_title=16'h7e56;
 9'd197:decode_presentation_title=16'h7ff9;
 9'd198:decode_presentation_title=16'h7fff;
 9'd199:decode_presentation_title=16'h8000;
 9'd200:decode_presentation_title=16'h8001;
 9'd201:decode_presentation_title=16'h8003;
 9'd202:decode_presentation_title=16'h8007;
 9'd203:decode_presentation_title=16'h800b;
 9'd204:decode_presentation_title=16'h802f;
 9'd205:decode_presentation_title=16'h8050;
 9'd206:decode_presentation_title=16'h80ab;
 9'd207:decode_presentation_title=16'h80b4;
 9'd208:decode_presentation_title=16'h80b8;
 9'd209:decode_presentation_title=16'h81e0;
 9'd210:decode_presentation_title=16'h81e1;
 9'd211:decode_presentation_title=16'h82d0;
 9'd212:decode_presentation_title=16'h82ff;
 9'd213:decode_presentation_title=16'h83db;
 9'd214:decode_presentation_title=16'h83fa;
 9'd215:decode_presentation_title=16'h8780;
 9'd216:decode_presentation_title=16'h9000;
 9'd217:decode_presentation_title=16'h9079;
 9'd218:decode_presentation_title=16'h9400;
 9'd219:decode_presentation_title=16'h9403;
 9'd220:decode_presentation_title=16'h9500;
 9'd221:decode_presentation_title=16'h952f;
 9'd222:decode_presentation_title=16'h96d0;
 9'd223:decode_presentation_title=16'h9b82;
 9'd224:decode_presentation_title=16'ha001;
 9'd225:decode_presentation_title=16'ha400;
 9'd226:decode_presentation_title=16'ha800;
 9'd227:decode_presentation_title=16'ha838;
 9'd228:decode_presentation_title=16'haa00;
 9'd229:decode_presentation_title=16'haa69;
 9'd230:decode_presentation_title=16'haae2;
 9'd231:decode_presentation_title=16'haae4;
 9'd232:decode_presentation_title=16'haaf4;
 9'd233:decode_presentation_title=16'habd0;
 9'd234:decode_presentation_title=16'habe0;
 9'd235:decode_presentation_title=16'habff;
 9'd236:decode_presentation_title=16'had00;
 9'd237:decode_presentation_title=16'haf80;
 9'd238:decode_presentation_title=16'hafcb;
 9'd239:decode_presentation_title=16'hafea;
 9'd240:decode_presentation_title=16'hafff;
 9'd241:decode_presentation_title=16'hb400;
 9'd242:decode_presentation_title=16'hb402;
 9'd243:decode_presentation_title=16'hb407;
 9'd244:decode_presentation_title=16'hb40b;
 9'd245:decode_presentation_title=16'hb800;
 9'd246:decode_presentation_title=16'hb82f;
 9'd247:decode_presentation_title=16'hb85b;
 9'd248:decode_presentation_title=16'hb95b;
 9'd249:decode_presentation_title=16'hb969;
 9'd250:decode_presentation_title=16'hbaab;
 9'd251:decode_presentation_title=16'hbc3d;
 9'd252:decode_presentation_title=16'hbd00;
 9'd253:decode_presentation_title=16'hbd06;
 9'd254:decode_presentation_title=16'hbd3d;
 9'd255:decode_presentation_title=16'hbd6a;
 9'd256:decode_presentation_title=16'hbdb8;
 9'd257:decode_presentation_title=16'hbf40;
 9'd258:decode_presentation_title=16'hbf80;
 9'd259:decode_presentation_title=16'hbfd0;
 9'd260:decode_presentation_title=16'hbffa;
 9'd261:decode_presentation_title=16'hbffd;
 9'd262:decode_presentation_title=16'hbfff;
 9'd263:decode_presentation_title=16'hc000;
 9'd264:decode_presentation_title=16'hc002;
 9'd265:decode_presentation_title=16'hc081;
 9'd266:decode_presentation_title=16'hc0f0;
 9'd267:decode_presentation_title=16'hc1f9;
 9'd268:decode_presentation_title=16'hd000;
 9'd269:decode_presentation_title=16'hd006;
 9'd270:decode_presentation_title=16'hd00b;
 9'd271:decode_presentation_title=16'hd00f;
 9'd272:decode_presentation_title=16'hd01f;
 9'd273:decode_presentation_title=16'hd02f;
 9'd274:decode_presentation_title=16'hd078;
 9'd275:decode_presentation_title=16'hd147;
 9'd276:decode_presentation_title=16'hd149;
 9'd277:decode_presentation_title=16'hd6f8;
 9'd278:decode_presentation_title=16'hd7a5;
 9'd279:decode_presentation_title=16'hdbc0;
 9'd280:decode_presentation_title=16'hdbfa;
 9'd281:decode_presentation_title=16'he000;
 9'd282:decode_presentation_title=16'he007;
 9'd283:decode_presentation_title=16'he00b;
 9'd284:decode_presentation_title=16'he078;
 9'd285:decode_presentation_title=16'he1e1;
 9'd286:decode_presentation_title=16'he2c0;
 9'd287:decode_presentation_title=16'he343;
 9'd288:decode_presentation_title=16'he400;
 9'd289:decode_presentation_title=16'he7e0;
 9'd290:decode_presentation_title=16'he96e;
 9'd291:decode_presentation_title=16'heaa5;
 9'd292:decode_presentation_title=16'head0;
 9'd293:decode_presentation_title=16'heae5;
 9'd294:decode_presentation_title=16'hebd2;
 9'd295:decode_presentation_title=16'hf000;
 9'd296:decode_presentation_title=16'hf001;
 9'd297:decode_presentation_title=16'hf00b;
 9'd298:decode_presentation_title=16'hf400;
 9'd299:decode_presentation_title=16'hf407;
 9'd300:decode_presentation_title=16'hf40b;
 9'd301:decode_presentation_title=16'hf42d;
 9'd302:decode_presentation_title=16'hf42f;
 9'd303:decode_presentation_title=16'hf458;
 9'd304:decode_presentation_title=16'hf478;
 9'd305:decode_presentation_title=16'hf47f;
 9'd306:decode_presentation_title=16'hf5e7;
 9'd307:decode_presentation_title=16'hf5fb;
 9'd308:decode_presentation_title=16'hf5ff;
 9'd309:decode_presentation_title=16'hf7d0;
 9'd310:decode_presentation_title=16'hf800;
 9'd311:decode_presentation_title=16'hf801;
 9'd312:decode_presentation_title=16'hf900;
 9'd313:decode_presentation_title=16'hf907;
 9'd314:decode_presentation_title=16'hfa7f;
 9'd315:decode_presentation_title=16'hfabf;
 9'd316:decode_presentation_title=16'hfae0;
 9'd317:decode_presentation_title=16'hfafa;
 9'd318:decode_presentation_title=16'hfaff;
 9'd319:decode_presentation_title=16'hfbff;
 9'd320:decode_presentation_title=16'hfd00;
 9'd321:decode_presentation_title=16'hfd7d;
 9'd322:decode_presentation_title=16'hfdb8;
 9'd323:decode_presentation_title=16'hfe00;
 9'd324:decode_presentation_title=16'hfe2d;
 9'd325:decode_presentation_title=16'hfe3d;
 9'd326:decode_presentation_title=16'hfe93;
 9'd327:decode_presentation_title=16'hfebf;
 9'd328:decode_presentation_title=16'hfee7;
 9'd329:decode_presentation_title=16'hfef9;
 9'd330:decode_presentation_title=16'hff00;
 9'd331:decode_presentation_title=16'hff80;
 9'd332:decode_presentation_title=16'hffa9;
 9'd333:decode_presentation_title=16'hffab;
 9'd334:decode_presentation_title=16'hffd0;
 9'd335:decode_presentation_title=16'hffd7;
 9'd336:decode_presentation_title=16'hffdb;
 9'd337:decode_presentation_title=16'hffe0;
 9'd338:decode_presentation_title=16'hffe1;
 9'd339:decode_presentation_title=16'hffe5;
 9'd340:decode_presentation_title=16'hfff0;
 9'd341:decode_presentation_title=16'hfffa;
 9'd342:decode_presentation_title=16'hfffb;
 9'd343:decode_presentation_title=16'hfffd;
 9'd344:decode_presentation_title=16'hfffe;
 9'd345:decode_presentation_title=16'hffff;
 default:decode_presentation_title=16'd0;endcase end endfunction
function [15:0] decode_presentation_body;input [8:0] code;begin case(code)
 9'd0:decode_presentation_body=16'h0;
 9'd1:decode_presentation_body=16'h1;
 9'd2:decode_presentation_body=16'h2;
 9'd3:decode_presentation_body=16'h3;
 9'd4:decode_presentation_body=16'h5;
 9'd5:decode_presentation_body=16'h7;
 9'd6:decode_presentation_body=16'ha;
 9'd7:decode_presentation_body=16'hb;
 9'd8:decode_presentation_body=16'hd;
 9'd9:decode_presentation_body=16'he;
 9'd10:decode_presentation_body=16'hf;
 9'd11:decode_presentation_body=16'h10;
 9'd12:decode_presentation_body=16'h11;
 9'd13:decode_presentation_body=16'h18;
 9'd14:decode_presentation_body=16'h1a;
 9'd15:decode_presentation_body=16'h1b;
 9'd16:decode_presentation_body=16'h1c;
 9'd17:decode_presentation_body=16'h1d;
 9'd18:decode_presentation_body=16'h1e;
 9'd19:decode_presentation_body=16'h1f;
 9'd20:decode_presentation_body=16'h24;
 9'd21:decode_presentation_body=16'h28;
 9'd22:decode_presentation_body=16'h2b;
 9'd23:decode_presentation_body=16'h2c;
 9'd24:decode_presentation_body=16'h2d;
 9'd25:decode_presentation_body=16'h2e;
 9'd26:decode_presentation_body=16'h2f;
 9'd27:decode_presentation_body=16'h38;
 9'd28:decode_presentation_body=16'h39;
 9'd29:decode_presentation_body=16'h3a;
 9'd30:decode_presentation_body=16'h3e;
 9'd31:decode_presentation_body=16'h3f;
 9'd32:decode_presentation_body=16'h40;
 9'd33:decode_presentation_body=16'h50;
 9'd34:decode_presentation_body=16'h55;
 9'd35:decode_presentation_body=16'h60;
 9'd36:decode_presentation_body=16'h6a;
 9'd37:decode_presentation_body=16'h6f;
 9'd38:decode_presentation_body=16'h7a;
 9'd39:decode_presentation_body=16'h7c;
 9'd40:decode_presentation_body=16'h7d;
 9'd41:decode_presentation_body=16'h7e;
 9'd42:decode_presentation_body=16'h7f;
 9'd43:decode_presentation_body=16'h9e;
 9'd44:decode_presentation_body=16'ha0;
 9'd45:decode_presentation_body=16'ha1;
 9'd46:decode_presentation_body=16'ha6;
 9'd47:decode_presentation_body=16'ha7;
 9'd48:decode_presentation_body=16'haa;
 9'd49:decode_presentation_body=16'hab;
 9'd50:decode_presentation_body=16'hb0;
 9'd51:decode_presentation_body=16'hb4;
 9'd52:decode_presentation_body=16'hba;
 9'd53:decode_presentation_body=16'hbb;
 9'd54:decode_presentation_body=16'hbc;
 9'd55:decode_presentation_body=16'hbd;
 9'd56:decode_presentation_body=16'hbe;
 9'd57:decode_presentation_body=16'hbf;
 9'd58:decode_presentation_body=16'he0;
 9'd59:decode_presentation_body=16'hf4;
 9'd60:decode_presentation_body=16'hf8;
 9'd61:decode_presentation_body=16'hfc;
 9'd62:decode_presentation_body=16'hfd;
 9'd63:decode_presentation_body=16'hfe;
 9'd64:decode_presentation_body=16'hff;
 9'd65:decode_presentation_body=16'h100;
 9'd66:decode_presentation_body=16'h103;
 9'd67:decode_presentation_body=16'h104;
 9'd68:decode_presentation_body=16'h107;
 9'd69:decode_presentation_body=16'h154;
 9'd70:decode_presentation_body=16'h155;
 9'd71:decode_presentation_body=16'h16e;
 9'd72:decode_presentation_body=16'h17e;
 9'd73:decode_presentation_body=16'h180;
 9'd74:decode_presentation_body=16'h1aa;
 9'd75:decode_presentation_body=16'h1b4;
 9'd76:decode_presentation_body=16'h1ea;
 9'd77:decode_presentation_body=16'h1f8;
 9'd78:decode_presentation_body=16'h1fd;
 9'd79:decode_presentation_body=16'h1fe;
 9'd80:decode_presentation_body=16'h1ff;
 9'd81:decode_presentation_body=16'h200;
 9'd82:decode_presentation_body=16'h240;
 9'd83:decode_presentation_body=16'h24e;
 9'd84:decode_presentation_body=16'h24f;
 9'd85:decode_presentation_body=16'h2d0;
 9'd86:decode_presentation_body=16'h2de;
 9'd87:decode_presentation_body=16'h2df;
 9'd88:decode_presentation_body=16'h2eb;
 9'd89:decode_presentation_body=16'h2ed;
 9'd90:decode_presentation_body=16'h2ef;
 9'd91:decode_presentation_body=16'h2fb;
 9'd92:decode_presentation_body=16'h2ff;
 9'd93:decode_presentation_body=16'h34e;
 9'd94:decode_presentation_body=16'h39d;
 9'd95:decode_presentation_body=16'h3ae;
 9'd96:decode_presentation_body=16'h3b5;
 9'd97:decode_presentation_body=16'h3d0;
 9'd98:decode_presentation_body=16'h3dd;
 9'd99:decode_presentation_body=16'h3ed;
 9'd100:decode_presentation_body=16'h3ee;
 9'd101:decode_presentation_body=16'h3fd;
 9'd102:decode_presentation_body=16'h3ff;
 9'd103:decode_presentation_body=16'h400;
 9'd104:decode_presentation_body=16'h47d;
 9'd105:decode_presentation_body=16'h506;
 9'd106:decode_presentation_body=16'h51c;
 9'd107:decode_presentation_body=16'h554;
 9'd108:decode_presentation_body=16'h55b;
 9'd109:decode_presentation_body=16'h55e;
 9'd110:decode_presentation_body=16'h590;
 9'd111:decode_presentation_body=16'h5ab;
 9'd112:decode_presentation_body=16'h5ba;
 9'd113:decode_presentation_body=16'h66d;
 9'd114:decode_presentation_body=16'h680;
 9'd115:decode_presentation_body=16'h697;
 9'd116:decode_presentation_body=16'h6be;
 9'd117:decode_presentation_body=16'h6bf;
 9'd118:decode_presentation_body=16'h6ef;
 9'd119:decode_presentation_body=16'h707;
 9'd120:decode_presentation_body=16'h740;
 9'd121:decode_presentation_body=16'h747;
 9'd122:decode_presentation_body=16'h75f;
 9'd123:decode_presentation_body=16'h79a;
 9'd124:decode_presentation_body=16'h79b;
 9'd125:decode_presentation_body=16'h7ae;
 9'd126:decode_presentation_body=16'h7d0;
 9'd127:decode_presentation_body=16'h7eb;
 9'd128:decode_presentation_body=16'h7ed;
 9'd129:decode_presentation_body=16'h7fa;
 9'd130:decode_presentation_body=16'h7fe;
 9'd131:decode_presentation_body=16'h7ff;
 9'd132:decode_presentation_body=16'h800;
 9'd133:decode_presentation_body=16'h97a;
 9'd134:decode_presentation_body=16'ha1f;
 9'd135:decode_presentation_body=16'haa5;
 9'd136:decode_presentation_body=16'hb00;
 9'd137:decode_presentation_body=16'hb0f;
 9'd138:decode_presentation_body=16'hb1a;
 9'd139:decode_presentation_body=16'hb1c;
 9'd140:decode_presentation_body=16'hb1e;
 9'd141:decode_presentation_body=16'hb2f;
 9'd142:decode_presentation_body=16'hb90;
 9'd143:decode_presentation_body=16'hbab;
 9'd144:decode_presentation_body=16'hbae;
 9'd145:decode_presentation_body=16'hbaf;
 9'd146:decode_presentation_body=16'hbfb;
 9'd147:decode_presentation_body=16'hbfd;
 9'd148:decode_presentation_body=16'hbfe;
 9'd149:decode_presentation_body=16'hbff;
 9'd150:decode_presentation_body=16'hd0d;
 9'd151:decode_presentation_body=16'hd4e;
 9'd152:decode_presentation_body=16'he00;
 9'd153:decode_presentation_body=16'heb7;
 9'd154:decode_presentation_body=16'heff;
 9'd155:decode_presentation_body=16'hf5c;
 9'd156:decode_presentation_body=16'hfad;
 9'd157:decode_presentation_body=16'hfd7;
 9'd158:decode_presentation_body=16'h1000;
 9'd159:decode_presentation_body=16'h14e0;
 9'd160:decode_presentation_body=16'h151f;
 9'd161:decode_presentation_body=16'h1540;
 9'd162:decode_presentation_body=16'h1800;
 9'd163:decode_presentation_body=16'h1aad;
 9'd164:decode_presentation_body=16'h1c00;
 9'd165:decode_presentation_body=16'h1d00;
 9'd166:decode_presentation_body=16'h1e00;
 9'd167:decode_presentation_body=16'h1ee5;
 9'd168:decode_presentation_body=16'h1f40;
 9'd169:decode_presentation_body=16'h1fc0;
 9'd170:decode_presentation_body=16'h1fee;
 9'd171:decode_presentation_body=16'h1ff0;
 9'd172:decode_presentation_body=16'h1ffd;
 9'd173:decode_presentation_body=16'h1fff;
 9'd174:decode_presentation_body=16'h2800;
 9'd175:decode_presentation_body=16'h29d0;
 9'd176:decode_presentation_body=16'h2bff;
 9'd177:decode_presentation_body=16'h2c00;
 9'd178:decode_presentation_body=16'h2d00;
 9'd179:decode_presentation_body=16'h2d50;
 9'd180:decode_presentation_body=16'h2e00;
 9'd181:decode_presentation_body=16'h2ee0;
 9'd182:decode_presentation_body=16'h2f00;
 9'd183:decode_presentation_body=16'h2f40;
 9'd184:decode_presentation_body=16'h2f6e;
 9'd185:decode_presentation_body=16'h2ff0;
 9'd186:decode_presentation_body=16'h3400;
 9'd187:decode_presentation_body=16'h3800;
 9'd188:decode_presentation_body=16'h38e0;
 9'd189:decode_presentation_body=16'h3b40;
 9'd190:decode_presentation_body=16'h3c00;
 9'd191:decode_presentation_body=16'h3fe0;
 9'd192:decode_presentation_body=16'h4000;
 9'd193:decode_presentation_body=16'h4040;
 9'd194:decode_presentation_body=16'h41f4;
 9'd195:decode_presentation_body=16'h4b40;
 9'd196:decode_presentation_body=16'h5000;
 9'd197:decode_presentation_body=16'h5400;
 9'd198:decode_presentation_body=16'h5410;
 9'd199:decode_presentation_body=16'h5500;
 9'd200:decode_presentation_body=16'h55d0;
 9'd201:decode_presentation_body=16'h5900;
 9'd202:decode_presentation_body=16'h5d00;
 9'd203:decode_presentation_body=16'h5e00;
 9'd204:decode_presentation_body=16'h6180;
 9'd205:decode_presentation_body=16'h6b54;
 9'd206:decode_presentation_body=16'h6d00;
 9'd207:decode_presentation_body=16'h6f80;
 9'd208:decode_presentation_body=16'h7800;
 9'd209:decode_presentation_body=16'h79a0;
 9'd210:decode_presentation_body=16'h79d0;
 9'd211:decode_presentation_body=16'h79e0;
 9'd212:decode_presentation_body=16'h7ad0;
 9'd213:decode_presentation_body=16'h7d00;
 9'd214:decode_presentation_body=16'h7e00;
 9'd215:decode_presentation_body=16'h7ed0;
 9'd216:decode_presentation_body=16'h7ee0;
 9'd217:decode_presentation_body=16'h7fc0;
 9'd218:decode_presentation_body=16'h7fd0;
 9'd219:decode_presentation_body=16'h8000;
 9'd220:decode_presentation_body=16'h81c0;
 9'd221:decode_presentation_body=16'h82c0;
 9'd222:decode_presentation_body=16'h8e00;
 9'd223:decode_presentation_body=16'h9780;
 9'd224:decode_presentation_body=16'h9e00;
 9'd225:decode_presentation_body=16'ha000;
 9'd226:decode_presentation_body=16'ha100;
 9'd227:decode_presentation_body=16'ha540;
 9'd228:decode_presentation_body=16'ha7d0;
 9'd229:decode_presentation_body=16'haae0;
 9'd230:decode_presentation_body=16'hab40;
 9'd231:decode_presentation_body=16'habd0;
 9'd232:decode_presentation_body=16'had00;
 9'd233:decode_presentation_body=16'hae00;
 9'd234:decode_presentation_body=16'haff0;
 9'd235:decode_presentation_body=16'hb000;
 9'd236:decode_presentation_body=16'hb2d0;
 9'd237:decode_presentation_body=16'hb400;
 9'd238:decode_presentation_body=16'hb6c0;
 9'd239:decode_presentation_body=16'hb800;
 9'd240:decode_presentation_body=16'hbad0;
 9'd241:decode_presentation_body=16'hbb40;
 9'd242:decode_presentation_body=16'hbbc0;
 9'd243:decode_presentation_body=16'hbc00;
 9'd244:decode_presentation_body=16'hbc28;
 9'd245:decode_presentation_body=16'hbc70;
 9'd246:decode_presentation_body=16'hbda0;
 9'd247:decode_presentation_body=16'hbe00;
 9'd248:decode_presentation_body=16'hbf00;
 9'd249:decode_presentation_body=16'hbf40;
 9'd250:decode_presentation_body=16'hbf80;
 9'd251:decode_presentation_body=16'hbfd0;
 9'd252:decode_presentation_body=16'hc000;
 9'd253:decode_presentation_body=16'hc100;
 9'd254:decode_presentation_body=16'hd000;
 9'd255:decode_presentation_body=16'hd010;
 9'd256:decode_presentation_body=16'hd1d0;
 9'd257:decode_presentation_body=16'hd700;
 9'd258:decode_presentation_body=16'hd740;
 9'd259:decode_presentation_body=16'hde00;
 9'd260:decode_presentation_body=16'hde74;
 9'd261:decode_presentation_body=16'hdf40;
 9'd262:decode_presentation_body=16'he000;
 9'd263:decode_presentation_body=16'he0b0;
 9'd264:decode_presentation_body=16'he1c0;
 9'd265:decode_presentation_body=16'he354;
 9'd266:decode_presentation_body=16'he6d0;
 9'd267:decode_presentation_body=16'he740;
 9'd268:decode_presentation_body=16'he7a0;
 9'd269:decode_presentation_body=16'he7b8;
 9'd270:decode_presentation_body=16'he7c0;
 9'd271:decode_presentation_body=16'he950;
 9'd272:decode_presentation_body=16'hea00;
 9'd273:decode_presentation_body=16'hea40;
 9'd274:decode_presentation_body=16'head0;
 9'd275:decode_presentation_body=16'heb40;
 9'd276:decode_presentation_body=16'hebe0;
 9'd277:decode_presentation_body=16'hed10;
 9'd278:decode_presentation_body=16'hee00;
 9'd279:decode_presentation_body=16'heec0;
 9'd280:decode_presentation_body=16'heed0;
 9'd281:decode_presentation_body=16'hef00;
 9'd282:decode_presentation_body=16'hef40;
 9'd283:decode_presentation_body=16'hefe0;
 9'd284:decode_presentation_body=16'hf000;
 9'd285:decode_presentation_body=16'hf1d0;
 9'd286:decode_presentation_body=16'hf400;
 9'd287:decode_presentation_body=16'hf4d0;
 9'd288:decode_presentation_body=16'hf500;
 9'd289:decode_presentation_body=16'hf800;
 9'd290:decode_presentation_body=16'hf950;
 9'd291:decode_presentation_body=16'hf9d0;
 9'd292:decode_presentation_body=16'hf9e0;
 9'd293:decode_presentation_body=16'hfaa0;
 9'd294:decode_presentation_body=16'hfad0;
 9'd295:decode_presentation_body=16'hfb40;
 9'd296:decode_presentation_body=16'hfb80;
 9'd297:decode_presentation_body=16'hfbc0;
 9'd298:decode_presentation_body=16'hfbd0;
 9'd299:decode_presentation_body=16'hfc00;
 9'd300:decode_presentation_body=16'hfd00;
 9'd301:decode_presentation_body=16'hfde0;
 9'd302:decode_presentation_body=16'hfde4;
 9'd303:decode_presentation_body=16'hfe00;
 9'd304:decode_presentation_body=16'hfe40;
 9'd305:decode_presentation_body=16'hfea4;
 9'd306:decode_presentation_body=16'hfed0;
 9'd307:decode_presentation_body=16'hff00;
 9'd308:decode_presentation_body=16'hff40;
 9'd309:decode_presentation_body=16'hff80;
 9'd310:decode_presentation_body=16'hffa0;
 9'd311:decode_presentation_body=16'hffc0;
 9'd312:decode_presentation_body=16'hffd0;
 9'd313:decode_presentation_body=16'hffe0;
 9'd314:decode_presentation_body=16'hfff0;
 9'd315:decode_presentation_body=16'hfff4;
 default:decode_presentation_body=16'd0;endcase end endfunction
 wire [15:0] wa=INIT_FILE=="presentation_title.mif" ? decode_presentation_title(qa):decode_presentation_body(qa);
 wire [15:0] wb=INIT_FILE=="presentation_title.mif" ? decode_presentation_title(qb):decode_presentation_body(qb);
 assign pixel=wa[lo_a*2+:2];assign pixel_b=wb[lo_b*2+:2];
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(9),.DATA_WIDTH_B(9),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
 .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE(MEMORY_FILE),.FILL_ALL("NONE")) indices(
 .dia(9'd0),.addra(addr[12:3]),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(qa),
 .dib(9'd0),.addrb(addr_b[12:3]),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(qb));
endmodule

// Only the three current read-A-only WIDTH2 callers use this wrapper.
// Four consecutive coverage values share one8bit word. Registering the
// low address bits preserves the original single-clock pixel latency.
module presentation_packed_rom #(parameter WIDTH=2,parameter INIT_FILE="presentation_title.mif")(
 input clk,input [12:0] addr,addr_b,output [WIDTH-1:0] pixel,pixel_b);
 localparam MEMORY_FILE=INIT_FILE=="presentation_title.mif"?"presentation_title_packed.mif":
  INIT_FILE=="presentation_body.mif"?"presentation_body_packed.mif":
  INIT_FILE=="osd_latin_aa.mif"?"osd_latin_aa_packed.mif":
  INIT_FILE=="presentation_title.dat"?"presentation_title_packed.dat":
  INIT_FILE=="presentation_body.dat"?"presentation_body_packed.dat":
  INIT_FILE=="osd_latin_aa.dat"?"osd_latin_aa_packed.dat":"UNSUPPORTED_PACKED_ASSET";
 wire [7:0] word;reg [1:0] low;
 always @(posedge clk)low<=addr[1:0];
 assign pixel=word[{low,1'b0}+:2];assign pixel_b=0;
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(8),.DATA_WIDTH_B(8),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
  .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),
  .INIT_FILE(MEMORY_FILE),.FILL_ALL("NONE")) memory(
  .dia(8'd0),.addra(addr[12:2]),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(word),
  .dib(8'd0),.addrb(11'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob());
endmodule

// Each word has four title pixels below four body pixels. PortA reads the
// title byte and portB reads the body byte at independent word addresses.
module presentation_title_body_rom #(parameter INIT_FILE="presentation_title_body_packed.mif")(
 input clk,input [12:0] title_addr,body_addr,output [1:0] title_pixel,body_pixel);
 wire [15:0] title_word,body_word;reg [1:0] title_low,body_low;
 always @(posedge clk)begin title_low<=title_addr[1:0];body_low<=body_addr[1:0];end
 assign title_pixel=title_word[{title_low,1'b0}+:2];
 assign body_pixel=body_word[{1'b1,body_low,1'b0}+:2];
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),
  .DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),
  .INIT_FILE(INIT_FILE),.FILL_ALL("NONE")) memory(
  .dia(16'd0),.addra(title_addr[12:2]),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(title_word),
  .dib(16'd0),.addrb(body_addr[12:2]),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(body_word));
endmodule
