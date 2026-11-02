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
    input [9:0] x,y,
    output wire paint,hotspot_paint
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
            if (mode && armed && !key_next_n) begin
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

    wire title_pixel,body_pixel,icon_pixel;
    presentation_bitrom #(.INIT_FILE("presentation_title.mif")) u_title_rom (
        .clk(clk),.addr({y[4:0],title_cell[2:0],x[4:0]}),.pixel(title_pixel),
        .addr_b(13'd0),.pixel_b());
    presentation_bitrom #(.INIT_FILE("presentation_body.mif")) u_body_rom (
        .clk(clk),.addr({body2_area,y[3:0],body_cell[3:0],x[3:0]}),.pixel(body_pixel),
        .addr_b(13'd0),.pixel_b());
    wire arrow_pixel;
    presentation_bitrom #(.INIT_FILE("presentation_icons.mif")) u_icon_rom (
        .clk(clk),.addr({ICON_ID,y[4:0],x[4:0]}),.pixel(icon_pixel),
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
    assign paint=(title_d && title_pixel && fade_d) ||
                 (body_d && body_pixel) || (icon_d && icon_pixel);

    // A shared two-pixel border and optional leader are placed at each step.
    wire [11:0] hot_cfg=step==1 ? HOTSPOT1 : step==2 ? HOTSPOT2 :
                        step==3 ? HOTSPOT3 : HOTSPOT4;
    wire [4:0] hx=hot_cfg[11:7],hy=hot_cfg[6:2];
    wire [1:0] hot_style=hot_cfg[1:0];
    wire box=tx==hx && ty==hy &&
        (x[4:0]<2 || x[4:0]>=30 || y[4:0]<2 || y[4:0]>=30);
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

module presentation_bitrom #(parameter INIT_FILE="presentation_title.mif") (
    input clk,input [12:0] addr,addr_b,output pixel,pixel_b
);
    EG_LOGIC_BRAM #(.DATA_WIDTH_A(1),.DATA_WIDTH_B(1),
        .ADDR_WIDTH_A(13),.ADDR_WIDTH_B(13),
        .DATA_DEPTH_A(8192),.DATA_DEPTH_B(8192),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
        .RESETMODE("SYNC"),.IMPLEMENT("9K"),
        .INIT_FILE(INIT_FILE),.FILL_ALL("NONE")) u_rom (
        .dia(1'b0),.addra(addr),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),
        .rsta(1'b0),.bea(1'b0),.doa(pixel),
        .dib(1'b0),.addrb(addr_b),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(1'b0),
        .rstb(1'b0),.beb(1'b0),.dob(pixel_b));
endmodule
