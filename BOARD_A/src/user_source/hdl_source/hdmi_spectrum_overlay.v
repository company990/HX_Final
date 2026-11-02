// Compact current-frame 100..8000Hz colored mountains with rounded tips.
// No rectangular background or white contour. Width is 64 bins x 4 pixels.
module hdmi_spectrum_overlay #(
    parameter integer REGISTER_CURVE=0, RUNTIME_ENABLE=0,
    parameter integer INDEXED=0,
    parameter integer SPECTRUM_BIN_NUM     = 64,
    parameter integer SPECTRUM_X_START     = 368,
    parameter integer SPECTRUM_X_END       = 623,
    parameter integer SPECTRUM_Y_TOP       = 16,
    parameter integer SPECTRUM_Y_BOTTOM    = 111,
    parameter integer SPECTRUM_BAR_WIDTH   = 2,
    parameter integer SPECTRUM_BAR_GAP     = 0,
    parameter integer SPECTRUM_HEIGHT_SCALE= 2,    // 0..31 -> 像素(2=>max62)
    parameter integer SPECTRUM_DEBUG_BYPASS= 0,
    // ---- 视觉美化(参数均由 top 传入, 禁止子模块内部 `define) ----
    parameter integer SPECTRUM_COLOR_MODE  = 1,     // 1=列渐变(蓝->红)
    parameter integer SPECTRUM_FADE_ENABLE = 1,     // 1=顶部 4 级 fade 融入背景
    parameter integer SPECTRUM_FADE_PERCENT= 20,    // 文档用(近似 25%=px>>2)
    parameter integer SPECTRUM_FADE_LEVELS = 4,     // 4 级: 25/50/75/100%
    parameter integer SPECTRUM_SMOOTH_STEP = 1,
    parameter [23:0] SPECTRUM_COLOR0 = 24'h0080FF,  // blue
    parameter [23:0] SPECTRUM_COLOR1 = 24'h00FFFF,  // cyan
    parameter [23:0] SPECTRUM_COLOR2 = 24'h00FF40,  // green
    parameter [23:0] SPECTRUM_COLOR3 = 24'hFFFF00,  // yellow
    parameter [23:0] SPECTRUM_COLOR4 = 24'hFF8000,  // orange
    parameter [23:0] SPECTRUM_COLOR5 = 24'hFF2020   // red
)(
    input  wire                          I_clk,    // video_clk 25MHz
    input  wire                          I_rst,
    input  wire                          I_vs,
    input  wire                          I_de,
    input  wire [23:0]                   I_rgb_bg, // 原 BMP RGB(当前像素, fade 混入目标)
    input  wire [SPECTRUM_BIN_NUM*6-1:0] I_height, // 64x6 柱高 0..31(与 HUB75 共享)
    output wire [5:0] O_read_addr,
    input wire [5:0] I_read_height,
    output wire O_freeze,
    input wire I_enable,
    output wire O_vs,O_de,
    output wire [23:0]                   O_rgb
);

    // 256px wide. Coherent current-frame heights; no historical fill union.
    localparam X0=SPECTRUM_X_START, X1=SPECTRUM_X_END+1,
               BASE=SPECTRUM_Y_TOP+56, TOP=SPECTRUM_Y_TOP, BOTTOM=SPECTRUM_Y_BOTTOM+1;
    reg vs_d, de_d, seen;
    reg [9:0] x,y;
    wire frame_start = I_vs && !vs_d;
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin vs_d<=0; de_d<=0; seen<=0; x<=0; y<=0; end
        else begin
            vs_d<=I_vs; de_d<=I_de;
            if(frame_start) begin x<=0; y<=0; seen<=0; end
            else if(I_de) begin
                x<=x+1'b1;
                if(!de_d) begin
                    seen<=1;
                    if(seen) y<=y+1'b1;
                end
            end else x<=0;
        end
    end
    // y counter advances on first active clock; compensate before that edge.
    wire [9:0] py = y + ((I_de && !de_d && seen) ? 10'd1 : 10'd0);
    // One coherent video-frame snapshot. No historical maximum envelope:
    // old peaks must not fill the valleys of the current spectrum.
    wire [9:0] off=x-X0;
    wire [5:0] bin=off[7:2];
    wire [5:0] next_bin=bin==63 ? 6'd63 : bin+6'd1;
    wire [4:0] h0,h1;
    reg frame_valid;
    always @(posedge I_clk or posedge I_rst)
        if(I_rst) frame_valid<=0; else if(frame_start) frame_valid<=1;
    generate if(INDEXED) begin : indexed
        reg copying;
        reg [6:0] count;
        reg [5:0] copy_addr;
        reg copy_valid;
        reg initialized;
        reg [5:0] init_addr;
        // Freeze the source before its first snapshot read. The whole copy
        // completes in vertical blank, long before the visible spectrum area.
        assign O_freeze = copying || frame_start;
        assign O_read_addr = count[5:0];
        wire write_frame = !I_rst && (!initialized || copy_valid);
        wire [5:0] frame_addr = !initialized ? init_addr : copy_addr;
        wire [5:0] frame_data = !initialized ? 6'd0 : (I_read_height>31 ? 6'd23 : I_read_height>8 ? I_read_height-6'd8 : 6'd0);
        wire [9:0] next_x = (frame_start || !I_de) ? 10'd0 : x + 1'b1;
        wire [15:0] next_off = next_x - SPECTRUM_X_START;
        wire [5:0] next_band=next_off[7:2];
        wire [5:0] adjacent=next_band==63 ? 6'd63 : next_band+6'd1;
        wire [5:0] q0,q1;
        spectrum_ram64 frame_ram(I_clk,write_frame,frame_addr,frame_data,
            next_band,q0,adjacent,q1);
        assign h0=initialized ? q0[4:0] : 0;
        assign h1=initialized ? q1[4:0] : 0;
        always @(posedge I_clk or posedge I_rst) begin
            if (I_rst) begin
                copying<=0; count<=0; copy_addr<=0; copy_valid<=0;
                initialized<=0; init_addr<=0;
            end else begin
                if (!initialized) begin
                    if (init_addr==SPECTRUM_BIN_NUM-1) initialized<=1;
                    else init_addr<=init_addr+1'b1;
                end
                copy_valid<=0;
                if (frame_start) begin copying<=1; count<=0; end
                else if (copying) begin
                    if (count < SPECTRUM_BIN_NUM) begin
                        copy_valid<=1; copy_addr<=count[5:0]; count<=count+1'b1;
                    end else copying<=0;
                end
            end
        end
    end else begin : legacy
    assign O_read_addr=0; assign O_freeze=0;
    reg [319:0] frame_height;
    integer k;
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin frame_height<=0;  end
        else if(frame_start) begin
            for(k=0;k<64;k=k+1)
                frame_height[k*5 +: 5] <= I_height[k*6 +: 6]>31 ? 5'd23 :
                    I_height[k*6 +: 6]>8 ? I_height[k*6 +: 6]-6'd8 : 5'd0;
        end
    end
    assign h0=frame_height[bin*5 +: 5];
    assign h1=frame_height[next_bin*5 +: 5];
    end endgenerate
    // Join the actual frequency samples without averaging across neighbours.
    // Fixed-point height (quarter units). Only the two OUTER tips are tapered.
    wire [7:0] a={3'd0,h0}, b={3'd0,h1};
    reg [7:0] curve;
    always @* begin
        case(off[1:0])
            0:curve=a<<2;
            1:curve=(a<<1)+a+b;
            2:curve=(a+b)<<1;
            3:curve=a+(b<<1)+b;
        endcase
    end
    wire in_area=I_de && x>=X0 && x<X1 && py>=TOP && py<BOTTOM;
    // Smooth symmetric taper over the outer 16 pixels. It applies to the
    // entire silhouette, including the quiet floor, rather than a hard crop.
    wire [7:0] tip_distance=off[7] ? ~off[7:0] : off[7:0];
    reg [4:0] taper;
    always @* begin
        if(tip_distance>=16) taper=16;
        else case(tip_distance[3:0])
            0:taper=0; 1:taper=2; 2:taper=3; 3:taper=5;
            4:taper=6; 5:taper=8; 6:taper=9; 7:taper=10;
            8:taper=11; 9:taper=12; 10:taper=13; 11:taper=14;
            12:taper=15; 13:taper=15; default:taper=16;
        endcase
    end
    // Share the exact curve*taper product between upper and lower extents.
    // curve <= 124; all discarded bits match the original floor division.
    wire [12:0] tapered_curve;
    // One optional pixel stage. Spectrum RAM addressing and frame snapshot
    // continue on their original clock; all render operands travel together.
    wire [12:0] render_curve;
    wire [4:0] render_taper;
    wire [9:0] render_y;
    wire [7:0] render_off;
    wire [23:0] render_bg;
    wire render_area,render_valid,render_enable;
    generate if(REGISTER_CURVE)begin:curve_stage
        reg [12:0] product;
        reg [4:0] taper_q;
        reg [9:0] y_q;
        reg [7:0] off_q;
        reg [23:0] bg_q;
        reg area_q,valid_q,enable_q,vs_q,de_q;
        always @(posedge I_clk or posedge I_rst)begin
            if(I_rst)begin product<=0;taper_q<=0;y_q<=0;off_q<=0;bg_q<=0;
                area_q<=0;valid_q<=0;enable_q<=0;vs_q<=0;de_q<=0;end
            else begin product<=tapered_curve;taper_q<=taper;y_q<=py;off_q<=off[7:0];bg_q<=I_rgb_bg;
                area_q<=in_area;valid_q<=frame_valid;enable_q<=RUNTIME_ENABLE?I_enable:1'b1;vs_q<=I_vs;de_q<=I_de;end
        end
        assign render_curve=product;assign render_taper=taper_q;assign render_y=y_q;
        assign render_off=off_q;assign render_bg=bg_q;assign render_area=area_q;
        assign render_valid=valid_q;assign render_enable=enable_q;assign O_vs=vs_q;assign O_de=de_q;
    end else begin:curve_bypass
        assign render_curve=tapered_curve;assign render_taper=taper;assign render_y=py;
        assign render_off=off[7:0];assign render_bg=I_rgb_bg;assign render_area=in_area;
        assign render_valid=frame_valid;assign render_enable=RUNTIME_ENABLE?I_enable:1'b1;
        assign O_vs=I_vs;assign O_de=I_de;
    end endgenerate
    wire [7:0] render_tip=render_off[7] ? ~render_off:render_off;
    (* use_ripple=1 *) wire [10:0] upper_extent = (render_curve >> 1) + ({6'd0,render_taper} << 1) + render_taper;
    (* use_ripple=1 *) wire [10:0] lower_extent = (render_curve >> 2) + ({6'd0,render_taper} << 1);
    (* use_ripple=1 *) wire [12:0] crest=BASE*16-upper_extent;
    (* use_ripple=1 *) wire [12:0] mirror=BASE*16+lower_extent;
    wire [12:0] pixel_y={render_y[8:0],4'b0000};
    wire filled=render_valid && pixel_y>=crest && pixel_y<=mirror;
    (* use_ripple=1 *) wire [12:0] inside_top=pixel_y-crest, inside_bottom=mirror-pixel_y;
    wire [2:0] glow=!filled?3'd0:
        (inside_top<16 || inside_bottom<16 || render_tip<3)?3'd1:
        (render_y>BASE ? 3'd3 : 3'd6);
    // Continuous cyan/blue/pink/gold palette over the 256-pixel width.
    reg [7:0] r,gr,bl;
    always @* begin
        case(render_off[7:6])
            0:begin r=32+render_off[5:0];gr=208-render_off[5:0];bl=240;end
            1:begin r=96+{render_off[5:0],1'b0};gr=144-(render_off[5:0]>>1);bl=240-render_off[5:0];end
            2:begin r=224+(render_off[5:0]>>2);gr=112+(render_off[5:0]>>1);bl=176-render_off[5:0];end
            3:begin r=240;gr=144+render_off[5:0];bl=112-render_off[5:0];end
        endcase
    end
    wire [23:0] painted;
    // Both 9x9 products in each native DSP are independent. The live
    // curve product and the registered pixel's blue blend share a block;
    // no state, pixel latency or clock is introduced by that packing.
    spectrum_dual9_products products(curve,taper,{r,gr,bl},render_bg,glow,
        tapered_curve,painted);
    assign O_rgb=(!render_enable || SPECTRUM_DEBUG_BYPASS || !render_area || !render_valid || glow==0)?render_bg:painted;
endmodule

// Four independent signed 9x9 products, packed into two physical DSPs.
// Color deltas fit -255..255; alpha is 0,1,3,6. Curve/taper operands
// are explicitly zero extended, so signed mode preserves their full range.
module spectrum_dual9_products(input [7:0] curve,input [4:0] taper,
 input [23:0] color,background,input [2:0] alpha,
 output [12:0] curve_product,output [23:0] pixel);
 wire signed [8:0] dr=$signed({1'b0,color[23:16]})-$signed({1'b0,background[23:16]});
 wire signed [8:0] dg=$signed({1'b0,color[15:8]})-$signed({1'b0,background[15:8]});
 wire signed [8:0] db=$signed({1'b0,color[7:0]})-$signed({1'b0,background[7:0]});
 wire [8:0] weight=(alpha==1 || alpha==3 || alpha==6) ? {6'd0,alpha}:9'd0;
 wire [35:0] rg,bc;
 EG_PHY_MULT18 #(.INPUTREGA("DISABLE"),.INPUTREGB("DISABLE"),
  .OUTPUTREG("DISABLE"),.MODE("MULT9X9C"),.SOURCEAMUX("1"),.SOURCEBMUX("1")) red_green(
  .a({dg,dr}),.b({weight,weight}),.p(rg),.signeda(1'b1),.signedb(1'b1),
  .acin(),.bcin(),.acout(),.bcout(),.sourcea(1'b1),.sourceb(1'b1),
  .cea(1'b0),.ceb(1'b0),.cepd(1'b0),.clk(1'b0),.rstan(1'b1),.rstbn(1'b1),.rstpdn(1'b1));
 EG_PHY_MULT18 #(.INPUTREGA("DISABLE"),.INPUTREGB("DISABLE"),
  .OUTPUTREG("DISABLE"),.MODE("MULT9X9C"),.SOURCEAMUX("1"),.SOURCEBMUX("1")) blue_curve(
  .a({1'b0,curve,db}),.b({4'd0,taper,weight}),.p(bc),.signeda(1'b1),.signedb(1'b1),
  .acin(),.bcin(),.acout(),.bcout(),.sourcea(1'b1),.sourceb(1'b1),
  .cea(1'b0),.ceb(1'b0),.cepd(1'b0),.clk(1'b0),.rstan(1'b1),.rstbn(1'b1),.rstpdn(1'b1));
 wire [11:0] sr={1'b0,background[23:16],3'b000}+rg[11:0];
 wire [11:0] sg={1'b0,background[15:8],3'b000}+rg[29:18];
 wire [11:0] sb={1'b0,background[7:0],3'b000}+bc[11:0];
 assign pixel={sr[10:3],sg[10:3],sb[10:3]};
 assign curve_product=bc[30:18];
endmodule
