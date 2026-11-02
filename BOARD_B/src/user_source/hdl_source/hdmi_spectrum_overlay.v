// Compact current-frame 100..8000Hz colored mountains with rounded tips.
// No rectangular background or white contour. Width is 64 bins x 4 pixels.
module hdmi_spectrum_overlay #(
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
    wire [12:0] tapered_curve = curve * taper;
    wire [10:0] upper_extent = (tapered_curve >> 1) + ({6'd0,taper} << 1) + taper;
    wire [10:0] lower_extent = (tapered_curve >> 2) + ({6'd0,taper} << 1);
    wire [12:0] crest=BASE*16-upper_extent;
    wire [12:0] mirror=BASE*16+lower_extent;
    wire [12:0] pixel_y={py[8:0],4'b0000};
    wire filled=frame_valid && pixel_y>=crest && pixel_y<=mirror;
    wire [12:0] inside_top=pixel_y-crest, inside_bottom=mirror-pixel_y;
    wire [2:0] glow=!filled?3'd0:
        (inside_top<16 || inside_bottom<16 || tip_distance<3)?3'd1:
        (py>BASE ? 3'd3 : 3'd6);
    // Continuous cyan/blue/pink/gold palette over the 256-pixel width.
    reg [7:0] r,gr,bl;
    always @* begin
        case(off[7:6])
            0:begin r=32+off[5:0];gr=208-off[5:0];bl=240;end
            1:begin r=96+{off[5:0],1'b0};gr=144-(off[5:0]>>1);bl=240-off[5:0];end
            2:begin r=224+(off[5:0]>>2);gr=112+(off[5:0]>>1);bl=176-off[5:0];end
            3:begin r=240;gr=144+off[5:0];bl=112-off[5:0];end
        endcase
    end
    function [7:0] shade;
        input [7:0] color,bg;
        input [2:0] alpha;
        reg signed [8:0] delta;
        reg signed [10:0] triple;
        reg signed [11:0] weighted,total;
        begin
            // Only alpha 0,1,3,6 are generated. Signed differences avoid
            // duplicating the background and color adders for each weight.
            delta=$signed({1'b0,color})-$signed({1'b0,bg});
            triple=($signed(delta) <<< 1)+$signed(delta);
            case(alpha)
                6:weighted=$signed(triple) <<< 1;
                3:weighted=$signed(triple);
                1:weighted=$signed(delta);
                default:weighted=0;
            endcase
            total=$signed({1'b0,bg,3'b000})+weighted;
            shade=total[10:3];
        end
    endfunction
    wire [23:0] painted={shade(r,I_rgb_bg[23:16],glow),shade(gr,I_rgb_bg[15:8],glow),shade(bl,I_rgb_bg[7:0],glow)};
    assign O_rgb=(SPECTRUM_DEBUG_BYPASS || !in_area || !frame_valid || glow==0)?I_rgb_bg:painted;
endmodule
