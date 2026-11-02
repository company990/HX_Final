`timescale 1ns/1ps

module osd_overlay_640x480 #(
    parameter FONT_BRAM = 0,
    parameter PRESENTATION_ENABLE = 0,
    parameter [2:0] PRESENTATION_ICON_ID = 3'd0,
    parameter [4:0] PRESENTATION_ICON_COL = 5'd15,
    parameter [4:0] PRESENTATION_ICON_ROW = 5'd7,
    parameter [11:0] PRESENTATION_HOTSPOT1={5'd1,5'd4,2'b00},
    parameter [11:0] PRESENTATION_HOTSPOT2={5'd1,5'd7,2'b01},
    parameter [11:0] PRESENTATION_HOTSPOT3={5'd13,5'd8,2'b11},
    parameter [11:0] PRESENTATION_HOTSPOT4={5'd15,5'd7,2'b11}
) (
    input             clk,
    input             rst,
    input             vs,
    input             de,
    input             presentation_mode,
    input             presentation_key_next_n,

    // ===================== G1-B2 dynamic status =====================
    input             auto_play,
    input      [1:0]  img_idx,
    input      [2:0]  img_count,
    
    // ===================== G1-C2 image adjustment OSD =====================
    input      [2:0]  brightness_level,
    input      [2:0]  contrast_level,
    
    input      [23:0] rgb_in,
    output     [23:0] rgb_out
);

    // =========================================================
    // 640x480 active-video coordinates
    // =========================================================
    reg [9:0] x_cnt;
    reg [9:0] y_cnt;
    reg       de_d;

    wire presentation_paint,presentation_hotspot;
    generate if (PRESENTATION_ENABLE) begin : presentation
        presentation_overlay #(
            .ICON_ID(PRESENTATION_ICON_ID),
            .ICON_COL(PRESENTATION_ICON_COL),
            .ICON_ROW(PRESENTATION_ICON_ROW),
            .HOTSPOT1(PRESENTATION_HOTSPOT1),
            .HOTSPOT2(PRESENTATION_HOTSPOT2),
            .HOTSPOT3(PRESENTATION_HOTSPOT3),
            .HOTSPOT4(PRESENTATION_HOTSPOT4)
        ) u_reveal (
            .clk(clk),.rst(rst),.mode(presentation_mode),
            .key_next_n(presentation_key_next_n),.vs(vs),.de(de),
            .x(x_cnt),.y(y_cnt),.paint(presentation_paint),
            .hotspot_paint(presentation_hotspot)
        );
    end else begin : no_presentation
        assign presentation_paint=1'b0;
        assign presentation_hotspot=1'b0;
    end endgenerate


    // =========================================================
    // G1-B3 frame-synchronous animation control
    // =========================================================
    reg  vs_d;
    wire frame_tick;

    assign frame_tick = vs && !vs_d;

    always @(posedge clk or posedge rst) begin
        if (rst)
            vs_d <= 1'b0;
        else
            vs_d <= vs;
    end


// =========================================================
// G1-B3 bilingual scrolling phase generator
//
// scroll_lang:
//   0 = English
//   1 = Chinese
//
// English:
//   ANLOGIC FPGA MULTIMEDIA PLAYER
//   30 cells * 12 px = 360 px
//   640 + 360 = 1000 px
//   phase = 0 ... 999
//
// Chinese:
//   安路多媒体播放系统
//   9 cells * 18 px = 162 px
//   640 + 162 = 802 px
//   phase = 0 ... 801
//
// Move 1 pixel every 2 video frames.
// =========================================================

reg [9:0] scroll_phase;
reg       scroll_div;
reg       scroll_lang;

localparam [9:0] ENG_PHASE_MAX = 10'd999;
localparam [9:0] CHN_PHASE_MAX = 10'd801;

wire [9:0] scroll_phase_max;

assign scroll_phase_max =
    scroll_lang ?
        CHN_PHASE_MAX :
        ENG_PHASE_MAX;


always @(posedge clk or posedge rst) begin

    if (rst) begin

        scroll_phase <= 10'd0;
        scroll_div   <= 1'b0;
        scroll_lang  <= 1'b0;

    end
    else if (frame_tick) begin

        if (scroll_div == 1'b0) begin

            scroll_div <= 1'b1;

        end
        else begin

            scroll_div <= 1'b0;

            // Current language has completely scrolled out.
            if (scroll_phase == scroll_phase_max) begin

                scroll_phase <= 10'd0;
                scroll_lang  <= ~scroll_lang;

            end
            else begin

                scroll_phase <= scroll_phase + 10'd1;

            end

        end

    end

end
    // =========================================================
    // Active-video coordinate counters
    // =========================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x_cnt <= 10'd0;
            y_cnt <= 10'd0;
            de_d  <= 1'b0;
        end
        else begin

            de_d <= de;

            // Horizontal active-pixel counter
            if (de) begin
                if (x_cnt != 10'd639)
                    x_cnt <= x_cnt + 10'd1;
                else
                    x_cnt <= 10'd639;
            end
            else begin
                x_cnt <= 10'd0;
            end

            // One active line finished
            if (de_d && !de) begin
                if (y_cnt == 10'd479)
                    y_cnt <= 10'd0;
                else
                    y_cnt <= y_cnt + 10'd1;
            end

        end
    end


    // =========================================================
    // G1-B2 dynamic OSD
    //
    // MODE: MANUAL
    // IMAGE: 1/4
    //
    // MODE: AUTO
    // IMAGE: 2/4
    //
    // Font : 5x7
    // Scale: 2x
    // Glyph: 10x14
    // Cell : 12 pixels wide
    // =========================================================

    localparam [9:0] OSD_X = 10'd32;
    localparam [9:0] OSD_Y = 10'd32;

    localparam [9:0] OSD_W = 10'd144;
    localparam [9:0] OSD_H = 10'd68;

    wire in_osd_x;
    wire in_osd_y;

    assign in_osd_x =
        (x_cnt >= OSD_X) &&
        (x_cnt <  OSD_X + OSD_W);

    assign in_osd_y =
        (y_cnt >= OSD_Y) &&
        (y_cnt <  OSD_Y + OSD_H);


    wire [9:0] rel_x;
    wire [9:0] rel_y;

    assign rel_x = x_cnt - OSD_X;
    assign rel_y = y_cnt - OSD_Y;


    // =========================================================
    // Decode 12 horizontal OSD character cells
    // =========================================================

    // Decode a 12-pixel cell for coordinates below 360.
    function [8:0] cell12;
        input [8:0] v;
        reg [4:0] q;
        reg [7:0] r1; reg [6:0] r2; reg [5:0] r3; reg [4:0] r4; reg [3:0] r5;
        begin
            q[4]=(v>=9'd192); r1=q[4] ? v-9'd192 : v;
            q[3]=(r1>=8'd96); r2=q[3] ? r1-8'd96 : r1;
            q[2]=(r2>=7'd48); r3=q[2] ? r2-7'd48 : r2;
            q[1]=(r3>=6'd24); r4=q[1] ? r3-6'd24 : r3;
            q[0]=(r4>=5'd12); r5=q[0] ? r4-5'd12 : r4;
            cell12={q,r5};
        end
    endfunction
    // The status box has only 12 cells, so its decoder needs four stages.
    function [7:0] cell12_osd;
        input [7:0] v;
        reg [3:0] q;
        reg [6:0] r1; reg [5:0] r2; reg [4:0] r3; reg [3:0] r4;
        begin
            q[3]=(v>=8'd96); r1=q[3] ? v-8'd96 : v;
            q[2]=(r1>=7'd48); r2=q[2] ? r1-7'd48 : r1;
            q[1]=(r2>=6'd24); r3=q[1] ? r2-6'd24 : r2;
            q[0]=(r3>=5'd12); r4=q[0] ? r3-5'd12 : r3;
            cell12_osd={q,r4};
        end
    endfunction
    // Decode an 18-pixel cell for coordinates below 162.
    function [8:0] cell18;
        input [7:0] v;
        reg [3:0] q;
        reg [7:0] r1; reg [6:0] r2; reg [5:0] r3; reg [4:0] r4;
        begin
            q[3]=(v>=8'd144); r1=q[3] ? v-8'd144 : v;
            q[2]=(r1>=8'd72); r2=q[2] ? r1-8'd72 : r1;
            q[1]=(r2>=7'd36); r3=q[1] ? r2-7'd36 : r2;
            q[0]=(r3>=6'd18); r4=q[0] ? r3-6'd18 : r3;
            cell18={q,r4};
        end
    endfunction
    reg [3:0] osd_char_index;
    reg [3:0] osd_char_pos_x;

    always @(*) begin
        osd_char_index = 4'd15;
        osd_char_pos_x = 0;
        if (rel_x < 10'd144)
            {osd_char_index,osd_char_pos_x} = cell12_osd(rel_x[7:0]);
    end

    // =========================================================
    // Decode four OSD text lines
    //
    // line 0 : MODE
    // y = 0  .. 13
    //
    // line 1 : IMAGE
    // y = 18 .. 31
    //
    // line 2 : BRI
    // y = 36 .. 49
    //
    // line 3 : CON
    // y = 54 .. 67
    // =========================================================
    
    reg [1:0] osd_text_line;
    reg [3:0] osd_char_pos_y;
    reg       osd_in_text_line;
    
    always @(*) begin
    
        osd_text_line    = 2'd0;
        osd_char_pos_y   = 4'd0;
        osd_in_text_line = 1'b0;
    
        if (rel_y < 10'd14) begin
    
            osd_text_line    = 2'd0;
            osd_char_pos_y   = rel_y[3:0];
            osd_in_text_line = 1'b1;
    
        end
        else if (
            (rel_y >= 10'd18) &&
            (rel_y <  10'd32)
        ) begin
    
            osd_text_line    = 2'd1;
            osd_char_pos_y   = rel_y - 10'd18;
            osd_in_text_line = 1'b1;
    
        end
        else if (
            (rel_y >= 10'd36) &&
            (rel_y <  10'd50)
        ) begin
    
            osd_text_line    = 2'd2;
            osd_char_pos_y   = rel_y - 10'd36;
            osd_in_text_line = 1'b1;
    
        end
        else if (
            (rel_y >= 10'd54) &&
            (rel_y <  10'd68)
        ) begin
    
            osd_text_line    = 2'd3;
            osd_char_pos_y   = rel_y - 10'd54;
            osd_in_text_line = 1'b1;
    
        end
    
    end
    
    

    wire [2:0] osd_char_x;
    wire [2:0] osd_char_y;
    wire       osd_in_char_x;

    assign osd_char_x =
        osd_char_pos_x[3:1];

    assign osd_char_y =
        osd_char_pos_y[3:1];

    assign osd_in_char_x =
        (osd_char_pos_x < 4'd10);


    // =========================================================
    // Character codes
    // =========================================================

    localparam [4:0] CH_SPACE = 5'd0;
    localparam [4:0] CH_M     = 5'd1;
    localparam [4:0] CH_O     = 5'd2;
    localparam [4:0] CH_D     = 5'd3;
    localparam [4:0] CH_E     = 5'd4;
    localparam [4:0] CH_COLON = 5'd5;
    localparam [4:0] CH_A     = 5'd6;
    localparam [4:0] CH_N     = 5'd7;
    localparam [4:0] CH_U     = 5'd8;
    localparam [4:0] CH_L     = 5'd9;
    localparam [4:0] CH_T     = 5'd10;
    localparam [4:0] CH_I     = 5'd11;
    localparam [4:0] CH_G     = 5'd12;
    localparam [4:0] CH_SLASH = 5'd13;
    localparam [4:0] CH_0     = 5'd14;
    localparam [4:0] CH_1     = 5'd15;
    localparam [4:0] CH_2     = 5'd16;
    localparam [4:0] CH_3     = 5'd17;
    localparam [4:0] CH_4     = 5'd18;

    // G1-B3 additional scrolling subtitle characters

    localparam [4:0] CH_C     = 5'd19;
    localparam [4:0] CH_F     = 5'd20;
    localparam [4:0] CH_P     = 5'd21;
    localparam [4:0] CH_Y     = 5'd22;
    localparam [4:0] CH_R     = 5'd23;
    // ===================== G1-C2 OSD characters =====================
    
    localparam [4:0] CH_5     = 5'd24;
    localparam [4:0] CH_6     = 5'd25;
    localparam [4:0] CH_7     = 5'd26;
    localparam [4:0] CH_B     = 5'd27;
    localparam [4:0] CH_PLUS  = 5'd28;
    localparam [4:0] CH_MINUS = 5'd29;
    localparam [4:0] CH_DOT   = 5'd30;
    localparam [4:0] CH_X     = 5'd31;
    // =========================================================
    // G1-B3 Chinese subtitle codes
    //
    // 16x16 Hanzi:
    // 安 路 多 媒 体 播 放 系 统
    // =========================================================
    
    localparam [3:0] HZ_AN   = 4'd0;   // 安
    localparam [3:0] HZ_LU   = 4'd1;   // 路
    localparam [3:0] HZ_DUO  = 4'd2;   // 多
    localparam [3:0] HZ_MEI  = 4'd3;   // 媒
    localparam [3:0] HZ_TI   = 4'd4;   // 体
    localparam [3:0] HZ_BO   = 4'd5;   // 播
    localparam [3:0] HZ_FANG = 4'd6;   // 放
    localparam [3:0] HZ_XI   = 4'd7;   // 系
    localparam [3:0] HZ_TONG = 4'd8;   // 统
    
    // =========================================================
    // Dynamic current-image digit
    // =========================================================

    reg [4:0] current_img_char;

    always @(*) begin
        case (img_idx)

            2'd0:
                current_img_char = CH_1;

            2'd1:
                current_img_char = CH_2;

            2'd2:
                current_img_char = CH_3;

            2'd3:
                current_img_char = CH_4;

            default:
                current_img_char = CH_1;

        endcase
    end


    // =========================================================
    // Dynamic image-count digit
    // =========================================================

    reg [4:0] img_count_char;

    always @(*) begin
        case (img_count)

            3'd0:
                img_count_char = CH_0;

            3'd1:
                img_count_char = CH_1;

            3'd2:
                img_count_char = CH_2;

            3'd3:
                img_count_char = CH_3;

            default:
                img_count_char = CH_4;

        endcase
    end
    // =========================================================
    // G1-C2 brightness display
    //
    // level 0 = -64
    // level 1 = -32
    // level 2 =   0
    // level 3 = +32
    // level 4 = +64
    // =========================================================
    
    reg [4:0] bri_sign_char;
    reg [4:0] bri_tens_char;
    reg [4:0] bri_ones_char;
    
    always @(*) begin
    
        bri_sign_char = CH_SPACE;
        bri_tens_char = CH_SPACE;
        bri_ones_char = CH_0;
    
        case (brightness_level)
    
            3'd0: begin
                bri_sign_char = CH_MINUS;
                bri_tens_char = CH_6;
                bri_ones_char = CH_4;
            end
    
            3'd1: begin
                bri_sign_char = CH_MINUS;
                bri_tens_char = CH_3;
                bri_ones_char = CH_2;
            end
    
            3'd2: begin
                bri_sign_char = CH_SPACE;
                bri_tens_char = CH_SPACE;
                bri_ones_char = CH_0;
            end
    
            3'd3: begin
                bri_sign_char = CH_PLUS;
                bri_tens_char = CH_3;
                bri_ones_char = CH_2;
            end
    
            3'd4: begin
                bri_sign_char = CH_PLUS;
                bri_tens_char = CH_6;
                bri_ones_char = CH_4;
            end
    
            default: begin
                bri_sign_char = CH_SPACE;
                bri_tens_char = CH_SPACE;
                bri_ones_char = CH_0;
            end
    
        endcase
    
    end
    // =========================================================
    // G1-C2 contrast display
    //
    // level 0 = 0.50X
    // level 1 = 0.75X
    // level 2 = 1.00X
    // level 3 = 1.25X
    // level 4 = 1.50X
    // =========================================================
    
    reg [4:0] con_int_char;
    reg [4:0] con_dec1_char;
    reg [4:0] con_dec2_char;
    
    always @(*) begin
    
        con_int_char  = CH_1;
        con_dec1_char = CH_0;
        con_dec2_char = CH_0;
    
        case (contrast_level)
    
            3'd0: begin
                con_int_char  = CH_0;
                con_dec1_char = CH_5;
                con_dec2_char = CH_0;
            end
    
            3'd1: begin
                con_int_char  = CH_0;
                con_dec1_char = CH_7;
                con_dec2_char = CH_5;
            end
    
            3'd2: begin
                con_int_char  = CH_1;
                con_dec1_char = CH_0;
                con_dec2_char = CH_0;
            end
    
            3'd3: begin
                con_int_char  = CH_1;
                con_dec1_char = CH_2;
                con_dec2_char = CH_5;
            end
    
            3'd4: begin
                con_int_char  = CH_1;
                con_dec1_char = CH_5;
                con_dec2_char = CH_0;
            end
    
            default: begin
                con_int_char  = CH_1;
                con_dec1_char = CH_0;
                con_dec2_char = CH_0;
            end
    
        endcase
    
    end
    // =========================================================
    // G1-B2 OSD character selection
    // =========================================================

    reg [4:0] osd_char_code;

    always @(*) begin
    
        osd_char_code = CH_SPACE;
    
        case (osd_text_line)
    
            // =====================================================
            // Line 0 : MODE: MANUAL / MODE: AUTO
            // =====================================================
            2'd0: begin
    
                case (osd_char_index)
    
                    4'd0:  osd_char_code = CH_M;
                    4'd1:  osd_char_code = CH_O;
                    4'd2:  osd_char_code = CH_D;
                    4'd3:  osd_char_code = CH_E;
                    4'd4:  osd_char_code = CH_COLON;
                    4'd5:  osd_char_code = CH_SPACE;
    
                    4'd6:
                        osd_char_code =
                            auto_play ? CH_A : CH_M;
    
                    4'd7:
                        osd_char_code =
                            auto_play ? CH_U : CH_A;
    
                    4'd8:
                        osd_char_code =
                            auto_play ? CH_T : CH_N;
    
                    4'd9:
                        osd_char_code =
                            auto_play ? CH_O : CH_U;
    
                    4'd10:
                        osd_char_code =
                            auto_play ? CH_SPACE : CH_A;
    
                    4'd11:
                        osd_char_code =
                            auto_play ? CH_SPACE : CH_L;
    
                    default:
                        osd_char_code = CH_SPACE;
    
                endcase
    
            end
    
    
            // =====================================================
            // Line 1 : IMAGE: 1/4
            // =====================================================
            2'd1: begin
    
                case (osd_char_index)
    
                    4'd0: osd_char_code = CH_I;
                    4'd1: osd_char_code = CH_M;
                    4'd2: osd_char_code = CH_A;
                    4'd3: osd_char_code = CH_G;
                    4'd4: osd_char_code = CH_E;
                    4'd5: osd_char_code = CH_COLON;
                    4'd6: osd_char_code = CH_SPACE;
                    4'd7: osd_char_code = current_img_char;
                    4'd8: osd_char_code = CH_SLASH;
                    4'd9: osd_char_code = img_count_char;
    
                    default:
                        osd_char_code = CH_SPACE;
    
                endcase
    
            end
    
    
            // =====================================================
            // Line 2 : BRI: +32
            // =====================================================
            2'd2: begin
    
                case (osd_char_index)
    
                    4'd0: osd_char_code = CH_B;
                    4'd1: osd_char_code = CH_R;
                    4'd2: osd_char_code = CH_I;
                    4'd3: osd_char_code = CH_COLON;
                    4'd4: osd_char_code = CH_SPACE;
    
                    4'd5: osd_char_code = bri_sign_char;
                    4'd6: osd_char_code = bri_tens_char;
                    4'd7: osd_char_code = bri_ones_char;
    
                    default:
                        osd_char_code = CH_SPACE;
    
                endcase
    
            end
    
    
            // =====================================================
            // Line 3 : CON: 1.25X
            // =====================================================
            2'd3: begin
    
                case (osd_char_index)
    
                    4'd0: osd_char_code = CH_C;
                    4'd1: osd_char_code = CH_O;
                    4'd2: osd_char_code = CH_N;
                    4'd3: osd_char_code = CH_COLON;
                    4'd4: osd_char_code = CH_SPACE;
    
                    4'd5: osd_char_code = con_int_char;
                    4'd6: osd_char_code = CH_DOT;
                    4'd7: osd_char_code = con_dec1_char;
                    4'd8: osd_char_code = con_dec2_char;
                    4'd9: osd_char_code = CH_X;
    
                    default:
                        osd_char_code = CH_SPACE;
    
                endcase
    
            end
    
            default:
                osd_char_code = CH_SPACE;
    
        endcase
    
    end
    

    // =========================================================
    // G1-B3 formal scrolling subtitle geometry
    //
    // Bottom darkened subtitle bar:
    // y = 430 ... 449
    //
    // Text glyph:
    // y = 433 ... 446
    //
    // Virtual text horizontal range:
    // 640 ... 999
    //
    // Total = 360 pixels
    // =========================================================

    localparam [9:0]  SCROLL_BAR_Y   = 10'd430;
    localparam [9:0]  SCROLL_BAR_H   = 10'd20;
    localparam [9:0]  SCROLL_TEXT_Y  = 10'd433;

    localparam [10:0] SCROLL_TEXT_X0 = 11'd640;
    localparam [10:0] SCROLL_TEXT_X1 = 11'd1000;


    wire [10:0] scroll_virtual_x;
    
    // =========================================================
    // G1-B3 Chinese scrolling geometry
    //
    // 9 Hanzi * 18 pixels = 162 pixels
    // Each Hanzi: 16x16 + 2-pixel spacing
    //
    // Chinese text vertical range:
    // y = 432 ... 447
    // =========================================================
    
    localparam [9:0]  HANZI_TEXT_Y  = 10'd432;
    localparam [10:0] HANZI_TEXT_X0 = 11'd640;
    localparam [10:0] HANZI_TEXT_X1 = 11'd802;
    
    wire        hanzi_in_text_x;
    wire        hanzi_in_text_y;
    
    wire [10:0] hanzi_rel_x;
    wire [9:0]  hanzi_rel_y;
    
    assign hanzi_in_text_x =
        (scroll_virtual_x >= HANZI_TEXT_X0) &&
        (scroll_virtual_x <  HANZI_TEXT_X1);
    
    assign hanzi_in_text_y =
        (y_cnt >= HANZI_TEXT_Y) &&
        (y_cnt <  HANZI_TEXT_Y + 10'd16);
    
    assign hanzi_rel_x =
        scroll_virtual_x - HANZI_TEXT_X0;
    
    assign hanzi_rel_y =
        y_cnt - HANZI_TEXT_Y;
        
    wire        scroll_bar_region;
    wire        scroll_in_text_x;
    wire        scroll_in_text_y;

    wire [10:0] scroll_rel_x;
    wire [9:0]  scroll_rel_y;


    assign scroll_virtual_x =
        {1'b0, x_cnt} +
        {1'b0, scroll_phase};


    assign scroll_bar_region =

        de &&

        (y_cnt >= SCROLL_BAR_Y) &&

        (y_cnt <  SCROLL_BAR_Y + SCROLL_BAR_H);


    assign scroll_in_text_x =

        (scroll_virtual_x >= SCROLL_TEXT_X0) &&

        (scroll_virtual_x <  SCROLL_TEXT_X1);


    assign scroll_in_text_y =

        (y_cnt >= SCROLL_TEXT_Y) &&

        (y_cnt <  SCROLL_TEXT_Y + 10'd14);


    assign scroll_rel_x =
        scroll_virtual_x - SCROLL_TEXT_X0;

    assign scroll_rel_y =
        y_cnt - SCROLL_TEXT_Y;


    // =========================================================
    // Decode 30 scrolling character cells
    //
    // No division
    // No modulo
    // =========================================================

    reg [4:0] scroll_char_index;
    reg [3:0] scroll_char_pos_x;

    always @(*) begin
        scroll_char_index = 5'd31;
        scroll_char_pos_x = 0;
        if (scroll_rel_x < 11'd360)
            {scroll_char_index,scroll_char_pos_x} = cell12(scroll_rel_x[8:0]);
    end

    // =========================================================
    // Decode 9 Chinese character cells
    //
    // Each cell = 18 pixels
    // Hanzi area = first 16 pixels
    // Gap        = last 2 pixels
    // =========================================================
    
    reg [3:0] hanzi_char_index;
    reg [4:0] hanzi_char_pos_x;
    
    always @(*) begin
        hanzi_char_index = 4'd15;
        hanzi_char_pos_x = 0;
        if (hanzi_rel_x < 11'd162)
            {hanzi_char_index,hanzi_char_pos_x} = cell18(hanzi_rel_x[7:0]);
    end

    // =========================================================
    // G1-B3 scrolling text string
    //
    // Index:
    //
    // 00 A
    // 01 N
    // 02 L
    // 03 O
    // 04 G
    // 05 I
    // 06 C
    // 07 SPACE
    // 08 F
    // 09 G
    // 10 P
    // 11 A
    // 12 SPACE
    // 13 M
    // 14 U
    // 15 L
    // 16 T
    // 17 I
    // 18 M
    // 19 E
    // 20 D
    // 21 I
    // 22 A
    // 23 SPACE
    // 24 P
    // 25 L
    // 26 A
    // 27 Y
    // 28 E
    // 29 R
    //
    // ANLOGIC FPGA MULTIMEDIA PLAYER
    // =========================================================

    reg [4:0] scroll_char_code;

    always @(*) begin

        case (scroll_char_index)

            5'd0:
                scroll_char_code = CH_A;

            5'd1:
                scroll_char_code = CH_N;

            5'd2:
                scroll_char_code = CH_L;

            5'd3:
                scroll_char_code = CH_O;

            5'd4:
                scroll_char_code = CH_G;

            5'd5:
                scroll_char_code = CH_I;

            5'd6:
                scroll_char_code = CH_C;

            5'd7:
                scroll_char_code = CH_SPACE;

            5'd8:
                scroll_char_code = CH_F;

            5'd9:
                scroll_char_code = CH_G;

            5'd10:
                scroll_char_code = CH_P;

            5'd11:
                scroll_char_code = CH_A;

            5'd12:
                scroll_char_code = CH_SPACE;

            5'd13:
                scroll_char_code = CH_M;

            5'd14:
                scroll_char_code = CH_U;

            5'd15:
                scroll_char_code = CH_L;

            5'd16:
                scroll_char_code = CH_T;

            5'd17:
                scroll_char_code = CH_I;

            5'd18:
                scroll_char_code = CH_M;

            5'd19:
                scroll_char_code = CH_E;

            5'd20:
                scroll_char_code = CH_D;

            5'd21:
                scroll_char_code = CH_I;

            5'd22:
                scroll_char_code = CH_A;

            5'd23:
                scroll_char_code = CH_SPACE;

            5'd24:
                scroll_char_code = CH_P;

            5'd25:
                scroll_char_code = CH_L;

            5'd26:
                scroll_char_code = CH_A;

            5'd27:
                scroll_char_code = CH_Y;

            5'd28:
                scroll_char_code = CH_E;

            5'd29:
                scroll_char_code = CH_R;

            default:
                scroll_char_code = CH_SPACE;

        endcase

    end


    wire [2:0] scroll_char_x;
    wire [2:0] scroll_char_y;

    wire       scroll_in_char_x;
    wire       use_scroll_font;


    assign scroll_char_x =
        scroll_char_pos_x[3:1];

    assign scroll_char_y =
        scroll_rel_y[3:1];

    assign scroll_in_char_x =
        (scroll_char_pos_x < 4'd10);


    assign use_scroll_font =
    
        (!scroll_lang) &&
    
        scroll_bar_region &&
    
        scroll_in_text_x &&
    
        scroll_in_text_y;

    // =========================================================
    // Shared font ROM input multiplexer
    //
    // Bottom scrolling text and top OSD reuse ONE font ROM.
    //
    // Keep names:
    // char_code
    // char_x
    // char_y
    //
    // for testbench access.
    // =========================================================

    wire [4:0] char_code;
    wire [2:0] char_x;
    wire [2:0] char_y;


    assign char_code =

        use_scroll_font ?

            scroll_char_code :

            osd_char_code;


    assign char_x =

        use_scroll_font ?

            scroll_char_x :

            osd_char_x;


    assign char_y =

        use_scroll_font ?

            scroll_char_y :

            osd_char_y;


    // =========================================================
    // Shared 5x7 font ROM
    // =========================================================

    reg [34:0] font_bits;

    always @(*) begin

        case (char_code)

            // SPACE
            CH_SPACE:
                font_bits = 35'b0;


            // M
            CH_M: font_bits = {
                5'b10001,
                5'b11011,
                5'b10101,
                5'b10101,
                5'b10001,
                5'b10001,
                5'b10001
            };


            // O
            CH_O: font_bits = {
                5'b01110,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b01110
            };


            // D
            CH_D: font_bits = {
                5'b11110,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b11110
            };


            // E
            CH_E: font_bits = {
                5'b11111,
                5'b10000,
                5'b10000,
                5'b11110,
                5'b10000,
                5'b10000,
                5'b11111
            };


            // :
            CH_COLON: font_bits = {
                5'b00000,
                5'b00100,
                5'b00100,
                5'b00000,
                5'b00100,
                5'b00100,
                5'b00000
            };


            // A
            CH_A: font_bits = {
                5'b01110,
                5'b10001,
                5'b10001,
                5'b11111,
                5'b10001,
                5'b10001,
                5'b10001
            };


            // N
            CH_N: font_bits = {
                5'b10001,
                5'b11001,
                5'b10101,
                5'b10011,
                5'b10001,
                5'b10001,
                5'b10001
            };


            // U
            CH_U: font_bits = {
                5'b10001,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b10001,
                5'b01110
            };


            // L
            CH_L: font_bits = {
                5'b10000,
                5'b10000,
                5'b10000,
                5'b10000,
                5'b10000,
                5'b10000,
                5'b11111
            };


            // T
            CH_T: font_bits = {
                5'b11111,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b00100
            };


            // I
            CH_I: font_bits = {
                5'b11111,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b11111
            };


            // G
            CH_G: font_bits = {
                5'b01110,
                5'b10001,
                5'b10000,
                5'b10111,
                5'b10001,
                5'b10001,
                5'b01110
            };


            // /
            CH_SLASH: font_bits = {
                5'b00001,
                5'b00010,
                5'b00100,
                5'b01000,
                5'b10000,
                5'b00000,
                5'b00000
            };


            // 0
            CH_0: font_bits = {
                5'b01110,
                5'b10001,
                5'b10011,
                5'b10101,
                5'b11001,
                5'b10001,
                5'b01110
            };


            // 1
            CH_1: font_bits = {
                5'b00100,
                5'b01100,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b01110
            };


            // 2
            CH_2: font_bits = {
                5'b01110,
                5'b10001,
                5'b00001,
                5'b00010,
                5'b00100,
                5'b01000,
                5'b11111
            };


            // 3
            CH_3: font_bits = {
                5'b11110,
                5'b00001,
                5'b00001,
                5'b01110,
                5'b00001,
                5'b00001,
                5'b11110
            };


            // 4
            CH_4: font_bits = {
                5'b00010,
                5'b00110,
                5'b01010,
                5'b10010,
                5'b11111,
                5'b00010,
                5'b00010
            };


            // C
            CH_C: font_bits = {
                5'b01110,
                5'b10001,
                5'b10000,
                5'b10000,
                5'b10000,
                5'b10001,
                5'b01110
            };


            // F
            CH_F: font_bits = {
                5'b11111,
                5'b10000,
                5'b10000,
                5'b11110,
                5'b10000,
                5'b10000,
                5'b10000
            };


            // P
            CH_P: font_bits = {
                5'b11110,
                5'b10001,
                5'b10001,
                5'b11110,
                5'b10000,
                5'b10000,
                5'b10000
            };


            // Y
            CH_Y: font_bits = {
                5'b10001,
                5'b10001,
                5'b01010,
                5'b00100,
                5'b00100,
                5'b00100,
                5'b00100
            };


            // R
            CH_R: font_bits = {
                5'b11110,
                5'b10001,
                5'b10001,
                5'b11110,
                5'b10100,
                5'b10010,
                5'b10001
            };
            // 5
            CH_5: font_bits = {
                5'b11111,
                5'b10000,
                5'b10000,
                5'b11110,
                5'b00001,
                5'b00001,
                5'b11110
            };
            
            // 6
            CH_6: font_bits = {
                5'b01110,
                5'b10000,
                5'b10000,
                5'b11110,
                5'b10001,
                5'b10001,
                5'b01110
            };
            
            // 7
            CH_7: font_bits = {
                5'b11111,
                5'b00001,
                5'b00010,
                5'b00100,
                5'b01000,
                5'b01000,
                5'b01000
            };
            
            // B
            CH_B: font_bits = {
                5'b11110,
                5'b10001,
                5'b10001,
                5'b11110,
                5'b10001,
                5'b10001,
                5'b11110
            };
            
            // +
            CH_PLUS: font_bits = {
                5'b00000,
                5'b00100,
                5'b00100,
                5'b11111,
                5'b00100,
                5'b00100,
                5'b00000
            };
            
            // -
            CH_MINUS: font_bits = {
                5'b00000,
                5'b00000,
                5'b00000,
                5'b11111,
                5'b00000,
                5'b00000,
                5'b00000
            };
            
            // .
            CH_DOT: font_bits = {
                5'b00000,
                5'b00000,
                5'b00000,
                5'b00000,
                5'b00000,
                5'b00110,
                5'b00110
            };
            
            // X
            CH_X: font_bits = {
                5'b10001,
                5'b10001,
                5'b01010,
                5'b00100,
                5'b01010,
                5'b10001,
                5'b10001
            };  
    
            default:
                font_bits = 35'b0;

        endcase

    end
    // =========================================================
    // Chinese scrolling string
    //
    // 安 路 多 媒 体 播 放 系 统
    // =========================================================
    
    reg [3:0] hanzi_code;
    
    always @(*) begin
    
        case (hanzi_char_index)
    
            4'd0:
                hanzi_code = HZ_AN;
    
            4'd1:
                hanzi_code = HZ_LU;
    
            4'd2:
                hanzi_code = HZ_DUO;
    
            4'd3:
                hanzi_code = HZ_MEI;
    
            4'd4:
                hanzi_code = HZ_TI;
    
            4'd5:
                hanzi_code = HZ_BO;
    
            4'd6:
                hanzi_code = HZ_FANG;
    
            4'd7:
                hanzi_code = HZ_XI;
    
            4'd8:
                hanzi_code = HZ_TONG;
    
            default:
                hanzi_code = HZ_AN;
    
        endcase
    
    end
    
    
    // =========================================================
    // 16x16 Chinese font ROM
    // =========================================================
    
    reg [255:0] hanzi_bits;
    
    always @(*) begin
    
        case (hanzi_code)
    
            // =====================================================
            // 安
            // =====================================================
            HZ_AN: hanzi_bits = {
                16'b0000001110000000,
                16'b0000000100000000,
                16'b0111111111111100,
                16'b0110000000001100,
                16'b0110001000001100,
                16'b0000011000000000,
                16'b0111111111111100,
                16'b0000110001100000,
                16'b0001100001100000,
                16'b0001110011000000,
                16'b0000111110000000,
                16'b0000000111100000,
                16'b0000011000111000,
                16'b0011110000001100,
                16'b0010000000000100,
                16'b0000000000000000
            };
    
    
            // =====================================================
            // 路
            // =====================================================
            HZ_LU: hanzi_bits = {
                16'b0000000001000000,
                16'b0111110001100000,
                16'b0100010011111000,
                16'b0100010110001000,
                16'b0100011110110000,
                16'b0111111011100000,
                16'b0001100001110000,
                16'b0101100011111000,
                16'b0101111100011000,
                16'b0101101111111000,
                16'b0101100010001000,
                16'b0101100010001000,
                16'b0111110010001000,
                16'b0110000011111000,
                16'b0000000001000000,
                16'b0000000000000000
            };
    
    
            // =====================================================
            // 多
            // =====================================================
            HZ_DUO: hanzi_bits = {
                16'b0000000100000000,
                16'b0000001100000000,
                16'b0000011111110000,
                16'b0001100001100000,
                16'b0011010011000000,
                16'b0000001110000000,
                16'b0000111011000000,
                16'b0111000111111100,
                16'b0000001100001100,
                16'b0000111000011000,
                16'b0001100110110000,
                16'b0000000011100000,
                16'b0000011110000000,
                16'b0011110000000000,
                16'b0111000000000000,
                16'b0000000000000000
            };
    
    
            // =====================================================
            // 媒
            // =====================================================
            HZ_MEI: hanzi_bits = {
                16'b0000010000010000,
                16'b0011001000010000,
                16'b0010001111111000,
                16'b0010001000010000,
                16'b0111110111110000,
                16'b0010110100010000,
                16'b0110110100010000,
                16'b0110110111110000,
                16'b0100110001100000,
                16'b0111111111111000,
                16'b0011100011110000,
                16'b0011110011111000,
                16'b0010011101101100,
                16'b0110001101100100,
                16'b0100000001100000,
                16'b0000000000000000
            };
    
    
            // =====================================================
            // 体
            // =====================================================
            HZ_TI: hanzi_bits = {
                16'b0000100000100000,
                16'b0001000000100000,
                16'b0001000000100000,
                16'b0011111111111100,
                16'b0011000011100000,
                16'b0111000111100000,
                16'b1111000111110000,
                16'b0011001100101000,
                16'b0011001000101100,
                16'b0011011000100100,
                16'b0011010000100110,
                16'b0011111111110100,
                16'b0011000000100000,
                16'b0011000000100000,
                16'b0011000000100000,
                16'b0000000000000000
            };
    
    
            // =====================================================
            // 播
            // =====================================================
            HZ_BO: hanzi_bits = {
                16'b0011000000011000,
                16'b0011011111111000,
                16'b0011001101001000,
                16'b0111100101010000,
                16'b0011011111111100,
                16'b0011000011110000,
                16'b0011001001011000,
                16'b0011111101001100,
                16'b0111000000000100,
                16'b1111011111111000,
                16'b0011010010011000,
                16'b0011011111111000,
                16'b0011010010011000,
                16'b0011011111111000,
                16'b0110010000001000,
                16'b0000000000000000
            };
    
    
            // =====================================================
            // 放
            // =====================================================
            HZ_FANG: hanzi_bits = {
                16'b0001000000100000,
                16'b0001100000100000,
                16'b0000100001100000,
                16'b0111111011111100,
                16'b0011000011001000,
                16'b0011000111001000,
                16'b0011111111001000,
                16'b0011011101011000,
                16'b0011011001110000,
                16'b0011011000110000,
                16'b0010011000110000,
                16'b0010011001110000,
                16'b0110011001011000,
                16'b0100011111001100,
                16'b0101111010000100,
                16'b0000000000000000
            };
    
    
            // =====================================================
            // 系
            // =====================================================
            HZ_XI: hanzi_bits = {
                16'b0000000000010000,
                16'b0000011111111000,
                16'b0011111110000000,
                16'b0000011000100000,
                16'b0000100001100000,
                16'b0011111111000000,
                16'b0000011100010000,
                16'b0000110000001000,
                16'b0011111111110000,
                16'b0011100010001100,
                16'b0000100010010000,
                16'b0001100010110000,
                16'b0011000010011000,
                16'b0110000010001100,
                16'b0000011110000000,
                16'b0000000000000000
            };
    
    
            // =====================================================
            // 统
            // =====================================================
            HZ_TONG: hanzi_bits = {
                16'b0001000000100000,
                16'b0001000000110000,
                16'b0011001111111000,
                16'b0010010001100000,
                16'b0110110001001100,
                16'b1111100011000100,
                16'b0101001111111100,
                16'b0011001111000100,
                16'b0010010011010000,
                16'b0111110011010000,
                16'b0100000011010000,
                16'b0000000010010000,
                16'b0011111010010100,
                16'b0111000110010100,
                16'b0000011100001100,
                16'b0000000000000000
            };
    
    
            default:
                hanzi_bits = 256'b0;
    
        endcase
    
    end
    // =========================================================
    // 16x16 Hanzi pixel lookup
    // =========================================================
    
    wire [3:0] hanzi_x;
    wire [3:0] hanzi_y;
    
    wire       hanzi_in_char_x;
    
    reg [15:0] hanzi_row;
    reg        hanzi_pixel;
    
    assign hanzi_x =
        hanzi_char_pos_x[3:0];
    
    assign hanzi_y =
        hanzi_rel_y[3:0];
    
    assign hanzi_in_char_x =
        (hanzi_char_pos_x < 5'd16);
    
    
    always @(*) begin
    
        case (hanzi_y)
    
            4'd0:  hanzi_row = hanzi_bits[255:240];
            4'd1:  hanzi_row = hanzi_bits[239:224];
            4'd2:  hanzi_row = hanzi_bits[223:208];
            4'd3:  hanzi_row = hanzi_bits[207:192];
            4'd4:  hanzi_row = hanzi_bits[191:176];
            4'd5:  hanzi_row = hanzi_bits[175:160];
            4'd6:  hanzi_row = hanzi_bits[159:144];
            4'd7:  hanzi_row = hanzi_bits[143:128];
            4'd8:  hanzi_row = hanzi_bits[127:112];
            4'd9:  hanzi_row = hanzi_bits[111:96];
            4'd10: hanzi_row = hanzi_bits[95:80];
            4'd11: hanzi_row = hanzi_bits[79:64];
            4'd12: hanzi_row = hanzi_bits[63:48];
            4'd13: hanzi_row = hanzi_bits[47:32];
            4'd14: hanzi_row = hanzi_bits[31:16];
            4'd15: hanzi_row = hanzi_bits[15:0];
    
            default:
                hanzi_row = 16'b0;
    
        endcase
    
    end
    
    
    always @(*) begin
    
        case (hanzi_x)
    
            4'd0:  hanzi_pixel = hanzi_row[15];
            4'd1:  hanzi_pixel = hanzi_row[14];
            4'd2:  hanzi_pixel = hanzi_row[13];
            4'd3:  hanzi_pixel = hanzi_row[12];
            4'd4:  hanzi_pixel = hanzi_row[11];
            4'd5:  hanzi_pixel = hanzi_row[10];
            4'd6:  hanzi_pixel = hanzi_row[9];
            4'd7:  hanzi_pixel = hanzi_row[8];
            4'd8:  hanzi_pixel = hanzi_row[7];
            4'd9:  hanzi_pixel = hanzi_row[6];
            4'd10: hanzi_pixel = hanzi_row[5];
            4'd11: hanzi_pixel = hanzi_row[4];
            4'd12: hanzi_pixel = hanzi_row[3];
            4'd13: hanzi_pixel = hanzi_row[2];
            4'd14: hanzi_pixel = hanzi_row[1];
            4'd15: hanzi_pixel = hanzi_row[0];
    
            default:
                hanzi_pixel = 1'b0;
    
        endcase
    
    end
    // =========================================================
    // Shared font lookup
    // =========================================================

    reg [4:0] font_row;
    reg       font_pixel;


    always @(*) begin

        case (char_y)

            3'd0:
                font_row = font_bits[34:30];

            3'd1:
                font_row = font_bits[29:25];

            3'd2:
                font_row = font_bits[24:20];

            3'd3:
                font_row = font_bits[19:15];

            3'd4:
                font_row = font_bits[14:10];

            3'd5:
                font_row = font_bits[9:5];

            3'd6:
                font_row = font_bits[4:0];

            default:
                font_row = 5'b00000;

        endcase

    end


    always @(*) begin

        case (char_x)

            3'd0:
                font_pixel = font_row[4];

            3'd1:
                font_pixel = font_row[3];

            3'd2:
                font_pixel = font_row[2];

            3'd3:
                font_pixel = font_row[1];

            3'd4:
                font_pixel = font_row[0];

            default:
                font_pixel = 1'b0;

        endcase

    end


    // =========================================================
    // Pixel masks
    // =========================================================

    wire osd_pixel;
    wire english_scroll_pixel;
    wire hanzi_scroll_pixel;
    wire scroll_text_pixel;
    // =========================================================
    // G1-B3 subtitle bar visual style
    //
    // Fake 50% alpha dark background:
    // dark_rgb = rgb_in / 2
    // =========================================================
    
    wire [23:0] scroll_bg_dark;
    
    assign scroll_bg_dark = {
        1'b0, rgb_in[23:17],
        1'b0, rgb_in[15:9],
        1'b0, rgb_in[7:1]
    };

    assign osd_pixel =

        de &&

        in_osd_x &&

        in_osd_y &&

        osd_in_text_line &&

        osd_in_char_x &&

        font_pixel;


    // English scrolling pixel
    assign english_scroll_pixel =
    
        (!scroll_lang) &&
    
        use_scroll_font &&
    
        scroll_in_char_x &&
    
        font_pixel;
    
    
    // Chinese scrolling pixel
    assign hanzi_scroll_pixel =
    
        scroll_lang &&
    
        scroll_bar_region &&
    
        hanzi_in_text_x &&
    
        hanzi_in_text_y &&
    
        hanzi_in_char_x &&
    
        hanzi_pixel;
    
    
    // Select current language
    assign scroll_text_pixel =
    
        scroll_lang ?
    
            hanzi_scroll_pixel :
    
            english_scroll_pixel;
// =========================================================
// Final RGB output
//
// Priority:
//
// 1. White scrolling subtitle
// 2. 50% darkened image inside subtitle bar
// 3. G1-B2 MODE / IMAGE OSD
// 4. Original BMP image
// =========================================================

wire [23:0] legacy_rgb;
assign legacy_rgb =

    scroll_bar_region ?

        (
            // White scrolling text
            scroll_text_pixel ?

                24'hFFFFFF :

            // 50% darkened original image
                scroll_bg_dark
        ) :

    (de && in_osd_x && in_osd_y) ?

        (
            osd_pixel ?

                24'hFFFFFF :

                24'h000000
        ) :

        rgb_in;
// A single dual-port BRAM stores the original glyph bits exactly.
// RGB and all eligibility masks are delayed with the synchronous font read.
generate if (FONT_BRAM) begin : bram_font
    wire latin_pixel, chinese_pixel;
    wire [12:0] latin_addr = {2'b00,char_code,char_y,char_x};
    wire [12:0] chinese_addr = {1'b1,hanzi_code,hanzi_y,hanzi_x};
    EG_LOGIC_BRAM #(.DATA_WIDTH_A(1),.DATA_WIDTH_B(1),
        .ADDR_WIDTH_A(13),.ADDR_WIDTH_B(13),.DATA_DEPTH_A(8192),.DATA_DEPTH_B(8192),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
        .RESETMODE("SYNC"),.IMPLEMENT("9K"),
        .INIT_FILE("priority12_osd_font.mif"),.FILL_ALL("NONE")) font_rom (
        .dia(1'b0),.addra(latin_addr),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),
        .rsta(1'b0),.bea(1'b0),.doa(latin_pixel),
        .dib(1'b0),.addrb(chinese_addr),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(1'b0),
        .rstb(1'b0),.beb(1'b0),.dob(chinese_pixel));
    reg [23:0] bg;
    reg bar, osd_box, osd_enable, latin_enable, chinese_enable;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bg<=0; bar<=0; osd_box<=0; osd_enable<=0;
            latin_enable<=0; chinese_enable<=0;
        end else begin
            bg<=rgb_in;
            bar<=scroll_bar_region;
            osd_box<=de && in_osd_x && in_osd_y;
            osd_enable<=de && in_osd_x && in_osd_y && osd_in_text_line && osd_in_char_x;
            latin_enable<=!scroll_lang && use_scroll_font && scroll_in_char_x;
            chinese_enable<=scroll_lang && scroll_bar_region &&
                hanzi_in_text_x && hanzi_in_text_y && hanzi_in_char_x;
        end
    end
    wire [23:0] dark_bg={1'b0,bg[23:17],1'b0,bg[15:9],1'b0,bg[7:1]};
    assign rgb_out = presentation_hotspot ? 24'hffff00 : presentation_paint ? 24'hffffff : bar ?
        ((latin_enable && latin_pixel || chinese_enable && chinese_pixel) ? 24'hffffff : dark_bg) :
        osd_box ? ((osd_enable && latin_pixel) ? 24'hffffff : 24'd0) : bg;
end else begin : combinational_font
    assign rgb_out = presentation_hotspot ? 24'hffff00 : presentation_paint ? 24'hffffff : legacy_rgb;
end endgenerate
endmodule
