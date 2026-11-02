`timescale 1ns/1ps

module osd_overlay_640x480 #(
    parameter FONT_BRAM = 0,
    parameter PRESENTATION_ENABLE = 0,
    parameter COMPARE_ENABLE = 0,
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
    input av_enable,av_mode_mic,av_step,input[23:0] av_levels,input[4:0] av_pulse,av_ring,
    input             presentation_mode,
    input             presentation_key_next_n,

    // ===================== G1-B2 dynamic status =====================
    input             auto_play,
    input      [7:0]  img_idx,
    input      [7:0]  img_count,
    
    // ===================== G1-C2 image adjustment OSD =====================
    input signed [8:0] brightness_value,
    input      [9:0] contrast_gain,
    input             compare_valid,
    input      [3:0]  compare_cmd,
    input      [23:0] rgb_original,
    input      [23:0] rgb_adjusted,
    output            compare_on_active,
    output     [2:0]  compare_mode_active,
    
    input      [23:0] rgb_in,
    output     [23:0] rgb_out
);

    // =========================================================
    // 640x480 active-video coordinates
    // =========================================================
    reg [9:0] x_cnt;
    reg [9:0] y_cnt;
    reg       de_d;
    wire frame_tick;

    // LCD commands are applied together at the next frame boundary.
    reg compare_on,compare_on_pending;
    // 0..39 map to x=0..624 in 16-pixel steps; 40 is the far edge x=639.
    reg [5:0] compare_step,compare_step_pending;
    wire [9:0] compare_x=compare_step==6'd40 ? 10'd639 :
                         {compare_step,4'b0000};
    reg [2:0] compare_mode,compare_mode_pending;
    assign compare_on_active=COMPARE_ENABLE && compare_on;
    assign compare_mode_active=compare_mode;
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            compare_on<=1'b1; compare_on_pending<=1'b1;
            compare_step<=6'd20; compare_step_pending<=6'd20;
            compare_mode<=3'd1; compare_mode_pending<=3'd1;
        end else begin
            if(compare_valid && COMPARE_ENABLE) begin
                case(compare_cmd)
                    4'd0:compare_on_pending<=~compare_on_pending;
                    4'd1:compare_step_pending<=compare_step_pending==0 ?
                                              6'd0 : compare_step_pending-1'b1;
                    4'd2:compare_step_pending<=6'd20;
                    4'd3:compare_step_pending<=compare_step_pending>=6'd40 ?
                                              6'd40 : compare_step_pending+1'b1;
                    4'd4:compare_mode_pending<=compare_mode_pending==3'd4 ?
                                              3'd0 : compare_mode_pending+1'b1;
                    default:begin end
                endcase
            end
            if(frame_tick) begin
                compare_on<=compare_on_pending;
                compare_step<=compare_step_pending;
                compare_mode<=compare_mode_pending;
            end
        end
    end
    wire [7:0] red=rgb_original[23:16],green=rgb_original[15:8],blue=rgb_original[7:0];
    wire [7:0] gray={2'b00,red[7:2]}+{1'b0,green[7:1]}+{2'b00,blue[7:2]};
    reg [7:0] previous_gray;
    always @(posedge clk or posedge rst)
        if(rst) previous_gray<=0;
        else if(de) previous_gray<=gray;
    // Coarse horizontal edge threshold uses the upper three gray bits.
    // It avoids wide per-pixel absolute-value arithmetic on the near-full FPGA.
    wire edge_found=gray[7:5]!=previous_gray[7:5];
    wire [23:0] compare_processed=
        compare_mode==3'd0 ? rgb_original :
        compare_mode==3'd1 ? {gray,gray,gray} :
        compare_mode==3'd2 || compare_mode==3'd3 ? rgb_adjusted :
        ((x_cnt!=0 && edge_found) ? 24'hffffff : 24'h000000);
    wire slider_pixel=compare_x==10'd639 ? x_cnt==10'd639 :
                      x_cnt[9:2]==compare_x[9:2];
    wire [23:0] compare_base=COMPARE_ENABLE && compare_on && de ?
        (slider_pixel ? 24'hffff00 :
         x_cnt<compare_x ? rgb_original : compare_processed) : rgb_in;

    wire[23:0] compare_rgb=compare_base;
    wire av_paint;wire[23:0] av_color;wire[4:0] av_opacity;
    audio_visual_mask u_av_mask(.clk(clk),.rst(rst),.de(de),.x(x_cnt),.y(y_cnt),
      .enabled(av_enable),.mode_mic(av_mode_mic),.levels(av_levels),.pulse(av_pulse),.ring(av_ring),
      .paint(av_paint),.color(av_color),.frame_tick(frame_tick),.opacity(av_opacity));
    // Signed linear interpolation, 0/16 transparent through 16/16 opaque.
    // Both inputs already have the same one-clock font/mask latency.
    function [7:0] star_blend;
      input[7:0] background,foreground;input[4:0] opacity;
      reg signed[8:0] delta;reg signed[14:0] weighted,total;
      begin
        delta=$signed({1'b0,foreground})-$signed({1'b0,background});
        weighted=delta*$signed({1'b0,opacity});
        total=$signed({1'b0,background,4'b0000})+weighted+15'sd8;
        star_blend=total[11:4];
      end
    endfunction
    wire[1:0] presentation_text_coverage;
    wire presentation_paint,presentation_hotspot,presentation_focus_active,presentation_focus;
    wire [2:0] presentation_focus_step;
    wire [23:0] magnifier_pixel;
    wire magnifier_inside,magnifier_border;
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
            .av_step(av_step),.av_zoom(av_enable && (av_pulse>=8 || av_levels[23:18]>=32)),
            .av_mid(av_enable ? av_levels[17:12] : 6'd0),
            .key_next_n(presentation_key_next_n),.vs(vs),.de(de),
            .x(x_cnt),.y(y_cnt),.paint(presentation_paint),.text_coverage(presentation_text_coverage),
            .hotspot_paint(presentation_hotspot),
            .focus_active(presentation_focus_active),.focus_area(presentation_focus),
            .focus_step(presentation_focus_step)
        );
        presentation_magnifier u_magnifier (
            .clk(clk),.rst(rst),.vs(vs),.de(de),
            .focus_active(presentation_focus_active),.focus_area(presentation_focus),
            .focus_step(presentation_focus_step),.x(x_cnt),.y(y_cnt),
            .rgb_in(rgb_in),.pixel(magnifier_pixel),
            .inside_d(magnifier_inside),.border_d(magnifier_border)
        );
    end else begin : no_presentation
        assign presentation_text_coverage=2'd0;
        assign presentation_paint=1'b0;
        assign presentation_hotspot=1'b0;
        assign presentation_focus_active=1'b0;
        assign presentation_focus=1'b0;
        assign presentation_focus_step=3'd0;
        assign magnifier_pixel=24'd0;
        assign magnifier_inside=1'b0;
        assign magnifier_border=1'b0;
    end endgenerate


    // =========================================================
    // G1-B3 frame-synchronous animation control
    // =========================================================
    reg  vs_d;

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

    // Restore the original left text position; retain the padded card styling.
    localparam [9:0] OSD_X = 10'd32;
    localparam [9:0] OSD_Y = 10'd16;

    wire[3:0] index_length,count_length;
    wire [5:0] status_cells=6'd8+index_length+count_length;
    wire [9:0] OSD_W=status_cells>12 ? ({4'd0,status_cells}<<3)+({4'd0,status_cells}<<2) : 10'd144;
    wire [9:0] card_right=OSD_X+OSD_W+10'd8;
    localparam [9:0] OSD_H = COMPARE_ENABLE ? 10'd86 : 10'd68;

    localparam [9:0] CARD_BOTTOM=OSD_Y+OSD_H+10'd8;
    wire card_area=de && x_cnt>=24 && x_cnt<card_right &&
        y_cnt>=8 && y_cnt<CARD_BOTTOM &&
        ((x_cnt>=28 && x_cnt<card_right-4) ||
         (y_cnt>=12 && y_cnt<CARD_BOTTOM-4) ||
         (x_cnt>=26 && x_cnt<card_right-2 && y_cnt>=10 && y_cnt<CARD_BOTTOM-2));
    wire card_edge=card_area && (x_cnt<26 || x_cnt>=card_right-2 ||
        y_cnt<10 || y_cnt>=CARD_BOTTOM-2 ||
        ((x_cnt<28 || x_cnt>=card_right-4) && (y_cnt<12 || y_cnt>=CARD_BOTTOM-4)));

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
    wire [8:0] osd_cell,scroll_cell,hanzi_cell;
    reg [4:0] osd_char_index;
    reg [3:0] osd_char_pos_x;

    always @(*) begin
        osd_char_index = 5'd31;
        osd_char_pos_x = 0;
        if (rel_x < OSD_W)
            {osd_char_index,osd_char_pos_x} = osd_cell;
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
    
    reg [2:0] osd_text_line;
    reg [3:0] osd_char_pos_y;
    reg       osd_in_text_line;
    
    always @(*) begin
    
        osd_text_line    = 3'd0;
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
        else if (COMPARE_ENABLE && rel_y>=10'd72 && rel_y<10'd86) begin
            osd_text_line=3'd4;
            osd_char_pos_y=rel_y-10'd72;
            osd_in_text_line=1'b1;
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
        (osd_char_pos_x < 4'd12);


    // =========================================================
    // Character codes
    // =========================================================

    localparam [5:0] CH_SPACE = 6'd0;
    localparam [5:0] CH_M     = 6'd1;
    localparam [5:0] CH_O     = 6'd2;
    localparam [5:0] CH_D     = 6'd3;
    localparam [5:0] CH_E     = 6'd4;
    localparam [5:0] CH_COLON = 6'd5;
    localparam [5:0] CH_A     = 6'd6;
    localparam [5:0] CH_N     = 6'd7;
    localparam [5:0] CH_U     = 6'd8;
    localparam [5:0] CH_L     = 6'd9;
    localparam [5:0] CH_T     = 6'd10;
    localparam [5:0] CH_I     = 6'd11;
    localparam [5:0] CH_G     = 6'd12;
    localparam [5:0] CH_SLASH = 6'd13;
    localparam [5:0] CH_0     = 6'd14;
    localparam [5:0] CH_1     = 6'd15;
    localparam [5:0] CH_2     = 6'd16;
    localparam [5:0] CH_3     = 6'd17;
    localparam [5:0] CH_4     = 6'd18;

    // G1-B3 additional scrolling subtitle characters

    localparam [5:0] CH_C     = 6'd19;
    localparam [5:0] CH_F     = 6'd20;
    localparam [5:0] CH_P     = 6'd21;
    localparam [5:0] CH_Y     = 6'd22;
    localparam [5:0] CH_R     = 6'd23;
    // ===================== G1-C2 OSD characters =====================
    
    localparam [5:0] CH_5     = 6'd24;
    localparam [5:0] CH_6     = 6'd25;
    localparam [5:0] CH_7     = 6'd26;
    localparam [5:0] CH_B     = 6'd27;
    localparam [5:0] CH_PLUS  = 6'd28;
    localparam [5:0] CH_MINUS = 6'd29;
    localparam [5:0] CH_DOT   = 6'd30;
    localparam [5:0] CH_X     = 6'd31;
    localparam [5:0] CH_8=6'd32, CH_9=6'd33;
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

    // Convert once at VS, finishing in < 70 pixel clocks, before the OSD
    // starts at y=16. Input status is committed by osd_status_cdc at VS.
    // Delay start one clock so that the converter sees that new status.
    wire [11:0] index_decimal, count_decimal;
    reg format_vs, format_start;
    osd_decimal_pair status_format(.clk(clk),.rst(rst),.start(format_start),
        .index_value(img_count==0 ? 8'd0 : img_idx+8'd1),.count_value(img_count),
        .index_decimal(index_decimal),.count_decimal(count_decimal));
    wire [11:0] index_frame=index_decimal,count_frame=count_decimal;
    always @(posedge clk or posedge rst)
        if(rst) begin format_vs<=0;format_start<=0;end
        else begin
            format_vs<=vs;
            format_start<=vs && !format_vs;
        end
    function [3:0] decimal_length;
        input[11:0] value;
        integer n;
        begin
            decimal_length=1;
            for(n=1;n<3;n=n+1)if(value[n*4+:4]!=0)decimal_length=n+1;
        end
    endfunction
    assign index_length=decimal_length(index_frame);
    assign count_length=decimal_length(count_frame);
    wire[3:0] index_digit_position=4'd6+index_length-osd_char_index;
    wire[3:0] count_digit_position=4'd7+index_length+count_length-osd_char_index;
    function[5:0] decimal_character;
        input[3:0] digit;
        begin case(digit)
            0:decimal_character=CH_0;1:decimal_character=CH_1;
            2:decimal_character=CH_2;3:decimal_character=CH_3;
            4:decimal_character=CH_4;5:decimal_character=CH_5;
            6:decimal_character=CH_6;7:decimal_character=CH_7;
            8:decimal_character=CH_8;9:decimal_character=CH_9;
            default:decimal_character=CH_SPACE;
        endcase end
    endfunction
    // =========================================================
    // G1-C2 continuous target values shown as two 64-step progress bars.
    // Brightness midpoint is zero; contrast midpoint is 1.00x.
    // =========================================================
    wire [8:0] brightness_offset=brightness_value+9'sd64;
    wire [6:0] brightness_fill=brightness_offset[7:1];
    wire [9:0] contrast_offset=contrast_gain-10'd128;
    wire [6:0] contrast_fill=contrast_offset[8:2];
    wire bar_brightness=rel_y>=10'd38 && rel_y<10'd48;
    wire bar_contrast=rel_y>=10'd56 && rel_y<10'd66;
    wire adjust_bar_region=de && in_osd_x && in_osd_y &&
                           rel_x>=10'd60 && rel_x<10'd128 &&
                           (bar_brightness || bar_contrast);
    wire adjust_bar_border=adjust_bar_region &&
        (rel_x==10'd60 || rel_x==10'd127 || rel_y==10'd38 ||
         rel_y==10'd47 || rel_y==10'd56 || rel_y==10'd65);
    wire [6:0] bar_position=rel_x[6:0]-7'd62;
    wire [6:0] bar_amount=bar_brightness ? brightness_fill : contrast_fill;
    wire adjust_bar_fill=adjust_bar_region && !adjust_bar_border &&
        rel_x>=10'd62 && rel_x<10'd126 && bar_position<bar_amount;
    // =========================================================

        // G1-B2 OSD character selection
    // =========================================================

    reg [5:0] osd_char_code;

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

    
                    default: begin
                        osd_char_code = CH_SPACE;
                        if(osd_char_index<7+index_length)
                            osd_char_code=decimal_character(index_frame >> {index_digit_position,2'b00});
                        else if(osd_char_index==7+index_length)osd_char_code=CH_SLASH;
                        else if(osd_char_index<8+index_length+count_length)
                            osd_char_code=decimal_character(count_frame >> {count_digit_position,2'b00});
                    end
    
                endcase
    
            end
    
    
            // =====================================================
            // Line 2 : BRI: [continuous bar]
            // =====================================================
            2'd2: begin
    
                case (osd_char_index)
    
                    4'd0: osd_char_code = CH_B;
                    4'd1: osd_char_code = CH_R;
                    4'd2: osd_char_code = CH_I;
                    4'd3: osd_char_code = CH_COLON;
                    4'd4: osd_char_code = CH_SPACE;
    
                    default:
                        osd_char_code = CH_SPACE;
    
                endcase
    
            end
    
    
            // =====================================================
            // Line 3 : CON: [continuous bar]
            // =====================================================
            2'd3: begin
    
                case (osd_char_index)
    
                    4'd0: osd_char_code = CH_C;
                    4'd1: osd_char_code = CH_O;
                    4'd2: osd_char_code = CH_N;
                    4'd3: osd_char_code = CH_COLON;
                    4'd4: osd_char_code = CH_SPACE;
    
                    default:
                        osd_char_code = CH_SPACE;
    
                endcase
    
            end
            3'd4: begin
                case(osd_char_index)
                    0:osd_char_code=CH_C;
                    1:osd_char_code=CH_M;
                    2:osd_char_code=CH_P;
                    3:osd_char_code=CH_COLON;
                    4:osd_char_code=CH_SPACE;
                    5:osd_char_code=compare_on ?
                        (compare_mode==3'd0 ? CH_O :
                         compare_mode==3'd1 ? CH_G :
                         compare_mode==3'd2 ? CH_B :
                         compare_mode==3'd3 ? CH_C : CH_E) : CH_O;
                    6:osd_char_code=compare_on ?
                        (compare_mode==3'd0 ? CH_R :
                         compare_mode==3'd1 ? CH_R :
                         compare_mode==3'd2 ? CH_R :
                         compare_mode==3'd3 ? CH_O : CH_D) : CH_F;
                    7:osd_char_code=compare_on ?
                        (compare_mode==3'd0 ? CH_I :
                         compare_mode==3'd1 ? CH_A :
                         compare_mode==3'd2 ? CH_I :
                         compare_mode==3'd3 ? CH_N : CH_G) : CH_F;
                    8:osd_char_code=compare_mode==3'd0 && compare_on ? CH_G :
                                     compare_mode==3'd1 && compare_on ? CH_Y :
                                     compare_mode==3'd4 && compare_on ? CH_E : CH_SPACE;
                    default:osd_char_code=CH_SPACE;
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
            {scroll_char_index,scroll_char_pos_x} = scroll_cell;
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
            {hanzi_char_index,hanzi_char_pos_x} = hanzi_cell;
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

    reg [5:0] scroll_char_code;

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
                scroll_char_code = CH_P;

            5'd10:
                scroll_char_code = CH_G;

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
        (scroll_char_pos_x < 4'd12);


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

    wire [5:0] char_code;
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
    // Dim only the image background. Text, subtitle and hotspot graphics are
    // composed afterwards so they remain readable during a presentation.
    wire [23:0] focus_rgb =
        !presentation_focus_active || presentation_focus || !de ? compare_rgb :
        {1'b0,compare_rgb[23:17],1'b0,compare_rgb[15:9],1'b0,compare_rgb[7:1]};
    
    assign scroll_bg_dark = {
        1'b0, focus_rgb[23:17],
        1'b0, focus_rgb[15:9],
        1'b0, focus_rgb[7:1]
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

    card_area ?

        (
            osd_pixel ?

                24'hFFFFFF :

            (adjust_bar_border || card_edge) ?

                24'h387088 :

            adjust_bar_fill ?

                (bar_contrast ? 24'hA080F0 : 24'h00D8FF) :

                24'h142838
        ) :

        focus_rgb;
// A single dual-port BRAM stores the original glyph bits exactly.
// RGB and all eligibility masks are delayed with the synchronous font read.
function[7:0] font_blend;
    input[7:0] base;input[1:0] alpha;
    reg[9:0] value;
    begin
        case(alpha)
            0:value={2'b0,base};
            1:value={2'b0,base}-(base>>2)+10'd63;
            2:value=(base>>1)+10'd127;
            default:value=10'd255;
        endcase
        font_blend=value[7:0];
    end
endfunction
generate if (FONT_BRAM) begin : bram_font
    wire [1:0] latin_pixel, chinese_pixel;
    wire[3:0] native_x=use_scroll_font ? scroll_char_pos_x : osd_char_pos_x;
    wire[3:0] native_y=use_scroll_font ? scroll_rel_y[3:0] : osd_char_pos_y;
    // (character*16 + row)*12 + column, modulo the same 13-bit address.
    // One existing DSP resource replaces multiple variable LUT adders.
    (* dsp_style="yes" *) wire [13:0] latin_row_product={char_code,native_y}*14'd12;
    wire [12:0] latin_addr=latin_row_product[12:0]+native_x;
    wire[12:0] chinese_addr={1'b0,hanzi_code,hanzi_y,hanzi_x};
    presentation_packed_rom #(.WIDTH(2),.INIT_FILE("osd_latin_aa.mif")) font_rom (
        .clk(clk),.addr(latin_addr),.pixel(latin_pixel),.addr_b(13'd0),.pixel_b());
    // 4096 x 2 fits one 9K block.
    EG_LOGIC_BRAM #(.DATA_WIDTH_A(2),.DATA_WIDTH_B(2),
        .ADDR_WIDTH_A(12),.ADDR_WIDTH_B(12),.DATA_DEPTH_A(4096),.DATA_DEPTH_B(4096),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),
        .IMPLEMENT("9K"),.INIT_FILE("osd_hanzi_aa.mif"),.FILL_ALL("NONE")) hanzi_rom (
        .dia(2'd0),.addra(chinese_addr[11:0]),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(1'b0),
        .rsta(1'b0),.bea(1'b0),.doa(chinese_pixel),
        .dib(2'd0),.addrb(12'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),
        .rstb(1'b0),.beb(1'b0),.dob());
    wire[23:0] bg;
    wire[2:0] av_amount=av_enable ? av_levels[5:3]:3'd0;
    av_tone_rom u_av_tone(.clk(clk),.amount(av_amount),.rgb(focus_rgb),.result(bg));
    reg bar, osd_box, osd_enable, latin_enable, chinese_enable;
    reg adjust_border_d,adjust_fill_d,contrast_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bar<=0; osd_box<=0; osd_enable<=0;
            latin_enable<=0; chinese_enable<=0;
            adjust_border_d<=0;adjust_fill_d<=0;contrast_d<=0;
        end else begin
            bar<=scroll_bar_region;
            osd_box<=card_area;
            osd_enable<=de && in_osd_x && in_osd_y && osd_in_text_line && osd_in_char_x;
            adjust_border_d<=adjust_bar_border || card_edge;
            adjust_fill_d<=adjust_bar_fill;
            contrast_d<=bar_contrast;
            latin_enable<=!scroll_lang && use_scroll_font && scroll_in_char_x;
            chinese_enable<=scroll_lang && scroll_bar_region &&
                hanzi_in_text_x && hanzi_in_text_y && hanzi_in_char_x;
        end
    end
    wire [23:0] dark_bg={1'b0,bg[23:17],1'b0,bg[15:9],1'b0,bg[7:1]};
    wire [23:0] star_rgb;
    osd_star_blend_dsp star_r(bg[23:16],av_color[23:16],av_opacity,star_rgb[23:16]);
    osd_star_blend_dsp star_g(bg[15:8],av_color[15:8],av_opacity,star_rgb[15:8]);
    osd_star_blend_dsp star_b(bg[7:0],av_color[7:0],av_opacity,star_rgb[7:0]);
    // Four-level coverage gives antialiased edges with shift/add blending.
    wire[1:0] bar_coverage=latin_enable ? latin_pixel : chinese_enable ? chinese_pixel : 2'd0;
    wire[23:0] card_bg=adjust_border_d ? 24'h387088 :
        adjust_fill_d ? (contrast_d ? 24'ha080f0 : 24'h00d8ff) : 24'h142838;
    wire[23:0] text_base=(presentation_text_coverage!=0) ? bg : bar ? dark_bg : card_bg;
    wire[1:0] text_coverage=(presentation_text_coverage!=0) ? presentation_text_coverage :
        bar ? bar_coverage : osd_enable ? latin_pixel : 2'd0;
    wire [23:0] text_rgb;
    osd_font_blend_native font_red(text_base[23:16],text_coverage,text_rgb[23:16]);
    osd_font_blend_native font_green(text_base[15:8],text_coverage,text_rgb[15:8]);
    osd_font_blend_native font_blue(text_base[7:0],text_coverage,text_rgb[7:0]);

    // Fixed white presentation glyphs reduce the final RGB selection logic.
    assign rgb_out = presentation_hotspot ? 24'hffff00 :
        presentation_paint ? 24'hffffff :
        (presentation_text_coverage!=0) ? text_rgb :
        magnifier_border ? 24'hffff00 : magnifier_inside ? magnifier_pixel : bar ?
        text_rgb :
        osd_box ? text_rgb : av_paint ? star_rgb : bg;
end else begin : combinational_font
    assign rgb_out = presentation_hotspot ? 24'hffff00 :
        presentation_paint ? 24'hffffff :
        magnifier_border ? 24'hffff00 : magnifier_inside ? magnifier_pixel : legacy_rgb;
end endgenerate
    // Instantiate after the coordinate declarations. A part-select of a
    // forward reference is rejected as an implicit scalar by strict Verilog.
    osd_cell12_dsp status_cell_decode(rel_x[8:0],osd_cell);
    osd_cell12_dsp scroll_cell_decode(scroll_rel_x[8:0],scroll_cell);
    osd_cell18_dsp hanzi_cell_decode(hanzi_rel_x[7:0],hanzi_cell);
endmodule

// Same combinational pixel latency and exact rounded /16 blend as before.
// Explicit TD inference uses available DSPs rather than congested LUT fabric.
module osd_star_blend_dsp(input[7:0] background,foreground,input[4:0] opacity,
    output[7:0] pixel);
    wire signed[8:0] delta=$signed({1'b0,foreground})-$signed({1'b0,background});
    (* dsp_style="yes" *) wire signed[14:0] weighted;
    assign weighted=delta*$signed({1'b0,opacity});
    wire signed[14:0] total=$signed({1'b0,background,4'b0000})+weighted+15'sd8;
    assign pixel=total[11:4];
endmodule

// Fixed-latency, 352-cycle binary-to-BCD conversion. The published result is
// atomic; changing inputs are picked up on the next conversion, not mid-word.
module osd_decimal32(input clk,rst,input[31:0] value,output reg[39:0] decimal);
    reg[31:0] binary;
    reg[39:0] digits;
    reg[5:0] remaining;
    reg[3:0] digit_step;
    wire[3:0] corrected=digits[3:0]>=5 ? digits[3:0]+4'd3 : digits[3:0];
    always @(posedge clk or posedge rst)
        if(rst)begin binary<=0;digits<=0;remaining<=0;decimal<=0;digit_step<=0;end
        else if(remaining==0)begin binary<=value;digits<=0;remaining<=32;digit_step<=0;end
        else if(digit_step<10)begin
            // Rotate once per decimal digit; ten rotations restore digit order.
            digits<={corrected,digits[39:4]};
            digit_step<=digit_step+1'b1;
        end
        else begin
            binary<={binary[30:0],1'b0};
            digits<={digits[38:0],binary[31]};
            remaining<=remaining-1'b1;
            digit_step<=0;
            if(remaining==1)decimal<={digits[38:0],binary[31]};
        end
endmodule

// Shared digit-serial converter alternates the two status fields.
module osd_decimal_pair(input clk,rst,start,input[7:0] index_value,count_value,
    output reg[11:0] index_decimal,count_decimal);
    reg[7:0] binary;
    reg[11:0] digits;
    reg[3:0] remaining;
    reg[1:0] digit_step;
    reg field;
    wire[3:0] corrected=digits[3:0]>=5 ? digits[3:0]+4'd3 : digits[3:0];
    always @(posedge clk or posedge rst)
        if(rst)begin remaining<=0;digit_step<=0;field<=0;index_decimal<=0;count_decimal<=0;end
        else if(remaining==0)begin
          if(start)begin
            binary<=index_value;field<=0;
            digits<=0;remaining<=8;digit_step<=0;
          end
        end else if(digit_step<3)begin
            digits<={corrected,digits[11:4]};digit_step<=digit_step+1'b1;
        end else begin
            binary<={binary[6:0],1'b0};digits<={digits[10:0],binary[7]};
            remaining<=remaining-1'b1;digit_step<=0;
            if(remaining==1)begin
                if(field)count_decimal<={digits[10:0],binary[7]};
                else begin
                    index_decimal<={digits[10:0],binary[7]};
                    binary<=count_value;digits<=0;remaining<=8;
                    field<=1;
                end
            end
        end
endmodule

// Callers guard twelve-pixel coordinates below 360 and eighteen-pixel
// coordinates below 162. Constant reciprocals are exact in these domains.
// No pixel clock or pipeline stage is added.
// Historical helper name retained. Arithmetic is entirely LUT logic.
module osd_cell12_dsp(input [8:0] v,output [8:0] cell_code);
 wire [7:0] quotient;
 osd_div3_radix4 division({1'b0,v[8:2]},quotient);
 // Preserve the native reciprocal's five-bit quotient truncation even
 // outside the actual call domain, and its modulo4 remainder expression.
 wire [4:0] q=quotient[4:0];
 wire [1:0] q3={q[0],1'b0}+q[1:0];
 wire [1:0] remainder=v[3:2]-q3;
 assign cell_code={q,remainder,v[1:0]};
endmodule
module osd_cell18_dsp(input [7:0] v,output [8:0] cell_code);
 wire [7:0] by3,by9;
 osd_div3_radix4 first({1'b0,v[7:1]},by3);
 osd_div3_radix4 second(by3,by9);
 wire [3:0] q=by9[3:0];
 wire [4:0] q18={q[0],4'd0}+{q,1'b0};
 wire [4:0] remainder=v[4:0]-q18;
 assign cell_code={q,remainder};
endmodule

// Exact original four-level white glyph coverage, with no pixel delay.
// q=base>>2; r=base&3. The wanted values are 4*q+r,
// 3*q+63+r, 2*q+127+(r>>1), and 255. Every result is <=255.
// (256*q+1)*(256*offset+weight) has that value in bits[15:8]:
// the low digit is weight<=4, the high digit is q*offset.
// Thus neither neighbouring base-256 digit carries into the result.
module osd_font_blend_native(input [7:0] base,input [1:0] alpha,output [7:0] pixel);
 reg [7:0] offset;reg [2:0] weight;
 always @*case(alpha)
  0:begin offset={6'd0,base[1:0]};weight=4;end
  1:begin offset=8'd63+{6'd0,base[1:0]};weight=3;end
  2:begin offset=8'd127+{7'd0,base[1]};weight=2;end
  default:begin offset=255;weight=0;end
 endcase
 wire [35:0] product;
 EG_PHY_MULT18 #(.INPUTREGA("DISABLE"),.INPUTREGB("DISABLE"),
  .OUTPUTREG("DISABLE"),.MODE("MULT18X18C"),.SOURCEAMUX("1"),.SOURCEBMUX("1")) coverage(
  .a({4'd0,base[7:2],8'd1}),.b({2'd0,offset,5'd0,weight}),.p(product),
  .signeda(1'b0),.signedb(1'b0),.acin(),.bcin(),.acout(),.bcout(),
  .sourcea(1'b1),.sourceb(1'b1),.cea(1'b0),.ceb(1'b0),.cepd(1'b0),
  .clk(1'b0),.rstan(1'b1),.rstbn(1'b1),.rstpdn(1'b1));
 assign pixel=product[15:8];
endmodule

// OSD helper multipliers use normal a/b inputs only. Cascade ports are unused.

// Exact unsigned division by3 in four radix4 digit steps.
// A step sees 4*remainder+next_digit in0..11, yielding a 2bit
// quotient digit and a remainder in0..2. No multiply or divider macro.
module osd_div3_radix4(input [7:0] number,output [7:0] quotient);
 function [3:0] digit;
  input [3:0] v;
  begin case(v)
   0:digit=4'b0000;1:digit=4'b0001;2:digit=4'b0010;
   3:digit=4'b0100;4:digit=4'b0101;5:digit=4'b0110;
   6:digit=4'b1000;7:digit=4'b1001;8:digit=4'b1010;
   9:digit=4'b1100;10:digit=4'b1101;11:digit=4'b1110;
   default:digit=4'b0000;
  endcase end
 endfunction
 wire [1:0] remainder [0:4];
 assign remainder[0]=0;
 genvar i;
 generate for(i=0;i<4;i=i+1)begin:step
  wire [3:0] decoded=digit({remainder[i],number[7-2*i -:2]});
  assign quotient[7-2*i -:2]=decoded[3:2];
  assign remainder[i+1]=decoded[1:0];
 end endgenerate
endmodule
