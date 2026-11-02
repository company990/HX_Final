// NT35510 16-bit indexed-register demo, portrait 480x800 by default.
// Does not include the panel-specific power/gamma initialization ROM.
// Touch input must already be in display pixel coordinates, same clk domain.
module lcd_ui_core #(
    parameter integer CLK_HZ=50000000, WIDTH=480, HEIGHT=800, FM_ENABLE=0,SCAN_ENABLE=0,SHARED_FONT=0,VIDEO_ENABLE=0,DSP_ENABLE=0,TIME_ENABLE=0
)(
    input wire clk, rst_n, profile_confirmed,
    output wire [9:0] init_addr, input wire [31:0] init_word,
    input wire touch_valid, touch_down,
    input wire [15:0] touch_x, touch_y,
    input wire [47:0] fm_status,
    output reg fm_action_valid, output reg [3:0] fm_action_id,
    output wire action_valid, output wire [3:0] action_id,
    output wire compare_valid, output wire [3:0] compare_cmd,
    input wire [2:0] transition_mode_display,
    output wire transition_prev_event,transition_next_event,
    output wire prev_image_event,next_image_event,
    output wire lcd_cs_n, lcd_rs, lcd_wr_n, lcd_rd_n, lcd_rst_n,
    inout wire [15:0] lcd_d,
    input wire scan_point_valid,input wire [31:0] scan_point,
    input wire [127:0] scan_status,output reg scan_action_valid,output reg [3:0] scan_action_id,output wire scan_page_active,
    input wire [127:0] dsp_status,output reg dsp_action_valid,output reg [3:0] dsp_action_id,
    input [23:0] time_status,output reg time_action_valid,output reg [3:0] time_action_id,
    output wire lcd_bl, init_done, init_fault,
    input wire [15:0] video_status,output wire video_action_valid,output wire [3:0] video_action_id,
    output wire [10:0] shared_font_addr,input wire [15:0] shared_font_bits
);
    wire iv, irs, br, bd;
    wire [15:0] idata, rd_unused;
    reg dv, drs;
    reg [15:0] ddata;
    lcd_init_engine #(.CLK_HZ(CLK_HZ)) init_ctl(
        .clk(clk),.rst_n(rst_n),.profile_confirmed(profile_confirmed),
        .lcd_rst_n(lcd_rst_n),.rom_addr(init_addr),.rom_word(init_word),
        .req_valid(iv),.req_rs(irs),.req_data(idata),
        .req_ready(br && !init_done),.bus_done(bd),
        .init_done(init_done),.init_fault(init_fault));
    // Conservative 50MHz pin timing: 100ns setup, 160ns WR low, 100ns hold.
    // The video engine increases concurrent switching on the physical board.
    lcd8080_bus #(.SETUP_CYCLES(5),.WR_LOW_CYCLES(8),.HOLD_CYCLES(5)) bus_ctl(.clk(clk),.rst_n(rst_n),
        .req_valid(init_done ? dv : iv),.req_read(1'b0),
        .req_rs(init_done ? drs : irs),.req_data(init_done ? ddata : idata),
        .req_ready(br),.done(bd),.rd_data(rd_unused),
        .lcd_cs_n(lcd_cs_n),.lcd_rs(lcd_rs),.lcd_wr_n(lcd_wr_n),
        .lcd_rd_n(lcd_rd_n),.lcd_d(lcd_d));

    // UI geometry matches the supplied sketch: title column, 3 button columns,
    // 5 rows in the upper 5/8, yellow background below, cyan title labels.
    // Buttons only generate an optional event: no application-side actions.
    reg time_page,ftime_page,time_back_lock;
    wire time_entry=TIME_ENABLE && touch_x>=192 && touch_x<288 && touch_y>=704 && touch_y<800;
    reg dsp_page,fdsp_page;
    wire dsp_entry=DSP_ENABLE && touch_x>=96 && touch_x<192 && touch_y>=704 && touch_y<800;
    reg fm_page,fm_held,fm_pressed,fpage;reg scan_page,fscan_page;reg scan_graph,fscan_graph;
    reg video_page,fvideo_page,video_event;reg [3:0] video_id;
    assign video_action_valid=video_event;assign video_action_id=video_id;
    wire video_entry=VIDEO_ENABLE && !fm_page && !dsp_page && !time_page && !time_back_lock && touch_x>=288 && touch_x<384 && touch_y>=704 && touch_y<800;
    assign scan_page_active=SCAN_ENABLE && scan_page;
    wire scan_entry=SCAN_ENABLE && fm_page && !video_page && !scan_page && !dsp_page && !time_page && touch_x>=32 && touch_x<336 && touch_y>=80 && touch_y<128;
    reg [3:0] fm_pressed_id;
    wire home_fm_entry=FM_ENABLE && touch_x>=160 && touch_x<320 && touch_y>=400 && touch_y<480;
    wire entry=home_fm_entry || (FM_ENABLE && touch_x>=384 && touch_x<480 && touch_y>=704 && touch_y<800);
    wire back=touch_x>=360 && touch_x<480 && touch_y<100;
    wire graph_controls=SCAN_ENABLE && scan_page && scan_graph;
    wire scan_view_toggle=SCAN_ENABLE && scan_page && touch_x>=32 && touch_x<336 && touch_y>=80 && touch_y<128;
    wire fm_button=touch_x<480 && (graph_controls ? (touch_y>=500 && touch_y<700) : (touch_y>=200 && touch_y<600));
    wire [1:0] fm_col=touch_x<120 ? 0 : touch_x<240 ? 1 : touch_x<360 ? 2 : 3;
    wire [1:0] fm_row=graph_controls ? (touch_y<600 ? 0 : 1) : touch_y<300 ? 0 : touch_y<400 ? 1 : touch_y<500 ? 2 : 3;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin video_page<=0;video_event<=0;video_id<=0;time_page<=0;time_back_lock<=0;time_action_valid<=0;time_action_id<=0;dsp_page<=0;dsp_action_valid<=0;dsp_action_id<=0;scan_graph<=1;scan_page<=0;scan_action_valid<=0;scan_action_id<=0;fm_page<=0;fm_held<=0;fm_action_valid<=0;fm_action_id<=0;fm_pressed<=0;fm_pressed_id<=0;end
        else begin
            video_event<=0;            time_action_valid<=0;fm_action_valid<=0;scan_action_valid<=0;dsp_action_valid<=0;
            if(touch_valid)begin
                fm_pressed<=(FM_ENABLE && fm_page || DSP_ENABLE && dsp_page || TIME_ENABLE && time_page) && touch_down && fm_button;
                if(!touch_down)begin fm_held<=0;time_back_lock<=0;end
                else if(!fm_held)begin
                    fm_held<=1;
                    if(!fm_page && !dsp_page && !time_page && !time_back_lock && video_entry)begin fm_page<=1;video_page<=1;end
                    else if(!fm_page && !dsp_page && !time_page && !time_back_lock && time_entry)time_page<=1;
                    else if(time_page && (back || fm_button && {fm_row,fm_col}==7))begin time_page<=0;time_back_lock<=1;end
                    else if(time_page && fm_button && {fm_row,fm_col}<7)begin time_action_valid<=1;time_action_id<={fm_row,fm_col};fm_pressed_id<={fm_row,fm_col};end
                    else if(!fm_page && !dsp_page && !time_page && !time_back_lock && dsp_entry)dsp_page<=1;
                    else if(dsp_page && (back || fm_button && {fm_row,fm_col}==15))begin dsp_page<=0;time_back_lock<=1;end
                    else if(dsp_page && fm_button)begin dsp_action_valid<=1;dsp_action_id<={fm_row,fm_col};fm_pressed_id<={fm_row,fm_col};end
                    else if(!fm_page && !dsp_page && !time_page && !time_back_lock && entry)fm_page<=1;
                    else if(scan_entry)begin scan_page<=1;scan_graph<=1;end
                    else if(scan_view_toggle)scan_graph<=~scan_graph;
                    else if(FM_ENABLE && fm_page && back)begin if(video_page)begin video_event<=1;video_id<=2;video_page<=0;fm_page<=0;time_back_lock<=1;if(video_status[15])begin fm_action_valid<=1;fm_action_id<=2;end end
                        else if(scan_page)scan_page<=0;else begin fm_page<=0;time_back_lock<=1;end end
                    else if(FM_ENABLE && fm_page && fm_button)begin
                        if(VIDEO_ENABLE && video_page)begin
                         if({fm_row,fm_col}<8)begin video_event<=1;video_id<={fm_row,fm_col};end
                         if({fm_row,fm_col}==0 || {fm_row,fm_col}==3 || {fm_row,fm_col}==4 || {fm_row,fm_col}==7 || ({fm_row,fm_col}==2 && video_status[15]))begin fm_action_valid<=1;fm_action_id<=2;end
                         else if({fm_row,fm_col}==5)begin fm_action_valid<=1;fm_action_id<=3;end
                        end else if(SCAN_ENABLE && scan_page)begin
                         if({fm_row,fm_col}==15)scan_page<=0;else begin scan_action_valid<=1;scan_action_id<={fm_row,fm_col};end
                        end else begin fm_action_valid<=1;fm_action_id<={fm_row,fm_col};end
                        fm_pressed_id<={fm_row,fm_col};
                    end
                end
            end
        end
    end
    wire media_action_valid;wire [3:0] media_action_id;
    assign action_valid=media_action_valid && !fm_page && !dsp_page && !time_page && !time_back_lock;
    assign action_id=media_action_id;
    reg pressed;
    reg [3:0] pressed_id;
    wire button;
    wire [3:0] hit;
    lcd_ui_layout #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) hit_test(
        .x(touch_x),.y(touch_y),.in_panel(),.is_button(button),
        .row(),.col(),.button_id(hit),.left(),.top(),.cell_width(),.label_id());
    lcd_adjust_repeat #(.CLK_HZ(CLK_HZ)) adjust_actions (
        .clk(clk),.rst_n(rst_n),.touch_valid(touch_valid),
        .touch_down(touch_down),.button(button && !home_fm_entry && !fm_page && !dsp_page && !time_page && !time_back_lock),.button_id(hit),
        .action_valid(media_action_valid),.action_id(media_action_id));
    // Five dedicated controls in the otherwise unused strip below the media UI.
    // 0: compare on/off, 1: left, 2: center, 3: right, 4: next mode.
    wire compare_button=!fm_page && !dsp_page && !time_page && !time_back_lock && touch_x<WIDTH && touch_y>=480 && touch_y<552;
    wire [3:0] compare_hit=(touch_x<WIDTH/5) ? 4'd0 :
                           (touch_x<2*WIDTH/5) ? 4'd1 :
                           (touch_x<3*WIDTH/5) ? 4'd2 :
                           (touch_x<4*WIDTH/5) ? 4'd3 : 4'd4;
    wire effect_area=!fm_page && !dsp_page && !time_page && !time_back_lock && touch_x<WIDTH && touch_y>=624 && touch_y<704;
    wire[1:0] effect_hit=touch_x<120 ? 0 : touch_x<240 ? 1 : touch_x<360 ? 2 : 3;
    reg[4:0] effect_settings;reg [2:0] beat_settings;
    wire transition_area=!fm_page && !dsp_page && !time_page && !time_back_lock && touch_x<WIDTH && touch_y>=552 && touch_y<624;
    // Two formerly empty transition cells now explicitly start/stop the border.
    // Commands 13/14 do not share the star, clap, prediction or transition keys.
    wire frame_area=transition_area && ((touch_x>=96 && touch_x<192) || (touch_x>=288 && touch_x<384));
    wire beat_area=1'b0; // Prediction remains disabled; the border uses observed events.
    reg compare_valid_r,compare_held;
    reg [3:0] compare_cmd_r;
    assign compare_valid=compare_valid_r;
    assign compare_cmd=compare_cmd_r;
    assign prev_image_event=action_valid && action_id==4'd3;
    assign next_image_event=action_valid && action_id==4'd5;
    reg transition_held,transition_prev_r,transition_next_r;
    assign transition_prev_event=transition_prev_r;
    assign transition_next_event=transition_next_r;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            transition_held<=0;transition_prev_r<=0;transition_next_r<=0;
        end else begin
            transition_prev_r<=0;transition_next_r<=0;
            if(touch_valid) begin
                if(!touch_down || !transition_area) transition_held<=0;
                else if(!transition_held) begin
                    transition_held<=1;
                    transition_prev_r<=touch_x<WIDTH/5;
                    transition_next_r<=touch_x>=4*WIDTH/5;
                end
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            compare_valid_r<=0;compare_held<=0;compare_cmd_r<=0;effect_settings<=5'b00011;beat_settings<=3'b010;
        end else begin
            compare_valid_r<=0;
            if(touch_valid) begin
                if(!touch_down || (!compare_button && !effect_area && !beat_area && !frame_area)) compare_held<=0;
                else if(!compare_held) begin
                    compare_held<=1;
                    compare_valid_r<=1;
                    compare_cmd_r<=frame_area ? (touch_x<192 ? 4'd13:4'd14) : beat_area ? (touch_x<2*WIDTH/5 ? 4'd11:4'd12) : effect_area ? 4'd7+effect_hit : compare_hit;
                    if(frame_area)beat_settings[0]<=touch_x<192;
                    if(beat_area)begin
                     if(touch_x<2*WIDTH/5)beat_settings[0]<=~beat_settings[0];
                     else beat_settings[2:1]<=beat_settings[2:1]==2 ? 0:beat_settings[2:1]+1'b1;
                    end
                    if(effect_area)case(effect_hit)
                     0:effect_settings[0]<=~effect_settings[0];
                     1:effect_settings[2:1]<=effect_settings[2:1]==2 ? 0:effect_settings[2:1]+1'b1;
                     2:effect_settings[3]<=~effect_settings[3];
                     3:effect_settings[4]<=~effect_settings[4];
                    endcase
                end
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pressed<=0; pressed_id<=0;
        end else begin
            if(touch_valid) begin
                pressed<=touch_down && button && !fm_page && !dsp_page && !time_page && !time_back_lock;
                if(touch_down && button) pressed_id<=hit;
            end
        end
    end
    lcd_backlight_pwm #(.DIVIDER(10)) pwm(.clk(clk),.rst_n(rst_n),
        .enable(init_done),.duty(9'd192),.pwm(lcd_bl));

    // Frame snapshot prevents a half-updated button. A short touch is latched
    // until shown for at least one frame; no short valid pulse gets lost.
    reg fpressed,pending_press;
    reg [3:0] fpressed_id,pending_id;
    // Panel coordinates never exceed WIDTH-1 / HEIGHT-1. Narrow counters also
    // narrow the renderer's address arithmetic after constant propagation.
    localparam integer X_BITS=WIDTH<2 ? 1 : $clog2(WIDTH);
    localparam integer Y_BITS=HEIGHT<2 ? 1 : $clog2(HEIGHT);
    reg [X_BITS-1:0] x;
    reg [Y_BITS-1:0] y;
    reg [5:0] index;
    reg [2:0] pixel_wait_phase;
    reg [1:0] state;
    localparam PREP=0, SEND=1, WAIT_BUS=2, PIXEL_WAIT=3;
    localparam [15:0] XEND=WIDTH-1;
    wire [15:0] row_address=y;
    wire [15:0] pixel,media_pixel,fm_pixel;
    // Full Chinese navigation captions replace the old two-letter badges.
    wire [15:0] selected_pixel=(fpage || fdsp_page || ftime_page) ? fm_pixel : media_pixel;
    // Separate renderer geometry/font selection from the 8080 data register.
    // FM uses two synchronous ROMs; allow one additional cycle for this stage.
    generate if(FM_ENABLE || DSP_ENABLE)begin:pixel_pipeline
        reg [15:0] rendered_pixel;
        always @(posedge clk or negedge rst_n)
            if(!rst_n)rendered_pixel<=0;else rendered_pixel<=selected_pixel;
        assign pixel=rendered_pixel;
    end else begin assign pixel=selected_pixel;end endgenerate
    generate if(FM_ENABLE || DSP_ENABLE || TIME_ENABLE || VIDEO_ENABLE)begin:fm_render
        fm_lcd_pixels #(.PIPELINE_FONT(FM_ENABLE || DSP_ENABLE),.VIDEO_ENABLE(VIDEO_ENABLE),.SCAN_ENABLE(SCAN_ENABLE),.SHARED_FONT(SHARED_FONT),.DSP_ENABLE(DSP_ENABLE),.TIME_ENABLE(TIME_ENABLE)) renderer(.clk(clk),.rst_n(rst_n),.x(x),.y(y),.status(fm_status),
            .time_page(ftime_page),.time_status(time_status),.dsp_page(fdsp_page),.dsp_status(dsp_status),.video_page(fvideo_page),.video_status(video_status),.scan_page(fscan_page),.scan_graph(fscan_graph),.scan_point_valid(scan_point_valid),.scan_point(scan_point),.scan_status(scan_status),.pressed(fm_pressed),.pressed_id(fm_pressed_id),.pixel(fm_pixel),.shared_font_addr(shared_font_addr),.shared_font_bits(shared_font_bits));
    end else begin assign fm_pixel=0;assign shared_font_addr=0;end endgenerate
    lcd_ui_pixels #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) pixels(
        .clk(clk),.x(x),.y(y),.pressed(fpressed),.pressed_id(fpressed_id),
        .beat_settings(beat_settings),.effect_settings(effect_settings),.transition_mode(transition_mode_display),.pixel(media_pixel));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin pending_press<=0;pending_id<=0;end
        else begin
            if(init_done && state==PREP) pending_press<=0;
            if(action_valid) begin pending_press<=1;pending_id<=action_id;end
        end
    end
    always @* begin
        dv=(state==SEND); drs=0; ddata=0;
        // Re-address each 480-pixel row, so a missed/extra physical WR edge
        // cannot turn into an accumulated stride error across the whole screen.
        // NT35510 uses 16-bit REGISTER INDICES;
        // do not replace these with ILI9341's 8-bit command encoding.
        case(index)
            0:ddata=16'h2a00; 1:begin drs=1; ddata=0; end
            2:ddata=16'h2a01; 3:begin drs=1; ddata=0; end
            4:ddata=16'h2a02; 5:begin drs=1; ddata={8'b0,XEND[15:8]}; end
            6:ddata=16'h2a03; 7:begin drs=1; ddata={8'b0,XEND[7:0]}; end
            8:ddata=16'h2b00; 9:begin drs=1; ddata={8'b0,row_address[15:8]}; end
            10:ddata=16'h2b01; 11:begin drs=1; ddata={8'b0,row_address[7:0]}; end
            12:ddata=16'h2b02; 13:begin drs=1; ddata={8'b0,row_address[15:8]}; end
            14:ddata=16'h2b03; 15:begin drs=1; ddata={8'b0,row_address[7:0]}; end
            16:ddata=16'h2c00;
            default:begin drs=1; ddata=pixel; end
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pixel_wait_phase<=0;state<=PREP; index<=0; x<=0; y<=0; fpressed<=0; fpressed_id<=0; fpage<=0;fscan_page<=0;fscan_graph<=1;fvideo_page<=0;fdsp_page<=0;ftime_page<=0;
        end else if(init_done) case(state)
            PREP: begin
                fpage<=fm_page;fscan_page<=scan_page;fscan_graph<=scan_graph;fvideo_page<=video_page;fdsp_page<=dsp_page;ftime_page<=time_page;
                fpressed<=pressed || pending_press;
                fpressed_id<=pressed ? pressed_id : pending_id;
                index<=0; x<=0; y<=0; state<=SEND;
            end
            SEND: if(br) state<=WAIT_BUS;
            PIXEL_WAIT: if(!(FM_ENABLE || DSP_ENABLE) || pixel_wait_phase==6)state<=SEND;else pixel_wait_phase<=pixel_wait_phase+1'b1;
            WAIT_BUS: if(bd) begin
                state<=SEND;
                if(index<17) index<=index+1'b1;
                else begin
                    pixel_wait_phase<=0;state<=PIXEL_WAIT;
                    if(x==WIDTH-1) begin
                        x<=0;
                        if(y==HEIGHT-1) state<=PREP;
                        else begin y<=y+1'b1;index<=0;state<=SEND;end
                    end else x<=x+1'b1;
                end
            end
            default: state<=PREP;
        endcase
    end
endmodule

// Only the four +/- adjustment buttons repeat while held. Other LCD actions
// remain one event per touch, so media controls cannot retrigger on a hold.
module lcd_adjust_repeat #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer DELAY_MS=300,
    parameter integer PERIOD_MS=80
    ,parameter integer COMPARE_REPEAT=0
) (
    input wire clk,rst_n,touch_valid,touch_down,button,
    input wire [3:0] button_id,
    output reg action_valid,
    output reg [3:0] action_id
);
    localparam integer DELAY_CYCLES=(CLK_HZ/1000)*DELAY_MS;
    localparam integer PERIOD_CYCLES=(CLK_HZ/1000)*PERIOD_MS;
    localparam integer MAX_CYCLES=DELAY_CYCLES>PERIOD_CYCLES ?
                                  DELAY_CYCLES : PERIOD_CYCLES;
    localparam integer CW=MAX_CYCLES<2 ? 1 : $clog2(MAX_CYCLES);
    reg held,repeat_armed;
    reg [3:0] held_id;
    reg [CW-1:0] count;
    wire adjustment=COMPARE_REPEAT ? (button_id==1 || button_id==3) :
                    (button_id==6 || button_id==7 ||
                     button_id==9 || button_id==11);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            held<=0;repeat_armed<=0;held_id<=0;count<=0;
            action_valid<=0;action_id<=0;
        end else begin
            action_valid<=0;
            if (touch_valid) begin
                held<=touch_down;
                if (!touch_down || !button) begin
                    repeat_armed<=0;
                    count<=0;
                end else if (!held) begin
                    held_id<=button_id;
                    action_valid<=1;
                    action_id<=button_id;
                    repeat_armed<=adjustment;
                    count<=DELAY_CYCLES-1;
                end else if (button_id!=held_id) begin
                    held_id<=button_id;
                    repeat_armed<=0;
                    count<=0;
                end
            end else if (held && repeat_armed) begin
                if (count==0) begin
                    action_valid<=1;
                    action_id<=held_id;
                    count<=PERIOD_CYCLES-1;
                end else count<=count-1'b1;
            end
        end
    end
endmodule

module lcd_backlight_pwm #(parameter integer DIVIDER=10)(
    input wire clk, rst_n, enable, input wire [8:0] duty,
    output wire pwm
);
    localparam integer DIV_BITS=DIVIDER<2 ? 1 : $clog2(DIVIDER);
    reg [DIV_BITS-1:0] div_count;
    reg [7:0] phase;
    // duty=0 is true off; duty=256 is true 100%.
    assign pwm=enable && ({1'b0,phase}<duty);
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin div_count<=0; phase<=0; end
        else if(div_count==DIVIDER-1) begin div_count<=0; phase<=phase+1'b1; end
        else div_count<=div_count+1'b1;
endmodule
