// Confirmed NT35510 display + user-confirmed GT9147 touch; no vendor primitives.
// Add to a separate TD project first. Do not use unverified package pin maps.
module lcd_touch_demo_top #(
    parameter integer CLK_HZ=50000000, FM_ENABLE=0,SCAN_ENABLE=0,SHARED_FONT=0,VIDEO_ENABLE=0,DSP_ENABLE=0,TIME_ENABLE=0,
    parameter integer PROFILE_CONFIRMED=1,
    parameter integer SWAP_XY=0, INVERT_X=0, INVERT_Y=0
)(
    input wire clk,rst_n,
    inout wire [15:0] lcd_d,
    output wire lcd_cs_n,lcd_rs,lcd_wr_n,lcd_rd_n,lcd_rst_n,lcd_bl,
    inout wire tp_scl,tp_sda,tp_int_n,
    output wire tp_rst_n,
    input wire [47:0] fm_status,
    output wire fm_action_valid, output wire [3:0] fm_action_id,
    output wire action_valid, output wire [3:0] action_id,
    output wire compare_valid, output wire [3:0] compare_cmd,
    input wire [2:0] transition_mode_video,
    output wire transition_prev_event,transition_next_event,
    output wire prev_image_event,next_image_event,
    input wire scan_point_valid,input wire [31:0] scan_point,
    input wire [127:0] scan_status,output wire scan_action_valid,output wire [3:0] scan_action_id,output wire scan_page_active,
    input wire [127:0] dsp_status,output wire dsp_action_valid,output wire [3:0] dsp_action_id,
    input [23:0] time_status,output time_action_valid,output [3:0] time_action_id,
    
    output wire init_done,init_fault,touch_fault,touch_identified,
    input wire [15:0] video_status,output wire video_action_valid,output wire [3:0] video_action_id,
    output wire [10:0] shared_font_addr,input wire [15:0] shared_font_bits
);
    // Asynchronous assertion, synchronous release into the only logic clock.
    reg [1:0] reset_sync;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) reset_sync<=0; else reset_sync<={reset_sync[0],1'b1};
    wire reset_n=reset_sync[1];
    wire [9:0] rom_addr;
    wire [31:0] rom_word;
    wire valid,down;
    wire [15:0] tx,ty;
    wire [31:0] product_id;
    wire [15:0] ax=SWAP_XY ? ty : tx;
    wire [15:0] ay=SWAP_XY ? tx : ty;
    wire in_range=(ax<480 && ay<800);
    // The touch engine reports 16-bit coordinates, but only in-range touches
    // can generate actions. Pass bounded coordinates to the UI arithmetic.
    wire [8:0] px=INVERT_X ? 9'd479-ax[8:0] : ax[8:0];
    wire [9:0] py=INVERT_Y ? 10'd799-ay[9:0] : ay[9:0];
    reg [2:0] transition_mode_meta,transition_mode_sync;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin transition_mode_meta<=3'd1;transition_mode_sync<=3'd1;end
        else begin
            transition_mode_meta<=transition_mode_video;
            transition_mode_sync<=transition_mode_meta;
        end
    end
    lcd_nt35510_init_rom #(.CLK_HZ(CLK_HZ)) rom(.clk(clk),.addr(rom_addr),.word(rom_word));
    touch_gt9147 #(.CLK_HZ(CLK_HZ),
        .SENSOR_WIDTH(SWAP_XY ? 800 : 480),.SENSOR_HEIGHT(SWAP_XY ? 480 : 800)) touch(
        .clk(clk),.rst_n(reset_n),.enable(1'b1),
        .scl(tp_scl),.sda(tp_sda),.int_n(tp_int_n),.touch_rst_n(tp_rst_n),
        .valid(valid),.down(down),.x(tx),.y(ty),
        .identified(touch_identified),.fault(touch_fault),.product_id(product_id));
    lcd_ui_core #(.CLK_HZ(CLK_HZ),.FM_ENABLE(FM_ENABLE),.SCAN_ENABLE(SCAN_ENABLE),.SHARED_FONT(SHARED_FONT),.VIDEO_ENABLE(VIDEO_ENABLE),.DSP_ENABLE(DSP_ENABLE),.TIME_ENABLE(TIME_ENABLE)) ui(
        .clk(clk),.rst_n(reset_n),.profile_confirmed(PROFILE_CONFIRMED!=0),
        .init_addr(rom_addr),.init_word(rom_word),
        .touch_valid(valid),.touch_down(down && in_range),.touch_x(px),.touch_y(py),
        .time_status(time_status),.time_action_valid(time_action_valid),.time_action_id(time_action_id),.dsp_status(dsp_status),.dsp_action_valid(dsp_action_valid),.dsp_action_id(dsp_action_id),.scan_point_valid(scan_point_valid),.scan_point(scan_point),.scan_status(scan_status),.scan_action_valid(scan_action_valid),.scan_action_id(scan_action_id),.scan_page_active(scan_page_active),
        .video_status(video_status),.video_action_valid(video_action_valid),.video_action_id(video_action_id),
        .fm_status(fm_status),.fm_action_valid(fm_action_valid),.fm_action_id(fm_action_id),
        .action_valid(action_valid),.action_id(action_id),
        .compare_valid(compare_valid),.compare_cmd(compare_cmd),
        .transition_mode_display(transition_mode_sync),
        .transition_prev_event(transition_prev_event),
        .transition_next_event(transition_next_event),
        .prev_image_event(prev_image_event),.next_image_event(next_image_event),
        .lcd_cs_n(lcd_cs_n),.lcd_rs(lcd_rs),.lcd_wr_n(lcd_wr_n),.lcd_rd_n(lcd_rd_n),
        .lcd_rst_n(lcd_rst_n),.lcd_d(lcd_d),.lcd_bl(lcd_bl),
        .init_done(init_done),.init_fault(init_fault),.shared_font_addr(shared_font_addr),.shared_font_bits(shared_font_bits));
endmodule
