// Composes two aligned 640x480 RGB888 streams. Both inputs must describe the
// same pixel_x/pixel_y; selection and state are constant across a video frame.
module transition_compositor #(parameter POSITION_FROM_COUNTER=0)(
    input wire [23:0] old_rgb,new_rgb,
    input wire [9:0] pixel_x,pixel_y,
    input wire [2:0] transition_mode,
    input wire [9:0] transition_pos,
    input wire [5:0] transition_counter,
    input wire transition_busy,
    output wire [23:0] transition_rgb,
    output wire select_new
);
    wire [5:0] alpha=transition_counter>6'd32 ? 6'd32 : transition_counter;
    wire [5:0] old_alpha=6'd32-alpha;
    wire [13:0] red_mix=old_rgb[23:16]*old_alpha+new_rgb[23:16]*alpha;
    wire [13:0] green_mix=old_rgb[15:8]*old_alpha+new_rgb[15:8]*alpha;
    wire [13:0] blue_mix=old_rgb[7:0]*old_alpha+new_rgb[7:0]*alpha;
    wire [23:0] blended={red_mix[12:5],green_mix[12:5],blue_mix[12:5]};
    // FADE selects the whole old/new frame at its black midpoint.
    // The full RGB alpha blend remains available when two RGB streams exist.
    wire [5:0] dissolve_threshold={pixel_x[0],pixel_y[0],pixel_x[1],
                                  pixel_y[1],pixel_x[2],pixel_y[2]};
    wire choose_new;
    generate if(POSITION_FROM_COUNTER) begin : gen_narrow_read_mask
        wire [6:0] top_edge={transition_counter,1'b0}+
                            {1'b0,transition_counter};
        wire [6:0] center_group=pixel_x[9:3];
        wire [6:0] center_distance=center_group<7'd40 ?
                                    7'd39-center_group : center_group-7'd40;
        wire new_ltr=pixel_x[9:4]<transition_counter;
        wire new_rtl=pixel_x[9:4]>=6'd40-transition_counter;
        wire new_top=pixel_y[8:2]<top_edge;
        wire new_center=center_distance<{1'b0,transition_counter};
        wire new_blinds=pixel_x[5:1]<transition_counter;
        assign choose_new=(transition_mode==3'd0) ||
                          (transition_mode==3'd1 && transition_counter>=6'd32) ||
                          (transition_mode==3'd2 && new_ltr) ||
                          (transition_mode==3'd3 && new_rtl) ||
                          (transition_mode==3'd4 && new_top) ||
                          (transition_mode==3'd5 && new_center) ||
                          (transition_mode==3'd6 && new_blinds);
    end else begin : gen_position_mask
        assign choose_new=(transition_mode==3'd1) ?
                              transition_counter>=6'd32 :
                          (transition_mode==3'd2) ? pixel_x<transition_pos :
                          (transition_mode==3'd3) ? pixel_x>=10'd640-transition_pos :
                          (transition_mode==3'd4) ? pixel_y<transition_pos :
                          (transition_mode==3'd5) ?
                              (pixel_x>=10'd320-transition_pos &&
                               pixel_x<10'd320+transition_pos) :
                          (transition_mode==3'd6) ? {4'b0,pixel_x[5:0]}<transition_pos :
                          1'b1;
    end endgenerate
    assign select_new=!transition_busy || choose_new;
    assign transition_rgb=!transition_busy || transition_mode==3'd0 ? new_rgb :
                          transition_mode==3'd1 ? blended :
                          choose_new ? new_rgb : old_rgb;
endmodule
