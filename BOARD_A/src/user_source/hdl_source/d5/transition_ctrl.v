// Video-clock transition state. Mode changes select the *next* transition;
// the active mode is frozen when a prepared image starts animating.
module transition_ctrl #(
    parameter integer DEFAULT_SPEED=1
)(
    input wire clk,rst,frame_tick,
    input wire transition_prev_event,transition_next_event,
    input wire transition_prepare,
    input wire transition_start,
    input wire speed_valid,input wire [2:0] speed_value,
    output reg [2:0] transition_mode,
    output reg [2:0] active_mode,
    output reg transition_busy,transition_done,
    output reg [5:0] transition_counter,
    output wire [9:0] transition_pos,
    output reg [2:0] transition_speed
);
    reg [2:0] frame_div;
    reg mode_prepared;
    wire shorter_mode=active_mode==3'd6;
    wire fade_mode=active_mode==3'd1;
    wire [2:0] frame_period=fade_mode ? 3'd2 : transition_speed;
    wire last_step=active_mode==3'd0 ||
                   (fade_mode && transition_counter==6'd63) ||
                   (shorter_mode && transition_counter==6'd31) ||
                   (!fade_mode && !shorter_mode && transition_counter==6'd39);
    assign transition_pos=(active_mode==3'd0) ?
                              (transition_counter==0 ? 10'd0 : 10'd640) :
                          (active_mode==3'd4) ?
                              ({1'b0,transition_counter,3'b0}+
                               {2'b0,transition_counter,2'b0}) :
                          (active_mode==3'd5) ? {1'b0,transition_counter,3'b0} :
                          (active_mode==3'd6) ? {3'b0,transition_counter,1'b0} :
                          (active_mode==3'd1) ?
                              ({transition_counter,4'b0}+
                               {2'b0,transition_counter,2'b0}) :
                              {transition_counter,4'b0};
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            transition_mode<=3'd1;
            active_mode<=3'd1;
            transition_busy<=0; transition_done<=0;
            transition_counter<=0;
            transition_speed<=DEFAULT_SPEED;
            frame_div<=0;mode_prepared<=0;
        end else begin
            transition_done<=0;
            if(transition_next_event)
                transition_mode<=transition_mode==3'd6 ? 3'd0 : transition_mode+1'b1;
            else if(transition_prev_event)
                transition_mode<=transition_mode==3'd0 ? 3'd6 : transition_mode-1'b1;
            if(speed_valid) transition_speed<=speed_value==0 ? 3'd1 : speed_value;
            if(transition_prepare) begin
                active_mode<=transition_mode;
                mode_prepared<=1;
            end
            if(transition_start && !transition_busy) begin
                if(!mode_prepared) active_mode<=transition_mode;
                mode_prepared<=0;
                transition_counter<=0;frame_div<=0;
                transition_busy<=1;
            end else if(frame_tick && transition_busy) begin
                if(frame_div+1'b1 < frame_period) frame_div<=frame_div+1'b1;
                else begin
                    frame_div<=0;
                    if(last_step) begin
                        transition_counter<=active_mode==3'd0 ? 6'd1 :
                                            fade_mode ? 6'd63 : shorter_mode ? 6'd32 : 6'd40;
                        transition_busy<=0;transition_done<=1;
                    end else begin
                        transition_counter<=transition_counter+1'b1;
                    end
                end
            end
        end
    end
endmodule
