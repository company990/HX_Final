`timescale 1ns/1ps

module video_transition_fade (

    input             clk,
    input             rst,

    // One-clock pulse for each new video frame
    input             frame_tick,

    // Start one complete:
    // fade-out -> black -> fade-in
    input             transition_start,

    // Original video RGB
    input      [23:0] rgb_in,

    // Faded video RGB
    output     [23:0] rgb_out,

    // Status
    output reg        transition_busy,
    output reg        midpoint_pulse,
    output reg        transition_done,

    // 0  = completely black
    // 32 = 100% original brightness
    output reg [5:0]  fade_level
);


    // =========================================================
    // State machine
    // =========================================================

    localparam [1:0] ST_IDLE     = 2'd0;
    localparam [1:0] ST_FADE_OUT = 2'd1;
    localparam [1:0] ST_FADE_IN  = 2'd2;

    reg [1:0] state;


    // =========================================================
    // transition_start rising-edge detection
    // =========================================================

    reg transition_start_d;

    wire transition_start_pulse;

    assign transition_start_pulse =
        transition_start &&
        !transition_start_d;


    // =========================================================
    // Fade speed divider
    //
    // Change one fade level every TWO video frames.
    //
    // At 60 Hz:
    //
    // 32 steps * 2 frames / 60 Hz
    // ~= 1.07 seconds
    //
    // for fade-out.
    //
    // Fade-in is another ~1.07 seconds.
    // =========================================================

    reg frame_div;


    // =========================================================
    // Fade state machine
    // =========================================================

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            transition_start_d <= 1'b0;

            state           <= ST_IDLE;
            frame_div       <= 1'b0;

            fade_level      <= 6'd32;

            transition_busy <= 1'b0;
            midpoint_pulse  <= 1'b0;
            transition_done <= 1'b0;

        end
        else begin

            transition_start_d <= transition_start;

            // These two outputs are one-clock pulses.
            midpoint_pulse  <= 1'b0;
            transition_done <= 1'b0;


            case (state)

                // =============================================
                // Normal full-brightness display
                // =============================================

                ST_IDLE: begin

                    transition_busy <= 1'b0;

                    fade_level <= 6'd32;
                    frame_div  <= 1'b0;


                    if (transition_start_pulse) begin

                        state <= ST_FADE_OUT;

                        transition_busy <= 1'b1;

                    end

                end


                // =============================================
                // Fade OUT
                //
                // 32 -> 31 -> ... -> 1 -> 0
                // =============================================

                ST_FADE_OUT: begin

                    transition_busy <= 1'b1;


                    if (frame_tick) begin

                        if (frame_div == 1'b0) begin

                            frame_div <= 1'b1;

                        end
                        else begin

                            frame_div <= 1'b0;


                            if (fade_level > 6'd1) begin

                                fade_level <=
                                    fade_level - 6'd1;

                            end
                            else begin

                                // Reach completely black.
                                fade_level <= 6'd0;

                                // Later G1-C1-C will use this
                                // pulse to switch the BMP image
                                // exactly at the black midpoint.
                                midpoint_pulse <= 1'b1;

                                state <= ST_FADE_IN;

                            end

                        end

                    end

                end


                // =============================================
                // Fade IN
                //
                // 0 -> 1 -> ... -> 31 -> 32
                // =============================================

                ST_FADE_IN: begin

                    transition_busy <= 1'b1;


                    if (frame_tick) begin

                        if (frame_div == 1'b0) begin

                            frame_div <= 1'b1;

                        end
                        else begin

                            frame_div <= 1'b0;


                            if (fade_level < 6'd31) begin

                                fade_level <=
                                    fade_level + 6'd1;

                            end
                            else begin

                                // Back to 100% brightness.
                                fade_level <= 6'd32;

                                transition_busy <= 1'b0;
                                transition_done <= 1'b1;

                                state <= ST_IDLE;

                            end

                        end

                    end

                end


                // =============================================
                // Safety fallback
                // =============================================

                default: begin

                    state <= ST_IDLE;

                    frame_div <= 1'b0;

                    fade_level <= 6'd32;

                    transition_busy <= 1'b0;

                end

            endcase

        end

    end


    // =========================================================
    // RGB brightness scaling
    //
    // output = input * fade_level / 32
    //
    // fade_level = 32:
    // output = input exactly
    //
    // fade_level = 16:
    // output = 50%
    //
    // fade_level = 0:
    // output = black
    // =========================================================

    wire [12:0] red_mult;
    wire [12:0] green_mult;
    wire [12:0] blue_mult;


    assign red_mult =
        rgb_in[23:16] * fade_level;

    assign green_mult =
        rgb_in[15:8] * fade_level;

    assign blue_mult =
        rgb_in[7:0] * fade_level;


    // Exact bypass at full brightness.
    assign rgb_out =

        (fade_level == 6'd32) ?

            rgb_in :

            {
                red_mult[12:5],
                green_mult[12:5],
                blue_mult[12:5]
            };


endmodule