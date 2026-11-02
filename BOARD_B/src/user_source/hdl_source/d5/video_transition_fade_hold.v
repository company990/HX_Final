`timescale 1ns/1ps

module video_transition_fade_hold (

    input             clk,
    input             rst,

    input             frame_tick,

    // Start one complete transition
    input             transition_start,

    // One-clock pulse:
    // new BMP frame has been completely loaded
    // and is ready for display.
    input             image_ready,

    input      [23:0] rgb_in,
    output     [23:0] rgb_out,

    output            transition_busy,
    output            midpoint_pulse,
    output            transition_done,

    output     [5:0]  fade_level,

    // 1 = currently holding completely black
    output            black_hold
);


    // =========================================================
    // Latched black-hold state
    //
    // midpoint_pulse itself is also OR'ed into black_hold,
    // so black_hold becomes effective immediately in the same
    // cycle that the fade core reaches level 0.
    // =========================================================

    reg black_hold_latched;


    assign black_hold =
        black_hold_latched |
        midpoint_pulse;


    // =========================================================
    // Core frame tick
    //
    // Once midpoint is reached, stop sending frame ticks
    // to the fade core so it remains at level 0.
    // =========================================================

    wire fade_frame_tick;

    assign fade_frame_tick =
        frame_tick &&
        !black_hold;


    // =========================================================
    // Original verified fade core
    //
    // DO NOT modify video_transition_fade.v
    // =========================================================

    video_transition_fade u_video_transition_fade (

        .clk              (clk),
        .rst              (rst),

        .frame_tick       (fade_frame_tick),

        .transition_start (transition_start),

        .rgb_in           (rgb_in),
        .rgb_out          (rgb_out),

        .transition_busy  (transition_busy),
        .midpoint_pulse   (midpoint_pulse),
        .transition_done  (transition_done),

        .fade_level       (fade_level)

    );


    // =========================================================
    // Black-hold latch
    //
    // midpoint_pulse:
    //   fade-out has reached completely black.
    //
    // image_ready:
    //   new BMP has really finished loading and is ready.
    // =========================================================

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            black_hold_latched <= 1'b0;

        end
        else begin

            // -------------------------------------------------
            // Fade-out reached black.
            //
            // Latch the black state.
            // -------------------------------------------------

            if (midpoint_pulse) begin

                black_hold_latched <= 1'b1;

            end

            // -------------------------------------------------
            // New image is ready.
            //
            // Release black hold and allow fade-in.
            // -------------------------------------------------

            else if (image_ready) begin

                black_hold_latched <= 1'b0;

            end

        end

    end


endmodule