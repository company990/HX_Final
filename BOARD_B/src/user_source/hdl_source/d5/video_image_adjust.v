`timescale 1ns/1ps

module video_image_adjust (

    input      [23:0] rgb_in,

    input signed [8:0] brightness,
    input        [9:0] contrast_gain,

    output     [23:0] rgb_out
);


    wire [23:0] rgb_after_contrast;


    // =========================================================
    // Processing order:
    //
    // RGB
    //  -> Contrast
    //  -> Brightness
    //  -> Output
    //
    // Equivalent conceptually to:
    //
    // out = contrast(in) + brightness
    // =========================================================

    video_contrast_adjust u_video_contrast_adjust (

        .rgb_in        (rgb_in),
        .contrast_gain (contrast_gain),
        .rgb_out       (rgb_after_contrast)

    );


    video_brightness_adjust u_video_brightness_adjust (

        .rgb_in      (rgb_after_contrast),
        .brightness  (brightness),
        .rgb_out     (rgb_out)

    );


endmodule