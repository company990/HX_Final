`timescale 1ns/1ps

module video_brightness_adjust (

    input      [23:0] rgb_in,

    // Signed brightness offset:
    //
    //   0   = unchanged
    //  +32  = brighter
    //  -32  = darker
    //
    input signed [8:0] brightness,

    output     [23:0] rgb_out
);


    // =========================================================
    // Extend RGB channels before signed arithmetic
    // =========================================================

    wire signed [9:0] red_value;
    wire signed [9:0] green_value;
    wire signed [9:0] blue_value;


    assign red_value =
        $signed({1'b0, rgb_in[23:16]}) +
        brightness;

    assign green_value =
        $signed({1'b0, rgb_in[15:8]}) +
        brightness;

    assign blue_value =
        $signed({1'b0, rgb_in[7:0]}) +
        brightness;


    // =========================================================
    // Saturation
    //
    // value < 0   -> 0
    // value > 255 -> 255
    // otherwise   -> original result
    // =========================================================

    function [7:0] clip8;

        input signed [9:0] value;

        begin

            if (value < 0)
                clip8 = 8'd0;

            else if (value > 255)
                clip8 = 8'd255;

            else
                clip8 = value[7:0];

        end

    endfunction


    assign rgb_out = {
        clip8(red_value),
        clip8(green_value),
        clip8(blue_value)
    };


endmodule