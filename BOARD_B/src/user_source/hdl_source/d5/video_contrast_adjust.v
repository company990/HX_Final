`timescale 1ns/1ps

module video_contrast_adjust (

    input      [23:0] rgb_in,

    // Q8 fixed-point contrast gain:
    //
    // 128 = 0.5x
    // 256 = 1.0x
    // 384 = 1.5x
    // 512 = 2.0x
    //
    input      [9:0] contrast_gain,

    output     [23:0] rgb_out
);


    // =========================================================
    // Contrast is adjusted around middle gray = 128
    //
    // out = 128 + ((in - 128) * gain) / 256
    // =========================================================

    function [7:0] adjust_channel;

        input [7:0] pixel;
        input [9:0] gain;

        reg signed [10:0] centered;
        reg signed [10:0] gain_signed;
        reg signed [21:0] multiplied;
        reg signed [21:0] adjusted;

        begin

            centered =
                $signed({1'b0, pixel}) - 11'sd128;

            gain_signed =
                $signed({1'b0, gain});

            multiplied =
                centered * gain_signed;

            adjusted =
                (multiplied >>> 8) + 22'sd128;


            if (adjusted < 0)
                adjust_channel = 8'd0;

            else if (adjusted > 255)
                adjust_channel = 8'd255;

            else
                adjust_channel = adjusted[7:0];

        end

    endfunction


    assign rgb_out = {

        adjust_channel(rgb_in[23:16], contrast_gain),
        adjust_channel(rgb_in[15:8],  contrast_gain),
        adjust_channel(rgb_in[7:0],   contrast_gain)

    };


endmodule