// Combined shifting dividend/quotient preserves the 26-cycle divider latency.
`timescale 1ns/1ps

// =============================================================
// G1-C3-D1
// Adaptive image fit geometry calculator
//
// Purpose:
//   Given an arbitrary BMP source width/height,
//   calculate a destination rectangle that:
//
//   1. fits inside HDMI 640x480
//   2. preserves aspect ratio
//   3. centers the image
//   4. allows both upscale and downscale
//
// Examples:
//
//   640x480 -> 640x480 offset(0,0)
//   320x240 -> 640x480 offset(0,0)
//   800x600 -> 640x480 offset(0,0)
//
//   640x360 -> 640x360 offset(0,60)
//   800x480 -> 640x384 offset(0,48)
//   480x480 -> 480x480 offset(80,0)
//
// Notes:
//   - No combinational divider.
//   - Division is performed sequentially in 26 clock cycles.
//   - Calculation is only needed once when a new BMP is loaded.
// =============================================================

module adaptive_image_fit (

    input  wire        clk,
    input  wire        rst,

    // One-clock pulse.
    input  wire        start,

    input  wire [15:0] src_width,
    input  wire [15:0] src_height,

    output reg         busy,
    output reg         done,
    output reg         valid,

    // Final display rectangle inside 640x480.
    output reg  [15:0] dst_width,
    output reg  [15:0] dst_height,

    output reg  [15:0] x_offset,
    output reg  [15:0] y_offset

);


// =============================================================
// Fixed HDMI output size
// =============================================================

localparam [15:0] HDMI_WIDTH  = 16'd640;
localparam [15:0] HDMI_HEIGHT = 16'd480;


// =============================================================
// State machine
// =============================================================

localparam [2:0]
    S_IDLE     = 3'd0,
    S_COMPARE  = 3'd1,
    S_DIV_INIT = 3'd2,
    S_DIV      = 3'd3,
    S_RESULT   = 3'd4;

reg [2:0] state;


// =============================================================
// Latched source size
// =============================================================

reg [15:0] src_width_latched;
reg [15:0] src_height_latched;


// =============================================================
// Aspect-ratio comparison
//
// Compare:
//
//     src_width / src_height
//
// with:
//
//     640 / 480
//
// Instead of division:
//
//     src_width * 480
//          ?
//     src_height * 640
//
// Constants are implemented using shifts:
//
//     x * 480 = x * (512 - 32)
//             = (x << 9) - (x << 5)
//
//     x * 640 = x * (512 + 128)
//             = (x << 9) + (x << 7)
// =============================================================

wire [25:0] src_width_ext;
wire [25:0] src_height_ext;

assign src_width_ext  = {10'd0, src_width_latched};
assign src_height_ext = {10'd0, src_height_latched};


wire [25:0] src_width_mul_480;
wire [25:0] src_height_mul_640;

assign src_width_mul_480 =
    (src_width_ext << 9) -
    (src_width_ext << 5);

assign src_height_mul_640 =
    (src_height_ext << 9) +
    (src_height_ext << 7);


// =============================================================
// Divider preparation
//
// Width limited:
//
//     dst_width  = 640
//     dst_height = src_height * 640 / src_width
//
// Height limited:
//
//     dst_height = 480
//     dst_width  = src_width * 480 / src_height
// =============================================================

reg        limited_by_width;

reg [25:0] division_numerator;
reg [15:0] division_denominator;


// =============================================================
// Sequential unsigned restoring divider
//
// numerator   : 26 bit
// denominator : 16 bit
// quotient    : 26 bit
//
// Maximum useful result is only 640.
// =============================================================

reg [15:0] divisor_work;

reg [16:0] remainder_work;
reg [25:0] quotient_work;

reg [4:0] div_bit;


// Current shifted remainder candidate.
wire [16:0] remainder_shift;

assign remainder_shift = {
    remainder_work[15:0],
    quotient_work[25]
};


// =============================================================
// Main state machine
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        state <= S_IDLE;

        busy  <= 1'b0;
        done  <= 1'b0;
        valid <= 1'b0;

        src_width_latched  <= 16'd0;
        src_height_latched <= 16'd0;

        limited_by_width <= 1'b0;

        division_numerator   <= 26'd0;
        division_denominator <= 16'd0;
        divisor_work   <= 16'd0;

        remainder_work <= 17'd0;
        quotient_work  <= 26'd0;

        div_bit <= 5'd0;

        dst_width  <= 16'd0;
        dst_height <= 16'd0;

        x_offset <= 16'd0;
        y_offset <= 16'd0;

    end
    else begin

        // done is a one-clock pulse.
        done <= 1'b0;


        case (state)


            // =================================================
            // IDLE
            // =================================================

            S_IDLE: begin

                busy <= 1'b0;

                if (start) begin

                    src_width_latched  <= src_width;
                    src_height_latched <= src_height;

                    busy  <= 1'b1;
                    valid <= 1'b0;

                    state <= S_COMPARE;

                end

            end


            // =================================================
            // Compare aspect ratios
            // =================================================

            S_COMPARE: begin

                // Invalid BMP geometry.
                if (
                    (src_width_latched  == 16'd0) ||
                    (src_height_latched == 16'd0)
                ) begin

                    dst_width  <= 16'd0;
                    dst_height <= 16'd0;

                    x_offset <= 16'd0;
                    y_offset <= 16'd0;

                    valid <= 1'b0;
                    busy  <= 1'b0;
                    done  <= 1'b1;

                    state <= S_IDLE;

                end

                // ---------------------------------------------
                // Source is wider than or equal to HDMI ratio.
                //
                // Width becomes 640.
                // Height is calculated.
                // ---------------------------------------------

                else if (
                    src_width_mul_480 >=
                    src_height_mul_640
                ) begin

                    limited_by_width <= 1'b1;

                    division_numerator <=
                        src_height_mul_640;

                    division_denominator <=
                        src_width_latched;

                    state <= S_DIV_INIT;

                end

                // ---------------------------------------------
                // Source is taller/narrower than HDMI ratio.
                //
                // Height becomes 480.
                // Width is calculated.
                // ---------------------------------------------

                else begin

                    limited_by_width <= 1'b0;

                    division_numerator <=
                        src_width_mul_480;

                    division_denominator <=
                        src_height_latched;

                    state <= S_DIV_INIT;

                end

            end


            // =================================================
            // Initialize divider
            // =================================================

            S_DIV_INIT: begin

                divisor_work  <= division_denominator;

                remainder_work <= 17'd0;
                quotient_work  <= division_numerator;

                div_bit <= 5'd25;

                state <= S_DIV;

            end


            // =================================================
            // 26-cycle restoring division
            // =================================================

            S_DIV: begin

                if (
                    remainder_shift >=
                    {1'b0, divisor_work}
                ) begin

                    remainder_work <=
                        remainder_shift -
                        {1'b0, divisor_work};

                    quotient_work <= {quotient_work[24:0],1'b1};

                end
                else begin

                    remainder_work <=
                        remainder_shift;

                    quotient_work <= {quotient_work[24:0],1'b0};

                end


                if (div_bit == 5'd0) begin

                    state <= S_RESULT;

                end
                else begin

                    div_bit <= div_bit - 1'b1;

                end

            end


            // =================================================
            // Generate final rectangle
            // =================================================

            S_RESULT: begin

                // ---------------------------------------------
                // Width limited
                // ---------------------------------------------

                if (limited_by_width) begin

                    dst_width <= HDMI_WIDTH;

                    // Prevent an extreme aspect ratio from
                    // producing zero visible rows.
                    if (quotient_work[15:0] == 16'd0) begin

                        dst_height <= 16'd1;

                        x_offset <= 16'd0;

                        y_offset <=
                            (HDMI_HEIGHT - 16'd1) >> 1;

                    end
                    else begin

                        dst_height <=
                            quotient_work[15:0];

                        x_offset <= 16'd0;

                        y_offset <=
                            (
                                HDMI_HEIGHT -
                                quotient_work[15:0]
                            ) >> 1;

                    end

                end


                // ---------------------------------------------
                // Height limited
                // ---------------------------------------------

                else begin

                    dst_height <= HDMI_HEIGHT;

                    if (quotient_work[15:0] == 16'd0) begin

                        dst_width <= 16'd1;

                        y_offset <= 16'd0;

                        x_offset <=
                            (HDMI_WIDTH - 16'd1) >> 1;

                    end
                    else begin

                        dst_width <=
                            quotient_work[15:0];

                        y_offset <= 16'd0;

                        x_offset <=
                            (
                                HDMI_WIDTH -
                                quotient_work[15:0]
                            ) >> 1;

                    end

                end


                valid <= 1'b1;
                busy  <= 1'b0;
                done  <= 1'b1;

                state <= S_IDLE;

            end


            // =================================================
            // Recovery
            // =================================================

            default: begin

                state <= S_IDLE;

                busy  <= 1'b0;
                valid <= 1'b0;

            end

        endcase

    end

end


endmodule