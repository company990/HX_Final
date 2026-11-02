`timescale 1ns/1ps

// =============================================================
// G1-C3
// Video zoom source-address mapper
//
// Output image is always:
//
//      640 x 480
//
// This module converts an output-screen coordinate:
//
//      out_x = 0 ... 639
//      out_y = 0 ... 479
//
// into the source-image coordinate which should be read
// from the 640x480 SDRAM framebuffer.
//
// zoom_level:
//
//      0 : 1.00X
//      1 : 1.25X
//      2 : 1.50X
//      3 : 2.00X
//
// All zoom modes use center crop + nearest-neighbour mapping.
//
// src_pixel_offset:
//
//      src_y * 640 + src_x
//
// This is only an address mapper.
// It does NOT contain SDRAM/FIFO control.
// =============================================================

module video_zoom_addr_mapper (

    input  wire [9:0]  out_x,
    input  wire [8:0]  out_y,

    input  wire [1:0]  zoom_level,

    output reg  [9:0]  src_x,
    output reg  [8:0]  src_y,

    output reg  [18:0] src_pixel_offset

);


// =============================================================
// Extended intermediate values
// =============================================================

reg [11:0] x_scaled;
reg [10:0] y_scaled;


// =============================================================
// Coordinate mapping
// =============================================================

always @(*) begin

    // ---------------------------------------------------------
    // Defaults: 1.00X
    // ---------------------------------------------------------

    src_x = out_x;
    src_y = out_y;

    x_scaled = 12'd0;
    y_scaled = 11'd0;


    case (zoom_level)

        // =====================================================
        // 1.00X
        //
        // Source window:
        //
        //      640 x 480
        //
        // Mapping:
        //
        //      sx = x
        //      sy = y
        // =====================================================

        2'd0: begin

            src_x = out_x;
            src_y = out_y;

        end


        // =====================================================
        // 1.25X
        //
        // Source window:
        //
        //      512 x 384
        //
        // Center crop:
        //
        //      X = 64  ... 575
        //      Y = 48  ... 431
        //
        // 640 -> 512 = 4/5
        // 480 -> 384 = 4/5
        // =====================================================

        2'd1: begin

            x_scaled =
                ({2'b00, out_x} << 2) / 12'd5;

            y_scaled =
                ({2'b00, out_y} << 2) / 11'd5;

            src_x =
                10'd64 + x_scaled[9:0];

            src_y =
                9'd48 + y_scaled[8:0];

        end


        // =====================================================
        // 1.50X
        //
        // Horizontal source width is approximately:
        //
        //      640 / 1.5 = 426.67
        //
        // We use 427 source pixels.
        //
        // Horizontal range:
        //
        //      X = 106 ... 532
        //
        // Vertical source height:
        //
        //      480 / 1.5 = 320
        //
        // Vertical range:
        //
        //      Y = 80 ... 399
        //
        // Mapping ratio:
        //
        //      2 / 3
        // =====================================================

        2'd2: begin

            x_scaled =
                ({1'b0, out_x} << 1) / 11'd3;

            y_scaled =
                ({1'b0, out_y} << 1) / 10'd3;

            src_x =
                10'd106 + x_scaled[9:0];

            src_y =
                9'd80 + y_scaled[8:0];

        end


        // =====================================================
        // 2.00X
        //
        // Source window:
        //
        //      320 x 240
        //
        // Center crop:
        //
        //      X = 160 ... 479
        //      Y = 120 ... 359
        //
        // Every source pixel appears twice horizontally
        // and twice vertically.
        // =====================================================

        2'd3: begin

            src_x =
                10'd160 + (out_x >> 1);

            src_y =
                9'd120 + (out_y >> 1);

        end


        default: begin

            src_x = out_x;
            src_y = out_y;

        end

    endcase

end


// =============================================================
// Convert source X/Y into linear framebuffer offset
//
// 640 = 512 + 128
//
// Therefore:
//
// src_y * 640
//
// = src_y * 512 + src_y * 128
//
// = (src_y << 9) + (src_y << 7)
//
// Avoids a general-purpose multiplier.
// =============================================================

always @(*) begin

    src_pixel_offset =
          ({10'd0, src_y} << 9)
        + ({10'd0, src_y} << 7)
        +  { 9'd0, src_x};

end


endmodule