`timescale 1ns/1ps

// ============================================================================
// G1-C3-D4-A
// Adaptive SDRAM read-address GOLDEN/REFERENCE mapper.
//
// Purpose:
//   Convert one HDMI output coordinate (640x480) into the corresponding
//   source-BMP word address stored in SDRAM.
//
// Storage convention established by D3-C:
//   - Raw BMP pixels are stored contiguously.
//   - WRITE_V_FLIP = 0.
//   - Positive-height BMP data is bottom-up in SDRAM.
//
// Display convention:
//   - dst_width/dst_height/x_offset/y_offset come from adaptive_image_fit.
//   - Aspect ratio is preserved.
//   - Pixels outside the fitted rectangle are letterbox/pillarbox.
//   - Nearest-neighbor mapping is used inside the fitted rectangle.
//
// IMPORTANT:
//   This module intentionally uses * and / as a simulation golden reference.
//   It is NOT the final optimized high-speed generator for frame_fifo_read.
//   D4-B will build a synthesizable sequential generator and compare it
//   against this reference.
// ============================================================================

module adaptive_sdram_addr_mapper #(
    parameter ADDR_BITS = 21
)(
    input  [15:0]              out_x,
    input  [15:0]              out_y,

    input  [ADDR_BITS-1:0]     base_addr,

    input  [15:0]              src_width,
    input  [15:0]              src_height,

    input  [15:0]              dst_width,
    input  [15:0]              dst_height,

    input  [15:0]              x_offset,
    input  [15:0]              y_offset,

    input                      geometry_valid,

    output reg                 inside_image,

    output reg [15:0]          src_x,
    output reg [15:0]          src_y_top,
    output reg [15:0]          src_y_mem,

    output reg [ADDR_BITS-1:0] read_addr
);

    reg [31:0] rel_x;
    reg [31:0] rel_y;

    reg [31:0] x_num;
    reg [31:0] y_num;

    reg [31:0] src_x_calc;
    reg [31:0] src_y_top_calc;
    reg [31:0] src_y_mem_calc;

    reg [31:0] addr_calc;

    always @* begin

        inside_image = 1'b0;

        src_x      = 16'd0;
        src_y_top  = 16'd0;
        src_y_mem  = 16'd0;

        read_addr  = base_addr;

        rel_x          = 32'd0;
        rel_y          = 32'd0;
        x_num          = 32'd0;
        y_num          = 32'd0;
        src_x_calc     = 32'd0;
        src_y_top_calc = 32'd0;
        src_y_mem_calc = 32'd0;
        addr_calc      = {{(32-ADDR_BITS){1'b0}}, base_addr};


        if (
            geometry_valid &&
            (src_width  != 16'd0) &&
            (src_height != 16'd0) &&
            (dst_width  != 16'd0) &&
            (dst_height != 16'd0) &&

            (out_x >= x_offset) &&
            (out_x <  (x_offset + dst_width)) &&

            (out_y >= y_offset) &&
            (out_y <  (y_offset + dst_height))
        ) begin

            inside_image = 1'b1;

            rel_x = out_x - x_offset;
            rel_y = out_y - y_offset;

            // Nearest-neighbor reference:
            //
            // src_x     = floor(rel_x * src_width  / dst_width)
            // src_y_top = floor(rel_y * src_height / dst_height)
            //
            // Source BMP was stored bottom-up, therefore:
            //
            // src_y_mem = src_height - 1 - src_y_top

            x_num = rel_x * src_width;
            y_num = rel_y * src_height;

            src_x_calc     = x_num / dst_width;
            src_y_top_calc = y_num / dst_height;

            // Defensive clamp.  With valid geometry this should normally
            // never be required, but it keeps the golden mapper safe.
            if (src_x_calc >= src_width)
                src_x_calc = src_width - 1'b1;

            if (src_y_top_calc >= src_height)
                src_y_top_calc = src_height - 1'b1;

            src_y_mem_calc = src_height - 1'b1 - src_y_top_calc;

            src_x     = src_x_calc[15:0];
            src_y_top = src_y_top_calc[15:0];
            src_y_mem = src_y_mem_calc[15:0];

            addr_calc =
                {{(32-ADDR_BITS){1'b0}}, base_addr} +
                (src_y_mem_calc * src_width) +
                src_x_calc;

            read_addr = addr_calc[ADDR_BITS-1:0];

        end

    end

endmodule
