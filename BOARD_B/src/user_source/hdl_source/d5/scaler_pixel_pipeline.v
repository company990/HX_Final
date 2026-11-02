`timescale 1ns/1ps

// ============================================================================
// D5-C1
// Complete scaler pixel pipeline wrapper.
//
// Integrates:
//   D5-A1  scaler_dst_scan_gen
//   D5-B1  bilinear_src_coord_gen
//   D5-B2  bilinear_rgb_interp
//
// This stage intentionally uses an ABSTRACT four-pixel source-memory
// transaction.  A later D5-C2 SDRAM fetch engine will translate one request
// into real SDRAM read commands.
//
// For every 640x480 destination pixel:
//   - border pixel:
//         directly emits RGB=0
//   - image pixel:
//         emits one request containing P00/P10/P01/P11 source addresses,
//         waits for the four returned source pixels,
//         performs bilinear RGB interpolation,
//         emits one destination pixel.
//
// The destination scan and bilinear phase advance ONLY when dst_valid and
// dst_ready handshake.  Therefore all coordinates remain frozen during
// source-memory stalls and destination backpressure.
//
// Verilog-2001 compatible.
// ============================================================================

module scaler_pixel_pipeline #(
    parameter ADDR_BITS = 21
)(
    input                       clk,
    input                       rst,

    // ------------------------------------------------------------------------
    // Geometry configuration
    // ------------------------------------------------------------------------
    input                       cfg_start,

    input      [ADDR_BITS-1:0]  src_base_addr,
    input      [15:0]           src_width,
    input      [15:0]           src_height,

    input      [15:0]           dst_width,
    input      [15:0]           dst_height,
    input      [15:0]           x_offset,
    input      [15:0]           y_offset,

    input                       geometry_valid,

    output                      cfg_busy,
    output                      cfg_done,
    output                      cfg_valid,

    // ------------------------------------------------------------------------
    // Frame control
    // ------------------------------------------------------------------------
    input                       frame_start,
    output reg                  frame_busy,
    output reg                  frame_done,

    // ------------------------------------------------------------------------
    // Abstract four-neighbour source-memory request
    //
    // Request handshake:
    //   src_req_valid && src_req_ready
    //
    // Response handshake:
    //   src_pixels_valid && src_pixels_ready
    // ------------------------------------------------------------------------
    output                      src_req_valid,
    input                       src_req_ready,

    output     [ADDR_BITS-1:0]  src_addr_p00,
    output     [ADDR_BITS-1:0]  src_addr_p10,
    output     [ADDR_BITS-1:0]  src_addr_p01,
    output     [ADDR_BITS-1:0]  src_addr_p11,

    input                       src_pixels_valid,
    output                      src_pixels_ready,

    input      [23:0]           src_pixel_p00,
    input      [23:0]           src_pixel_p10,
    input      [23:0]           src_pixel_p01,
    input      [23:0]           src_pixel_p11,

    // ------------------------------------------------------------------------
    // Destination 640x480 pixel stream
    //
    // dst_addr is always 0..307199 in raster order.
    // ------------------------------------------------------------------------
    output                      dst_valid,
    input                       dst_ready,

    output     [18:0]           dst_addr,
    output     [23:0]           dst_pixel,

    output     [9:0]            dst_x,
    output     [8:0]            dst_y
);

    // ========================================================================
    // D5-A1 destination scan
    // ========================================================================

    wire        scan_cfg_valid;

    wire        scan_active;
    wire        scan_done_unused;

    wire [9:0]  scan_x;
    wire [8:0]  scan_y;
    wire [18:0] scan_addr;

    wire        scan_inside;
    wire [15:0] scan_image_x;
    wire [15:0] scan_image_y;

    wire        scan_frame_last;

    wire        scan_start;
    wire        scan_advance;

    assign scan_start =
        frame_start &&
        cfg_valid &&
        !frame_busy;

    scaler_dst_scan_gen #(
        .OUT_WIDTH  (640),
        .OUT_HEIGHT (480),
        .ADDR_BITS  (19)
    ) u_dst_scan (
        .clk            (clk),
        .rst            (rst),

        .cfg_load       (cfg_start),
        .dst_width      (dst_width),
        .dst_height     (dst_height),
        .x_offset       (x_offset),
        .y_offset       (y_offset),
        .geometry_valid (geometry_valid),

        .cfg_valid      (scan_cfg_valid),

        .start          (scan_start),
        .advance        (scan_advance),

        .scan_active    (scan_active),
        .done           (scan_done_unused),

        .out_x          (scan_x),
        .out_y          (scan_y),
        .dst_addr       (scan_addr),

        .inside_image   (scan_inside),
        .image_x        (scan_image_x),
        .image_y        (scan_image_y),

        .frame_last     (scan_frame_last)
    );


    // ========================================================================
    // D5-B1 bilinear source mapping
    // ========================================================================

    wire        coord_cfg_busy;
    wire        coord_cfg_done;
    wire        coord_cfg_valid;

    wire        coord_sample_valid;

    wire [15:0] coord_src_x0_unused;
    wire [15:0] coord_src_x1_unused;
    wire [15:0] coord_src_y0_unused;
    wire [15:0] coord_src_y1_unused;

    wire [15:0] coord_frac_x;
    wire [15:0] coord_frac_y;

    wire [ADDR_BITS-1:0] coord_addr_p00;
    wire [ADDR_BITS-1:0] coord_addr_p10;
    wire [ADDR_BITS-1:0] coord_addr_p01;
    wire [ADDR_BITS-1:0] coord_addr_p11;

    bilinear_src_coord_gen #(
        .DST_BITS (10), // Fixed 640x480 destination; source coordinates stay 16-bit.
        .ADDR_BITS (ADDR_BITS)
    ) u_src_coord (
        .clk            (clk),
        .rst            (rst),

        .cfg_start      (cfg_start),

        .base_addr      (src_base_addr),
        .src_width      (src_width),
        .src_height     (src_height),
        .dst_width      (dst_width),
        .dst_height     (dst_height),
        .geometry_valid (geometry_valid),

        .cfg_busy       (coord_cfg_busy),
        .cfg_done       (coord_cfg_done),
        .cfg_valid      (coord_cfg_valid),

        .frame_start    (scan_start),
        .advance        (scan_advance),

        .inside_image   (scan_inside),
        .image_x        (scan_image_x),
        .image_y        (scan_image_y),

        .sample_valid   (coord_sample_valid),

        .src_x0         (coord_src_x0_unused),
        .src_x1         (coord_src_x1_unused),
        .src_y0         (coord_src_y0_unused),
        .src_y1         (coord_src_y1_unused),

        .frac_x         (coord_frac_x),
        .frac_y         (coord_frac_y),

        .addr_p00       (coord_addr_p00),
        .addr_p10       (coord_addr_p10),
        .addr_p01       (coord_addr_p01),
        .addr_p11       (coord_addr_p11)
    );

    assign cfg_busy  = coord_cfg_busy;
    assign cfg_done  = coord_cfg_done;
    assign cfg_valid = coord_cfg_valid && scan_cfg_valid;


    // ========================================================================
    // D5-B2 RGB bilinear interpolation
    // ========================================================================

    reg         interp_start;

    wire        interp_busy;
    wire        interp_done;
    wire [23:0] interp_pixel;

    reg [23:0] interp_p00;
    reg [23:0] interp_p10;
    reg [23:0] interp_p01;
    reg [23:0] interp_p11;

    reg [15:0] interp_frac_x;
    reg [15:0] interp_frac_y;

    bilinear_rgb_interp u_rgb_interp (
        .clk       (clk),
        .rst       (rst),

        .start     (interp_start),
        .busy      (interp_busy),
        .done      (interp_done),

        .p00       (interp_p00),
        .p10       (interp_p10),
        .p01       (interp_p01),
        .p11       (interp_p11),

        .frac_x    (interp_frac_x),
        .frac_y    (interp_frac_y),

        .pixel_out (interp_pixel)
    );


    // ========================================================================
    // Pixel sequencer
    // ========================================================================

    localparam ST_IDLE        = 3'd0;
    localparam ST_CLASSIFY    = 3'd1;
    localparam ST_SRC_REQ     = 3'd2;
    localparam ST_SRC_WAIT    = 3'd3;
    localparam ST_INTERP_WAIT = 3'd4;
    localparam ST_DST_OUT     = 3'd5;

    reg [2:0] state;

    reg [18:0] out_addr_r;
    reg [23:0] out_pixel_r;
    reg        out_last_r;

    reg [15:0] req_frac_x_r;
    reg [15:0] req_frac_y_r;

    assign src_req_valid =
        frame_busy &&
        (state == ST_SRC_REQ);

    assign src_pixels_ready =
        frame_busy &&
        (state == ST_SRC_WAIT);

    assign src_addr_p00 = coord_addr_p00;
    assign src_addr_p10 = coord_addr_p10;
    assign src_addr_p01 = coord_addr_p01;
    assign src_addr_p11 = coord_addr_p11;

    assign dst_valid =
        frame_busy &&
        (state == ST_DST_OUT);

    assign dst_addr  = out_addr_r;
    assign dst_pixel = out_pixel_r;

    assign dst_x = scan_x;
    assign dst_y = scan_y;

    assign scan_advance =
        dst_valid &&
        dst_ready;


    always @(posedge clk or posedge rst) begin
        if (rst) begin

            state <= ST_IDLE;

            frame_busy <= 1'b0;
            frame_done <= 1'b0;

            out_addr_r  <= 19'd0;
            out_pixel_r <= 24'd0;
            out_last_r  <= 1'b0;

            req_frac_x_r <= 16'd0;
            req_frac_y_r <= 16'd0;

            interp_start  <= 1'b0;

            interp_p00 <= 24'd0;
            interp_p10 <= 24'd0;
            interp_p01 <= 24'd0;
            interp_p11 <= 24'd0;

            interp_frac_x <= 16'd0;
            interp_frac_y <= 16'd0;

        end
        else begin

            frame_done   <= 1'b0;
            interp_start <= 1'b0;

            case (state)

                // ------------------------------------------------------------
                // Wait for a configured frame start.
                // ------------------------------------------------------------
                ST_IDLE:
                begin
                    frame_busy <= 1'b0;

                    if (scan_start) begin
                        frame_busy <= 1'b1;
                        state      <= ST_CLASSIFY;
                    end
                end


                // ------------------------------------------------------------
                // Current scan coordinate is stable here.
                // Border -> directly emit black.
                // Image  -> request P00/P10/P01/P11.
                // ------------------------------------------------------------
                ST_CLASSIFY:
                begin
                    if (scan_active) begin

                        out_addr_r <= scan_addr;
                        out_last_r <= scan_frame_last;

                        if (!scan_inside) begin

                            out_pixel_r <= 24'h000000;
                            state       <= ST_DST_OUT;

                        end
                        else if (coord_sample_valid) begin

                            state <= ST_SRC_REQ;
                        end
                    end
                end


                // ------------------------------------------------------------
                // Hold all four source addresses stable until accepted.
                // Latch interpolation fractions with the request.
                // ------------------------------------------------------------
                ST_SRC_REQ:
                begin
                    if (src_req_ready) begin

                        req_frac_x_r <= coord_frac_x;
                        req_frac_y_r <= coord_frac_y;

                        state <= ST_SRC_WAIT;
                    end
                end


                // ------------------------------------------------------------
                // Wait for the abstract source-memory response.
                // ------------------------------------------------------------
                ST_SRC_WAIT:
                begin
                    if (src_pixels_valid) begin

                        interp_p00 <= src_pixel_p00;
                        interp_p10 <= src_pixel_p10;
                        interp_p01 <= src_pixel_p01;
                        interp_p11 <= src_pixel_p11;

                        interp_frac_x <= req_frac_x_r;
                        interp_frac_y <= req_frac_y_r;

                        interp_start <= 1'b1;

                        state <= ST_INTERP_WAIT;
                    end
                end


                // ------------------------------------------------------------
                // Wait for D5-B2 to finish one RGB bilinear interpolation.
                // ------------------------------------------------------------
                ST_INTERP_WAIT:
                begin
                    if (interp_done) begin

                        out_pixel_r <= interp_pixel;
                        state       <= ST_DST_OUT;
                    end
                end


                // ------------------------------------------------------------
                // Hold output until the destination consumer accepts it.
                // Only this handshake advances BOTH D5-A1 and D5-B1.
                // ------------------------------------------------------------
                ST_DST_OUT:
                begin
                    if (dst_ready) begin

                        if (out_last_r) begin

                            frame_busy <= 1'b0;
                            frame_done <= 1'b1;
                            state      <= ST_IDLE;

                        end
                        else begin

                            state <= ST_CLASSIFY;
                        end
                    end
                end


                default:
                begin
                    state      <= ST_IDLE;
                    frame_busy <= 1'b0;
                end

            endcase
        end
    end

endmodule
