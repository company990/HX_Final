`timescale 1ns/1ps

// ============================================================================
// D5-C2-B
// Bilinear scaler + real-style SDRAM fetch integration wrapper.
//
// Integrates:
//   scaler_pixel_pipeline   (D5-A1 + D5-B1 + D5-B2)
//   bilinear_sdram_fetch    (D5-C2-A)
//
// The only source-memory interface exposed by this wrapper is the existing
// SDRAM user interface:
//      App_rd_en / App_rd_addr
//      Sdr_rd_en / Sdr_rd_dout
//
// Destination output remains a sequential 640x480 pixel stream.
// Verilog-2001 compatible.
// ============================================================================

module scaler_sdram_pipeline #(
    parameter ADDR_BITS = 21
)(
    input                       clk,
    input                       rst,

    // Geometry configuration
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

    // Frame control
    input                       frame_start,
    output                      frame_busy,
    output                      frame_done,

    // Existing SDRAM user interface
    input                       Sdr_init_done,
    input                       Sdr_init_ref_vld,
    input                       Sdr_busy,
    input                       App_wr_busy,

    output                      App_rd_en,
    output     [ADDR_BITS-1:0]  App_rd_addr,

    input                       Sdr_rd_en,
    input      [31:0]           Sdr_rd_dout,

    // Destination 640x480 stream
    output                      dst_valid,
    input                       dst_ready,

    output     [18:0]           dst_addr,
    output     [23:0]           dst_pixel,
    output     [9:0]            dst_x,
    output     [8:0]            dst_y,

    // Debug
    output                      fetch_memory_busy,
    output                      fetch_busy,
    output     [2:0]            fetch_issued_count,
    output     [2:0]            fetch_returned_count,
    output                      fetch_unexpected_return_sticky
);

    wire                       src_req_valid;
    wire                       src_req_ready;

    wire [ADDR_BITS-1:0]       src_addr_p00;
    wire [ADDR_BITS-1:0]       src_addr_p10;
    wire [ADDR_BITS-1:0]       src_addr_p01;
    wire [ADDR_BITS-1:0]       src_addr_p11;

    wire                       src_pixels_valid;
    wire                       src_pixels_ready;

    wire [23:0]                src_pixel_p00;
    wire [23:0]                src_pixel_p10;
    wire [23:0]                src_pixel_p01;
    wire [23:0]                src_pixel_p11;


    scaler_pixel_pipeline #(
        .FETCH_HOLDS(1), // Captured fetch pixels remain valid through interpolation.
        .SOURCE_HOLDS(1),
        .ADDR_BITS (ADDR_BITS)
    ) u_scaler (
        .clk              (clk),
        .rst              (rst),

        .cfg_start        (cfg_start),

        .src_base_addr    (src_base_addr),
        .src_width        (src_width),
        .src_height       (src_height),

        .dst_width        (dst_width),
        .dst_height       (dst_height),
        .x_offset         (x_offset),
        .y_offset         (y_offset),

        .geometry_valid   (geometry_valid),

        .cfg_busy         (cfg_busy),
        .cfg_done         (cfg_done),
        .cfg_valid        (cfg_valid),

        .frame_start      (frame_start),
        .frame_busy       (frame_busy),
        .frame_done       (frame_done),

        .src_req_valid    (src_req_valid),
        .src_req_ready    (src_req_ready),

        .src_addr_p00     (src_addr_p00),
        .src_addr_p10     (src_addr_p10),
        .src_addr_p01     (src_addr_p01),
        .src_addr_p11     (src_addr_p11),

        .src_pixels_valid (src_pixels_valid),
        .src_pixels_ready (src_pixels_ready),

        .src_pixel_p00    (src_pixel_p00),
        .src_pixel_p10    (src_pixel_p10),
        .src_pixel_p01    (src_pixel_p01),
        .src_pixel_p11    (src_pixel_p11),

        .dst_valid        (dst_valid),
        .dst_ready        (dst_ready),

        .dst_addr         (dst_addr),
        .dst_pixel        (dst_pixel),

        .dst_x            (dst_x),
        .dst_y            (dst_y)
    );


    bilinear_sdram_fetch #(
        .SOURCE_HOLDS(1),
        .ADDR_BITS     (ADDR_BITS),
        .MEM_DATA_BITS (32)
    ) u_fetch (
        .clk                      (clk),
        .rst                      (rst),
        .invalidate               (frame_start),

        .req_valid                (src_req_valid),
        .req_ready                (src_req_ready),

        .req_addr_p00             (src_addr_p00),
        .req_addr_p10             (src_addr_p10),
        .req_addr_p01             (src_addr_p01),
        .req_addr_p11             (src_addr_p11),

        .resp_valid               (src_pixels_valid),
        .resp_ready               (src_pixels_ready),

        .resp_pixel_p00           (src_pixel_p00),
        .resp_pixel_p10           (src_pixel_p10),
        .resp_pixel_p01           (src_pixel_p01),
        .resp_pixel_p11           (src_pixel_p11),

        .Sdr_init_done            (Sdr_init_done),
        .Sdr_init_ref_vld         (Sdr_init_ref_vld),
        .Sdr_busy                 (Sdr_busy),
        .App_wr_busy              (App_wr_busy),

        .App_rd_en                (App_rd_en),
        .App_rd_addr              (App_rd_addr),

        .Sdr_rd_en                (Sdr_rd_en),
        .Sdr_rd_dout              (Sdr_rd_dout),

        .memory_busy              (fetch_memory_busy),
        .fetch_busy               (fetch_busy),
        .issued_count             (fetch_issued_count),
        .returned_count           (fetch_returned_count),
        .unexpected_return_sticky (fetch_unexpected_return_sticky)
    );

endmodule
