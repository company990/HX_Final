`timescale 1ns/1ps

// ============================================================================
// D5-B1
// Bilinear source-coordinate + four-neighbour SDRAM address generator.
//
// This module is designed for the D5 architecture:
//
//   native BMP in SOURCE_BUFFER
//          |
//          v
//   scaler_dst_scan_gen  -> fixed 640x480 destination scan
//          |
//          v
//   bilinear_src_coord_gen  (this module)
//          |
//          +--> P00/P10/P01/P11 source addresses
//          +--> fx/fy Q0.16 interpolation fractions
//
// Mapping policy:
//   "align corners" bilinear mapping
//
//     src_x_fp = image_x * ((src_width  - 1) << 16) / (dst_width  - 1)
//     src_y_fp = image_y * ((src_height - 1) << 16) / (dst_height - 1)
//
// A quotient+remainder DDA is used at runtime, so:
//   - no runtime division
//   - no runtime coordinate multiplication for X/Y phase generation
//   - row-address multiplication is sequentialized once per source row
//   - the final fitted-image pixel maps exactly to the final source pixel
//
// Source BMP storage:
//   positive-height 24-bit BMP is stored natively in SDRAM bottom-up.
//   Therefore top-view source row y maps to memory row:
//       mem_y = src_height - 1 - y
//
// Four neighbours:
//   P00 = (x0, y0)
//   P10 = (x1, y0)
//   P01 = (x0, y1)
//   P11 = (x1, y1)
//
// Verilog-2001 compatible.
// ============================================================================

module bilinear_src_coord_gen #(
    parameter DST_BITS = 16,
    parameter ADDR_BITS = 21
)(
    input                       clk,
    input                       rst,

    // ------------------------------------------------------------------------
    // Configuration.
    // cfg_start is a one-cycle pulse.
    // ------------------------------------------------------------------------
    input                       cfg_start,

    input      [ADDR_BITS-1:0]  base_addr,
    input      [15:0]           src_width,
    input      [15:0]           src_height,
    input      [15:0]           dst_width,
    input      [15:0]           dst_height,
    input                       geometry_valid,

    output reg                  cfg_busy,
    output reg                  cfg_done,
    output reg                  cfg_valid,

    // ------------------------------------------------------------------------
    // Runtime scan interface.
    //
    // frame_start resets the bilinear phase to fitted-image coordinate (0,0).
    //
    // inside_image / image_x / image_y come from scaler_dst_scan_gen.
    // advance means the current destination pixel has been consumed.
    // ------------------------------------------------------------------------
    input                       frame_start,
    input                       advance,

    input                       inside_image,
    input      [15:0]           image_x,
    input      [15:0]           image_y,

    // ------------------------------------------------------------------------
    // Current source bilinear sample.
    // Valid only when sample_valid == 1.
    // ------------------------------------------------------------------------
    output                      sample_valid,

    output reg [15:0]           src_x0,
    output reg [15:0]           src_x1,
    output reg [15:0]           src_y0,
    output reg [15:0]           src_y1,

    // Q0.16 fractions.
    output reg [15:0]           frac_x,
    output reg [15:0]           frac_y,

    output reg [ADDR_BITS-1:0]  addr_p00,
    output reg [ADDR_BITS-1:0]  addr_p10,
    output reg [ADDR_BITS-1:0]  addr_p01,
    output reg [ADDR_BITS-1:0]  addr_p11
);

    // ========================================================================
    // Latched geometry
    // ========================================================================

    reg [ADDR_BITS-1:0] cfg_base_addr;

    reg [15:0] cfg_src_width;
    reg [15:0] cfg_src_height;
    reg [DST_BITS-1:0] cfg_dst_width;
    reg [DST_BITS-1:0] cfg_dst_height;

    // Stage4-B timing closure:
    // Precompute all frequently used "size - 1" values at configuration time.
    // Runtime logic compares directly against registered limits instead of
    // repeatedly building subtractors in the 125-MHz ext_mem_clk domain.
    reg [15:0] cfg_src_width_last;
    reg [15:0] cfg_src_height_last;
    reg [DST_BITS-1:0] cfg_dst_width_last;
    reg [DST_BITS-1:0] cfg_dst_height_last;


    // ========================================================================
    // Q16.16 source-coordinate step.
    //
    // Exact rational step:
    //   ((src_size - 1) << 16) / (dst_size - 1)
    //
    // quotient is the ordinary Q16.16 increment.
    // remainder is distributed by a second DDA accumulator, so repeated
    // additions equal floor(i * numerator / denominator) exactly.
    // ========================================================================

    reg [31:0] x_step_q16;
    reg [DST_BITS-1:0] x_step_rem;
    reg [DST_BITS-1:0] x_step_den;
    reg [DST_BITS-1:0] x_rem_threshold;

    reg [31:0] y_step_q16;
    reg [DST_BITS-1:0] y_step_rem;
    reg [DST_BITS-1:0] y_step_den;
    reg [DST_BITS-1:0] y_rem_threshold;


    // ========================================================================
    // Shared 32-bit / 16-bit sequential divider
    // ========================================================================

    reg         div_start;
    reg [31:0]  div_numerator;
    reg [15:0]  div_denominator;

    wire        div_busy;
    wire        div_done;
    wire [31:0] div_quotient;
    wire [15:0] div_remainder;

    u32_div_u16_seq u_div (
        .clk         (clk),
        .rst         (rst),
        .start       (div_start),
        .numerator   (div_numerator),
        .denominator (div_denominator),
        .busy        (div_busy),
        .done        (div_done),
        .quotient    (div_quotient),
        .remainder   (div_remainder)
    );


    // ========================================================================
    // Configuration FSM
    // ========================================================================

    localparam CFG_IDLE   = 3'd0;
    localparam CFG_KICK_X = 3'd1;
    localparam CFG_WAIT_X = 3'd2;
    localparam CFG_PREP_Y = 3'd3;
    localparam CFG_KICK_Y = 3'd4;
    localparam CFG_WAIT_Y = 3'd5;
    localparam CFG_COMMIT = 3'd6;

    reg [2:0] cfg_state;
    // D8-A-T: configuration predicates terminate here, not at step-register CE.
    // The input snapshot is consumed one cycle later; runtime mapping is unchanged.
    reg cfg_geometry_ok_r;
    reg cfg_x_single_r;
    reg cfg_y_single_r;

    always @(posedge clk or posedge rst) begin
        if (rst) begin

            cfg_state <= CFG_IDLE;
            cfg_geometry_ok_r <= 1'b0;
            cfg_x_single_r    <= 1'b0;
            cfg_y_single_r    <= 1'b0;

            cfg_busy  <= 1'b0;
            cfg_done  <= 1'b0;
            cfg_valid <= 1'b0;

            cfg_base_addr       <= {ADDR_BITS{1'b0}};
            cfg_src_width       <= 16'd0;
            cfg_src_height      <= 16'd0;
            cfg_dst_width       <= 16'd0;
            cfg_dst_height      <= 16'd0;
            cfg_src_width_last  <= 16'd0;
            cfg_src_height_last <= 16'd0;
            cfg_dst_width_last  <= 16'd0;
            cfg_dst_height_last <= 16'd0;

            x_step_q16      <= 32'd0;
            x_step_rem      <= 16'd0;
            x_step_den      <= 16'd1;
            x_rem_threshold <= 16'd1;

            y_step_q16      <= 32'd0;
            y_step_rem      <= 16'd0;
            y_step_den      <= 16'd1;
            y_rem_threshold <= 16'd1;

            div_start       <= 1'b0;
            div_numerator   <= 32'd0;
            div_denominator <= 16'd1;

        end
        else begin

            cfg_done  <= 1'b0;
            div_start <= 1'b0;

            case (cfg_state)

                CFG_IDLE:
                begin
                    if (cfg_start) begin

                        cfg_base_addr  <= base_addr;
                        cfg_src_width  <= src_width;
                        cfg_src_height <= src_height;
                        cfg_dst_width  <= dst_width;
                        cfg_dst_height <= dst_height;

                        cfg_src_width_last <=
                            (src_width > 16'd0) ?
                            (src_width - 16'd1) :
                            16'd0;

                        cfg_src_height_last <=
                            (src_height > 16'd0) ?
                            (src_height - 16'd1) :
                            16'd0;

                        cfg_dst_width_last <=
                            (dst_width > 16'd0) ?
                            (dst_width - 16'd1) :
                            16'd0;

                        cfg_dst_height_last <=
                            (dst_height > 16'd0) ?
                            (dst_height - 16'd1) :
                            16'd0;

                        cfg_valid <= 1'b0;
                        cfg_busy <= 1'b1;
                        cfg_geometry_ok_r <= geometry_valid &&
                            (src_width != 16'd0) && (src_height != 16'd0) &&
                            (dst_width != 16'd0) && (dst_height != 16'd0);
                        cfg_x_single_r <= !(|src_width[15:1]) ||
                                          !(|dst_width[15:1]);
                        cfg_y_single_r <= !(|src_height[15:1]) ||
                                          !(|dst_height[15:1]);
                        cfg_state <= CFG_COMMIT;
                    end
                end

                CFG_COMMIT:
                begin
                        if (!cfg_geometry_ok_r) begin

                            cfg_busy  <= 1'b0;
                            cfg_done  <= 1'b1;
                            cfg_valid <= 1'b0;
                            cfg_state <= CFG_IDLE;

                        end
                        else begin

                            cfg_busy <= 1'b1;

                            // X mapping degenerates to source x=0 when either
                            // source or destination has only one column.
                            if (cfg_x_single_r) begin

                                x_step_q16      <= 32'd0;
                                x_step_rem      <= 16'd0;
                                x_step_den      <= 16'd1;
                                x_rem_threshold <= 16'd1;

                                cfg_state <= CFG_PREP_Y;

                            end
                            else begin

                                div_numerator <=
                                    {cfg_src_width_last, 16'd0};

                                div_denominator <= cfg_dst_width_last;
                                x_step_den      <= cfg_dst_width_last;

                                cfg_state <= CFG_KICK_X;

                            end
                        end
                end


                CFG_KICK_X:
                begin
                    div_start <= 1'b1;
                    cfg_state <= CFG_WAIT_X;
                end


                CFG_WAIT_X:
                begin
                    if (div_done) begin
                        x_step_q16      <= div_quotient;
                        x_step_rem      <= div_remainder;
                        x_rem_threshold <=
                            x_step_den - div_remainder;

                        cfg_state <= CFG_PREP_Y;
                    end
                end


                CFG_PREP_Y:
                begin
                    if (cfg_y_single_r) begin

                        y_step_q16      <= 32'd0;
                        y_step_rem      <= 16'd0;
                        y_step_den      <= 16'd1;
                        y_rem_threshold <= 16'd1;

                        cfg_busy  <= 1'b0;
                        cfg_done  <= 1'b1;
                        cfg_valid <= 1'b1;

                        cfg_state <= CFG_IDLE;

                    end
                    else begin

                        div_numerator <=
                            {cfg_src_height_last, 16'd0};

                        div_denominator <= cfg_dst_height_last;
                        y_step_den      <= cfg_dst_height_last;

                        cfg_state <= CFG_KICK_Y;
                    end
                end


                CFG_KICK_Y:
                begin
                    div_start <= 1'b1;
                    cfg_state <= CFG_WAIT_Y;
                end


                CFG_WAIT_Y:
                begin
                    if (div_done) begin

                        y_step_q16      <= div_quotient;
                        y_step_rem      <= div_remainder;
                        y_rem_threshold <=
                            y_step_den - div_remainder;

                        cfg_busy  <= 1'b0;
                        cfg_done  <= 1'b1;
                        cfg_valid <= 1'b1;

                        cfg_state <= CFG_IDLE;
                    end
                end


                default:
                begin
                    cfg_state <= CFG_IDLE;
                    cfg_busy  <= 1'b0;
                    cfg_valid <= 1'b0;
                end

            endcase
        end
    end


    // ========================================================================
    // Runtime phase accumulators
    //
    // Stage4-B timing closure:
    //   1) row/column end tests use registered *_last limits.
    //   2) X/Y phase and remainder updates share arithmetic at runtime.
    //   3) the remainder carry test is rewritten from
    //
    //          rem + step_rem >= den
    //
    //      to
    //
    //          rem >= (den - step_rem)
    //
    //      where (den-step_rem) is also precomputed.
    //
    // The DDA arithmetic is mathematically identical, but the x_fp_acc/y_fp_acc
    // D paths no longer contain "add remainder -> compare -> add quotient -> +1"
    // as one long combinational cone.
    // ========================================================================

   reg [31:0] x_fp_acc;
    reg [DST_BITS-1:0] x_rem_acc;
    reg [31:0] y_fp_acc;
    reg [DST_BITS-1:0] y_rem_acc;

    // ========================================================================
    // Stage4-C timing closure
    //
    // Carry the fitted-row-end condition through the existing P1/P2 pixel
    // pipeline.  On the advance edge the DDA therefore sees only a local
    // registered 1-bit flag instead of the cross-module 16-bit
    //     image_x == cfg_dst_width_last
    // comparison.
    //
    // p2_row_last always belongs to the same destination pixel whose P2
    // bilinear payload is currently being held for consumption.
    // ========================================================================
    reg p1_row_last;
    reg p2_row_last;

    wire x_rem_carry;
    wire y_rem_carry;
    // X advances inside a row; Y advances only at its last pixel.
    wire [31:0] phase_base = p2_row_last ? y_fp_acc : x_fp_acc;
    wire [31:0] phase_step = p2_row_last ? y_step_q16 : x_step_q16;
    wire phase_carry = p2_row_last ? y_rem_carry : x_rem_carry;
    wire [31:0] phase_next = phase_base + phase_step + phase_carry;
    wire [DST_BITS-1:0] rem_base = p2_row_last ? y_rem_acc : x_rem_acc;
    wire [DST_BITS-1:0] rem_step = p2_row_last ? y_step_rem : x_step_rem;
    wire [DST_BITS-1:0] rem_threshold = p2_row_last ? y_rem_threshold : x_rem_threshold;
    wire [DST_BITS-1:0] rem_operand = phase_carry ? ~rem_threshold : rem_step;
    wire [DST_BITS-1:0] rem_next = rem_base + rem_operand + phase_carry;


    assign x_rem_carry =
        (x_step_rem != 16'd0) &&
        (x_rem_acc >= x_rem_threshold);

    assign y_rem_carry =
        (y_step_rem != 16'd0) &&
        (y_rem_acc >= y_rem_threshold);



    always @(posedge clk or posedge rst) begin
        if (rst) begin

            x_fp_acc  <= 32'd0;
            x_rem_acc <= 16'd0;

            y_fp_acc  <= 32'd0;
            y_rem_acc <= 16'd0;

        end
        else if (frame_start) begin

            x_fp_acc  <= 32'd0;
            x_rem_acc <= 16'd0;

            y_fp_acc  <= 32'd0;
            y_rem_acc <= 16'd0;

        end
      else if (advance && inside_image && cfg_valid) begin
    
        // ---------------------------------------------------------------
        // Stage4-C:
        // p2_row_last is registered together with the current pixel's
        // complete P2 payload.  Therefore the advance-edge DDA no longer
        // depends combinationally on image_x.
        // ---------------------------------------------------------------
    
        if (p2_row_last) begin
    
            x_fp_acc  <= 32'd0;
            x_rem_acc <= 16'd0;
    
            // Advance Y only if another fitted-image row remains.
            if (image_y < cfg_dst_height_last) begin
    
                if (y_rem_carry) begin
    
                    y_rem_acc <= rem_next;
                    y_fp_acc  <= phase_next;
    
                end
                else begin
    
                    y_rem_acc <= rem_next;
                    y_fp_acc  <= phase_next;
    
                end
    
            end
    
        end
    
        // ---------------------------------------------------------------
        // Advance one fitted-image pixel horizontally.
        // ---------------------------------------------------------------
    
        else begin
    
            if (x_rem_carry) begin
    
                x_rem_acc <= rem_next;
                x_fp_acc  <= phase_next;
    
            end
            else begin
    
                x_rem_acc <= rem_next;
                x_fp_acc  <= phase_next;
    
            end
    
        end
    
    end
    end


    // ========================================================================
    // Timing-optimized current bilinear coordinates and SDRAM addresses
    //
    // D5 timing fix:
    //   The original implementation formed
    //       (src_height-1-src_y0) * src_width
    //   and all four final addresses in one combinational path.  At 125 MHz
    //   this became the dominant Gate-STA path.
    //
    //   The memory row base is constant for an entire fitted-image row, so it
    //   is now prepared sequentially once per row using a 16-cycle shift/add
    //   multiplier.  The four per-pixel addresses then require only ONE
    //   base+x addition each.
    //
    // Interface compatibility is preserved.  sample_valid now means:
    //   "the current image pixel coordinates/fractions AND row bases are ready".
    // scaler_pixel_pipeline already waits in ST_CLASSIFY until sample_valid.
    // ========================================================================

    // Current clamped top-view Y used only when a new row-base calculation
    // starts.  These paths terminate in local row-preparation registers rather
    // than in the SDRAM fetch address registers.
    wire [15:0] current_src_y0_clamped;
    wire [15:0] current_y0_mem;
    wire        current_y_has_next;

    assign current_src_y0_clamped =
        (y_fp_acc[31:16] > cfg_src_height_last) ?
        cfg_src_height_last :
        y_fp_acc[31:16];

    assign current_y0_mem =
        // Equivalent to last - min(raw,last), but subtraction and comparison
        // run in parallel; the clamp mux no longer feeds the subtractor.
        (y_fp_acc[31:16] > cfg_src_height_last) ? 16'd0 :
        (cfg_src_height_last - y_fp_acc[31:16]);

    assign current_y_has_next =
        (current_src_y0_clamped < cfg_src_height_last);


    // ------------------------------------------------------------------------
    // Per-row sequential memory-row-base preparation.
    // ------------------------------------------------------------------------

    localparam ROW_IDLE  = 2'd0;
    localparam ROW_MUL   = 2'd1;
    localparam ROW_BASE0 = 2'd2;
    localparam ROW_BASE1 = 2'd3;

    reg [1:0]  row_state;
    reg        row_recalc_pending;
    reg        row_base_valid;

    reg [4:0]  row_mul_bit;
    reg [ADDR_BITS-1:0] row_mul_acc;
    reg [ADDR_BITS-1:0] row_mul_multiplicand;
    reg [15:0] row_mul_multiplier;

    reg [ADDR_BITS-1:0] row_product_r;
    reg [ADDR_BITS-1:0] row1_offset_r;
    reg        row_y_has_next_r;

    reg [ADDR_BITS-1:0] row0_base_r;
    reg [ADDR_BITS-1:0] row1_base_r;

    wire [ADDR_BITS-1:0] row_mul_addend;
    wire [ADDR_BITS-1:0] row_mul_acc_next;

    assign row_mul_addend =
        row_mul_multiplier[0] ? row_mul_multiplicand : 32'd0;

    assign row_mul_acc_next =
        row_mul_acc + row_mul_addend;


    always @(posedge clk or posedge rst) begin
        if (rst) begin

            row_state          <= ROW_IDLE;
            row_recalc_pending <= 1'b0;
            row_base_valid     <= 1'b0;

            row_mul_bit          <= 5'd0;
            row_mul_acc          <= 32'd0;
            row_mul_multiplicand <= 32'd0;
            row_mul_multiplier   <= 16'd0;

            row_product_r     <= 32'd0;
            row1_offset_r     <= 32'd0;
            row_y_has_next_r  <= 1'b0;

            row0_base_r <= 32'd0;
            row1_base_r <= 32'd0;

        end
        else if (cfg_start) begin

            // Geometry is being replaced.  Do not expose stale row bases.
            row_state          <= ROW_IDLE;
            row_recalc_pending <= 1'b0;
            row_base_valid     <= 1'b0;

        end
        else if (frame_start) begin

            // Runtime phase is reset to source y=0 in the phase accumulator
            // block on the same edge.  Start row preparation on the next edge,
            // after y_fp_acc has taken that reset value.
            row_state          <= ROW_IDLE;
            row_recalc_pending <= 1'b1;
            row_base_valid     <= 1'b0;

        end
        else begin

            // The source Y phase changes only after the last fitted-image pixel
            // of a row is consumed.  Invalidate the old bases immediately and
            // prepare the next row on following cycles.
            if (
            advance &&
            inside_image &&
            cfg_valid &&
            p2_row_last
        ) begin
        
            row_base_valid <= 1'b0;
        
            if (image_y < cfg_dst_height_last)
                row_recalc_pending <= 1'b1;
            else
                row_recalc_pending <= 1'b0;
        
        end

            case (row_state)

                ROW_IDLE:
                begin
                    if (row_recalc_pending && cfg_valid) begin

                        row_recalc_pending <= 1'b0;

                        row_mul_bit          <= 5'd0;
                        row_mul_acc          <= 32'd0;
                        row_mul_multiplicand <= {16'd0, cfg_src_width};
                        row_mul_multiplier   <= current_y0_mem;

                        row_y_has_next_r <= current_y_has_next;

                        row_state <= ROW_MUL;
                    end
                end


                ROW_MUL:
                begin
                    // 16x16 -> 32 shift/add multiply, one 32-bit addition max
                    // per cycle.  This intentionally trades a few row-boundary
                    // cycles for a much shorter 125-MHz critical path.
                    row_mul_acc          <= row_mul_acc_next;
                    row_mul_multiplicand <= row_mul_multiplicand << 1;
                    row_mul_multiplier   <= row_mul_multiplier >> 1;

                    if (row_mul_bit == 5'd15) begin
                        row_product_r <= row_mul_acc_next;
                        row_state     <= ROW_BASE0;
                    end
                    else begin
                        row_mul_bit <= row_mul_bit + 5'd1;
                    end
                end


                ROW_BASE0:
                begin
                    // First base-address stage: one add/sub per result.
                    row0_base_r <=
                        {{(32-ADDR_BITS){1'b0}}, cfg_base_addr} +
                        row_product_r;

                    if (row_y_has_next_r)
                        row1_offset_r <= row_product_r - {16'd0, cfg_src_width};
                    else
                        row1_offset_r <= row_product_r;

                    row_state <= ROW_BASE1;
                end


                ROW_BASE1:
                begin
                    // Second base-address stage.  Once this register is written,
                    // all four per-pixel addresses are simple base+x additions.
                    row1_base_r <=
                        {{(32-ADDR_BITS){1'b0}}, cfg_base_addr} +
                        row1_offset_r;

                    row_base_valid <= 1'b1;
                    row_state      <= ROW_IDLE;
                end


                default:
                begin
                    row_state      <= ROW_IDLE;
                    row_base_valid <= 1'b0;
                end

            endcase
        end
    end


    // ------------------------------------------------------------------------
    // Stage3 P1/P2 current-pixel payload pipeline
    //
    // P1:
    //   - clamps x0/y0
    //   - prepares x1/y1
    //   - snapshots frac_x/frac_y
    //   - snapshots the already-prepared row bases
    //
    // P2:
    //   - shares one base+x adder over four preparation clocks
    //   - publishes all coordinates/fractions/addresses atomically
    //
    // sample_valid is asserted ONLY for a complete P2 payload.
    //
    // On advance/cfg_start/frame_start, both valid bits are flushed.  This is
    // intentional: the destination scan / DDA phase update on that same edge,
    // and the new pixel is captured on a following edge.  scaler_pixel_pipeline
    // already waits in ST_CLASSIFY for sample_valid.
    // ------------------------------------------------------------------------

    reg [1:0] addr_phase;
    reg p1_valid;
    reg p2_valid;

    wire coord_p1_valid;
    wire coord_p2_valid;

    assign coord_p1_valid = p1_valid;
    assign coord_p2_valid = p2_valid;
    
    reg [15:0] p1_src_x0;
    reg [15:0] p1_src_x1;
    reg [15:0] p1_src_y0;
    reg [15:0] p1_src_y1;

    reg [15:0] p1_frac_x;
    reg [15:0] p1_frac_y;

    reg [ADDR_BITS-1:0] p1_row0_base;
    reg [ADDR_BITS-1:0] p1_row1_base;

    wire [15:0] current_src_x_raw;
    wire [15:0] current_src_y_raw;

    wire [15:0] current_src_x0_clamped;
    wire [15:0] current_src_x1_clamped;
    wire [15:0] current_src_y1_clamped;

    wire [15:0] current_frac_x_clamped;
    wire [15:0] current_frac_y_clamped;

    wire [ADDR_BITS-1:0] addr_row = addr_phase[1] ? p1_row1_base : p1_row0_base;
    wire [15:0] addr_x = addr_phase[0] ? p1_src_x1 : p1_src_x0;
    wire [ADDR_BITS-1:0] shared_addr = addr_row + addr_x;


    assign current_src_x_raw = x_fp_acc[31:16];
    assign current_src_y_raw = y_fp_acc[31:16];

    // Direct single-level clamp forms.  In particular, x1/y1 do not depend on
    // an already-muxed x0/y0 signal, which keeps the P1 data path short.
    assign current_src_x0_clamped =
        (current_src_x_raw > cfg_src_width_last) ?
        cfg_src_width_last :
        current_src_x_raw;

    assign current_src_x1_clamped =
        (current_src_x_raw >= cfg_src_width_last) ?
        cfg_src_width_last :
        (current_src_x_raw + 16'd1);

    assign current_src_y1_clamped =
        (current_src_y_raw >= cfg_src_height_last) ?
        cfg_src_height_last :
        (current_src_y_raw + 16'd1);

    assign current_frac_x_clamped =
        (current_src_x_raw > cfg_src_width_last) ?
        16'd0 :
        x_fp_acc[15:0];

    assign current_frac_y_clamped =
        (current_src_y_raw > cfg_src_height_last) ?
        16'd0 :
        y_fp_acc[15:0];


    // Payload is retained while invalid; consumers must use sample_valid.
    assign sample_valid =
        p2_valid;


    always @(posedge clk or posedge rst) begin
        if (rst) begin

            addr_phase <= 0;
            p1_valid <= 1'b0;
            p2_valid <= 1'b0;
            
            p1_row_last <= 1'b0;
            p2_row_last <= 1'b0;

            p1_src_x0 <= 16'd0;
            p1_src_x1 <= 16'd0;
            

            p1_src_y0 <= 16'd0;
            p1_src_y1 <= 16'd0;

            p1_frac_x <= 16'd0;
            p1_frac_y <= 16'd0;

            p1_row0_base <= 32'd0;
            p1_row1_base <= 32'd0;

            src_x0 <= 16'd0;
            src_x1 <= 16'd0;
            src_y0 <= 16'd0;
            src_y1 <= 16'd0;

            frac_x <= 16'd0;
            frac_y <= 16'd0;

            addr_p00 <= {ADDR_BITS{1'b0}};
            addr_p10 <= {ADDR_BITS{1'b0}};
            addr_p01 <= {ADDR_BITS{1'b0}};
            addr_p11 <= {ADDR_BITS{1'b0}};

        end
        else if (cfg_start) begin

            // A new geometry invalidates every in-flight pixel payload.
            addr_phase <= 0;
            p1_valid <= 1'b0;
            p2_valid <= 1'b0;
            
            p1_row_last <= 1'b0;
            p2_row_last <= 1'b0;

        end
        else if (frame_start) begin

            // Phase registers are reset on this same edge.  Capture the first
            // fitted-image pixel only after the new phase / row base are ready.
            addr_phase <= 0;
            p1_valid <= 1'b0;
            p2_valid <= 1'b0;

            p1_row_last <= 1'b0;
            p2_row_last <= 1'b0;

        end
        else if (advance) begin

            // The current destination pixel has just been consumed.  Do not
            // reuse its payload for the next raster position.
            addr_phase <= 0;
            p1_valid <= 1'b0;
            p2_valid <= 1'b0;
            
            p1_row_last <= 1'b0;
            p2_row_last <= 1'b0;

        end
        else if (!cfg_valid || !inside_image || !row_base_valid) begin

            // Border pixels never need source memory.  A source-row transition
            // also invalidates the old row bases until ROW_BASE1 completes.
            addr_phase <= 0;
            p1_valid <= 1'b0;
            p2_valid <= 1'b0;
            
            p1_row_last <= 1'b0;
            p2_row_last <= 1'b0;
            
        end
        else begin

            // ---------------------------------------------------------------
            // P1: exactly one preparation cycle for the current scan pixel.
            // ---------------------------------------------------------------
           if (!p1_valid && !p2_valid) begin

            p1_src_x0 <= current_src_x0_clamped;
            p1_src_x1 <= current_src_x1_clamped;
        
            p1_src_y0 <= current_src_y0_clamped;
            p1_src_y1 <= current_src_y1_clamped;
        
            p1_frac_x <= current_frac_x_clamped;
            p1_frac_y <= current_frac_y_clamped;
        
            p1_row0_base <= row0_base_r;
            p1_row1_base <= row1_base_r;
        
            // Stage4-C:
            // Snapshot the destination fitted-row boundary together with P1.
            p1_row_last <= (image_x == cfg_dst_width_last);
        
            addr_phase <= 0;
            p1_valid <= 1'b1;
        
        end

            // ---------------------------------------------------------------
            // P2: publish one atomic payload from registered P1 operands.
            // ---------------------------------------------------------------
            else if (p1_valid && !p2_valid) begin

                // The consumer waits for sample_valid. Publish only after
                // all four neighbour addresses have been computed.
                case (addr_phase)
                    0: addr_p00 <= shared_addr;
                    1: addr_p10 <= shared_addr;
                    2: addr_p01 <= shared_addr;
                    3: addr_p11 <= shared_addr;
                endcase
                if (addr_phase==3) begin
                    src_x0<=p1_src_x0; src_x1<=p1_src_x1;
                    src_y0<=p1_src_y0; src_y1<=p1_src_y1;
                    frac_x<=p1_frac_x; frac_y<=p1_frac_y;
                    p2_row_last<=p1_row_last; p1_row_last<=0;
                    p1_valid<=0; p2_valid<=1;
                    addr_phase<=0;
                end else addr_phase<=addr_phase+1'b1;
            end

            // p2_valid==1: hold the complete payload until advance.
        end
    end

endmodule



// ============================================================================
// Unsigned 32-bit / 16-bit sequential divider.
//
// One quotient/remainder result after 32 busy clocks.
// Verilog-2001 compatible.
// ============================================================================

module u32_div_u16_seq (
    input             clk,
    input             rst,

    input             start,
    input      [31:0] numerator,
    input      [15:0] denominator,

    output reg        busy,
    output reg        done,

    output reg [31:0] quotient,
    output reg [15:0] remainder
);

    // The shift register contains remaining dividend bits followed by quotient bits.
    reg [15:0] divisor_reg;

    reg [31:0] quotient_work;
    reg [16:0] remainder_work;

    reg [5:0] count;

    reg [16:0] rem_next;
    reg [31:0] quot_next;

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;

            quotient  <= 32'd0;
            remainder <= 16'd0;

            divisor_reg    <= 16'd1;

            quotient_work  <= 32'd0;
            remainder_work <= 17'd0;

            count <= 6'd0;

        end
        else begin

            done <= 1'b0;

            if (start && !busy) begin

                if (denominator == 16'd0) begin

                    busy      <= 1'b0;
                    done      <= 1'b1;
                    quotient  <= 32'd0;
                    remainder <= 16'd0;

                end
                else begin

                    busy <= 1'b1;

                    divisor_reg    <= denominator;

                    quotient_work  <= numerator;
                    remainder_work <= 17'd0;

                    count <= 6'd0;
                end

            end
            else if (busy) begin

                rem_next =
                    {remainder_work[15:0], quotient_work[31]};

                if (rem_next >= {1'b0, divisor_reg}) begin

                    rem_next =
                        rem_next - {1'b0, divisor_reg};

                    quot_next =
                        {quotient_work[30:0], 1'b1};

                end
                else begin

                    quot_next =
                        {quotient_work[30:0], 1'b0};

                end

                quotient_work  <= quot_next;
                remainder_work <= rem_next;

                if (count == 6'd31) begin

                    busy <= 1'b0;
                    done <= 1'b1;

                    quotient  <= quot_next;
                    remainder <= rem_next[15:0];

                end
                else begin

                    count <= count + 1'b1;
                end
            end
        end
    end

endmodule
