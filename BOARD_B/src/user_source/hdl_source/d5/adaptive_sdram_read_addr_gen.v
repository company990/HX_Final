`timescale 1ns/1ps

// ============================================================================
// G1-C3-D4-B1
// Synthesizable sequential adaptive SDRAM read-address generator.
//
// Runtime mapping:
//   HDMI output is fixed at 640x480.
//   Source BMP pixels are stored contiguously in SDRAM in original
//   positive-height BMP bottom-up order (WRITE_V_FLIP=0).
//
// For each accepted output pixel (advance=1):
//   1) preserve aspect ratio using precomputed dst_width/dst_height/offsets
//   2) nearest-neighbor map output coordinate -> source coordinate
//   3) correct BMP bottom-up orientation during read-address generation
//   4) use base_addr as a safe dummy address in letterbox/pillarbox regions
//
// No runtime '*' or '/' is used.
// Two 16-cycle sequential dividers are performed only when cfg_start arrives,
// followed by a 16-cycle shift/add precomputation for the vertical row jump.
// ============================================================================

module adaptive_sdram_read_addr_gen #(
    parameter ADDR_BITS  = 21,
    parameter OUT_WIDTH  = 640,
    parameter OUT_HEIGHT = 480
)(
    input                       clk,
    input                       rst,

    // ---------------------------------------------------------
    // Geometry/configuration load.
    // Geometry must stay stable when cfg_start is pulsed.
    // ---------------------------------------------------------
    input                       cfg_start,

    input  [ADDR_BITS-1:0]      base_addr,
    input  [ADDR_BITS-1:0]      src_pixel_count,

    input  [15:0]               src_width,
    input  [15:0]               src_height,

    input  [15:0]               dst_width,
    input  [15:0]               dst_height,

    input  [15:0]               x_offset,
    input  [15:0]               y_offset,

    input                       geometry_valid,

    output reg                  cfg_busy,
    output reg                  cfg_done,
    output reg                  cfg_valid,

    // ---------------------------------------------------------
    // Runtime output-pixel address stream.
    //
    // frame_start resets the stream to output coordinate (0,0).
    // advance moves exactly one output pixel forward.
    // ---------------------------------------------------------
    input                       frame_start,
    input                       advance,

    output reg [15:0]           out_x,
    output reg [15:0]           out_y,

    output reg                  inside_image,
    output reg [ADDR_BITS-1:0]  read_addr
);

    // =========================================================
    // Latched configuration
    // =========================================================

    reg [ADDR_BITS-1:0] cfg_base_addr;
    reg [ADDR_BITS-1:0] cfg_src_pixel_count;

    reg [15:0] cfg_src_width;
    reg [15:0] cfg_src_height;

    reg [15:0] cfg_dst_width;
    reg [15:0] cfg_dst_height;

    reg [15:0] cfg_x_offset;
    reg [15:0] cfg_y_offset;


    // =========================================================
    // DDA step values
    //
    // src_width / dst_width  = x_step_q remainder x_step_r
    // src_height / dst_height = y_step_q remainder y_step_r
    //
    // Runtime:
    //   source += quotient;
    //   remainder += step_r;
    //   if remainder >= dst_size:
    //       source += 1;
    //       remainder -= dst_size;
    // =========================================================

    reg [15:0] x_step_q;
    reg [15:0] x_step_r;

    reg [15:0] y_step_q;
    reg [15:0] y_step_r;

    // y_step_q * src_width, precomputed by shift/add.
    reg [ADDR_BITS-1:0] y_row_jump_q;


    // =========================================================
    // Shared sequential divider
    // =========================================================

    reg        div_start;
    reg [15:0] div_numerator;
    reg [15:0] div_denominator;

    wire       div_busy;
    wire       div_done;
    wire [15:0] div_quotient;
    wire [15:0] div_remainder;

    u16_divmod_seq u_div (
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


    // =========================================================
    // Configuration FSM
    // =========================================================

    localparam CFG_IDLE   = 3'd0;
    localparam CFG_WAIT_X = 3'd1;
    localparam CFG_WAIT_Y = 3'd2;
    localparam CFG_MUL_Y  = 3'd3;

    reg [2:0] cfg_state;

    reg [31:0] mul_acc;
    reg [31:0] mul_multiplicand;
    reg [15:0] mul_multiplier;
    reg [4:0]  mul_count;

    reg [31:0] mul_acc_next;


    always @(posedge clk or posedge rst) begin

        if (rst) begin

            cfg_state <= CFG_IDLE;

            cfg_busy  <= 1'b0;
            cfg_done  <= 1'b0;
            cfg_valid <= 1'b0;

            cfg_base_addr       <= {ADDR_BITS{1'b0}};
            cfg_src_pixel_count <= {ADDR_BITS{1'b0}};

            cfg_src_width  <= 16'd0;
            cfg_src_height <= 16'd0;

            cfg_dst_width  <= 16'd0;
            cfg_dst_height <= 16'd0;

            cfg_x_offset <= 16'd0;
            cfg_y_offset <= 16'd0;

            x_step_q <= 16'd0;
            x_step_r <= 16'd0;

            y_step_q <= 16'd0;
            y_step_r <= 16'd0;

            y_row_jump_q <= {ADDR_BITS{1'b0}};

            div_start       <= 1'b0;
            div_numerator   <= 16'd0;
            div_denominator <= 16'd1;

            mul_acc          <= 32'd0;
            mul_multiplicand <= 32'd0;
            mul_multiplier   <= 16'd0;
            mul_count        <= 5'd0;

        end
        else begin

            cfg_done  <= 1'b0;
            div_start <= 1'b0;

            case (cfg_state)

                CFG_IDLE:
                begin

                    if (cfg_start) begin

                        // Latch the complete metadata snapshot.
                        cfg_base_addr       <= base_addr;
                        cfg_src_pixel_count <= src_pixel_count;

                        cfg_src_width       <= src_width;
                        cfg_src_height      <= src_height;

                        cfg_dst_width       <= dst_width;
                        cfg_dst_height      <= dst_height;

                        cfg_x_offset        <= x_offset;
                        cfg_y_offset        <= y_offset;

                        cfg_valid <= 1'b0;

                        // Reject incomplete/invalid geometry.
                        if (
                            !geometry_valid ||
                            (src_width       == 16'd0) ||
                            (src_height      == 16'd0) ||
                            (dst_width       == 16'd0) ||
                            (dst_height      == 16'd0) ||
                            (src_pixel_count == {ADDR_BITS{1'b0}})
                        ) begin

                            cfg_busy  <= 1'b0;
                            cfg_done  <= 1'b1;
                            cfg_valid <= 1'b0;

                        end
                        else begin

                            cfg_busy <= 1'b1;

                            // First divider:
                            // src_width / dst_width
                            div_numerator   <= src_width;
                            div_denominator <= dst_width;
                            div_start       <= 1'b1;

                            cfg_state <= CFG_WAIT_X;

                        end

                    end

                end


                CFG_WAIT_X:
                begin

                    if (div_done) begin

                        x_step_q <= div_quotient;
                        x_step_r <= div_remainder;

                        // Second divider:
                        // src_height / dst_height
                        div_numerator   <= cfg_src_height;
                        div_denominator <= cfg_dst_height;
                        div_start       <= 1'b1;

                        cfg_state <= CFG_WAIT_Y;

                    end

                end


                CFG_WAIT_Y:
                begin

                    if (div_done) begin

                        y_step_q <= div_quotient;
                        y_step_r <= div_remainder;

                        // Precompute:
                        // y_row_jump_q = y_step_q * src_width
                        //
                        // Use a 16-cycle shift/add multiply, so runtime
                        // address generation needs only add/subtract logic.
                        mul_acc          <= 32'd0;
                        mul_multiplicand <= {16'd0, cfg_src_width};
                        mul_multiplier   <= div_quotient;
                        mul_count        <= 5'd0;

                        cfg_state <= CFG_MUL_Y;

                    end

                end


                CFG_MUL_Y:
                begin

                    if (mul_multiplier[0])
                        mul_acc_next = mul_acc + mul_multiplicand;
                    else
                        mul_acc_next = mul_acc;

                    if (mul_count == 5'd15) begin

                        y_row_jump_q <= mul_acc_next[ADDR_BITS-1:0];

                        cfg_busy  <= 1'b0;
                        cfg_done  <= 1'b1;
                        cfg_valid <= 1'b1;

                        cfg_state <= CFG_IDLE;

                    end
                    else begin

                        mul_acc          <= mul_acc_next;
                        mul_multiplicand <= mul_multiplicand << 1;
                        mul_multiplier   <= mul_multiplier >> 1;
                        mul_count        <= mul_count + 1'b1;

                    end

                end


                default:
                begin

                    cfg_state <= CFG_IDLE;
                    cfg_busy  <= 1'b0;

                end

            endcase

        end

    end


    // =========================================================
    // Runtime DDA state
    // =========================================================

    reg [15:0] src_x_cur;
    reg [15:0] x_rem_cur;

    reg [15:0] src_y_top_cur;
    reg [15:0] y_rem_cur;

    // Address of source x=0 for the currently mapped source row.
    // Because source data is stored bottom-up, the first displayed
    // source row begins at:
    //
    // base + src_pixel_count - src_width
    reg [ADDR_BITS-1:0] row_base_cur;


    reg [16:0] x_rem_sum;
    reg [16:0] y_rem_sum;

    reg        x_carry;
    reg        y_carry;

    reg [16:0] x_source_step;
    reg [16:0] y_source_step;

    reg [ADDR_BITS:0] first_row_base_ext;
    reg [ADDR_BITS:0] row_jump_ext;


    // =========================================================
    // Current output address / inside flag
    // =========================================================

    reg [16:0] image_x_end;
    reg [16:0] image_y_end;

    reg [ADDR_BITS:0] read_addr_ext;

    always @* begin

        image_x_end = {1'b0, cfg_x_offset} + {1'b0, cfg_dst_width};
        image_y_end = {1'b0, cfg_y_offset} + {1'b0, cfg_dst_height};

        inside_image = 1'b0;
        read_addr    = cfg_base_addr;

        if (
            cfg_valid &&

            ({1'b0, out_x} >= {1'b0, cfg_x_offset}) &&
            ({1'b0, out_x} <  image_x_end) &&

            ({1'b0, out_y} >= {1'b0, cfg_y_offset}) &&
            ({1'b0, out_y} <  image_y_end)
        ) begin

            inside_image = 1'b1;

            read_addr_ext =
                {1'b0, row_base_cur} +
                {{(ADDR_BITS-16+1){1'b0}}, src_x_cur};

            read_addr = read_addr_ext[ADDR_BITS-1:0];

        end
        else begin

            read_addr_ext = {1'b0, cfg_base_addr};

        end

    end


    // =========================================================
    // Runtime sequencing
    // =========================================================

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            out_x <= 16'd0;
            out_y <= 16'd0;

            src_x_cur <= 16'd0;
            x_rem_cur <= 16'd0;

            src_y_top_cur <= 16'd0;
            y_rem_cur     <= 16'd0;

            row_base_cur <= {ADDR_BITS{1'b0}};

        end
        else begin

            // Compute the source-memory address of displayed source row 0.
            first_row_base_ext =
                {1'b0, cfg_base_addr} +
                {1'b0, cfg_src_pixel_count} -
                {{(ADDR_BITS-16+1){1'b0}}, cfg_src_width};

            if (frame_start) begin

                out_x <= 16'd0;
                out_y <= 16'd0;

                src_x_cur <= 16'd0;
                x_rem_cur <= 16'd0;

                src_y_top_cur <= 16'd0;
                y_rem_cur     <= 16'd0;

                if (cfg_valid)
                    row_base_cur <= first_row_base_ext[ADDR_BITS-1:0];
                else
                    row_base_cur <= cfg_base_addr;

            end
            else if (advance) begin

                // -------------------------------------------------
                // End of one HDMI output row.
                // -------------------------------------------------
                if (out_x == OUT_WIDTH - 1) begin

                    out_x <= 16'd0;

                    src_x_cur <= 16'd0;
                    x_rem_cur <= 16'd0;

                    // End of complete HDMI output frame.
                    if (out_y == OUT_HEIGHT - 1) begin

                        out_y <= 16'd0;

                        src_y_top_cur <= 16'd0;
                        y_rem_cur     <= 16'd0;

                        if (cfg_valid)
                            row_base_cur <= first_row_base_ext[ADDR_BITS-1:0];
                        else
                            row_base_cur <= cfg_base_addr;

                    end
                    else begin

                        out_y <= out_y + 1'b1;

                        // Advance source Y only after a displayed image row,
                        // and not after the final fitted image row.
                        if (
                            cfg_valid &&
                            ({1'b0, out_y} >= {1'b0, cfg_y_offset}) &&
                            ({1'b0, out_y} <
                                ({1'b0, cfg_y_offset} +
                                 {1'b0, cfg_dst_height} - 1'b1))
                        ) begin

                            y_rem_sum = {1'b0, y_rem_cur} +
                                        {1'b0, y_step_r};

                            if (y_rem_sum >= {1'b0, cfg_dst_height}) begin
                                y_carry   = 1'b1;
                                y_rem_sum = y_rem_sum -
                                            {1'b0, cfg_dst_height};
                            end
                            else begin
                                y_carry = 1'b0;
                            end

                            y_source_step =
                                {1'b0, y_step_q} +
                                y_carry;

                            src_y_top_cur <=
                                src_y_top_cur +
                                y_source_step[15:0];

                            y_rem_cur <= y_rem_sum[15:0];

                            row_jump_ext =
                                {1'b0, y_row_jump_q};

                            if (y_carry)
                                row_jump_ext =
                                    row_jump_ext +
                                    {{(ADDR_BITS-16+1){1'b0}},
                                     cfg_src_width};

                            row_base_cur <=
                                row_base_cur -
                                row_jump_ext[ADDR_BITS-1:0];

                        end

                    end

                end

                // -------------------------------------------------
                // Advance within one HDMI output row.
                // -------------------------------------------------
                else begin

                    out_x <= out_x + 1'b1;

                    // Advance source X only while leaving one fitted
                    // image pixel for the next fitted image pixel.
                    if (
                        cfg_valid &&

                        ({1'b0, out_y} >= {1'b0, cfg_y_offset}) &&
                        ({1'b0, out_y} <
                            ({1'b0, cfg_y_offset} +
                             {1'b0, cfg_dst_height})) &&

                        ({1'b0, out_x} >= {1'b0, cfg_x_offset}) &&
                        ({1'b0, out_x} <
                            ({1'b0, cfg_x_offset} +
                             {1'b0, cfg_dst_width} - 1'b1))
                    ) begin

                        x_rem_sum =
                            {1'b0, x_rem_cur} +
                            {1'b0, x_step_r};

                        if (x_rem_sum >= {1'b0, cfg_dst_width}) begin
                            x_carry   = 1'b1;
                            x_rem_sum = x_rem_sum -
                                        {1'b0, cfg_dst_width};
                        end
                        else begin
                            x_carry = 1'b0;
                        end

                        x_source_step =
                            {1'b0, x_step_q} +
                            x_carry;

                        src_x_cur <=
                            src_x_cur +
                            x_source_step[15:0];

                        x_rem_cur <= x_rem_sum[15:0];

                    end

                end

            end

        end

    end

endmodule



// ============================================================================
// 16-bit unsigned sequential quotient/remainder divider.
// One result in 16 clocks after start.
// Verilog-2001 compatible.
// ============================================================================

module u16_divmod_seq (
    input             clk,
    input             rst,

    input             start,
    input      [15:0] numerator,
    input      [15:0] denominator,

    output reg        busy,
    output reg        done,

    output reg [15:0] quotient,
    output reg [15:0] remainder
);

    reg [15:0] dividend_shift;
    reg [15:0] divisor_reg;

    reg [15:0] quotient_work;
    reg [16:0] remainder_work;

    reg [4:0] count;

    reg [16:0] rem_next;
    reg [15:0] quot_next;


    always @(posedge clk or posedge rst) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;

            quotient  <= 16'd0;
            remainder <= 16'd0;

            dividend_shift <= 16'd0;
            divisor_reg    <= 16'd1;

            quotient_work  <= 16'd0;
            remainder_work <= 17'd0;

            count <= 5'd0;

        end
        else begin

            done <= 1'b0;

            if (start && !busy) begin

                if (denominator == 16'd0) begin

                    busy      <= 1'b0;
                    done      <= 1'b1;
                    quotient  <= 16'd0;
                    remainder <= 16'd0;

                end
                else begin

                    busy <= 1'b1;

                    dividend_shift <= numerator;
                    divisor_reg    <= denominator;

                    quotient_work  <= 16'd0;
                    remainder_work <= 17'd0;

                    count <= 5'd0;

                end

            end
            else if (busy) begin

                rem_next =
                    {remainder_work[15:0], dividend_shift[15]};

                dividend_shift <=
                    {dividend_shift[14:0], 1'b0};

                if (rem_next >= {1'b0, divisor_reg}) begin

                    rem_next =
                        rem_next - {1'b0, divisor_reg};

                    quot_next =
                        {quotient_work[14:0], 1'b1};

                end
                else begin

                    quot_next =
                        {quotient_work[14:0], 1'b0};

                end

                quotient_work  <= quot_next;
                remainder_work <= rem_next;

                if (count == 5'd15) begin

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
