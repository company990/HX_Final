`timescale 1ns/1ps

// ============================================================================
// D5-B2
// Resource-conscious RGB bilinear interpolation engine.
//
// Inputs:
//   P00 P10
//   P01 P11
//
// D5-B1 supplies Q0.16 frac_x / frac_y. This engine deliberately uses the
// upper 8 fractional bits (Q0.8 coefficient precision).
//
// Exact integer form used per R/G/B channel:
//
//   fx = frac_x[15:8]     // 0..255
//   fy = frac_y[15:8]     // 0..255
//
//   top_q8 = (P00 << 8) + (P10 - P00) * fx
//   bot_q8 = (P01 << 8) + (P11 - P01) * fx
//
//   out_q16 = (top_q8 << 8) + (bot_q8 - top_q8) * fy
//   out      = round(out_q16 / 65536)
//
// Stage4-A timing optimization:
//
//   Old critical style:
//
//       registered operand
//              |
//              v
//          multiplier
//              |
//              v
//            adder
//              |
//              v
//         destination reg
//
//   New structure:
//
//       registered operand
//              |
//              v
//          multiplier
//              |
//              v
//       product register
//              |
//              v
//            adder
//              |
//              v
//         destination reg
//
// The same three multipliers are still shared across R/G/B and reused for:
//   1. horizontal top
//   2. horizontal bottom
//   3. vertical blend
//
// External start/busy/done protocol is unchanged.
// Only internal latency is increased.
//
// Pixel format:
//   [23:16] = R
//   [15:8]  = G
//   [7:0]   = B
//
// Verilog-2001 compatible.
// ============================================================================

module bilinear_rgb_interp (

    input              clk,
    input              rst,

    input              start,
    output reg         busy,
    output reg         done,

    input      [23:0]  p00,
    input      [23:0]  p10,
    input      [23:0]  p01,
    input      [23:0]  p11,

    input      [15:0]  frac_x,
    input      [15:0]  frac_y,

    output reg [23:0]  pixel_out

);


    // ========================================================================
    // FSM
    // ========================================================================

    localparam S_IDLE       = 4'd0;
    localparam S_TOP_MUL    = 4'd1;
    localparam S_TOP_ADD    = 4'd2;
    localparam S_BOTTOM_MUL = 4'd3;
    localparam S_BOTTOM_ADD = 4'd4;
    localparam S_VERT_PREP  = 4'd5;
    localparam S_VERT_MUL   = 4'd6;
    localparam S_VERT_ADD   = 4'd7;
    localparam S_OUT        = 4'd8;

    reg [3:0] state;


    // ========================================================================
    // Latched pixels / coefficients
    // ========================================================================

    reg [7:0] p00_r;
    reg [7:0] p00_g;
    reg [7:0] p00_b;

    reg [7:0] p10_r;
    reg [7:0] p10_g;
    reg [7:0] p10_b;

    reg [7:0] p01_r;
    reg [7:0] p01_g;
    reg [7:0] p01_b;

    reg [7:0] p11_r;
    reg [7:0] p11_g;
    reg [7:0] p11_b;

    reg [7:0] fx8;
    reg [7:0] fy8;


    // ========================================================================
    // Three shared signed multipliers
    //
    // mul_a:
    //
    // horizontal:
    //      -255 ... +255
    //
    // vertical:
    //      -65280 ... +65280
    //
    // Therefore signed 17-bit is sufficient.
    //
    // mul_b:
    //      0 ... 255
    //
    // Converted to positive signed 9-bit before multiplication.
    //
    // Result:
    //      17 bit x 9 bit = 26 bit signed
    // ========================================================================

    reg signed [16:0] mul_a_r;
    reg signed [16:0] mul_a_g;
    reg signed [16:0] mul_a_b;

    reg [7:0] mul_b;


    wire signed [8:0] mul_b_signed;

    assign mul_b_signed =
        $signed({1'b0, mul_b});


    wire signed [25:0] mul_prod_r;
    wire signed [25:0] mul_prod_g;
    wire signed [25:0] mul_prod_b;


    assign mul_prod_r =
        $signed(mul_a_r) * mul_b_signed;

    assign mul_prod_g =
        $signed(mul_a_g) * mul_b_signed;

    assign mul_prod_b =
        $signed(mul_a_b) * mul_b_signed;


    // ========================================================================
    // Stage4-A timing cut
    //
    // Register the multiplier outputs before any horizontal/vertical add.
    // ========================================================================

    reg signed [25:0] mul_prod_r_r;
    reg signed [25:0] mul_prod_g_r;
    reg signed [25:0] mul_prod_b_r;


    // ========================================================================
    // Horizontal interpolation results
    //
    // Q8 format:
    //
    //      pixel_value * 256
    //
    // Valid range:
    //
    //      0 ... 65280
    // ========================================================================

    reg signed [17:0] top_r_q8;
    reg signed [17:0] top_g_q8;
    reg signed [17:0] top_b_q8;

    reg signed [17:0] bot_r_q8;
    reg signed [17:0] bot_g_q8;
    reg signed [17:0] bot_b_q8;


    // TOP calculation.
    //
    // Both inputs here are registered:
    //
    //      p00_*
    //      mul_prod_*_r

    wire signed [25:0] top_calc_r;
    wire signed [25:0] top_calc_g;
    wire signed [25:0] top_calc_b;


    assign top_calc_r =
        $signed({10'd0, p00_r, 8'd0}) +
        $signed(mul_prod_r_r);

    assign top_calc_g =
        $signed({10'd0, p00_g, 8'd0}) +
        $signed(mul_prod_g_r);

    assign top_calc_b =
        $signed({10'd0, p00_b, 8'd0}) +
        $signed(mul_prod_b_r);


    // BOTTOM calculation.
    //
    // Again, multiplier output is already registered.

    wire signed [25:0] bot_calc_r;
    wire signed [25:0] bot_calc_g;
    wire signed [25:0] bot_calc_b;


    assign bot_calc_r =
        $signed({10'd0, p01_r, 8'd0}) +
        $signed(mul_prod_r_r);

    assign bot_calc_g =
        $signed({10'd0, p01_g, 8'd0}) +
        $signed(mul_prod_g_r);

    assign bot_calc_b =
        $signed({10'd0, p01_b, 8'd0}) +
        $signed(mul_prod_b_r);


    // ========================================================================
    // Vertical final Q16 value
    //
    // final =
    //
    //      (top_q8 << 8)
    //
    //            +
    //
    //      vertical_product
    //
    // IMPORTANT:
    //
    // Vertical product may be negative.
    // Explicit sign extension must be preserved.
    // ========================================================================

    wire signed [26:0] final_q16_r;
    wire signed [26:0] final_q16_g;
    wire signed [26:0] final_q16_b;


    assign final_q16_r =
        $signed({1'b0, top_r_q8, 8'd0}) +
        $signed({mul_prod_r_r[25], mul_prod_r_r});

    assign final_q16_g =
        $signed({1'b0, top_g_q8, 8'd0}) +
        $signed({mul_prod_g_r[25], mul_prod_g_r});

    assign final_q16_b =
        $signed({1'b0, top_b_q8, 8'd0}) +
        $signed({mul_prod_b_r[25], mul_prod_b_r});


    // ========================================================================
    // Registered final Q16 result
    //
    // Preserve the previous timing fix:
    //
    //      final adder
    //          |
    //          v
    //      final_q16_*_r
    //          |
    //          v
    //      round/saturate
    // ========================================================================

    reg [23:0] final_q16_r_r;
    reg [23:0] final_q16_g_r;
    reg [23:0] final_q16_b_r;


    // ========================================================================
    // Round to nearest integer
    //
    //      +32768
    //      >> 16
    // ========================================================================

    // Every stage is a convex interpolation of unsigned 8-bit channels.
    // Thus 0 <= final_q16 <= 255*65536. Rounding cannot reach 256,
    // and 24 bits hold the full value without losing fractional precision.
    wire [23:0] rounded_r=final_q16_r_r+24'd32768;
    wire [23:0] rounded_g=final_q16_g_r+24'd32768;
    wire [23:0] rounded_b=final_q16_b_r+24'd32768;

    // ========================================================================
    // Main FSM
    // ========================================================================

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            state <= S_IDLE;

            busy <= 1'b0;
            done <= 1'b0;

            pixel_out <= 24'd0;


            // ---------------------------------------------------------------
            // Input registers
            // ---------------------------------------------------------------

            p00_r <= 8'd0;
            p00_g <= 8'd0;
            p00_b <= 8'd0;

            p10_r <= 8'd0;
            p10_g <= 8'd0;
            p10_b <= 8'd0;

            p01_r <= 8'd0;
            p01_g <= 8'd0;
            p01_b <= 8'd0;

            p11_r <= 8'd0;
            p11_g <= 8'd0;
            p11_b <= 8'd0;

            fx8 <= 8'd0;
            fy8 <= 8'd0;


            // ---------------------------------------------------------------
            // Multiplier input registers
            // ---------------------------------------------------------------

            mul_a_r <= 17'sd0;
            mul_a_g <= 17'sd0;
            mul_a_b <= 17'sd0;

            mul_b <= 8'd0;


            // ---------------------------------------------------------------
            // Stage4-A multiplier result registers
            // ---------------------------------------------------------------

            mul_prod_r_r <= 26'sd0;
            mul_prod_g_r <= 26'sd0;
            mul_prod_b_r <= 26'sd0;


            // ---------------------------------------------------------------
            // Horizontal interpolation registers
            // ---------------------------------------------------------------

            top_r_q8 <= 18'sd0;
            top_g_q8 <= 18'sd0;
            top_b_q8 <= 18'sd0;

            bot_r_q8 <= 18'sd0;
            bot_g_q8 <= 18'sd0;
            bot_b_q8 <= 18'sd0;


            // ---------------------------------------------------------------
            // Final vertical result registers
            // ---------------------------------------------------------------

            final_q16_r_r <= 27'sd0;
            final_q16_g_r <= 27'sd0;
            final_q16_b_r <= 27'sd0;

        end

        else begin

            // done is always a single-clock pulse.
            done <= 1'b0;


            case (state)


                // ============================================================
                // IDLE
                //
                // Latch the new pixel job.
                //
                // At the same edge prepare TOP horizontal multiplication:
                //
                //      (P10 - P00) * fx
                // ============================================================

                S_IDLE:
                begin

                    busy <= 1'b0;


                    if (start) begin


                        // ---------------------------------------------------
                        // Latch pixels
                        // ---------------------------------------------------

                        p00_r <= p00[23:16];
                        p00_g <= p00[15:8];
                        p00_b <= p00[7:0];

                        p10_r <= p10[23:16];
                        p10_g <= p10[15:8];
                        p10_b <= p10[7:0];

                        p01_r <= p01[23:16];
                        p01_g <= p01[15:8];
                        p01_b <= p01[7:0];

                        p11_r <= p11[23:16];
                        p11_g <= p11[15:8];
                        p11_b <= p11[7:0];


                        // ---------------------------------------------------
                        // Latch interpolation coefficients
                        // ---------------------------------------------------

                        fx8 <= frac_x[15:8];
                        fy8 <= frac_y[15:8];


                        // ---------------------------------------------------
                        // Prepare TOP multiplier operands
                        //
                        // R
                        // ---------------------------------------------------

                        mul_a_r <=
                            $signed({1'b0, p10[23:16]}) -
                            $signed({1'b0, p00[23:16]});


                        // G

                        mul_a_g <=
                            $signed({1'b0, p10[15:8]}) -
                            $signed({1'b0, p00[15:8]});


                        // B

                        mul_a_b <=
                            $signed({1'b0, p10[7:0]}) -
                            $signed({1'b0, p00[7:0]});


                        mul_b <= frac_x[15:8];


                        busy <= 1'b1;

                        state <= S_TOP_MUL;

                    end

                end


                // ============================================================
                // TOP multiplier stage
                //
                // The multiplier output is captured into an explicit register.
                //
                // There is NO horizontal Q8 addition in this clock.
                // ============================================================

                S_TOP_MUL:
                begin

                    mul_prod_r_r <= mul_prod_r;
                    mul_prod_g_r <= mul_prod_g;
                    mul_prod_b_r <= mul_prod_b;

                    state <= S_TOP_ADD;

                end


                // ============================================================
                // TOP ADD stage
                //
                // product register
                //      +
                // P00 << 8
                //
                // At the same edge, prepare BOTTOM multiplier operands.
                // ============================================================

                S_TOP_ADD:
                begin

                    top_r_q8 <= top_calc_r[17:0];
                    top_g_q8 <= top_calc_g[17:0];
                    top_b_q8 <= top_calc_b[17:0];


                    // -------------------------------------------------------
                    // Prepare BOTTOM:
                    //
                    //      (P11 - P01) * fx
                    // -------------------------------------------------------

                mul_a_r <=
                    $signed({1'b0, p11_r}) -
                    $signed({1'b0, p01_r});
                
                mul_a_g <=
                    $signed({1'b0, p11_g}) -
                    $signed({1'b0, p01_g});
                
                mul_a_b <=
                    $signed({1'b0, p11_b}) -
                    $signed({1'b0, p01_b});
                
                mul_b <= fx8;
                
                state <= S_BOTTOM_MUL;
    
                end


                // ============================================================
                // BOTTOM multiplier stage
                //
                // Explicit timing cut after multiplier.
                // ============================================================

                S_BOTTOM_MUL:
                begin

                    mul_prod_r_r <= mul_prod_r;
                    mul_prod_g_r <= mul_prod_g;
                    mul_prod_b_r <= mul_prod_b;

                    state <= S_BOTTOM_ADD;

                end


                // ============================================================
                // BOTTOM ADD stage
                //
                //      bot_q8 = (P01 << 8) + registered_product
                // ============================================================

                S_BOTTOM_ADD:
                begin

                    bot_r_q8 <= bot_calc_r[17:0];
                    bot_g_q8 <= bot_calc_g[17:0];
                    bot_b_q8 <= bot_calc_b[17:0];

                    state <= S_VERT_PREP;

                end


                // ============================================================
                // Vertical operand preparation
                //
                // Both TOP and BOTTOM are now already registered.
                //
                //      vertical_delta = bot_q8 - top_q8
                // ============================================================

                S_VERT_PREP:
                begin

                    mul_a_r <=
                        $signed(bot_r_q8[16:0]) -
                        $signed(top_r_q8[16:0]);


                    mul_a_g <=
                        $signed(bot_g_q8[16:0]) -
                        $signed(top_g_q8[16:0]);


                    mul_a_b <=
                        $signed(bot_b_q8[16:0]) -
                        $signed(top_b_q8[16:0]);


                    mul_b <= fy8;


                    state <= S_VERT_MUL;

                end


                // ============================================================
                // Vertical multiplier stage
                //
                //      (bot_q8 - top_q8) * fy
                //
                // Again capture the multiplier output first.
                // ============================================================

                S_VERT_MUL:
                begin

                    mul_prod_r_r <= mul_prod_r;
                    mul_prod_g_r <= mul_prod_g;
                    mul_prod_b_r <= mul_prod_b;

                    state <= S_VERT_ADD;

                end


                // ============================================================
                // Final Q16 ADD
                //
                //      (top_q8 << 8)
                //             +
                //      registered vertical product
                //
                // Then register complete Q16 result.
                // ============================================================

                S_VERT_ADD:
                begin

                    final_q16_r_r <= final_q16_r;
                    final_q16_g_r <= final_q16_g;
                    final_q16_b_r <= final_q16_b;

                    state <= S_OUT;

                end


                // ============================================================
                // Final output
                //
                // Registered Q16
                //      ->
                // round
                //      ->
                // saturation
                //      ->
                // RGB output
                // ============================================================

                S_OUT:
                begin

                    pixel_out[23:16] <= rounded_r[23:16];
                    pixel_out[15:8]  <= rounded_g[23:16];
                    pixel_out[7:0]   <= rounded_b[23:16];


                    busy <= 1'b0;
                    done <= 1'b1;


                    state <= S_IDLE;

                end


                // ============================================================
                // Safety default
                // ============================================================

                default:
                begin

                    state <= S_IDLE;

                    busy <= 1'b0;

                end


            endcase

        end

    end


endmodule