`timescale 1ns/1ps

// =============================================================
// G1-C3-B
// Sequential zoom framebuffer read-address generator
//
// One SDRAM read command = one output pixel.
//
// frame_start:
//     start a new 640x480 output frame
//     latch base_addr and zoom_level
//
// advance:
//     current read_addr has just been issued
//     move to the address for the NEXT output pixel
//
// zoom_level:
//
//     0 = 1.00X
//     1 = 1.25X
//     2 = 1.50X
//     3 = 2.00X
//
// Important:
//     no division
//     no multiplication
//     only counters, small phase accumulators and add/subtract
// =============================================================

module video_zoom_read_addr_gen (

    input  wire        clk,
    input  wire        rst,

    input  wire        frame_start,
    input  wire        advance,

    input  wire [20:0] base_addr,
    input  wire [1:0]  zoom_level,

    output reg  [20:0] read_addr,

    output reg  [1:0]  zoom_level_active,

    output reg  [9:0]  out_x,
    output reg  [8:0]  out_y

);


// =============================================================
// Latched framebuffer base
// =============================================================

reg [20:0] base_addr_latched;


// =============================================================
// Fractional phase accumulators
//
// 1.25X : source/output = 4/5
// 1.50X : source/output = 2/3
// 2.00X : source/output = 1/2
// =============================================================

reg [2:0] x_phase;
reg [2:0] y_phase;


// =============================================================
// Center-crop first-pixel offset
//
// 1.00X:
//     (0,0)
//     offset = 0
//
// 1.25X:
//     (64,48)
//     48*640 + 64 = 30784
//
// 1.50X:
//     (106,80)
//     80*640 + 106 = 51306
//
// 2.00X:
//     (160,120)
//     120*640 + 160 = 76960
// =============================================================

function [20:0] zoom_origin_offset;

    input [1:0] level;

    begin

        case (level)

            2'd0:
                zoom_origin_offset = 21'd0;

            2'd1:
                zoom_origin_offset = 21'd30784;

            2'd2:
                zoom_origin_offset = 21'd51306;

            2'd3:
                zoom_origin_offset = 21'd76960;

            default:
                zoom_origin_offset = 21'd0;

        endcase

    end

endfunction


// =============================================================
// Address sequencer
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        base_addr_latched <= 21'd0;

        zoom_level_active <= 2'd0;

        read_addr         <= 21'd0;

        out_x             <= 10'd0;
        out_y             <= 9'd0;

        x_phase           <= 3'd0;
        y_phase           <= 3'd0;

    end

    // ---------------------------------------------------------
    // Start a new frame.
    //
    // zoom_level is latched here so the zoom factor cannot
    // change halfway through one displayed frame.
    // ---------------------------------------------------------

    else if (frame_start) begin

        base_addr_latched <= base_addr;

        zoom_level_active <= zoom_level;

        read_addr <=
            base_addr +
            zoom_origin_offset(zoom_level);

        out_x   <= 10'd0;
        out_y   <= 9'd0;

        x_phase <= 3'd0;
        y_phase <= 3'd0;

    end


    // ---------------------------------------------------------
    // One current pixel address has just been issued.
    //
    // Generate address for the NEXT output pixel.
    // ---------------------------------------------------------

    else if (advance) begin


        // =====================================================
        // End of output line
        // =====================================================

        if (out_x == 10'd639) begin

            out_x   <= 10'd0;
            x_phase <= 3'd0;


            // -------------------------------------------------
            // End of complete 640x480 frame
            // -------------------------------------------------

            if (out_y == 9'd479) begin

                out_y   <= 9'd0;
                y_phase <= 3'd0;

                read_addr <=
                    base_addr_latched +
                    zoom_origin_offset(zoom_level_active);

            end


            // -------------------------------------------------
            // Move to first pixel of next output line
            // -------------------------------------------------

            else begin

                out_y <= out_y + 1'b1;

                case (zoom_level_active)


                    // =========================================
                    // 1.00X
                    //
                    // Framebuffer is naturally contiguous:
                    //
                    // previous row x=639
                    // next row     x=0
                    //
                    // therefore simply +1.
                    // =========================================

                    2'd0: begin

                        read_addr <=
                            read_addr + 21'd1;

                        y_phase <= 3'd0;

                    end


                    // =========================================
                    // 1.25X
                    //
                    // Y ratio = 4/5
                    //
                    // Source row is sometimes repeated.
                    //
                    // Last source X = 575
                    // First source X = 64
                    //
                    // same source row:
                    //
                    //     64 - 575 = -511
                    //
                    // next source row:
                    //
                    //     640 + 64 - 575 = +129
                    // =========================================

                     2'd1: begin
        
                    // 1.25X vertical: 4/5
                
                    if (y_phase >= 3'd1) begin
                
                        y_phase <=
                            y_phase - 3'd1;
                
                        // next source row
                        read_addr <=
                            read_addr + 21'd129;
                
                    end
                    else begin
                
                        y_phase <=
                            3'd4;
                
                        // repeat current source row
                        read_addr <=
                            read_addr - 21'd511;
                
                    end
                
                end
        
                    // =========================================
                    // 1.50X
                    //
                    // Y ratio = 2/3
                    //
                    // Last source X = 532
                    // First source X = 106
                    //
                    // same source row:
                    //
                    //     106 - 532 = -426
                    //
                    // next source row:
                    //
                    //     640 + 106 - 532 = +214
                    // =========================================

                    2'd2: begin

                        if ((y_phase + 3'd2) >= 3'd3) begin

                            y_phase <=
                                y_phase + 3'd2 - 3'd3;

                            read_addr <=
                                read_addr + 21'd214;

                        end
                        else begin

                            y_phase <=
                                y_phase + 3'd2;

                            read_addr <=
                                read_addr - 21'd426;

                        end

                    end


                    // =========================================
                    // 2.00X
                    //
                    // Y ratio = 1/2
                    //
                    // Last source X = 479
                    // First source X = 160
                    //
                    // same source row:
                    //
                    //     160 - 479 = -319
                    //
                    // next source row:
                    //
                    //     640 + 160 - 479 = +321
                    // =========================================

                    2'd3: begin

                        if ((y_phase + 3'd1) >= 3'd2) begin

                            y_phase <=
                                y_phase + 3'd1 - 3'd2;

                            read_addr <=
                                read_addr + 21'd321;

                        end
                        else begin

                            y_phase <=
                                y_phase + 3'd1;

                            read_addr <=
                                read_addr - 21'd319;

                        end

                    end


                    default: begin

                        read_addr <=
                            read_addr + 21'd1;

                        y_phase <= 3'd0;

                    end

                endcase

            end

        end


        // =====================================================
        // Normal horizontal pixel
        // =====================================================

        else begin

            out_x <= out_x + 1'b1;

            case (zoom_level_active)


                // =============================================
                // 1.00X
                // =============================================

                2'd0: begin

                    read_addr <=
                        read_addr + 21'd1;

                    x_phase <= 3'd0;

                end


                // =============================================
                // 1.25X
                //
                // source/output = 4/5
                //
                // address increment pattern:
                //
                //     0,1,1,1,1 ...
                // =============================================
                2'd1: begin
                
                    // 1.25X horizontal: source/output = 4/5
                    //
                    // source address increment sequence:
                    //
                    // 0, 1, 1, 1, 1,
                    // 0, 1, 1, 1, 1 ...
                
                    if (x_phase >= 3'd1) begin
                
                        x_phase <=
                            x_phase - 3'd1;
                
                        read_addr <=
                            read_addr + 21'd1;
                
                    end
                    else begin
                
                        x_phase <=
                            3'd4;
                
                        read_addr <=
                            read_addr;
                
                    end
                
                end
                // =============================================
                // 1.50X
                //
                // source/output = 2/3
                //
                // address increment pattern:
                //
                //     0,1,1 ...
                // =============================================

                2'd2: begin

                    if ((x_phase + 3'd2) >= 3'd3) begin

                        x_phase <=
                            x_phase + 3'd2 - 3'd3;

                        read_addr <=
                            read_addr + 21'd1;

                    end
                    else begin

                        x_phase <=
                            x_phase + 3'd2;

                        read_addr <=
                            read_addr;

                    end

                end


                // =============================================
                // 2.00X
                //
                // source/output = 1/2
                //
                // address increment pattern:
                //
                //     0,1,0,1 ...
                // =============================================

                2'd3: begin

                    if ((x_phase + 3'd1) >= 3'd2) begin

                        x_phase <=
                            x_phase + 3'd1 - 3'd2;

                        read_addr <=
                            read_addr + 21'd1;

                    end
                    else begin

                        x_phase <=
                            x_phase + 3'd1;

                        read_addr <=
                            read_addr;

                    end

                end


                default: begin

                    read_addr <=
                        read_addr + 21'd1;

                    x_phase <= 3'd0;

                end

            endcase

        end

    end

end


endmodule