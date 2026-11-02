`timescale 1ns/1ps

// ============================================================================
// G1-C3-D4-C2-A
// SDRAM return-data mask aligner.
//
// Problem solved:
//   adaptive_inside_image is valid when an SDRAM read command is ISSUED
//   (App_rd_en), while Sdr_rd_dout is valid later when Sdr_rd_en arrives.
//
// This module queues one inside/bar bit for every issued read command and
// removes one bit for every returned SDRAM word.  Because SDRAM returns are
// in-order, the returned data word is masked with the matching command bit.
//
// cmd_inside = 1 -> pass SDRAM data
// cmd_inside = 0 -> force returned word to 32'd0 (true black)
//
// Verilog-2001 compatible.
// ============================================================================

module adaptive_sdram_return_mask #(
    parameter DATA_BITS  = 32,
    parameter FIFO_DEPTH = 512,
    parameter PTR_BITS   = 9
)(
    input                       clk,
    input                       rst,
    input                       clear,

    // Issued SDRAM read command.
    input                       cmd_valid,
    input                       cmd_inside,

    // Returned SDRAM data.
    input                       data_valid,
    input  [DATA_BITS-1:0]      data_in,

    output [DATA_BITS-1:0]      data_out,

    output reg [PTR_BITS:0]     queued_count,
    output reg                  overflow_sticky,
    output reg                  underflow_sticky
);

    reg mask_mem [0:FIFO_DEPTH-1];

    reg [PTR_BITS-1:0] wr_ptr;
    reg [PTR_BITS-1:0] rd_ptr;

    wire queue_empty = (queued_count == 0);
    wire queue_full  = (queued_count == FIFO_DEPTH);

    // Normal case: returned data consumes the oldest queued command mask.
    //
    // Defensive zero-latency bypass:
    // if a controller ever returns data in the same cycle as the very first
    // command (queue empty + simultaneous push/pop), use cmd_inside directly.
    wire return_inside =
        !queue_empty ? mask_mem[rd_ptr] :
        (cmd_valid ? cmd_inside : 1'b0);

    assign data_out =
        return_inside ? data_in : {DATA_BITS{1'b0}};


    always @(posedge clk or posedge rst) begin

        if (rst) begin

            wr_ptr <= {PTR_BITS{1'b0}};
            rd_ptr <= {PTR_BITS{1'b0}};

            queued_count <= {(PTR_BITS+1){1'b0}};

            overflow_sticky  <= 1'b0;
            underflow_sticky <= 1'b0;

        end
        else if (clear) begin

            wr_ptr <= {PTR_BITS{1'b0}};
            rd_ptr <= {PTR_BITS{1'b0}};

            queued_count <= {(PTR_BITS+1){1'b0}};

            overflow_sticky  <= 1'b0;
            underflow_sticky <= 1'b0;

        end
        else begin

            case ({cmd_valid, data_valid})

                2'b10:
                begin

                    if (!queue_full) begin

                        mask_mem[wr_ptr] <= cmd_inside;
                        wr_ptr           <= wr_ptr + 1'b1;
                        queued_count     <= queued_count + 1'b1;

                    end
                    else begin

                        overflow_sticky <= 1'b1;

                    end

                end


                2'b01:
                begin

                    if (!queue_empty) begin

                        rd_ptr       <= rd_ptr + 1'b1;
                        queued_count <= queued_count - 1'b1;

                    end
                    else begin

                        underflow_sticky <= 1'b1;

                    end

                end


                2'b11:
                begin

                    if (queue_empty) begin

                        // Zero-latency bypass: nothing needs to be stored.
                        wr_ptr       <= wr_ptr;
                        rd_ptr       <= rd_ptr;
                        queued_count <= queued_count;

                    end
                    else begin

                        // Pop oldest bit and push newest bit in the same cycle.
                        // This is also safe when the queue is full.
                        mask_mem[wr_ptr] <= cmd_inside;

                        wr_ptr <= wr_ptr + 1'b1;
                        rd_ptr <= rd_ptr + 1'b1;

                        queued_count <= queued_count;

                    end

                end


                default:
                begin

                    wr_ptr       <= wr_ptr;
                    rd_ptr       <= rd_ptr;
                    queued_count <= queued_count;

                end

            endcase

        end

    end

endmodule
