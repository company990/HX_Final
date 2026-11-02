`timescale 1ns/1ps

module cdc_toggle_to_pulse (

    input  wire dst_clk,
    input  wire dst_rst,

    // Toggle signal generated in another clock domain.
    // Every source event flips this bit once.
    input  wire src_toggle_async,

    // One-dst-clock pulse for every detected toggle.
    output wire dst_pulse
);


    // =========================================================
    // Three-stage synchronizer
    //
    // sync_ff[0] : first metastability-catching stage
    // sync_ff[1] : second synchronized stage
    // sync_ff[2] : delayed copy for edge/toggle detection
    // =========================================================

    reg [2:0] sync_ff;


    always @(posedge dst_clk or posedge dst_rst) begin

        if (dst_rst) begin

            sync_ff <= 3'b000;

        end
        else begin

            sync_ff <= {
                sync_ff[1:0],
                src_toggle_async
            };

        end

    end


    // =========================================================
    // Detect a change of the synchronized toggle.
    //
    // Source:
    //
    //     0 -> 1
    //
    // or:
    //
    //     1 -> 0
    //
    // Both generate exactly one pulse.
    // =========================================================

    assign dst_pulse =
        sync_ff[2] ^
        sync_ff[1];


endmodule