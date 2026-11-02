// Configuration-time initialization is intentional: PROGRAM_SRAM may leave
// external reset and the existing PLL lock high throughout reconfiguration.
// The core still receives a complete reset before normal operation begins.
module startup_reset_guard #(
    parameter integer HOLD_BITS=18
)(
    input wire clk, reset_n, pll_locked,
    output reg core_reset=1'b1
);
    reg [1:0] lock_sync=2'b00;
    reg [HOLD_BITS-1:0] stable_count={HOLD_BITS{1'b0}};
    wire async_ok=reset_n && pll_locked;
    always @(posedge clk or negedge async_ok) begin
        if(!async_ok) begin
            lock_sync<=0;
            stable_count<=0;
            core_reset<=1;
        end else begin
            lock_sync<={lock_sync[0],1'b1};
            if(!lock_sync[1]) begin
                stable_count<=0;
                core_reset<=1;
            end else if(!(&stable_count)) begin
                stable_count<=stable_count+1'b1;
                core_reset<=1;
            end else core_reset<=0;
        end
    end
endmodule
