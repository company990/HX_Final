module cdc_level_sync (
    input  wire dst_clk,
    input  wire dst_rst,
    input  wire async_level,
    output reg  sync_level
);

reg level_meta;

always @(posedge dst_clk or posedge dst_rst) begin
    if (dst_rst) begin
        level_meta <= 1'b0;
        sync_level <= 1'b0;
    end else begin
        level_meta <= async_level;
        sync_level <= level_meta;
    end
end

endmodule
