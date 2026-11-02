module reset_sync (
    input  wire clk,
    input  wire arst,
    output wire srst
);

reg [1:0] sync_ff;

// Asynchronous assertion, two-clock synchronous release.
always @(posedge clk or posedge arst) begin
    if (arst)
        sync_ff <= 2'b11;
    else
        sync_ff <= {sync_ff[0], 1'b0};
end

assign srst = sync_ff[1];

endmodule
