module sticky_fault (
    input  wire clk,
    input  wire rst,
    input  wire fault_event,
    output reg  fault_sticky
);

always @(posedge clk or posedge rst) begin
    if (rst)
        fault_sticky <= 1'b0;
    else if (fault_event)
        fault_sticky <= 1'b1;
end

endmodule
