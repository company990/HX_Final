module uart_rx_8n1 #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD = 115200
)(
    input clk,
    input rst,
    input rx,
    output reg valid,
    output reg [7:0] data
);
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD;
    reg r0,r1;
    reg busy;
    reg [15:0] cnt;
    reg [3:0] bit_idx;
    reg [7:0] shreg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            r0<=1'b1; r1<=1'b1;
            busy<=1'b0; cnt<=0; bit_idx<=0; shreg<=0; valid<=0; data<=0;
        end else begin
            r0 <= rx; r1 <= r0;
            valid <= 1'b0;
            if (!busy) begin
                if (r1 == 1'b0) begin
                    busy <= 1'b1;
                    cnt <= CLKS_PER_BIT/2;
                    bit_idx <= 4'd0;
                end
            end else if (cnt != 0) begin
                cnt <= cnt - 1'b1;
            end else begin
                cnt <= CLKS_PER_BIT-1;
                if (bit_idx == 0) begin
                    bit_idx <= 4'd1;
                end else if (bit_idx <= 8) begin
                    shreg[bit_idx-1] <= r1;
                    bit_idx <= bit_idx + 1'b1;
                end else begin
                    busy <= 1'b0;
                    data <= shreg;
                    valid <= 1'b1;
                end
            end
        end
    end
endmodule
