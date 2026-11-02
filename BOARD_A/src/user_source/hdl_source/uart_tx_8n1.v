module uart_tx_8n1 #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD = 115200
)(
    input clk,
    input rst,
    input tx_start,
    input [7:0] tx_data,
    output reg tx_busy,
    output reg tx
);
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD;
    reg [15:0] cnt;
    reg [3:0] bit_idx;
    reg [9:0] shreg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt<=0; bit_idx<=0; shreg<=10'b1111111111; tx_busy<=0; tx<=1'b1;
        end else begin
            if (!tx_busy) begin
                tx <= 1'b1;
                if (tx_start) begin
                    tx_busy <= 1'b1;
                    shreg <= {1'b1, tx_data, 1'b0};
                    cnt <= CLKS_PER_BIT-1;
                    bit_idx <= 4'd0;
                    tx <= 1'b0;
                end
            end else if (cnt != 0) begin
                cnt <= cnt - 1'b1;
            end else begin
                cnt <= CLKS_PER_BIT-1;
                bit_idx <= bit_idx + 1'b1;
                shreg <= {1'b1, shreg[9:1]};
                tx <= shreg[1];
                if (bit_idx == 4'd9) begin
                    tx_busy <= 1'b0;
                    tx <= 1'b1;
                end
            end
        end
    end
endmodule
