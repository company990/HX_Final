// ONLY for confirmed XPT2046/compatible resistive panels. NOT GT911/GT9147.
// SPI mode 0, MSB first, 24 clocks (8 command + 16 receive), CS high between
// axes. 0xD0 / 0x90: 12-bit differential X/Y, power-down enables PENIRQ.
// Raw output requires per-panel calibration and axis/orientation conversion.
module touch_spi_xpt2046 #(
    parameter integer CLK_HZ=50000000, SPI_HZ=500000, POLL_HZ=100
)(
    input wire clk, rst_n, pen_n, miso,
    output reg sclk, mosi, cs_n,
    output reg valid, down,
    output reg [11:0] raw_x, raw_y
);
    localparam integer HALF=CLK_HZ/(2*SPI_HZ);
    localparam IDLE=0, START=1, SHIFT=2, NEXT=3, REPORT=4;
    reg [2:0] state;
    reg [1:0] ps, ms;
    reg [31:0] poll;
    reg [15:0] divider;
    reg [4:0] bits;
    reg [23:0] tx, rx;
    reg axis;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE; ps<=3; ms<=0; poll<=0; divider<=0; bits<=0;
            tx<=0; rx<=0; axis<=0; sclk<=0; mosi<=0; cs_n<=1;
            valid<=0; down<=0; raw_x<=0; raw_y<=0;
        end else begin
            ps<={ps[0],pen_n}; ms<={ms[0],miso}; valid<=0;
            case(state)
                IDLE: if(poll!=0) poll<=poll-1'b1;
                else if(ps[1]) begin down<=0; valid<=1; poll<=CLK_HZ/POLL_HZ-1; end
                else begin axis<=0; state<=START; end
                START: begin
                    tx<=axis ? 24'h900000 : 24'hd00000;
                    rx<=0; bits<=0; mosi<=1; sclk<=0; cs_n<=0;
                    divider<=HALF-1; state<=SHIFT;
                end
                SHIFT: if(divider!=0) divider<=divider-1'b1;
                else begin
                    divider<=HALF-1;
                    if(!sclk) begin sclk<=1; rx<={rx[22:0],ms[1]}; end
                    else begin
                        sclk<=0;
                        if(bits==23) begin cs_n<=1; mosi<=0; state<=NEXT; end
                        else begin bits<=bits+1'b1; tx<={tx[22:0],1'b0}; mosi<=tx[22]; end
                    end
                end
                NEXT: if(divider!=0) divider<=divider-1'b1;
                else begin
                    if(axis) begin raw_y<=rx[14:3]; state<=REPORT; end
                    else begin raw_x<=rx[14:3]; axis<=1; state<=START; end
                end
                REPORT: begin
                    down<=!ps[1]; valid<=1; poll<=CLK_HZ/POLL_HZ-1; state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
