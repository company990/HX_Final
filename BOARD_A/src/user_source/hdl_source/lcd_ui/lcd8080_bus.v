// Single-clock, 16-bit 8080 bus. req_* is accepted on req_valid && req_ready.
// All cycle parameters must be >= 1. rd_data is valid with done after a read.
module lcd8080_bus #(
    parameter integer SETUP_CYCLES=3, WR_LOW_CYCLES=5,
    parameter integer RD_LOW_CYCLES=25, HOLD_CYCLES=3
)(
    input wire clk, rst_n,
    input wire req_valid, req_read, req_rs,
    input wire [15:0] req_data,
    output wire req_ready,
    output reg done,
    output reg [15:0] rd_data,
    output reg lcd_cs_n, lcd_rs, lcd_wr_n, lcd_rd_n,
    inout wire [15:0] lcd_d
);
    localparam IDLE=0, SETUP=1, LOW=2, HOLD=3;
    localparam integer MAX_LOW=WR_LOW_CYCLES>RD_LOW_CYCLES ?
                               WR_LOW_CYCLES:RD_LOW_CYCLES;
    localparam integer MAX_CYCLES=SETUP_CYCLES>MAX_LOW ?
                                  (SETUP_CYCLES>HOLD_CYCLES ?
                                   SETUP_CYCLES:HOLD_CYCLES) :
                                  (MAX_LOW>HOLD_CYCLES ?
                                   MAX_LOW:HOLD_CYCLES);
    localparam integer COUNT_BITS=MAX_CYCLES<2 ? 1 : $clog2(MAX_CYCLES);
    reg [1:0] state;
    reg [COUNT_BITS-1:0] count;
    reg [15:0] data_latch;
    reg read_latch, drive;
    assign req_ready=(state==IDLE);
    assign lcd_d=drive ? data_latch : 16'hzzzz;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; count<=0; data_latch<=0; read_latch<=0;
            drive<=0; done<=0; rd_data<=0;
            lcd_cs_n<=1; lcd_rs<=0; lcd_wr_n<=1; lcd_rd_n<=1;
        end else begin
            done<=0;
            case(state)
                IDLE: if(req_valid) begin
                    data_latch<=req_data; read_latch<=req_read;
                    drive<=!req_read; lcd_rs<=req_rs; lcd_cs_n<=0;
                    count<=SETUP_CYCLES-1; state<=SETUP;
                end
                SETUP: if(count!=0) count<=count-1'b1;
                else begin
                    lcd_rd_n<=!read_latch; lcd_wr_n<=read_latch;
                    count<=read_latch ? RD_LOW_CYCLES-1 : WR_LOW_CYCLES-1;
                    state<=LOW;
                end
                LOW: if(count!=0) count<=count-1'b1;
                else begin
                    if(read_latch) rd_data<=lcd_d;
                    lcd_wr_n<=1; lcd_rd_n<=1;
                    count<=HOLD_CYCLES-1; state<=HOLD;
                end
                HOLD: if(count!=0) count<=count-1'b1;
                else begin
                    lcd_cs_n<=1; drive<=0; done<=1; state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
