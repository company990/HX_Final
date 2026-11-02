`timescale 1ns/1ps
// Sequential-address I2C transactions, with byte-stream writes and reads.
// Both pads are open drain: only zero or high impedance is ever driven.
// Clock stretching is honored; no generated clock is used inside the FPGA.
module fm_i2c_master #(
    parameter integer CLK_HZ=25000000,
    parameter integer I2C_HZ=100000,
    parameter integer TIMEOUT_US=20000
)(
    input wire clk,rst,start,read_transfer,
    input wire [6:0] address,length,
    input wire [7:0] write_data,
    output reg [6:0] byte_index,
    output reg [7:0] read_data,
    output reg [6:0] read_index,
    output reg read_valid,busy,done,
    output reg [2:0] error,
    inout wire scl,sda
);
    localparam integer DIV=(CLK_HZ+4*I2C_HZ-1)/(4*I2C_HZ);
    localparam integer DW=DIV<2 ? 1 : $clog2(DIV);
    localparam integer LIMIT=(CLK_HZ/1000000)*TIMEOUT_US/DIV;
    localparam integer TW=LIMIT<2 ? 1 : $clog2(LIMIT+1);
    localparam integer HW=$clog2(2*DIV+1);
    // Two synchronized pad stages plus one counter cycle precede this count.
    // Normal clocks stay at 2*DIV high; stretched clocks retain that high time.
    localparam integer HIGH_MIN=2*DIV>3 ? 2*DIV-3 : 1;
    localparam IDLE=0,START_CHECK=1,START_PULL=2,START_LOW=3,
               LOAD=4,BIT_SETUP=5,BIT_RISE=6,BIT_SAMPLE=7,BIT_FALL=8,
               ACK_SETUP=9,ACK_RISE=10,ACK_SAMPLE=11,ACK_FALL=12,
               STOP_LOW=13,STOP_RISE=14,STOP_RELEASE=15,STOP_DONE=16,
               BIT_WAIT_LOW=17,ACK_WAIT_LOW=18;
    reg [4:0] state;
    reg [DW-1:0] divider;
    reg [TW-1:0] wait_count;
    reg [HW-1:0] high_count;
    reg scl_low,sda_low;
    reg [1:0] scl_sync,sda_sync;
    reg [7:0] shift;
    reg [2:0] bit_index;
    reg [6:0] count;
    reg reading,address_byte,receiving,ack_failed;
    wire tick=divider==DIV-1;
    assign scl=scl_low ? 1'b0 : 1'bz;
    assign sda=sda_low ? 1'b0 : 1'bz;
    // Synchronous pad sampling; these signals never clock internal registers.
    always @(posedge clk or posedge rst)
        if(rst) begin scl_sync<=3;sda_sync<=3;end
        else begin scl_sync<={scl_sync[0],scl};sda_sync<={sda_sync[0],sda};end
    wire waiting_high=(state==START_CHECK || state==BIT_SAMPLE ||
                       state==ACK_SAMPLE || state==STOP_RELEASE) && !scl_sync[1];
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state<=IDLE;divider<=0;wait_count<=0;high_count<=0;
            scl_low<=0;sda_low<=0;byte_index<=0;read_data<=0;read_index<=0;
            read_valid<=0;busy<=0;done<=0;error<=0;shift<=0;
            bit_index<=7;count<=0;reading<=0;address_byte<=0;
            receiving<=0;ack_failed<=0;
        end else begin
            read_valid<=0;done<=0;
            if(scl_low || !scl_sync[1]) high_count<=0;
            else if(high_count<2*DIV) high_count<=high_count+1'b1;
            if(!busy || tick) divider<=0;else divider<=divider+1'b1;
            if(state==IDLE) begin
                scl_low<=0;sda_low<=0;wait_count<=0;
                if(start) begin
                    busy<=1;error<=0;byte_index<=0;reading<=read_transfer;
                    count<=length;address_byte<=1;receiving<=0;
                    shift<={address,read_transfer};bit_index<=7;
                    if(length==0) begin error<=4;done<=1;busy<=0;end
                    else state<=START_CHECK;
                end
            end else if(state==BIT_SETUP) begin
                // Change SDA one system clock after SCL falls, meeting the
                // datasheet's 0..900 ns data hold time as well as setup time.
                sda_low<=receiving ? 1'b0 : !shift[7];state<=BIT_WAIT_LOW;
            end else if(state==ACK_SETUP) begin
                sda_low<=receiving && byte_index!=count-1'b1;state<=ACK_WAIT_LOW;
            end else if(tick) begin
                if(waiting_high && wait_count==LIMIT-1) begin
                    scl_low<=0;sda_low<=0;busy<=0;done<=1;error<=3;state<=IDLE;
                end else begin
                    if(waiting_high) wait_count<=wait_count+1'b1;
                    else wait_count<=0;
                    case(state)
                        START_CHECK: if(scl_sync[1]) begin
                            if(!sda_sync[1]) begin error<=4;busy<=0;done<=1;state<=IDLE;end
                            else state<=START_PULL;
                        end
                        START_PULL: begin sda_low<=1;state<=START_LOW;end
                        START_LOW: begin scl_low<=1;state<=BIT_SETUP;end
                        LOAD: begin shift<=write_data;bit_index<=7;state<=BIT_SETUP;end
                        BIT_WAIT_LOW: state<=BIT_RISE;
                        BIT_RISE: begin scl_low<=0;state<=BIT_SAMPLE;end
                        BIT_SAMPLE: if(scl_sync[1]) begin
                            if(receiving) shift<={shift[6:0],sda_sync[1]};
                            state<=BIT_FALL;
                        end
                        BIT_FALL: if(high_count>=HIGH_MIN) begin
                            scl_low<=1;
                            if(!receiving) shift<={shift[6:0],1'b0};
                            if(bit_index==0) state<=ACK_SETUP;
                            else begin bit_index<=bit_index-1'b1;state<=BIT_SETUP;end
                        end
                        ACK_WAIT_LOW: state<=ACK_RISE;
                        ACK_RISE: begin scl_low<=0;state<=ACK_SAMPLE;end
                        ACK_SAMPLE: if(scl_sync[1]) begin
                            ack_failed<=!receiving && sda_sync[1];state<=ACK_FALL;
                        end
                        ACK_FALL: if(high_count>=HIGH_MIN) begin
                            scl_low<=1;
                            if(ack_failed) begin error<=address_byte ? 3'd1 : 3'd2;state<=STOP_LOW;end
                            else if(address_byte) begin
                                address_byte<=0;bit_index<=7;
                                if(reading) begin receiving<=1;shift<=0;state<=BIT_SETUP;end
                                else state<=LOAD;
                            end else begin
                                if(receiving) begin read_data<=shift;read_index<=byte_index;read_valid<=1;end
                                if(byte_index==count-1'b1) state<=STOP_LOW;
                                else begin
                                    byte_index<=byte_index+1'b1;bit_index<=7;
                                    if(receiving) begin shift<=0;state<=BIT_SETUP;end
                                    else state<=LOAD;
                                end
                            end
                        end
                        STOP_LOW: begin scl_low<=1;sda_low<=1;state<=STOP_RISE;end
                        STOP_RISE: begin scl_low<=0;state<=STOP_RELEASE;end
                        STOP_RELEASE: if(scl_sync[1]) begin sda_low<=0;state<=STOP_DONE;end
                        STOP_DONE: begin busy<=0;done<=1;state<=IDLE;end
                        default: begin scl_low<=0;sda_low<=0;busy<=0;error<=4;done<=1;state<=IDLE;end
                    endcase
                end
            end
        end
    end
endmodule
