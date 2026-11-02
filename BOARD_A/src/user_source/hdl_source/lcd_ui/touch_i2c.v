// Single-master I2C primitives. Both lines MUST have external pull-ups to 3.3V.
// op: 0 START/repeated START, 1 STOP, 2 write byte, 3 read byte.
// last=1 sends NACK after a received byte. SDA changes only while SCL is low,
// except START/STOP. Includes clock stretching and a bounded timeout.
module touch_i2c_byte #(
    parameter integer CLK_HZ=50000000, I2C_HZ=100000
)(
    input wire clk,rst_n,start, input wire [1:0] op,
    input wire [7:0] tx, input wire last,
    output reg busy,done,error,nack, output reg [7:0] rx,
    inout wire scl,sda
);
    localparam integer QUARTER=CLK_HZ/(4*I2C_HZ);
    reg scl_low,sda_low;
    reg [1:0] ss,ds,phase,operation;
    reg [3:0] bitno;
    reg [7:0] data;
    reg final_byte;
    localparam integer WATCHDOG_CYCLES=CLK_HZ/100;
    localparam integer WATCHDOG_BITS=WATCHDOG_CYCLES<2 ? 1 :
                                     $clog2(WATCHDOG_CYCLES);
    localparam integer DIVIDER_BITS=QUARTER<2 ? 1 : $clog2(QUARTER);
    reg [WATCHDOG_BITS-1:0] watchdog;
    reg [DIVIDER_BITS-1:0] divider;
    assign scl=scl_low ? 1'b0 : 1'bz;
    assign sda=sda_low ? 1'b0 : 1'bz;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            scl_low<=0;sda_low<=0;ss<=3;ds<=3;phase<=0;operation<=0;
            bitno<=0;data<=0;final_byte<=0;watchdog<=0;divider<=0;
            busy<=0;done<=0;error<=0;nack<=0;rx<=0;
        end else begin
            ss<={ss[0],scl}; ds<={ds[0],sda}; done<=0;
            if(!busy) begin
                if(start) begin
                    busy<=1;error<=0;nack<=0;phase<=0;operation<=op;
                    bitno<=0;data<=tx;final_byte<=last;rx<=0;
                    watchdog<=0;divider<=QUARTER-1;
                    if(op==0) begin scl_low<=1;sda_low<=0; end
                    if(op==1) begin scl_low<=1;sda_low<=1; end
                end
            end else if(watchdog==CLK_HZ/100-1) begin
                // Failed transaction: release bus. Upper layer retries/reset.
                busy<=0;done<=1;error<=1;scl_low<=0;sda_low<=0;
            end else begin
                watchdog<=watchdog+1'b1;
                if(divider!=0) divider<=divider-1'b1;
                else begin
                    divider<=QUARTER-1;
                    case(operation)
                        0: case(phase)
                            0:begin scl_low<=0;phase<=1;end
                            1:if(ss[1]) phase<=2;
                            2:begin
                                if(!ds[1]) begin
                                    error<=1;busy<=0;done<=1;scl_low<=0;sda_low<=0;
                                end else begin sda_low<=1;phase<=3;end
                            end
                            3:begin scl_low<=1;busy<=0;done<=1;end
                        endcase
                        1: case(phase)
                            0:begin scl_low<=0;phase<=1;end
                            1:if(ss[1]) phase<=2;
                            2:begin sda_low<=0;phase<=3;end
                            3:begin busy<=0;done<=1;end
                        endcase
                        default: case(phase)
                            0:begin
                                scl_low<=1;
                                if(operation==2) sda_low<=(bitno==8) ? 0 : !data[7];
                                else sda_low<=(bitno==8) ? !final_byte : 0;
                                phase<=1;
                            end
                            1:begin scl_low<=0;phase<=2;end
                            2:if(ss[1]) begin
                                if(operation==2 && bitno==8) nack<=ds[1];
                                if(operation==3 && bitno<8) rx<={rx[6:0],ds[1]};
                                phase<=3;
                            end
                            3:begin
                                scl_low<=1;
                                if(bitno==8) begin busy<=0;done<=1; end
                                else begin bitno<=bitno+1'b1;data<={data[6:0],1'b0};phase<=0;end
                            end
                        endcase
                    endcase
                end
            end
        end
    end
endmodule

// GT-style register master: 7-bit device address, 16-bit big-endian register
// address, read 1..9 bytes (byte 0 in rd_bytes[7:0]) or write ONE byte.
// Repeated START for read, ACK all but final byte, STOP on NACK/error.
module touch_i2c_reg #(
    parameter integer CLK_HZ=50000000,I2C_HZ=100000
)(
    input wire clk,rst_n,req,read_op,
    input wire [6:0] address,
    input wire [15:0] reg_addr,
    input wire [3:0] length,
    input wire [7:0] wr_byte,
    output reg busy,done,error, output reg [71:0] rd_bytes,
    inout wire scl,sda
);
    reg launch;
    wire byte_busy,byte_done,byte_error,nack;
    wire [7:0] rx;
    reg [1:0] op;
    reg [7:0] tx;
    reg last;
    reg [3:0] stage,idx,len;
    reg [1:0] state;
    reg rd;
    reg [6:0] dev;
    reg [15:0] addr;
    reg [7:0] wd;
    touch_i2c_byte #(.CLK_HZ(CLK_HZ),.I2C_HZ(I2C_HZ)) phy(
        .clk(clk),.rst_n(rst_n),.start(launch),.op(op),.tx(tx),.last(last),
        .busy(byte_busy),.done(byte_done),.error(byte_error),.nack(nack),
        .rx(rx),.scl(scl),.sda(sda));
    always @* begin
        op=2;tx=0;last=0;
        case(stage)
            0:op=0;
            1:tx={dev,1'b0};
            2:tx=addr[15:8];
            3:tx=addr[7:0];
            4:if(rd) op=0; else tx=wd;
            5:tx={dev,1'b1};
            6:begin op=3;last=(idx==len-1'b1);end
            default:op=1;
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            launch<=0;busy<=0;done<=0;error<=0;rd_bytes<=0;
            stage<=0;idx<=0;len<=1;state<=0;rd<=0;dev<=0;addr<=0;wd<=0;
        end else begin
            launch<=0;done<=0;
            case(state)
                0:if(req) begin
                    if(read_op && (length==0 || length>9)) begin done<=1;error<=1;end
                    else begin
                        rd<=read_op;dev<=address;addr<=reg_addr;len<=length;wd<=wr_byte;
                        busy<=1;error<=0;rd_bytes<=0;idx<=0;stage<=0;state<=1;
                    end
                end
                1:if(!byte_busy) begin launch<=1;state<=2;end
                2:if(byte_done) begin
                    if(byte_error || (op==2 && nack)) begin
                        error<=1;
                        if(stage==7 || byte_error) begin busy<=0;done<=1;state<=0;end
                        else begin stage<=7;state<=1;end
                    end else if(stage==7) begin busy<=0;done<=1;state<=0;end
                    else begin
                        state<=1;
                        if(stage==6) begin
                            rd_bytes[idx*8 +: 8]<=rx;
                            if(idx==len-1'b1) stage<=7;else idx<=idx+1'b1;
                        end else if(stage==4 && !rd) stage<=7;
                        else stage<=stage+1'b1;
                    end
                end
                default:begin state<=0;busy<=0;end
            endcase
        end
    end
endmodule
