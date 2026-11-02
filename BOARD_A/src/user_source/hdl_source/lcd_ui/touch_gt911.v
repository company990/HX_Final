// Candidate ONLY for verified GT911 with factory configuration already valid.
// Selects address 0x5D: INT low during reset, release as input after 50 ms.
// Reads product ID before accepting touch reports. GT9147/OTT2001A require
// their own ID/configuration verification; do not silently treat as GT911.
module touch_gt911 #(parameter integer CLK_HZ=50000000,
    SENSOR_WIDTH=480,SENSOR_HEIGHT=800)(
    input wire clk,rst_n,enable,
    inout wire scl,sda,int_n,
    output reg touch_rst_n,
    output reg valid,down,
    output reg [15:0] x,y,
    output reg identified,fault,
    output reg [31:0] product_id
);
    reg int_low;
    assign int_n=int_low ? 1'b0 : 1'bz;
    reg [1:0] irq;
    reg [3:0] state;
    reg [31:0] timer,poll;
    reg req,read_op;
    reg [15:0] addr;
    reg [3:0] len;
    wire busy,done,error;
    wire [71:0] bytes;
    reg [71:0] report;
    localparam RESET=0,INT_HOLD=1,BOOT=2,ID_REQ=3,ID_WAIT=4,
        IDLE=5,READ_WAIT=6,CLEAR_REQ=7,CLEAR_WAIT=8,BAD_ID=9,
        SIZE_REQ=10,SIZE_WAIT=11;
    touch_i2c_reg #(.CLK_HZ(CLK_HZ)) master(
        .clk(clk),.rst_n(rst_n),.req(req),.read_op(read_op),.address(7'h5d),
        .reg_addr(addr),.length(len),.wr_byte(8'h00),
        .busy(busy),.done(done),.error(error),.rd_bytes(bytes),.scl(scl),.sda(sda));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=RESET;timer<=CLK_HZ/50-1;poll<=0;int_low<=1;touch_rst_n<=0;
            irq<=3;req<=0;read_op<=1;addr<=0;len<=1;report<=0;
            valid<=0;down<=0;x<=0;y<=0;identified<=0;fault<=0;product_id<=0;
        end else begin
            irq<={irq[0],int_n};req<=0;valid<=0;
            case(state)
                RESET:if(enable) begin
                    if(timer!=0) timer<=timer-1'b1;
                    else begin touch_rst_n<=1;timer<=CLK_HZ/20-1;state<=INT_HOLD;end
                end
                INT_HOLD:if(timer!=0) timer<=timer-1'b1;
                    else begin int_low<=0;timer<=CLK_HZ/20-1;state<=BOOT;end
                BOOT:if(timer!=0) timer<=timer-1'b1;else state<=ID_REQ;
                ID_REQ:if(!busy) begin req<=1;read_op<=1;addr<=16'h8140;len<=4;state<=ID_WAIT;end
                ID_WAIT:if(done) begin
                    product_id<=bytes[31:0];
                    if(error) begin fault<=1;timer<=CLK_HZ/20-1;state<=BOOT;end
                    else if(bytes[23:0]!=24'h313139) begin fault<=1;state<=BAD_ID;end
                    else state<=SIZE_REQ;
                end
                SIZE_REQ:if(!busy) begin req<=1;read_op<=1;addr<=16'h8048;len<=4;state<=SIZE_WAIT;end
                SIZE_WAIT:if(done) begin
                    if(error || bytes[15:0]!=SENSOR_WIDTH || bytes[31:16]!=SENSOR_HEIGHT)
                        begin fault<=1;state<=BAD_ID;end
                    else begin fault<=0;identified<=1;poll<=0;state<=IDLE;end
                end
                // Rate-limited level IRQ + 10 ms polling fallback. Polling also
                // obtains release reports when the IRQ polarity/config differs.
                IDLE:if(poll!=0 && !(irq[1] && !irq[0])) poll<=poll-1'b1;
                else if(!busy) begin
                    req<=1;read_op<=1;addr<=16'h814e;len<=9;state<=READ_WAIT;
                end
                READ_WAIT:if(done) begin
                    if(error) begin
                        fault<=1;valid<=1;down<=0;poll<=CLK_HZ/100-1;state<=IDLE;
                    end else if(bytes[7]) begin report<=bytes;state<=CLEAR_REQ;end
                    else begin poll<=CLK_HZ/100-1;state<=IDLE;end
                end
                CLEAR_REQ:if(!busy) begin
                    req<=1;read_op<=0;addr<=16'h814e;len<=1;state<=CLEAR_WAIT;
                end
                CLEAR_WAIT:if(done) begin
                    fault<=error;valid<=1;
                    down<=!error && report[3:0]!=0 && report[3:0]<=5;
                    if(!error && report[3:0]!=0 && report[3:0]<=5) begin
                        x<=report[31:16];y<=report[47:32];
                    end
                    poll<=CLK_HZ/100-1;state<=IDLE;
                end
                default:state<=BAD_ID;
            endcase
        end
    end
endmodule
