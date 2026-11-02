// Deterministic RTL8211E RGMII delay setup through Clause 22 MDIO.
// Reference: Realtek RTL8211E datasheet; Linux v6.12 rtl8211e_config_init.
// Both internal delays are enabled; the FPGA forwards an unshifted TX clock.
module rtl8211e_init #(
    parameter integer CLK_HZ=50000000,
    parameter integer WAIT_MS=100,
    parameter integer MDC_HALF_CYCLES=50,
    parameter [4:0] PHY_ADDR=5'd1
)(
    input wire clk,rst,
    output wire mdc,mdio_out,mdio_oe,
    input wire mdio_in,
    output reg ready, fault
);
    localparam integer WAIT_CYCLES=(CLK_HZ/1000)*WAIT_MS;
    localparam integer WAIT_BITS=WAIT_CYCLES<2 ? 1 : $clog2(WAIT_CYCLES+1);
    reg [WAIT_BITS-1:0] wait_count;
    reg [3:0] step;
    reg [1:0] state;
    reg go,read_cmd;
    reg [4:0] reg_addr;
    reg [15:0] write_data,delay_value;
    wire busy,done,read_error;
    wire [15:0] read_data;
    mdio_clause22 #(.HALF_CYCLES(MDC_HALF_CYCLES)) bus(
        .clk(clk),.rst(rst),.go(go),.read_cmd(read_cmd),.phy_addr(PHY_ADDR),
        .reg_addr(reg_addr),.write_data(write_data),.busy(busy),.done(done),
        .read_data(read_data),.read_error(read_error),.mdc(mdc),
        .mdio_in(mdio_in),.mdio_out(mdio_out),.mdio_oe(mdio_oe));
    always @(posedge clk) begin
        if(rst) begin
            state<=0; step<=0; wait_count<=0; ready<=0; fault<=0;
            go<=0; read_cmd<=0; reg_addr<=0; write_data<=0; delay_value<=0;
        end else begin
            go<=0;
            case(state)
                0: if(wait_count>=WAIT_CYCLES) begin wait_count<=0; step<=0; state<=1; end
                   else wait_count<=wait_count+1'b1;
                1: begin
                    read_cmd<=0; write_data<=0;
                    case(step)
                        0: begin reg_addr<=31; write_data<=0; end
                        1: begin reg_addr<=2; read_cmd<=1; end
                        2: begin reg_addr<=3; read_cmd<=1; end
                        3: begin reg_addr<=31; write_data<=7; end
                        4: begin reg_addr<=30; write_data<=16'h00a4; end
                        5: begin reg_addr<=28; read_cmd<=1; end
                        6: begin reg_addr<=28; write_data<=delay_value; end
                        7: begin reg_addr<=28; read_cmd<=1; end
                        8: begin reg_addr<=31; write_data<=0; end
                        9: begin reg_addr<=31; read_cmd<=1; end
                        default: reg_addr<=31;
                    endcase
                    go<=1; state<=2;
                end
                2: if(done) begin
                    if(read_error || (step==1 && read_data!=16'h001c) ||
                        (step==2 && read_data!=16'hc915) ||
                        (step==7 && read_data!=delay_value) ||
                        (step==9 && read_data!=0)) begin
                        fault<=1; state<=0; wait_count<=0;
                    end else if(step==9) begin ready<=1; fault<=0; state<=3; end
                    else begin
                        if(step==5) delay_value<=read_data | 16'h3800;
                        step<=step+1'b1; state<=1;
                    end
                end
                3: ready<=1;
                default: state<=0;
            endcase
        end
    end
endmodule

// MDIO changes while MDC is low. Read data is sampled halfway through the
// high phase, allowing RTL8211E's maximum 300 ns response time at 500 kHz.
module mdio_clause22 #(parameter integer HALF_CYCLES=50)(
    input wire clk,rst,go,read_cmd,
    input wire [4:0] phy_addr,reg_addr,
    input wire [15:0] write_data,
    output reg busy,done,read_error,
    output reg [15:0] read_data,
    output reg mdc,mdio_out,mdio_oe,
    input wire mdio_in
);
    localparam integer COUNT_BITS=$clog2(2*HALF_CYCLES);
    reg [COUNT_BITS-1:0] phase;
    reg [5:0] bit_index;
    reg [63:0] shift;
    reg reading;
    reg mdio_s1,mdio_s2;
    always @(posedge clk) begin
        if(rst) begin
            busy<=0; done<=0; mdc<=0; mdio_out<=1; mdio_oe<=0;
            phase<=0; bit_index<=0; shift<=0; reading<=0; read_data<=0; read_error<=0;
            mdio_s1<=1; mdio_s2<=1;
        end else begin
            mdio_s1<=mdio_in; mdio_s2<=mdio_s1;
            done<=0;
            if(!busy) begin
                if(go) begin
                    shift<={32'hffffffff,2'b01,(read_cmd ? 2'b10 : 2'b01),
                        phy_addr,reg_addr,2'b10,write_data};
                    reading<=read_cmd; busy<=1; phase<=0; bit_index<=0;
                    read_error<=0; read_data<=0; mdio_oe<=1; mdio_out<=1; mdc<=0;
                end
            end else begin
                if(phase==HALF_CYCLES-1) mdc<=1;
                if(phase==HALF_CYCLES+HALF_CYCLES/2-1 && reading) begin
                    if(bit_index==47 && mdio_s2!=1'b0) read_error<=1;
                    if(bit_index>=48) read_data<={read_data[14:0],mdio_s2};
                end
                if(phase==2*HALF_CYCLES-1) begin
                    phase<=0; mdc<=0;
                    if(bit_index==63) begin busy<=0; done<=1; mdio_oe<=0; end
                    else begin
                        bit_index<=bit_index+1'b1; shift<={shift[62:0],1'b0};
                        mdio_out<=shift[62];
                        if(reading && bit_index>=45) mdio_oe<=0;
                    end
                end else phase<=phase+1'b1;
            end
        end
    end
endmodule
