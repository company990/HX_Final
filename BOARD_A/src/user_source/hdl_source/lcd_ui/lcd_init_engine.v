// Synchronous ROM input: address is held for FETCH and DECODE.
// word[31:28]: 0=command, 1=data, 2=delay in clk cycles, F=end.
// END is accepted ONLY with profile_confirmed=1. This is intentionally not
// a fabricated panel initialization table. Supply the exact panel vendor ROM.
module lcd_init_engine #(
    parameter integer CLK_HZ=50000000, ROM_ADDR_BITS=10
)(
    input wire clk, rst_n, profile_confirmed,
    output reg lcd_rst_n,
    output reg [ROM_ADDR_BITS-1:0] rom_addr,
    input wire [31:0] rom_word,
    output wire req_valid, output wire req_rs,
    output wire [15:0] req_data,
    input wire req_ready, bus_done,
    output reg init_done, init_fault
);
    localparam RESET=0, RELEASE=1, FETCH=2, DECODE=3, SEND=4,
               WAIT_BUS=5, DELAY=6, FINISH=7, FAULT=8;
    reg [3:0] state;
    // The bytecode carries at most 28 delay bits. Reset/release loads
    // can require more for unusually high CLK_HZ; preserve those as well.
    // CLK_HZ<50 retains the legacy 32-bit underflow behavior unchanged.
    localparam integer RELEASE_BITS=CLK_HZ<50 ? 32:$clog2(CLK_HZ/5);
    localparam integer COUNT_BITS=RELEASE_BITS>28 ? RELEASE_BITS:28;
    reg [COUNT_BITS-1:0] count;
    reg [31:0] instruction;
    assign req_valid=(state==SEND);
    assign req_rs=instruction[28];
    assign req_data=instruction[15:0];
    task advance;
        begin
            if (&rom_addr) begin init_fault<=1; state<=FAULT; end
            else begin rom_addr<=rom_addr+1'b1; state<=FETCH; end
        end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=RESET; count<=CLK_HZ/50-1; lcd_rst_n<=0;
            rom_addr<=0; instruction<=0; init_done<=0; init_fault<=0;
        end else case(state)
            RESET: if(count!=0) count<=count-1'b1;
            else begin lcd_rst_n<=1; count<=CLK_HZ/5-1; state<=RELEASE; end
            RELEASE: if(count!=0) count<=count-1'b1; else state<=FETCH;
            FETCH: state<=DECODE;
            DECODE: begin
                instruction<=rom_word;
                case(rom_word[31:28])
                    0,1: state<=SEND;
                    2: if(rom_word[27:0]==0) advance;
                       else begin count<={4'b0,rom_word[27:0]}-1'b1; state<=DELAY; end
                    15: if(profile_confirmed) begin init_done<=1; state<=FINISH; end
                        else begin init_fault<=1; state<=FAULT; end
                    default: begin init_fault<=1; state<=FAULT; end
                endcase
            end
            SEND: if(req_ready) state<=WAIT_BUS;
            WAIT_BUS: if(bus_done) advance;
            DELAY: if(count!=0) count<=count-1'b1; else advance;
            FINISH: state<=FINISH;
            default: begin state<=FAULT; init_fault<=1; end
        endcase
    end
endmodule
