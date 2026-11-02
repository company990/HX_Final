// Project-local binding for the public APM simple-dual-port RAM interface.
// APUG013 v1.7 uses exactly one such RAM: the 2048x8 ICMP echo FIFO.
// TD 6.2's generic APM model maps this configuration to distributed RAM even
// with MEMORY_PRIMITIVE="bram". Bind the documented interface to EG4 BRAM.
// The encrypted network stack and its FIFO controller remain unchanged.
// Compile after the official APM simulation library; include explicitly in
// synthesis. Unsupported configurations fail elaboration instead of silently
// changing the semantics of another APM memory instance.
module apm_memory_sdpram #(
    parameter MEMORY_PRIMITIVE="auto", MEMORY_INIT_FILE="none",
    parameter MEMORY_FILL_REMAIN="none", CLOCKING_MODE="common_clock",
    parameter ECC_MODE="no_ecc", FORCE_KEEP=0, ALGORITHM="min_area", ECC_PIPE=0,
    parameter ADDR_WIDTH_A=5, DATA_DEPTH_A=1<<ADDR_WIDTH_A,
    parameter WRITE_DATA_WIDTH_A=5, BYTE_WIDTH_A=WRITE_DATA_WIDTH_A,
    parameter ADDR_WIDTH_B=5, DATA_DEPTH_B=1<<ADDR_WIDTH_B,
    parameter READ_DATA_WIDTH_B=WRITE_DATA_WIDTH_A, RESET_VALUE_B=5'b0,
    parameter RESETMODE_B="sync", READ_LATENCY_B=2
)(
    input wire clka,cea,
    input wire [WRITE_DATA_WIDTH_A/BYTE_WIDTH_A-1:0] wea,
    input wire [ADDR_WIDTH_A-1:0] addra,
    input wire [WRITE_DATA_WIDTH_A-1:0] dina,
    input wire clkb,rstb,ceb,oceb,
    input wire [ADDR_WIDTH_B-1:0] addrb,
    output wire [READ_DATA_WIDTH_B-1:0] doutb,
    input wire injectsbiterra,injectdbiterra,
    output wire sbiterrb,dbiterrb
);
    generate if(ADDR_WIDTH_A==11 && ADDR_WIDTH_B==11 &&
        DATA_DEPTH_A==2048 && DATA_DEPTH_B==2048 &&
        WRITE_DATA_WIDTH_A==8 && READ_DATA_WIDTH_B==8 && BYTE_WIDTH_A==8 &&
        READ_LATENCY_B==1 && RESETMODE_B=="sync" &&
        MEMORY_INIT_FILE=="none" && MEMORY_FILL_REMAIN=="none" &&
        ECC_MODE=="no_ecc" && ECC_PIPE==0 &&
        (CLOCKING_MODE=="independent_clock" || CLOCKING_MODE=="common_clock")) begin : eg4
        wire read_clk=CLOCKING_MODE=="common_clock" ? clka : clkb;
        wire [7:0] data;
        reg reset_output;
        // The APM reset takes priority over CE and holds RESET_VALUE until the
        // next enabled read. Preserve that behavior outside the BRAM itself.
        always @(posedge read_clk) begin
            if(rstb) reset_output<=1'b1;
            else if(ceb) reset_output<=1'b0;
        end
        assign doutb=reset_output ? RESET_VALUE_B : data;
        EG_LOGIC_BRAM #(.DATA_WIDTH_A(8),.DATA_WIDTH_B(8),
            .ADDR_WIDTH_A(11),.ADDR_WIDTH_B(11),.DATA_DEPTH_A(2048),.DATA_DEPTH_B(2048),
            .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
            .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
            .RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
            .dia(dina),.addra(addra),.cea(cea),.ocea(1'b0),.clka(clka),.wea(wea[0]),
            .rsta(1'b0),.bea(1'b0),.doa(),
            .dib(8'd0),.addrb(addrb),.ceb(ceb && !rstb),.oceb(1'b0),.clkb(read_clk),
            .web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(data));
    end else begin : unsupported
        APUG013_EG4_APM_CONFIGURATION_NOT_SUPPORTED invalid_configuration();
    end endgenerate
    assign sbiterrb=1'b0;
    assign dbiterrb=1'b0;
endmodule
