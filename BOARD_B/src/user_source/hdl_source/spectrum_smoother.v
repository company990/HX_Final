// Indexed production mode uses two 64x6 BRAMs and a three-cycle update.
// I_band_* writes target entries; I_update starts one complete smoothing pass.
// I_freeze pauses writes while the display snapshots the current heights.
// The packed legacy interface is retained for HUB75/test-pattern builds.
module spectrum_smoother #(
    parameter integer INDEXED = 0,
    parameter integer SPECTRUM_BIN_NUM = 64,
    parameter integer HUB75_HEIGHT     = 32,    // 柱高上限
    parameter integer HUB75_SMOOTH_STEP= 1
)(
    input  wire                                I_clk,
    input  wire                                I_rst,
    input  wire                                I_update,
    input  wire [SPECTRUM_BIN_NUM*6-1:0]       I_target,
    input wire I_band_valid,
    input wire [5:0] I_band_addr,
    input wire [5:0] I_band_height,
    input wire I_freeze,
    input wire [5:0] I_read_addr,
    output wire [5:0] O_read_height,
    output wire [SPECTRUM_BIN_NUM*6-1:0]       O_height
);
    generate if (INDEXED) begin : indexed
        reg [5:0] index;
        reg [1:0] phase;
        reg initializing, pending, busy;
        wire [5:0] target, current;
        wire [5:0] step = HUB75_SMOOTH_STEP;
        wire [5:0] next_height = (current < target) ?
            (((current + step) < target) ? (current + step) : target) :
            (current > target) ?
            (((current - step) > target) ? (current - step) : target) : current;
        wire write_current = !I_rst && (initializing || (busy && phase==2 && !I_freeze));
        wire [5:0] unused_target;
        spectrum_ram64 target_ram(I_clk, I_band_valid && !I_rst,
            I_band_addr, I_band_height, index, target, 6'd0, unused_target);
        spectrum_ram64 current_ram(I_clk, write_current, index,
            initializing ? 6'd0 : next_height, index, current,
            I_read_addr, O_read_height);
        assign O_height = 0;
        always @(posedge I_clk or posedge I_rst) begin
            if (I_rst) begin
                index<=0; phase<=0; initializing<=1; pending<=0; busy<=0;
            end else begin
                if (initializing) begin
                    if (index==SPECTRUM_BIN_NUM-1) begin
                        initializing<=0; index<=0;
                    end else index<=index+1'b1;
                end else if (!I_freeze) begin
                    if (busy) begin
                        if (phase==2) begin
                            phase<=0;
                            if (index==SPECTRUM_BIN_NUM-1) begin busy<=0; index<=0; end
                            else index<=index+1'b1;
                        end else phase<=phase+1'b1;
                    end else if (pending) begin
                        busy<=1; pending<=0; phase<=0; index<=0;
                    end
                end
                if (I_update) pending<=1;
            end
        end
    end else begin : legacy
        assign O_read_height = 0;
    reg [5:0] cur [0:SPECTRUM_BIN_NUM-1];
    reg [7:0] idx;              // 当前更新的 band 0..SPECTRUM_BIN_NUM-1
    reg [8:0] base;             // idx*6 (递加避免乘法)
    reg       busy;             // 正在扫 64 个 band
    reg       pend;             // busy 期间到达的 I_update 待处理(不丢)
    integer   i;

    // 单份数据通路: 与并行版每 band 逐位相同的运算
    wire [5:0] tg   = I_target[base +: 6];
    wire [5:0] cv   = cur[idx];
    wire [5:0] step = HUB75_SMOOTH_STEP[5:0];
    wire [5:0] nv   =
        (cv < tg) ? (((cv + step) < tg) ? (cv + step) : tg) :
        (cv > tg) ? (((cv - step) > tg) ? (cv - step) : tg) : cv;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            for (i = 0; i < SPECTRUM_BIN_NUM; i = i + 1)
                cur[i] <= 6'd0;
            idx  <= 8'd0;
            base <= 9'd0;
            busy <= 1'b0;
            pend <= 1'b0;
        end else begin
            if (busy) begin
                cur[idx] <= nv;                        // 本拍更新 1 个 band
                if (idx == (SPECTRUM_BIN_NUM-1)) begin
                    busy <= 1'b0;                      // 64 个 band 全部更新完毕
                    pend <= 1'b0;
                end else begin
                    idx  <= idx  + 8'd1;
                    base <= base + 9'd6;
                end
            end else if (pend) begin
                busy <= 1'b1;
                idx  <= 8'd0;
                base <= 9'd0;
                pend <= 1'b0;
            end
            if (I_update) pend <= 1'b1;   // 最后赋值: 完成拍同时到达的新请求也不丢
        end
    end

    genvar g;
        for (g = 0; g < SPECTRUM_BIN_NUM; g = g + 1) begin : gen_out
            assign O_height[g*6 +: 6] = cur[g];
        end
    end endgenerate
endmodule

// One 64x6 true dual-port BRAM; both outputs have one clock latency.
module spectrum_ram64(input wire clk, input wire we,
    input wire [5:0] wa, input wire [5:0] wd,
    input wire [5:0] ra, output wire [5:0] qa,
    input wire [5:0] rb, output wire [5:0] qb);
`ifdef PRIORITY12_SIM
    reg [5:0] mem[0:63];
    reg [5:0] a, b;
    always @(posedge clk) begin
        if (we) mem[wa] <= wd;
        a <= mem[we ? wa : ra];
        b <= mem[rb];
    end
    assign qa=a; assign qb=b;
`else
    EG_LOGIC_BRAM #(.DATA_WIDTH_A(6),.DATA_WIDTH_B(6),
        .ADDR_WIDTH_A(6),.ADDR_WIDTH_B(6),.DATA_DEPTH_A(64),.DATA_DEPTH_B(64),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
        .RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram (
        .dia(wd),.addra(we ? wa : ra),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(we),
        .rsta(1'b0),.bea(1'b0),.doa(qa),
        .dib(6'd0),.addrb(rb),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(1'b0),
        .rstb(1'b0),.beb(1'b0),.dob(qb));
`endif
endmodule
