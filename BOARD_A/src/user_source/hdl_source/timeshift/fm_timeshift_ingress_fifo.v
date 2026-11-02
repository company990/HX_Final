// Timeshift-only variant of the unchanged original audio_async_fifo.
// Source Gray pointers stay registered. First write-pointer capture is on
// falling mem_clk, second stage and RAM/read control stay on rising mem_clk.
// This provides a real half-cycle boundary for PLL-related 25->50MHz clocks,
// without timing exceptions or changes to existing WAV/MIC FIFO instances.
//=====================================================================
// audio_async_fifo.v  (Version 24: Audio Multi-Clock CDC Async FIFO)
//
//  标准 Gray 指针异步 FIFO, 用于音频 32bit PCM 帧跨时钟域:
//    A(sd_card_clk) -> B(ext_mem_clk)   : WAV 读出 PCM -> Audio SDRAM Ring
//    B(ext_mem_clk) -> C(video_clk)     : Ring 读回 -> 48k Consumer
//  一帧 = {L16,R16} = 32bit。
//
//  - 每域各自维护自己的二进制指针(wbin/rbin), 只把对端的 *Gray* 指针经 2FF
//    同步(多 bit 指针绝不用裸 reg/2FF 直接跨域)。
//  - Full(写域)/Empty(读域) 用 Gray 指针 MSB 两位取反比较(环形满/空判定)。
//  - Data 只在 !empty 时被读 => 无随机/旧数据泄漏; 空读不推进指针, 报 underflow。
//  - 满写不推进指针, 报 overflow; 绝不覆盖未读数据/不越界。
//  - almost_full/almost_empty = 由各自域把对端 gray 还原成 bin 估算水位(阈值).
//  - 复位: 写/读各自独立异步复位(rst_w_n / rst_r_n), 各自只清自己的指针/同步器。
//  - EG4 dual-port BRAM storage; Gray pointers and handshake latency are unchanged.
//=====================================================================
module fm_timeshift_ingress_fifo #(
    parameter integer DATA_W      = 32,
    parameter integer DEPTH       = 64,    // 2 的幂
    parameter integer ADDR_W      = 6,     // $clog2(DEPTH)
    parameter integer AEMPTY_LEV  = 2,     // used<=2 => almost_empty
    parameter integer AFULL_LEV   = 2      // (DEPTH-used)<=2 => almost_full
)(
    input  wire                clk_w,
    input  wire                rst_w_n,     // 写域异步复位(低有效)
    input  wire                i_wr,        // 写使能
    input  wire [DATA_W-1:0]   i_data,
    output wire                o_wr_acked,  // 1 拍: 本拍成功写入 1 帧(背压计数用)
    output wire                o_full,
    output wire                o_almost_full,
    output wire                o_overflow,  // 满时仍写 => 丢弃 1 拍

    input  wire                clk_r,
    input  wire                rst_r_n,     // 读域异步复位(低有效)
    input  wire                i_rd,        // 读使能(请求弹出一帧)
    output wire [DATA_W-1:0]   o_data,      // 弹出的帧(与 o_rd_valid 同步)
    output wire                o_rd_valid,  // 1 拍: 本拍 o_data 有效(已弹出 1 帧)
    output wire                o_empty,
    output wire                o_almost_empty,
    output wire                o_underflow  // 空时仍读 => 本拍无数据
);
    // 指针(含 1 位额外最高位; 单调递增)
    reg  [ADDR_W:0] wbin;
    reg  [ADDR_W:0] rbin;
    // Register Gray in its SOURCE domain. XORing an asynchronous binary
    // counter at the destination can capture glitches at carry transitions.
    reg [ADDR_W:0] wgray /* synthesis keep="true" */;
    reg [ADDR_W:0] rgray /* synthesis keep="true" */;
    wire [ADDR_W:0] wnext = wbin + 1'b1;
    wire [ADDR_W:0] rnext = rbin + 1'b1;
    reg             ov_r;    // 已满仍写(丢弃) -- 注册, 反映写前状态
    reg             und_r;   // 已空仍读(无数据) -- 注册, 反映读前状态
    reg             wack_r;  // 写接受 1 拍

    // 各自域把对端 gray 2FF 同步(绝不裸传多 bit 指针/数据)
    reg  [ADDR_W:0] rgray_s1, rgray_s2;   // clk_w 域: gray(rbin)
    reg  [ADDR_W:0] wgray_s1, wgray_s2;   // clk_r 域: gray(wbin)

    function [ADDR_W:0] g2b;
        input [ADDR_W:0] g;
        integer k;
        begin
            g2b = g;
            for (k = ADDR_W-1; k >= 0; k = k - 1)
                g2b[k] = g2b[k+1] ^ g[k];
        end
    endfunction

    reg [ADDR_W:0] rbin_of_w;   // clk_w 域: 还原的读指针(同步滞后 => 偏小)
    reg [ADDR_W:0] wbin_of_r;   // clk_r 域: 还原的写指针(同步滞后 => 偏小)
    always @(*) rbin_of_w = g2b(rgray_s2);
    always @(*) wbin_of_r = g2b(wgray_s2);

    // used 估算(各自域): 同步滞后只会让 full/empty 提前 => 安全方向(不覆写/不空读)
    wire [ADDR_W:0] used_w = wbin - rbin_of_w;   // 写域: >=实际 used => full 提前
    wire [ADDR_W:0] used_r = wbin_of_r - rbin;   // 读域: <=实际 used => empty 提前
    wire full  = (used_w >= DEPTH);
    wire empty = (used_r == 0);

    // ---- 复位同步: 任一侧复位都收敛两侧指针(防单侧复位后 epoch 错位读到旧数据) ----
    // 各自域 2FF 同步对端复位; 有效复位 = 本域复位 & 对端已释放(经同步)。
    reg rrs1, rrs2;   // clk_w 域: 同步 rst_r_n
    reg wrs1, wrs2;   // clk_r 域: 同步 rst_w_n
    always @(posedge clk_w) begin rrs1 <= rst_r_n; rrs2 <= rrs1; end
    always @(posedge clk_r) begin wrs1 <= rst_w_n; wrs2 <= wrs1; end
    wire eff_w_n = rst_w_n & rrs2;
    wire eff_r_n = rst_r_n & wrs2;

    // ---- 写域(同步复位) ----
    always @(posedge clk_w) begin
        if (!eff_w_n) begin
            rgray_s1 <= 0; rgray_s2 <= 0;
            wbin     <= 0;
            wgray    <= 0;
            ov_r     <= 1'b0;
            wack_r   <= 1'b0;
        end else begin
            rgray_s1 <= rgray;
            rgray_s2 <= rgray_s1;
            ov_r     <= i_wr && full;   // 写前已满才报(丢弃)
            wack_r   <= i_wr && !full;  // 成功写入
            if (i_wr && !full) begin
                wbin <= wnext;
                wgray <= wnext ^ (wnext >> 1);
            end
        end
    end

    // Dual-clock BRAM stores the same 64 PCM words; pointer logic is unchanged.
    wire [DATA_W-1:0] ram_data;
    wire ram_write=eff_w_n && i_wr && !full;
    wire ram_read=eff_r_n && i_rd && !empty;
    reg data_seen;
    always @(posedge clk_r)
        if(!eff_r_n) data_seen<=1'b0;
        else if(ram_read) data_seen<=1'b1;
    EG_LOGIC_BRAM #(.DATA_WIDTH_A(DATA_W),.DATA_WIDTH_B(DATA_W),
        .ADDR_WIDTH_A(ADDR_W),.ADDR_WIDTH_B(ADDR_W),
        .DATA_DEPTH_A(DEPTH),.DATA_DEPTH_B(DEPTH),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
        .RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) storage (
        .dia(i_data),.addra(wbin[ADDR_W-1:0]),.cea(ram_write),.ocea(1'b0),
        .clka(clk_w),.wea(ram_write),.rsta(1'b0),.bea(1'b0),.doa(),
        .dib({DATA_W{1'b0}}),.addrb(rbin[ADDR_W-1:0]),.ceb(ram_read),.oceb(1'b0),
        .clkb(clk_r),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(ram_data));
    reg              rd_valid_r;
    always @(negedge clk_r) begin
        if (!eff_r_n) wgray_s1 <= 0;
        else wgray_s1 <= wgray;
    end
    // ---- 读域(同步复位) ----
    always @(posedge clk_r) begin
        if (!eff_r_n) begin
            wgray_s2 <= 0;
            rbin      <= 0;
            rgray     <= 0;
            rd_valid_r <= 1'b0;
            und_r      <= 1'b0;
        end else begin
            wgray_s2 <= wgray_s1;
            und_r     <= i_rd && empty;   // 读前已空才报(无数据)
            if (i_rd && !empty) begin
                rbin       <= rbin + 1'b1;
                rgray      <= rnext ^ (rnext >> 1);
                rd_valid_r <= 1'b1;
            end else begin
                rd_valid_r <= 1'b0;
            end
        end
    end
    assign o_data      = data_seen ? ram_data : {DATA_W{1'b0}};
    assign o_rd_valid  = rd_valid_r;

    assign o_underflow = und_r;
    assign o_overflow  = ov_r;
    assign o_wr_acked  = wack_r;
    assign o_full  = full;
    assign o_empty = empty;
    assign o_almost_full  = (used_w >= (DEPTH - AFULL_LEV));
    assign o_almost_empty = (used_r <= AEMPTY_LEV);
endmodule
