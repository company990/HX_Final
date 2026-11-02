//=====================================================================
// audio_sdram_ring_buffer.v  (Phase3: SDRAM Audio Ring Buffer)
//
//   PCM Producer -> SDRAM Ring(32KB, 可配置) -> PCM Consumer
//
// Version 16:
//  - FRAMES 可配置(默认 8192 = 32KB; 每帧 32bit {L16,R16})
//  - 数据存 SDRAM(抽象接口); 不用大 reg array / 大 BRAM 缓存整曲
//  - Ring: write_ptr/read_ptr 自回绕; used_frames 主状态
//  - Producer valid/data/ready; full => ready=0, overflow 统计
//  - Consumer req -> valid+data; used==0 => underflow(=valid0)+统计
//  - 抽象 SDRAM: mem_wr_* / mem_rd_*(读延迟可 1..N)
//  - 读延迟吸收: OFDEPTH 深 FF/LUT 输出缓存 + 单次在途读预取
//    (单在途读 => response 与 request 一一对应, 抗任意读延迟)
//  - 关键: 同拍 push/pop 或写提交/pop 对 ocnt/used 采用"一次合并提交",
//    避免 NBA last-write-wins 丢增量导致计数/回绕错乱.
//  - 不改 frame_read_write / SDRAM PHY / HDMI / FFT / HUB75 / Startup / MUX
//=====================================================================
module audio_sdram_ring_buffer #(
    parameter FRAMES  = 8192,
    parameter ADDRW   = 13,       // 2^ADDRW >= FRAMES
    parameter LOW     = 2048,
    parameter HIGH    = 6144,
    parameter OFDEPTH = 8
)(
    input  wire clk,
    input  wire rst,

    // ---------------- Producer ----------------
    input  wire        producer_valid,
    input  wire [31:0] producer_data,
    output wire        producer_ready,

    // ---------------- Consumer ----------------
    input  wire        consumer_req,
    output reg         consumer_valid,
    output reg  [31:0] consumer_data,

    // ---------------- Status ----------------
    output wire [ADDRW:0] used_frames,
    output wire        empty,
    output wire        full,
    output wire        almost_empty,
    output wire        almost_full,
    output wire        underflow,
    output wire        overflow,
    output reg  [15:0] underflow_count,
    output reg  [15:0] overflow_count,

    // ---------------- 抽象 SDRAM 写口 ----------------
    output reg         mem_wr_req,
    output reg  [ADDRW-1:0] mem_wr_addr,
    output reg  [31:0] mem_wr_data,
    input  wire        mem_wr_ready,

    // ---------------- 抽象 SDRAM 读口 ----------------
    output reg         mem_rd_req,
    output reg  [ADDRW-1:0] mem_rd_addr,
    input  wire [31:0] mem_rd_data,
    input  wire        mem_rd_valid
);
    // ---- 主状态 ----
    reg  [ADDRW-1:0] wptr, rptr;
    reg  [ADDRW:0]   used;
    assign used_frames  = used;
    assign full         = (used == FRAMES);
    assign empty        = (used == 0);
    assign almost_empty = (used <= LOW);
    assign almost_full  = (used >= HIGH);

    // ---- 写侧 ----
    reg [31:0] wdata;
    reg        whold;
    assign producer_ready = !whold && !full;
    assign overflow       = producer_valid && full;
    assign underflow      = consumer_req && (used == 0);

    // ---- 读侧输出缓存 ----
    reg [31:0] ofifo [0:OFDEPTH-1];
    reg [3:0]  owp, orp;
    reg [3:0]  ocnt;                        // 0..OFDEPTH
    reg        rd_busy;
    reg        want, pop_pend;

    wire can_issue = !rd_busy && (ocnt < OFDEPTH) && (used > ocnt);

    always @(posedge clk or posedge rst) begin : main
        reg        push_evt, pop_evt, wr_evt;
        reg [ADDRW:0] nu;
        reg [3:0]    no;
        if (rst) begin
            wptr<=0; rptr<=0; used<=0;
            whold<=0; wdata<=0;
            mem_wr_req<=0; mem_wr_addr<=0; mem_wr_data<=0;
            mem_rd_req<=0; mem_rd_addr<=0;
            owp<=0; orp<=0; ocnt<=0; rd_busy<=0; want<=0; pop_pend<=0;
            consumer_valid<=0; consumer_data<=0;
            underflow_count<=0; overflow_count<=0;
        end else begin
            // ---- 默认 ----
            mem_wr_req <= 1'b0;
            mem_rd_req <= 1'b0;
            consumer_valid <= 1'b0;
            push_evt = 1'b0; pop_evt = 1'b0; wr_evt = 1'b0;

            // ===== 写侧(接受 + mem 提交) =====
            if (whold) begin
                mem_wr_req  <= 1'b1;
                mem_wr_data <= wdata;
                if (mem_wr_ready) begin
                    mem_wr_req <= 1'b0;
                    whold <= 1'b0;
                    wptr  <= wptr + 1'b1;
                    wr_evt = 1'b1;                 // used+1
                end
            end else if (producer_valid && !full) begin
                whold       <= 1'b1;
                wdata       <= producer_data;
                mem_wr_addr <= wptr;
                mem_wr_data <= producer_data;
                mem_wr_req  <= 1'b1;
            end
            if (producer_valid && full) overflow_count <= overflow_count + 1'b1;

            // ===== 读侧预取(单在途) =====
            if (can_issue) begin
                rd_busy     <= 1'b1;
                mem_rd_req  <= 1'b1;
                mem_rd_addr <= rptr;
                rptr        <= rptr + 1'b1;
            end
            if (mem_rd_valid) begin
                rd_busy     <= 1'b0;
                ofifo[owp]  <= mem_rd_data;
                if (owp == (OFDEPTH-1)) owp <= 0; else owp <= owp + 1'b1;
                push_evt = 1'b1;                   // ocnt+1
            end

            // ===== Consumer =====
            if (consumer_req) begin
                if (used == 0) begin
                    underflow_count <= underflow_count + 1'b1;
                end else if (ocnt != 0) begin
                    pop_pend <= 1'b1;
                end else begin
                    want <= 1'b1;
                end
            end
            if (want && (ocnt != 0)) begin
                pop_pend <= 1'b1;
                want     <= 1'b0;
            end
            if (pop_pend) begin
                consumer_valid <= 1'b1;
                consumer_data  <= ofifo[orp];
                if (orp == (OFDEPTH-1)) orp <= 0; else orp <= orp + 1'b1;
                pop_evt = 1'b1;                    // ocnt-1, used-1
                pop_pend <= 1'b0;
            end

            // ===== 合并提交 ocnt / used(避免同拍 push&pop / wr&pop 竞态) =====
            no = ocnt + (push_evt ? 1'b1 : 1'b0) - (pop_evt ? 1'b1 : 1'b0);
            nu = used + (wr_evt ? 1'b1 : 1'b0) - (pop_evt ? 1'b1 : 1'b0);
            ocnt <= no;
            used <= nu;
        end
    end
endmodule
