//=====================================================================
// sd_read_arbiter.v  (Version 23 + V25 Step2: SD Read Arbiter)
//
//  共享唯一 SD 控制器(sd_card_sec_read_write / sd_card_top 的读侧):
//  两个读 master 复用同一 sd_sec_read 字节流口, 禁止第二套 SD PHY/初始化。
//    - m0 = BMP 图片链(bmp_read)         - m1 = WAV/FAT 链(下一阶段接入)
//  单控制器一次只服务一个 512B 块读(见 sd_card_sec_read_write 语义:
//  仅在 S_WAIT_READ_WRITE 采样 sd_sec_read, 忙时新脉冲会被丢 => 不能简单把
//  两个 master 的脉冲并到一个 sd_sec_read 上)。
//
//  设计:
//    - 每个 master 的请求(sd_sec_read 高脉冲)被"锁存 pending"(请求=1 拍即可,
//      之后 arbiter 在控制器空闲时择机补发, master 不会因丢脉冲而死等)。
//    - 事务 owner: 仲裁获胜者独占直到收到 sd_sec_read_end(该块读完);
//      整个 512B 事务期间 owner 保持不变, 绝不中途切换。
//    - 数据/valid/end 按 owner 门控回对应 master => 非 owner 绝不看到别的
//      事务的字节流(不会误吞 BMP/WAV 数据)。
//    - 优先级: BMP(m0) 高; 但 WAV(m1) 只要 pending 就一直保留, 控制器每次
//      空闲先看 m0 再看 m1, 故 WAV 不可能永久饿死(BMP 空闲/突发结束后立刻轮到)。
//    - I_init_done: SD 初始化未完成(0)时禁止发任何控制器请求/事务(Test11);
//      rst: 清 owner/pending/req/输出门控(Test10 无旧数据泄漏)。
//
//  纯组合门控 + 少量寄存器; 无 BRAM/DSP; 一个时钟域(sd_card_clk)。
//=====================================================================
module sd_read_arbiter (
    input  wire        clk,
    input  wire        rst,

    input  wire        I_init_done,       // 1=SD 初始化完成, 允许事务

    // ---- m0: BMP 读 master ----
    input  wire        m0_req,             // 请求(sd_sec_read 高=请求)
    input  wire [31:0] m0_addr,
    output reg  [7:0]  m0_data,
    output reg         m0_valid,
    output reg         m0_end,

    // ---- m1: WAV/FAT 读 master ----
    input  wire        m1_req,
    input  wire [31:0] m1_addr,
    output reg  [7:0]  m1_data,
    output reg         m1_valid,
    output reg         m1_end,

    // ---- 共享 SD 控制器读口 ----
    output reg         ctr_req,
    output reg  [31:0] ctr_addr,
    input  wire [7:0]  ctr_data,
    input  wire        ctr_valid,
    input  wire        ctr_end,

    // ---- 观测 ----
    output wire [1:0]  o_owner,           // 0 none / 1=m0(BMP) / 2=m1(WAV)
    output wire        o_m1_idle,
    output wire        o_busy             // 1=有事务进行中
);
    localparam [1:0] S_IDLE = 2'd0, S_M0 = 2'd1, S_M1 = 2'd2;

    reg [1:0] st;
    reg       pend0, pend1;
    reg       m0_req_seen;
    reg [31:0] pending_addr0, pending_addr1;
    assign o_m1_idle = (st != S_M1) && !pend1 && !m1_req;

    assign o_owner = (st == S_M0) ? 2'd1 : (st == S_M1) ? 2'd2 : 2'd0;
    assign o_busy  = (st != S_IDLE);

    // ---- 输入请求锁存(仅当不是自己当前事务/非空闲也可挂起) ----
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pending_addr0<=0; pending_addr1<=0; m0_req_seen<=0;
            st     <= S_IDLE;
            pend0  <= 1'b0;
            pend1  <= 1'b0;
            ctr_req<= 1'b0;
            ctr_addr<= 32'd0;
            m0_data <= 8'd0; m0_valid <= 1'b0; m0_end <= 1'b0;
            m1_data <= 8'd0; m1_valid <= 1'b0; m1_end <= 1'b0;
        end else begin
            // 锁存非 owner 的请求; owner 自身下一次请求在其 end 后才会发,
            // 此时已回 IDLE, 直接按 pending 处理(避免把同一事务重复入队)。
            // BMP holds req through end. The registered m0_end arrives one cycle
            // after owner returns IDLE; do not enqueue that trailing old level.
            m0_req_seen <= m0_req;
            if (m0_req && !m0_req_seen && (st != S_M0)) begin pend0 <= 1'b1; pending_addr0<=m0_addr; end
            if (m1_req && (st != S_M1)) begin pend1 <= 1'b1; pending_addr1<=m1_addr; end

            // 输出默认: 门控关(非 owner 看不到数据)
            m0_valid <= 1'b0; m0_end <= 1'b0; m0_data <= 8'd0;
            m1_valid <= 1'b0; m1_end <= 1'b0; m1_data <= 8'd0;
            ctr_req  <= 1'b0;

            case (st)
                S_IDLE: begin
                    // 初始化完成才发; BMP 优先; WAV 次之; 有 pending 才发
                    if (I_init_done) begin
                        if (pend0) begin
                            st       <= S_M0;
                            pend0    <= 1'b0;
                            ctr_req  <= 1'b1;
                            ctr_addr <= pending_addr0;
                        end else if (pend1) begin
                            st       <= S_M1;
                            pend1    <= 1'b0;
                            ctr_req  <= 1'b1;
                            ctr_addr <= pending_addr1;
                        end
                    end
                end
                S_M0: begin
                    m0_data <= ctr_data;
                    m0_valid<= ctr_valid;
                    if (ctr_end) begin
                        m0_end <= 1'b1;
                        st     <= S_IDLE;
                    end
                end
                S_M1: begin
                    m1_data <= ctr_data;
                    m1_valid<= ctr_valid;
                    if (ctr_end) begin
                        m1_end <= 1'b1;
                        st     <= S_IDLE;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
