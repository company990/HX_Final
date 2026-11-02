//=====================================================================

// sqrt_integer.v

// 纯整数平方根（二进制恢复型逐位算法，非浮点、非查表）。

//

//   输入 value : 无符号 0..2^IN_WIDTH-1（本设计为 Re^2+Im^2，52bit）

//   输出 result: 无符号整数部分 sqrt(value)（26bit）

//

// 算法（每次迭代恢复 1 位结果）：

//   每次把当前余数左移 2 位并拼入 value 的最高 2 位（自上而下取位），

//   尝试减去 (Y<<2)+1，成功则结果位 1 否则 0，Y 左移 1 位。

//   迭代 OUT_WIDTH 次（每拍 1 次），本设计 26 拍得到一个 bin 的幅度。

//

// 时序：start 高脉冲启动 -> busy 拉高，done 高脉冲时 result 有效。

// 时钟域：video_clk(25MHz)。

//=====================================================================

module sqrt_integer #(

    parameter integer IN_WIDTH  = 52,   // 输入位宽（平方和）

    parameter integer OUT_WIDTH = 26    // 输出位宽（幅度）

)(

    input  wire                          I_clk,

    input  wire                          I_rst,     // 高有效

    input  wire                          I_start,   // 启动脉冲

    input  wire [IN_WIDTH-1:0]           I_value,   // 无符号

    output reg                           O_busy,

    output reg                           O_done,    // 完成脉冲

    output reg  [OUT_WIDTH-1:0]          O_result

);



    localparam integer NIT  = OUT_WIDTH;

    localparam integer WIDE = OUT_WIDTH + 2;          // 余数/试减宽



    function integer count_bits;
        input integer n;integer v;begin count_bits=0;v=n-1;
        while(v>0) begin count_bits=count_bits+1;v=v>>1;end
        if(count_bits<1) count_bits=1;end
    endfunction
    localparam integer COUNT_W=count_bits(NIT);
    // After k steps remainder < 2*partial_root+1; OUT_WIDTH+2 is sufficient.
    reg [COUNT_W-1:0]       S_cnt;      // 已处理迭代数 0..NIT-1

    reg [IN_WIDTH-1:0]      S_val;      // 输入移位寄存器

    reg [WIDE-1:0]          S_rem;      // 余数

    reg [OUT_WIDTH-1:0]     S_res;      // 结果累加



    // 组合：当前最高 2 位、试减值、次态

    wire [1:0] top2 = S_val[IN_WIDTH-1 -: 2];

    wire [WIDE-1:0] rem_n = (S_rem << 2) | { {(WIDE-2){1'b0}}, top2 };

    // 试减 = (Y<<2)+1

    wire [WIDE-1:0] trial =

        { {(WIDE-2-OUT_WIDTH){1'b0}}, S_res, 2'b00 } | {{(WIDE-1){1'b0}},1'b1};

    wire [WIDE:0] difference = {1'b0,rem_n} - {1'b0,trial};
    wire        do_sub = !difference[WIDE];

    wire [WIDE-1:0] rem_2  = do_sub ? difference[WIDE-1:0] : rem_n;

    wire [OUT_WIDTH-1:0] res_2 = do_sub ? ((S_res << 1) | 1'b1) : (S_res << 1);



    always @(posedge I_clk or posedge I_rst) begin

        if (I_rst) begin

            S_cnt    <= {OUT_WIDTH{1'b0}};

            S_val    <= {IN_WIDTH{1'b0}};

            S_rem    <= {WIDE{1'b0}};

            S_res    <= {OUT_WIDTH{1'b0}};

            O_busy   <= 1'b0;

            O_done   <= 1'b0;

            O_result <= {OUT_WIDTH{1'b0}};

        end else begin

            O_done <= 1'b0;

            if (I_start && !O_busy) begin

                O_busy <= 1'b1;

                S_val  <= I_value;

                S_rem  <= {WIDE{1'b0}};

                S_res  <= {OUT_WIDTH{1'b0}};

                S_cnt  <= {OUT_WIDTH{1'b0}};

            end else if (O_busy) begin

                S_val <= S_val << 2;

                S_rem <= rem_2;

                S_res <= res_2;

                if (S_cnt == (NIT - 1)) begin

                    O_busy   <= 1'b0;

                    O_done   <= 1'b1;

                    O_result <= res_2;

                end else begin

                    S_cnt <= S_cnt + 1'b1;

                end

            end

        end

    end



endmodule

