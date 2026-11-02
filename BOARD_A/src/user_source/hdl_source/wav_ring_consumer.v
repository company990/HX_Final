//=====================================================================
// wav_ring_consumer.v  (Version 23: WAV SDRAM Ring 48k Consumer)
//
//  从 audio_sdram_ring_buffer 消费端按 48k sample-enable 取 {L16,R16}
//  帧, 输出统一 24bit PCM(16->24 用 {sample,8'b0}, 与 HDMI Audio/FFT
//  输入位宽一致)。只在 I_enable(=WAV 播放使能)且 ring 非空时请求 =>
//  无 underflow; 不播放时不消费 => 预取期 ring 只增长。
//
//  play_done 判定(与 ctl 配合, 播完不循环):
//    I_enable && producer(wav pipeline)已 done && 已读过 >=1 帧 && ring 空
//    => O_play_done 高(ctl 由 PLAY_WAV->WAV_DONE 静音)。
//
//  不重实现 ring/背压; 纯消费端协调 + 16->24 上采样。
//  无 BRAM/DSP; 一个时钟域(消费侧 sample 时钟)。
//=====================================================================
module wav_ring_consumer #(
    parameter integer W = 13          // used_frames 位宽(ADDRW: 配合 ring FRAMES)
)(
    input  wire clk,
    input  wire rst,

    // 播放控制 / 状态
    input  wire        I_sample_enable,   // 48k sample tick(消费侧)
    input  wire        I_enable,          // = O_wav_play(1 才消费)
    input  wire [W:0]  I_used,            // ring used_frames
    input  wire        I_producer_done,   // = pipeline wav_done(生产者已结束)

    // ring 消费口
    output reg         O_req,             // -> ring consumer_req
    input  wire        I_valid,           // <- ring consumer_valid
    input  wire [31:0] I_data,            // <- ring consumer_data {L16,R16}

    // 统一 48k PCM 输出(24bit)
    output reg         O_pcm_valid,
    output reg signed [23:0] O_left,
    output reg signed [23:0] O_right,

    // 状态
    output reg         O_play_done,       // 全部读完(给 ctl)
    output reg  [15:0] O_gotframes        // 本段已消费帧数(调试/测试)
);
    // ---------------- 读请求(每 sample tick 一帧) ----------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            O_req <= 1'b0;
        end else begin
            O_req <= 1'b0;
            if (I_sample_enable && I_enable && (I_used != 0))
                O_req <= 1'b1;
        end
    end

    // ---------------- 弹出捕获 + 16->24 + 计数 ----------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            O_pcm_valid <= 1'b0;
            O_left      <= 24'sd0;
            O_right     <= 24'sd0;
            O_gotframes <= 16'd0;
        end else begin
            O_pcm_valid <= 1'b0;
            if (I_valid) begin
                O_pcm_valid <= 1'b1;
                O_left      <= {I_data[31:16], 8'd0};   // 16 -> 24 上采样
                O_right     <= {I_data[15:0],  8'd0};
                if (!O_gotframes[15]) O_gotframes <= O_gotframes + 16'd1;
            end
        end
    end

    // ---------------- play_done ----------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            O_play_done <= 1'b0;
        end else begin
            if (!I_enable) O_play_done <= 1'b0;
            else O_play_done <= I_producer_done && (I_used == 0) &&
                                (O_gotframes >= 16'd1) && I_enable;
        end
    end
endmodule
