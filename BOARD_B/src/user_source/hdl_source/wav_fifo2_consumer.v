// Prefetch PCM before the HDMI sampling edge. Valid is a transfer enable.
module wav_fifo2_consumer (
    input wire clk, rst,
    input wire I_sample_enable, I_enable,
    output reg O_fifo_rd,
    input wire I_fifo_valid,
    input wire [31:0] I_fifo_data,
    input wire I_fifo_empty, I_producer_done,
    output wire O_pcm_valid,
    output wire signed [23:0] O_left, O_right,
    output reg O_play_done,
    output reg [15:0] O_gotframes,
    output wire O_rd_pending
);
    reg rd_pend, have_frame;
    reg [31:0] frame;
    assign O_rd_pending = rd_pend;
    assign O_pcm_valid = I_enable && I_sample_enable && have_frame;
    assign O_left  = have_frame ? {frame[31:16],8'b0} : 24'd0;
    assign O_right = have_frame ? {frame[15:0],8'b0} : 24'd0;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            O_fifo_rd<=0; rd_pend<=0; have_frame<=0; frame<=0;
            O_play_done<=0; O_gotframes<=0;
        end else begin
            O_fifo_rd<=0;
            if (I_fifo_valid) begin
                rd_pend<=0; frame<=I_fifo_data; have_frame<=1;
            end
            if (I_enable && !have_frame && !rd_pend && !I_fifo_empty) begin
                O_fifo_rd<=1; rd_pend<=1;
            end
            if (O_pcm_valid) begin
                have_frame<=0;
                if (O_gotframes!=16'hffff) O_gotframes<=O_gotframes+1'b1;
            end
            // Producer done must include ALL upstream buffers, not just decoder.
            O_play_done <= I_enable && I_producer_done && I_fifo_empty &&
                           !rd_pend && !have_frame && !I_fifo_valid;
        end
    end
endmodule
