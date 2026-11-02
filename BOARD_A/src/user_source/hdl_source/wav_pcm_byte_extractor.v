//=====================================================================
// wav_pcm_byte_extractor.v  (Phase20: 文件字节流 -> 连续 PCM L/R 帧)
// 轻量字节流 sink: 从 MUSIC.WAV 的 data 区逐字节取 16bit LE Stereo PCM.
//   与 tf_wav_pcm_reader(直接驱动 SD sector) 不同, 本模块是纯字节流消费者:
//   上游(tf_wav_pcm_pipeline)已把"跳过 header 后从 data 第一字节开始"
//   的位置对齐好, 本模块只负责按 I_data_size 取 data_size 字节并组帧.
// 不缓存整文件/大 BRAM(逐字节组装), DSP+0 / PLL+0.
// 握手:
//   输入字节: I_byte_valid/I_byte[7:0] + O_byte_ready(背压)
//   输出帧  : pcm_valid + pcm_left/right(signed16) + I_sample_req(=ready)
// 帧字节序(文件顺序, LE): Llo Lhi Rlo Rhi -> L={Lhi,Llo} R={Rhi,Rlo}
// 错误码: 1 data_size%4!=0 ; 2 流提前结束(截断)。
//=====================================================================
module wav_pcm_byte_extractor (
    input  wire clk,
    input  wire rst,
    input  wire I_start,
    input  wire [31:0] I_data_size,
    input  wire I_byte_valid,
    input  wire [7:0] I_byte,
    input  wire I_stream_end,
    output wire O_byte_ready,
    output reg  pcm_valid,
    output reg  signed [15:0] pcm_left,
    output reg  signed [15:0] pcm_right,
    input  wire I_sample_req,
    output reg  wav_done,
    output reg  O_error,
    output reg  [3:0] O_error_code,
    output reg  [3:0] O_state
);
    localparam [3:0] S_IDLE=0, S_RUN=1, S_DONE=2, S_ERR=3;

    reg [3:0]  st;
    reg [31:0] rem;        // 尚未拉取的 data 字节数
    reg [1:0]  bc;         // 帧内字节 0..3
    reg [7:0]  llo, lhi, rlo;

    assign O_byte_ready = (st==S_RUN) && (!pcm_valid) && (rem!=0);

    always @(posedge clk or posedge rst) begin : main
        if (rst) begin
            st<=S_IDLE; rem<=0; bc<=0; llo<=0; lhi<=0; rlo<=0;
            pcm_valid<=0; pcm_left<=0; pcm_right<=0;
            wav_done<=0; O_error<=0; O_error_code<=0; O_state<=0;
        end else begin
            O_state <= st;
            case (st)
                S_IDLE: begin
                    if (I_start) begin
                        O_error<=0; O_error_code<=0; wav_done<=0;
                        pcm_valid<=0; bc<=0; rem<=I_data_size;
                        if (I_data_size==0) begin
                            wav_done<=1'b1;
                            st<=S_DONE;
                        end else if (I_data_size[1:0]!=2'd0) begin
                            O_error<=1'b1; O_error_code<=4'd1;
                            st<=S_ERR;
                        end else begin
                            st<=S_RUN;
                        end
                    end
                end
                S_RUN: begin
                    if (pcm_valid && I_sample_req) begin
                        // 消费者取走当前帧
                        pcm_valid <= 1'b0;
                        if (rem == 0) begin
                            wav_done <= 1'b1;
                            st <= S_DONE;
                        end
                    end else if (!pcm_valid && I_byte_valid && O_byte_ready) begin
                        // 取一个字节
                        case (bc)
                            2'd0: llo <= I_byte;
                            2'd1: lhi <= I_byte;
                            2'd2: rlo <= I_byte;
                            default: begin
                                // 第 4 字节 = Rhi; 组帧
                                pcm_valid  <= 1'b1;
                                pcm_left   <= {lhi, llo};
                                pcm_right  <= {I_byte, rlo};
                            end
                        endcase
                        bc <= bc + 2'd1;
                        rem <= rem - 32'd1;
                    end
                    if (I_stream_end && (rem!=0)) begin
                        O_error<=1'b1; O_error_code<=4'd2;
                        st<=S_ERR;
                    end
                end
                S_DONE: begin
                    st <= S_IDLE;       // wav_done 保持到下次 I_start
                end
                S_ERR: begin
                    st <= S_IDLE;       // O_error/O_error_code 保持到下次 I_start
                end
                default: st<=S_IDLE;
            endcase
        end
    end
endmodule
