//=====================================================================
// tf_wav_pcm_reader.v  (TF WAV -> 连续 PCM sample 流, 第二阶段)
// 输入: I_data_offset(文件字节偏移, L0 起始)/I_data_size(data 字节数, 须 %4=0)
//       与 I_file_start_sector(该 WAV 文件起始扇区; 本阶段 TEMPORARY 固定定位,
//       非 FAT 文件名解析)。
// 复用 SD 读口(与真实 sd_card_sec_read_write 读侧一致):
//   sd_sec_read/sd_sec_read_addr/sd_sec_read_data/sd_sec_read_data_valid/sd_sec_read_end
//   单块 512B 扇区读。
// 内部: 520B 环形字节 FIFO(可综合 0~1 BRAM) 解耦 SD(producer)与 48k consumer,
//       支持跨 sector 边界残帧(每 sector 起始前要求 fcnt<=8 防溢出)。
// 输出: 每 I_sample_req 一帧 pcm_left/right(signed16, LE); 读完 wav_done;
//       data_size%4!=0 -> error_code=1; data_size==0 -> 空即 done。
// 不做: SDRAM ring / 大 BRAM FIFO / MUX / 接入 FFT/HDMI/HUB75/Startup。
//=====================================================================
module tf_wav_pcm_reader (
    input  wire I_clk,
    input  wire I_rst,
    input  wire I_start,
    input  wire [31:0] I_data_offset,
    input  wire [31:0] I_data_size,
    input  wire [31:0] I_file_start_sector,
    output reg        sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0] sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,
    input  wire        I_sample_req,
    output reg         pcm_valid,
    output reg  signed [15:0] pcm_left,
    output reg  signed [15:0] pcm_right,
    output wire        pcm_ready,
    output reg         underflow,
    output reg         wav_done,
    output reg         O_error,
    output reg  [3:0]  O_error_code,
    output reg  [3:0]  O_state
);
    localparam [3:0] S_IDLE=0, S_INIT=1, S_REQ=2, S_RD=3, S_WFULL=4, S_DRAIN=5, S_DONE=6, S_ERR=7;
    localparam integer DEPTH = 520;
    localparam integer PTRW = 10;

    reg [7:0]  fifo[0:DEPTH-1];
    reg [PTRW-1:0] wp, rp;
    reg [PTRW:0]   fcnt;               // 0..520

    reg [3:0]  st;
    reg [31:0] produced;
    reg [31:0] cur_sec;
    reg [8:0]  bcnt;

    wire [31:0] rel_sec  = cur_sec - I_file_start_sector;
    wire [31:0] file_off = (rel_sec << 9) | {23'd0, bcnt};

    wire pop_ok = (fcnt >= 10'd4) && (st!=S_DONE) && (st!=S_ERR);
    assign pcm_ready = pop_ok;

    function [PTRW-1:0] nxtp; input [PTRW-1:0] p;
        begin if (p == (DEPTH-1)) nxtp = 0; else nxtp = p + 1'b1; end
    endfunction
    function [PTRW-1:0] ridx; input [PTRW-1:0] base; input [3:0] off;
        integer v;
        begin v = base + off; if (v >= DEPTH) v = v - DEPTH; ridx = v; end
    endfunction

    always @(posedge I_clk or posedge I_rst) begin : main
        reg w_en, p_en;
        reg [PTRW:0] nf;
        if (I_rst) begin
            st<=S_IDLE; wp<=0; rp<=0; fcnt<=0; produced<=0; cur_sec<=0; bcnt<=0;
            sd_sec_read<=0; sd_sec_read_addr<=0;
            pcm_valid<=0; pcm_left<=0; pcm_right<=0; underflow<=0;
            wav_done<=0; O_error<=0; O_error_code<=0; O_state<=0;
        end else begin
            O_state <= st;
            sd_sec_read <= 1'b0;
            pcm_valid   <= 1'b0;
            wav_done    <= 1'b0;
            underflow   <= 1'b0;

            // 计算本拍写/读
            w_en = 1'b0;
            if ((st==S_RD) && sd_sec_read_data_valid) begin
                if ((file_off >= I_data_offset) &&
                    (file_off - I_data_offset < I_data_size) &&
                    (fcnt < DEPTH))
                    w_en = 1'b1;
            end
            p_en = I_sample_req && pop_ok;
            nf   = fcnt + (w_en ? {{(PTRW){1'b0}},1'b1} : 0) - (p_en ? {{(PTRW){1'b0}},4'd4} : 0);

            if (w_en) begin
                fifo[wp] <= sd_sec_read_data;
                wp       <= nxtp(wp);
                produced <= produced + 32'd1;
            end
            if (p_en) begin
                pcm_valid <= 1'b1;
                pcm_left  <= {fifo[ridx(rp,4'd1)], fifo[ridx(rp,4'd0)]};
                pcm_right <= {fifo[ridx(rp,4'd3)], fifo[ridx(rp,4'd2)]};
                rp        <= nxtp(nxtp(nxtp(nxtp(rp))));
            end else if (I_sample_req) begin
                underflow <= 1'b1;
            end
            fcnt <= nf;

            case (st)
                S_IDLE: begin
                    if (I_start) begin
                        if (I_data_size[1:0] != 2'd0) begin
                            st<=S_ERR; O_error<=1'b1; O_error_code<=4'd1;
                        end else if (I_data_size == 32'd0) begin
                            st<=S_DONE; wav_done<=1'b1;
                        end else begin
                            st<=S_INIT;
                        end
                    end
                end
                S_INIT: begin
                    produced<=0; fcnt<=0; wp<=0; rp<=0; bcnt<=0;
                    cur_sec <= I_file_start_sector + (I_data_offset >> 9);
                    st<=S_REQ;
                end
                S_REQ: begin
                    sd_sec_read      <= 1'b1;
                    sd_sec_read_addr <= cur_sec;
                    bcnt             <= 9'd0;
                    st<=S_RD;
                end
                S_RD: begin
                    if (sd_sec_read_data_valid) bcnt <= bcnt + 9'd1;
                    if (sd_sec_read_end) begin
                        if ((produced + (w_en?32'd1:32'd0)) >= I_data_size) begin
                            st<=S_DRAIN;
                        end else if (fcnt <= 10'd8) begin
                            cur_sec <= cur_sec + 32'd1;
                            st<=S_REQ;
                        end else begin
                            st<=S_WFULL;
                        end
                    end
                end
                S_WFULL: begin
                    if (fcnt <= 10'd8) begin
                        cur_sec <= cur_sec + 32'd1;
                        st<=S_REQ;
                    end
                end
                S_DRAIN: begin
                    if (fcnt == 0) begin
                        st<=S_DONE; wav_done<=1'b1;
                    end
                end
                S_DONE: begin end
                S_ERR:  begin end
                default: st<=S_IDLE;
            endcase
        end
    end
endmodule
