//=====================================================================
// tf_wav_pcm_pipeline.v  (Phase20: TF FAT32 -> WAV -> PCM 完整链协调器)
//
// 职责: 只做"数据流连接 + 状态控制 + SD 主控切换 + 子模块启动握手 +
//       错误时排空 stream 使其回 IDLE", 不含 FAT/WAV/PCM 算法。
//
// 链:
//   fat32_file_locator      "文件在哪?"
//   fat32_file_stream_reader"连续字节?"  -> byte stream
//   tf_wav_parser           "是否支持WAV?" -> data_offset/data_size
//   wav_pcm_byte_extractor  "PCM 是什么?"  -> L16/R16
//
// 启动握手: 各子模块终态自动回 IDLE(done/error 粘性)。pipeline 每阶段
//   "先确认 done 拉低(已重启, 防旧粘性 done 误判) -> 再等 done 拉高(完成)"。
// 错误处理: parser/extractor 出错时, stream 可能仍卡在文件中途(无法经
//   I_start 从非 IDLE 重启), 故先 P_DRAIN 丢弃剩余字节直到 stream 回 IDLE。
//
// SD 读口(单一主控): locator / stream 阶段依次独占。
// 错误码: locator(4 无TF_AUDIO /5 无MUSIC) / stream(2..5) / parser(1..7)
//   / extractor(1) 原样上报。
//=====================================================================
module tf_wav_pcm_pipeline (
    input  wire clk,
    input  wire rst,
    input  wire I_start,
    input wire [87:0] I_target_name,

    output wire        sd_sec_read,
    output wire [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,

    output wire         pcm_valid,
    output wire  signed [15:0] pcm_left,
    output wire  signed [15:0] pcm_right,
    input  wire         I_sample_req,
    output wire         wav_done,
    output wire         O_busy,
    output wire         O_error,
    output wire  [3:0]  O_error_code,
    output wire  [3:0]  O_state,

    output wire [15:0] O_wav_count, O_wav_index,
    output wire         O_file_found,
    output wire  [31:0] O_file_size,
    output wire  [31:0] O_first_cluster,
    output wire  [15:0] O_file_clusters,
    output wire  [15:0] O_fmt,
    output wire  [15:0] O_channels,
    output wire  [31:0] O_sample_rate,
    output wire  [15:0] O_bits,
    output wire  [31:0] O_data_offset,
    output wire  [31:0] O_data_size
);
    // V27S8: LAZY_CHAIN 透传到 locator(默认 0 = 主链行为逐位不变; 专项 TB 可传 1)
    parameter integer LAZY_CHAIN = 1;
    parameter integer DYNAMIC_NAME = 0;

    localparam [3:0] P_IDLE=0, P_LOC=1, P_GO=2, P_HDR=3, P_EXST=4,
                     P_DATA=5, P_ERRS=6, P_DONE=7, P_ERR=8, P_DRAIN=9;

    reg [3:0] pst;
    reg loc_p, go_p, ex_p;
    reg loc_ack, go_ack, ex_ack;
    reg sd_sel_loc;
    reg strm_active;                 // stream 已启动且可能未回 IDLE(需排空)
    reg [31:0] fcl, fsz;
    reg [3:0] err_code;
    reg wav_done_r, o_err_r; reg [3:0] o_ec_r;

    // ---- locator ----
    wire loc_rd; wire [31:0] loc_raddr;
    wire loc_found, loc_done, loc_busy, loc_err; wire [3:0] loc_ec; wire [3:0] loc_st;
    wire [31:0] loc_fsize, loc_first; wire [15:0] loc_fcl;
    fat32_file_locator #(
        .DYNAMIC_NAME(DYNAMIC_NAME), .LAZY_CHAIN(LAZY_CHAIN) // V27S8: 默认 0(已验证行为); 专项 TB 诊断时传 1
    ) u_loc (
        .clk(clk), .rst(rst), .I_start(loc_p), .I_target_name(I_target_name),
        .sd_sec_read(loc_rd), .sd_sec_read_addr(loc_raddr),
        .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .wav_count(O_wav_count), .wav_index(O_wav_index),
        .file_found(loc_found), .file_size(loc_fsize), .first_cluster(loc_first),
        .file_clusters(loc_fcl), .done(loc_done), .busy(loc_busy),
        .error(loc_err), .error_code(loc_ec), .state(loc_st)
    );

    // ---- stream ----
    wire strm_rd; wire [31:0] strm_raddr;
    wire strm_v, strm_err, strm_done, strm_busy; wire [7:0] strm_d; wire [3:0] strm_ec, strm_st;
    wire [31:0] strm_cnt;
    wire strm_ready;
    fat32_file_stream_reader u_strm (
        .clk(clk), .rst(rst), .I_start(go_p),
        .I_file_first_cluster(fcl), .I_file_size(fsz),
        .sd_sec_read(strm_rd), .sd_sec_read_addr(strm_raddr),
        .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .O_byte_valid(strm_v), .O_byte_data(strm_d), .O_byte_ready(strm_ready),
        .O_file_end(), .O_done(strm_done), .O_busy(strm_busy),
        .O_error(strm_err), .O_error_code(strm_ec), .O_byte_cnt(strm_cnt), .O_state(strm_st)
    );

    // ---- parser ----
    wire pars_done, pars_err, pars_busy; wire [3:0] pars_ec, pars_st;
    wire [15:0] pars_fmt, pars_ch, pars_bits; wire [31:0] pars_rate, pars_doff, pars_dsz;
    wire pars_v;
    tf_wav_parser u_pars (
        .I_clk(clk), .I_rst(rst), .I_start(go_p),
        .I_byte_valid(pars_v), .I_byte(strm_d), .I_stream_end(1'b0),
        .O_busy(pars_busy), .O_done(pars_done), .O_error(pars_err), .O_error_code(pars_ec),
        .O_fmt(pars_fmt), .O_channels(pars_ch), .O_sample_rate(pars_rate), .O_bits(pars_bits),
        .O_data_offset(pars_doff), .O_data_size(pars_dsz), .O_state(pars_st)
    );

    // ---- extractor ----
    wire ex_ready, ex_valid, ex_done, ex_err; wire [3:0] ex_ec; wire [3:0] ex_st;
    wire signed [15:0] ex_l, ex_r;
    wire ex_v;
    wav_pcm_byte_extractor u_ex (
        .clk(clk), .rst(rst), .I_start(ex_p),
        .I_data_size(pars_dsz),
        .I_byte_valid(ex_v), .I_byte(strm_d), .I_stream_end(strm_done),
        .O_byte_ready(ex_ready),
        .pcm_valid(ex_valid), .pcm_left(ex_l), .pcm_right(ex_r),
        .I_sample_req(I_sample_req),
        .wav_done(ex_done), .O_error(ex_err), .O_error_code(ex_ec), .O_state(ex_st)
    );

    // ---- 阶段/字节切换 ----
    wire in_hdr = (pst==P_HDR);
    wire in_ex  = (pst==P_DATA);
    wire in_drain = (pst==P_DRAIN);
    wire feed_pars = in_hdr && !pars_done && !pars_err;
    wire feed_ex   = in_ex && !ex_done && !ex_err;

    assign pars_v     = feed_pars && strm_v;
    assign strm_ready = feed_pars || (feed_ex && ex_ready) || in_drain;
    assign ex_v       = feed_ex && ex_ready && strm_v;

    assign sd_sec_read      = sd_sel_loc ? loc_rd   : strm_rd;
    assign sd_sec_read_addr = sd_sel_loc ? loc_raddr : strm_raddr;

    assign pcm_valid  = ex_valid;
    assign pcm_left   = ex_l;
    assign pcm_right  = ex_r;
    assign wav_done   = wav_done_r;
    assign O_error    = o_err_r;
    assign O_error_code = o_ec_r;
    assign O_state    = pst;
    assign O_busy     = (pst==P_LOC)||(pst==P_GO)||(pst==P_HDR)||(pst==P_EXST)||
                        (pst==P_DATA)||(pst==P_DRAIN);

    assign O_file_found    = loc_found;
    assign O_file_size     = fsz;
    assign O_first_cluster = fcl;
    assign O_file_clusters = loc_fcl;
    assign O_fmt           = pars_fmt;
    assign O_channels      = pars_ch;
    assign O_sample_rate   = pars_rate;
    assign O_bits          = pars_bits;
    assign O_data_offset   = pars_doff;
    assign O_data_size     = pars_dsz;

    // ---- FSM ----
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pst<=P_IDLE; loc_p<=0; go_p<=0; ex_p<=0;
            loc_ack<=0; go_ack<=0; ex_ack<=0;
            sd_sel_loc<=0; strm_active<=0; fcl<=0; fsz<=0; err_code<=0;
            wav_done_r<=0; o_err_r<=0; o_ec_r<=0;
        end else begin
            loc_p<=0; go_p<=0; ex_p<=0;
            case (pst)
                P_IDLE: begin
                    loc_ack<=0; go_ack<=0; ex_ack<=0; strm_active<=0;
                    if (I_start) begin
                        wav_done_r<=0; o_err_r<=0; o_ec_r<=0;
                        sd_sel_loc<=1'b1;
                        pst<=P_LOC;
                    end
                end
                P_LOC: begin
                    if (!loc_ack) begin
                        loc_p<=1'b1;
                        if (!loc_done) loc_ack<=1'b1;
                    end else begin
                        loc_p<=1'b0;
                        if (loc_done) begin
                            sd_sel_loc<=0;
                            if (loc_err) begin
                                err_code<=loc_ec; pst<=P_ERRS;
                            end else if (!loc_found) begin
                                err_code<=4'd5; pst<=P_ERRS;
                            end else begin
                                fcl<=loc_first; fsz<=loc_fsize;
                                pst<=P_GO;
                            end
                        end
                    end
                end
                P_GO: begin
                    if (!go_ack) begin
                        go_p<=1'b1;
                        if (!strm_done && !pars_done) go_ack<=1'b1;
                    end else begin
                        go_p<=1'b0;
                        strm_active<=1'b1;
                        pst<=P_HDR;
                    end
                end
                P_HDR: begin
                    if (pars_err) begin
                        err_code<=pars_ec; pst<=P_ERRS;
                    end else if (strm_err) begin
                        err_code<=strm_ec; pst<=P_ERRS;
                    end else if (pars_done) begin
                        if (pars_doff > fsz || pars_dsz > (fsz - pars_doff)) begin
                            err_code<=4'd7; pst<=P_ERRS;
                        end else begin
                            pst<=P_EXST;
                        end
                    end else if (strm_done) begin
                        err_code<=4'd7; pst<=P_ERRS;
                    end
                end
                P_EXST: begin
                    if (!ex_ack) begin
                        ex_p<=1'b1;
                        if (!ex_done) ex_ack<=1'b1;
                    end else begin
                        ex_p<=1'b0;
                        pst<=P_DATA;
                    end
                end
                P_DATA: begin
                    if (ex_err) begin
                        err_code<=ex_ec; pst<=P_ERRS;
                    end else if (strm_err) begin
                        err_code<=strm_ec; pst<=P_ERRS;
                    end else if (ex_done) begin
                        wav_done_r<=1'b1;
                        pst<=P_DONE;
                    end
                end
                P_ERRS: begin
                    o_err_r<=1'b1; o_ec_r<=err_code;
                    // 若 stream 仍在文件中途(parser/extractor 出错), 先排空回 IDLE
                    if (strm_active && !strm_done) pst<=P_DRAIN;
                    else pst<=P_ERR;
                end
                P_DONE: begin
                    // 收尾: 若 stream 仍在文件中途(理论上正常路径已读完),
                    // 排空到其回 IDLE, 保证下次可 I_start 重启
                    if (strm_active && !strm_done) pst<=P_DRAIN;
                    else pst<=P_IDLE;
                end
                P_DRAIN: begin
                    if (strm_done) begin
                        strm_active<=0;
                        pst<=P_ERR;      // 到 P_ERR 后回 IDLE(o_err 已在 ERRS 置位)
                    end
                end
                P_ERR: begin
                    pst<=P_IDLE;
                end
                default: pst<=P_IDLE;
            endcase
        end
    end
endmodule
