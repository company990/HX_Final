//=====================================================================
// audio_wav_top_path.v  (V26 Step8 Phase-A: 最终 WAV 音频链正式 RTL 层次)
//
//  把 V14..V26 已验证模块在一个正式可综合 RTL 层次串成:
//    A(sd_card_clk): SD m1 client(经 top 唯一 sd_card_top+sd_read_arbiter.m1)
//      -> V20 tf_wav_pcm_pipeline(动态定位 /TF_AUDIO/MUSIC.WAV)
//      -> PCM{L16,R16} -> V24 audio_async_fifo #1(A 写/B 读, 单在途 ack)
//    B(ext_mem_clk)  : FIFO#1 读(rd_pend) -> audio_sdram_ring_buffer
//      (Ring mem_* 上提为端口; top 侧经 audio_sdram_access 接共享 SDRAM)
//      -> Ring Consumer(rd_pend) -> V24 audio_async_fifo #2(B 写/C 读)
//    C(video_clk)    : wav_fifo2_consumer(48k audio_valid tick, 16->24) = WAV 源
//      + Startup/Tone 源(外部输入) -> audio_mux3(sel 由 audio_startup_wav_ctl)
//      -> 24bit o_pcm_left/right/valid(HDMI Audio + FFT 同源)
//
//  边界(Phase-A): 只暴露 SD m1 client / Ring mem 接口 / video PCM / MUX+控制状态;
//    不接 SDRAM PHY、不驱动 App_*/Sdr_*; audio_sdram_access 只在 top 侧唯一桥;
//    SD Controller 仍唯一(top sd_card_top); A->B/B->C 只经 V24 async_fifo;
//    48k 仅 video_clk 域 tick, 不新建 clock/PLL。Startup 绝不同时与 WAV 输出;
//    Startup done 但 WAV 未 ready -> 静音等待; WAV 播完 -> pcm0+done 不循环。
//  纯 LUT/FF+复用模块。
//=====================================================================
module audio_wav_top_path #(
    parameter integer DYNAMIC_NAME = 0,
    parameter integer PAUSE_ENABLE = 0,
    parameter integer NETWORK_PREFILL_FRAMES = 4096,
    parameter integer AUDIO_FIFO_DEPTH = 64,   // FIFO#1/#2 深度
    parameter integer RING_FRAMES      = 8192, // Audio Ring 帧数(32KB)
    parameter integer RING_AW          = 13,   // clog2(RING_FRAMES)
    parameter integer RING_LOW         = 2048,
    parameter integer RING_HIGH        = 6144,
    parameter integer AUDIO_TEST_MODE  = 0     // 1=WAV 不可用输出 TestTone(DEBUG)
)(
    input  wire [87:0] i_target_name,
    input  wire        i_pause,
    input  wire clk_a,   // sd_card_clk (A)
    input  wire clk_b,   // ext_mem_clk (B)
    input  wire clk_c,   // video_clk   (C)
    input  wire rst_a, input  wire rst_b, input  wire rst_c,
    // Active-low HC-SR501 soft reset synchronized into each clock domain.
    input  wire rst_soft_a, input wire rst_soft_b, input wire rst_soft_c,

    // Runtime WAV source selection: 0=TF/SD card, 1=Ethernet.
    // source_sel is synchronized to clk_a by the top-level switch filter.
    input  wire source_sel,

    // Network file stream in clk_a. Ignored in legacy TF mode.
    input wire [7:0] net_file_byte,
    input wire net_file_valid, net_file_end,
    input wire [31:0] net_file_size,
    output wire net_file_ready, net_file_error,

    // ---------- A: SD m1 client(接 top sd_read_arbiter.m1) ----------
    output wire        sd_m1_req,
    output wire [31:0] sd_m1_addr,
    input  wire [7:0]  sd_m1_data,
    input  wire        sd_m1_valid,
    input  wire        sd_m1_end,

    // ---------- B: Ring mem 接口(接 top audio_sdram_access.a_*) ----------
    output wire              ring_mem_wr_req,
    output wire [RING_AW-1:0] ring_mem_wr_addr,
    output wire [31:0]        ring_mem_wr_data,
    input  wire               ring_mem_wr_ready,
    output wire               ring_mem_rd_req,
    output wire [RING_AW-1:0] ring_mem_rd_addr,
    input  wire [31:0]        ring_mem_rd_data,
    input  wire               ring_mem_rd_valid,

    // ---------- C: 控制输入(video 域) ----------
    input  wire              i_audio_valid,       // video_clk 域 48k sample tick(=audio_valid)
    input  wire              i_sd_init_done,      // V27S2: SD 初始化完成(已同步到本域) — 音频启动唯一起点
    input  wire              i_first_frame_ready, // (V27S2 起不再用于音频; 保留端口兼容)
    input  wire              i_startup_done,      // Startup Music 播完(粘性)
    input  wire              i_wav_start,         // (保留端口兼容: 已不再需要; 现由内部 A_SEARCH 触发)
    input  wire              i_no_bmp,            // (V27S2 起不再用于音频; 保留端口兼容)
    input  wire signed [23:0] i_startup_l, i_startup_r,   // Startup 音源(外部 player)
    input  wire signed [23:0] i_tone_l,   i_tone_r,       // Test Tone(外部 I2S 恢复音)

    // ---------- C: 统一 24bit PCM 输出 + WAV 帧输出 ----------
    output wire         o_pcm_valid,
    output wire signed [23:0] o_pcm_left, o_pcm_right,

    output wire         o_wav_valid,
    output wire signed [23:0] o_wav_left, o_wav_right,

    // ---------- C: MUX/控制状态 ----------
    output wire [1:0]  o_audio_sel,
    output wire        o_wav_play,
    output wire        o_unlock_image,
    output wire [2:0]  o_ctl_phase,
    output wire        o_music_start,     // V27S2: 单拍(启动 Startup Music)
    output wire        o_wav_search,      // V27S2: A_SEARCH 电平(启动 ROOT locator/预取)
    output wire [2:0]  o_audio_digit,     // V27S2: 音频启动 debug digit(0..4)
    output reg [15:0] o_wav_count_c, o_wav_index_c,
    output reg o_catalog_valid_c,
    output wire        o_wav_ready,       // FIFO#2 非空
    output wire        o_wav_found_c,
    output wire        o_wav_err_c,
    output wire        o_play_finished,
    output wire        o_wav_done_c,     // WAV 播完不循环

    // ---------- 状态(域内观测) ----------
    output wire [RING_AW:0] o_ring_used,   // (B) ring used
    output wire             o_fifo1_full,  // (A)
    output wire             o_fifo1_empty, // (B 读侧)
    output wire             o_fifo2_full,  // (B 写侧)
    output wire             o_fifo2_empty  // (C 读侧)
);
    // Keep the established high-active reset convention inside the data path.
    // Reset all buffered audio state whenever the selected source changes so
    // bytes or PCM frames from two different files can never be mixed.
    wire rst_a_base = rst_a | ~rst_soft_a;
    wire rst_b_base = rst_b | ~rst_soft_b;
    wire rst_c_base = rst_c | ~rst_soft_c;

    reg source_a_d=0;
    reg source_b_meta=0, source_b=0, source_b_d=0;
    reg source_c_meta=0, source_c=0, source_c_d=0;
    wire source_change_a = source_sel ^ source_a_d;
    wire source_change_b = source_b ^ source_b_d;
    wire source_change_c = source_c ^ source_c_d;
    wire rst_a_all = rst_a_base | source_change_a;
    wire rst_b_all = rst_b_base | source_change_b;
    wire rst_c_all = rst_c_base | source_change_c;

    always @(posedge clk_a or posedge rst_a_base) begin
        if (rst_a_base) source_a_d <= 1'b0;
        else            source_a_d <= source_sel;
    end
    always @(posedge clk_b or posedge rst_b_base) begin
        if (rst_b_base) begin
            source_b_meta <= 1'b0;
            source_b      <= 1'b0;
            source_b_d    <= 1'b0;
        end else begin
            source_b_meta <= source_sel;
            source_b      <= source_b_meta;
            source_b_d    <= source_b;
        end
    end
    always @(posedge clk_c or posedge rst_c_base) begin
        if (rst_c_base) begin
            source_c_meta <= 1'b0;
            source_c      <= 1'b0;
            source_c_d    <= 1'b0;
        end else begin
            source_c_meta <= source_sel;
            source_c      <= source_c_meta;
            source_c_d    <= source_c;
        end
    end

    // ==================================================================
    //  A 域: V20 pipeline + FIFO#1 写(单在途)
    // ==================================================================
    wire        wav_pcm_v; wire signed [15:0] wav_pcm_l, wav_pcm_r;
    wire        wav_done, wav_err; wire [3:0] wav_st, wav_ec; wire wav_found;
    reg         wav_start=0;
    reg         started=0;
    wire        sample_ok;

    // Select the raw WAV byte source before parsing. SD and Ethernet therefore
    // share one WAV parser/extractor instead of consuming two complete pipelines.
    wire        tf_req;
    wire [31:0] tf_addr;
    wire [7:0]  tf_file_byte;
    wire        tf_file_valid, tf_file_ready, tf_file_end;
    wire [31:0] tf_file_size;
    wire        tf_source_error;
    wire [3:0]  tf_source_error_code;

    wire [7:0]  selected_file_byte;
    wire        selected_file_valid, selected_file_ready, selected_file_end;
    wire [31:0] selected_file_size;
    wire        shared_pcm_valid, shared_done, shared_error, shared_found;
    wire signed [15:0] shared_left, shared_right;
    wire [3:0]  shared_state, shared_error_code;

    assign sd_m1_req  = source_sel ? 1'b0 : tf_req;
    assign sd_m1_addr = source_sel ? 32'd0 : tf_addr;

    assign selected_file_byte = source_sel ? net_file_byte : tf_file_byte;
    assign selected_file_valid = source_sel ? net_file_valid : tf_file_valid;
    assign selected_file_size = source_sel ? net_file_size : tf_file_size;
    assign selected_file_end = source_sel ? net_file_end :
                               (tf_file_end | tf_source_error);
    assign net_file_ready = source_sel ? selected_file_ready : 1'b0;
    assign tf_file_ready = source_sel ? 1'b0 : selected_file_ready;

    assign wav_pcm_v = shared_pcm_valid;
    assign wav_pcm_l = shared_left;
    assign wav_pcm_r = shared_right;
    assign wav_done = shared_done;
    assign wav_err = shared_error | (!source_sel && tf_source_error);
    assign wav_found = shared_found;
    assign wav_st = shared_state;
    assign wav_ec = shared_error ? shared_error_code :
                    ((!source_sel && tf_source_error) ? tf_source_error_code : 4'd0);
    assign net_file_error = source_sel && shared_error;

    wire tf_catalog_found;
    // Held A-domain result bundle; synchronize found before capturing it in C.
    // Each private session reset clears both the source and destination handshake.
    wire [15:0] catalog_count_a, catalog_index_a;
    reg [2:0] catalog_found_sync;
    reg catalog_seen;
    always @(posedge clk_c or posedge rst_c_all) begin
        if(rst_c_all) begin
            catalog_found_sync<=0; catalog_seen<=0; o_catalog_valid_c<=0;
            o_wav_count_c<=0; o_wav_index_c<=0;
        end else begin
            catalog_found_sync<={catalog_found_sync[1:0],tf_catalog_found};
            o_catalog_valid_c<=0;
            if(catalog_found_sync[2] && !catalog_seen) begin
                o_wav_count_c<=catalog_count_a; o_wav_index_c<=catalog_index_a;
                o_catalog_valid_c<=1; catalog_seen<=1;
            end
        end
    end
    tf_wav_file_source #(.DYNAMIC_NAME(DYNAMIC_NAME)) u_tf_file_source (
        .clk(clk_a), .rst(rst_a_all),
        .I_start(source_sel ? 1'b0 : wav_start), .I_target_name(i_target_name),
        .sd_sec_read(tf_req), .sd_sec_read_addr(tf_addr),
        .sd_sec_read_data(sd_m1_data),
        .sd_sec_read_data_valid(sd_m1_valid), .sd_sec_read_end(sd_m1_end),
        .file_byte(tf_file_byte), .file_valid(tf_file_valid),
        .file_ready(tf_file_ready), .file_size(tf_file_size),
        .file_end(tf_file_end), .file_found(tf_catalog_found),
        .wav_count(catalog_count_a), .wav_index(catalog_index_a),
        .error(tf_source_error), .error_code(tf_source_error_code), .state()
    );

    wav_stream_pcm_pipeline u_wav_pipe (
        .clk(clk_a), .rst(rst_a_all), .I_start(wav_start),
        .I_file_size(selected_file_size), .I_byte(selected_file_byte),
        .I_byte_valid(selected_file_valid), .I_stream_end(selected_file_end),
        .O_byte_ready(selected_file_ready), .I_sample_req(sample_ok),
        .pcm_valid(shared_pcm_valid), .pcm_left(shared_left), .pcm_right(shared_right),
        .wav_done(shared_done), .O_error(shared_error),
        .O_error_code(shared_error_code), .O_file_found(shared_found),
        .O_state(shared_state)
    );

    // FIFO#1 (A->B)
    wire [31:0] f1_din = {wav_pcm_l, wav_pcm_r};
    reg  f1_wr=0, f1_pend=0; wire f1_wack;
    wire f1_full, f1_afull, f1_ovf;
    wire [31:0] f1_dout; wire f1_rdv, f1_empty, f1_aempty, f1_und;
    reg  f1_rd=0;
    audio_async_fifo #(.DATA_W(32),.DEPTH(AUDIO_FIFO_DEPTH),.ADDR_W(6),
                       .AEMPTY_LEV(2),.AFULL_LEV(2)) u_fifo1 (
        .clk_w(clk_a), .rst_w_n(~rst_a_all), .i_wr(f1_wr), .i_data(f1_din), .o_wr_acked(f1_wack),
        .o_full(f1_full), .o_almost_full(f1_afull), .o_overflow(f1_ovf),
        .clk_r(clk_b), .rst_r_n(~rst_b_all), .i_rd(f1_rd), .o_data(f1_dout),
        .o_rd_valid(f1_rdv), .o_empty(f1_empty), .o_almost_empty(f1_aempty),
        .o_underflow(f1_und)
    );
    assign sample_ok = !f1_full && !f1_pend;
    always @(posedge clk_a or posedge rst_a_all) begin
        if (rst_a_all) begin f1_wr<=0; f1_pend<=0; end
        else begin
            f1_wr <= 1'b0;
            if (f1_wack) f1_pend <= 1'b0;
            if (wav_pcm_v && !f1_full && !f1_pend) begin f1_wr<=1'b1; f1_pend<=1'b1; end
        end
    end

    // ==================================================================
    //  B 域: FIFO#1 读(rd_pend) -> Ring Producer -> Ring -> Ring Consumer -> FIFO#2
    // ==================================================================
    reg rp_psk_v=0; reg [31:0] rp_psk=0; reg rp_pend=0;
    reg ring_pv=0; reg [31:0] ring_pd=0; wire ring_pready;
    always @(posedge clk_b or posedge rst_b_all) begin
        if (rst_b_all) begin f1_rd<=0; rp_psk_v<=0; rp_psk<=0; rp_pend<=0; ring_pv<=0; ring_pd<=0; end
        else begin
            f1_rd   <= 1'b0;
            ring_pv <= 1'b0;
            if (f1_rdv) begin rp_psk_v<=1'b1; rp_psk<=f1_dout; rp_pend<=1'b0; end
            if (!rp_psk_v && !rp_pend && !f1_empty) begin f1_rd<=1'b1; rp_pend<=1'b1; end
            if (rp_psk_v && ring_pready) begin
                ring_pv <= 1'b1; ring_pd <= rp_psk; rp_psk_v <= 1'b0;
            end
        end
    end

    reg ring_creq=0; wire ring_cvalid; wire [31:0] ring_cdata;
    wire ring_empty;
    audio_sdram_ring_buffer #(
        .FRAMES(RING_FRAMES), .ADDRW(RING_AW), .LOW(RING_LOW),
        .HIGH(RING_HIGH), .OFDEPTH(8)
    ) u_ring (
        .clk(clk_b), .rst(rst_b_all),
        .producer_valid(ring_pv), .producer_data(ring_pd), .producer_ready(ring_pready),
        .consumer_req(ring_creq), .consumer_valid(ring_cvalid), .consumer_data(ring_cdata),
        .used_frames(o_ring_used), .empty(ring_empty), .full(),
        .almost_empty(), .almost_full(), .underflow(), .overflow(),
        .underflow_count(), .overflow_count(),
        .mem_wr_req(ring_mem_wr_req), .mem_wr_addr(ring_mem_wr_addr),
        .mem_wr_data(ring_mem_wr_data), .mem_wr_ready(ring_mem_wr_ready),
        .mem_rd_req(ring_mem_rd_req), .mem_rd_addr(ring_mem_rd_addr),
        .mem_rd_data(ring_mem_rd_data), .mem_rd_valid(ring_mem_rd_valid)
    );

    reg  rp2_psk_v=0; reg [31:0] rp2_psk=0; reg rp2_pend=0;
    reg  f2_wr=0, f2_pend=0; reg [31:0] f2_din=0; wire f2_wack;
    wire f2_full, f2_afull, f2_ovf;
    wire [31:0] f2_dout; wire f2_rdv, f2_empty, f2_aempty, f2_und;
    wire f2_rd;   // FIFO#2 读请求, 由 wav_fifo2_consumer.O_fifo_rd 驱动
    audio_async_fifo #(.DATA_W(32),.DEPTH(AUDIO_FIFO_DEPTH),.ADDR_W(6),
                       .AEMPTY_LEV(2),.AFULL_LEV(2)) u_fifo2 (
        .clk_w(clk_b), .rst_w_n(~rst_b_all), .i_wr(f2_wr), .i_data(f2_din), .o_wr_acked(f2_wack),
        .o_full(f2_full), .o_almost_full(f2_afull), .o_overflow(f2_ovf),
        .clk_r(clk_c), .rst_r_n(~rst_c_all), .i_rd(f2_rd), .o_data(f2_dout),
        .o_rd_valid(f2_rdv), .o_empty(f2_empty), .o_almost_empty(f2_aempty),
        .o_underflow(f2_und)
    );
    always @(posedge clk_b or posedge rst_b_all) begin
        if (rst_b_all) begin ring_creq<=0; rp2_psk_v<=0; rp2_psk<=0; rp2_pend<=0;
                           f2_wr<=0; f2_pend<=0; f2_din<=0; end
        else begin
            ring_creq <= 1'b0;
            f2_wr     <= 1'b0;
            if (ring_cvalid) begin rp2_psk_v<=1'b1; rp2_psk<=ring_cdata; rp2_pend<=1'b0; end
            if (!rp2_psk_v && !rp2_pend && !ring_empty) begin ring_creq<=1'b1; rp2_pend<=1'b1; end
            if (rp2_psk_v && !f2_pend && !f2_full) begin
                f2_wr<=1'b1; f2_din<=rp2_psk; f2_pend<=1'b1; rp2_psk_v<=1'b0;
            end
            if (f2_wack) f2_pend<=1'b0;
        end
    end

    // ==================================================================
    //  C 域: 状态 CDC + ctl + 48k tick + consumer + mux
    // ==================================================================
    // A 域先打一拍; found 每轮粘性锁存(loc_found 非粘性, ctl 需要稳定 found)
    reg af_f=0, af_e=0, af_d=0;
    reg found_l=0;
    always @(posedge clk_a or posedge rst_a_all) begin
        if (rst_a_all) begin af_f<=0; af_e<=0; af_d<=0; found_l<=0; end
        else begin
            found_l <= found_l | wav_found;
            af_f<=found_l; af_e<=wav_err; af_d<=wav_done;
        end
    end
    // Decoder EOF is not playback EOF: drain all A/B buffers into FIFO2.
    reg bd1=0, bd2=0, drained_b=0;
    always @(posedge clk_b or posedge rst_b_all) begin
        if (rst_b_all) begin bd1<=0; bd2<=0; drained_b<=0; end
        else begin
            bd1<=af_d; bd2<=bd1;
            drained_b <= bd2 && f1_empty && !f1_rd && !f1_rdv &&
                !rp_pend && !rp_psk_v && !ring_pv && ring_pready &&
                ring_empty && !ring_creq && !ring_cvalid &&
                !rp2_pend && !rp2_psk_v && !f2_wr && !f2_pend;
        end
    end
    reg cf1=0,cf2=0, ce1=0,ce2=0, cd1=0,cd2=0;
    always @(posedge clk_c or posedge rst_c_all) begin
        if (rst_c_all) begin cf1<=0;cf2<=0;ce1<=0;ce2<=0;cd1<=0;cd2<=0; end
        else begin cf1<=af_f; cf2<=cf1; ce1<=af_e; ce2<=ce1; cd1<=drained_b; cd2<=cd1; end
    end
    wire wav_found_c = cf2;
    wire wav_err_c   = ce2;
    wire wav_done_c  = cd2;

    reg  first_frame_d1=0;
    // i_first_frame_ready(video 域电平) -> clk_c 同步寄存器(带复位, 单时钟边沿)
    // 注: V27S2 起音频启动不再看首帧, 此寄存器仅保留(未使用, 综合会优化掉)
    always @(posedge clk_c or posedge rst_c_all) begin
        if (rst_c_all) first_frame_d1 <= 1'b0;
        else       first_frame_d1 <= i_first_frame_ready;
    end

    // Start network playback after a cushion has accumulated. For short files,
    // producer EOF releases the gate even when the target cannot be reached.
    reg prefilled_b=0, prefilled_c1=0, prefilled_c2=0;
    always @(posedge clk_b or posedge rst_b_all) begin
        if(rst_b_all) prefilled_b<=0;
        else if(o_ring_used>=NETWORK_PREFILL_FRAMES || bd2) prefilled_b<=1;
    end
    always @(posedge clk_c or posedge rst_c_all) begin
        if(rst_c_all) begin prefilled_c1<=0; prefilled_c2<=0; end
        else begin prefilled_c1<=prefilled_b; prefilled_c2<=prefilled_c1; end
    end
    wire [7:0] wav_used_c = (!f2_empty && (!source_c || prefilled_c2)) ? 8'd1 : 8'd0;
    wire wav_play_done_w, unlock_w, wav_play_w;
    wire [2:0] ctl_ph; wire [1:0] sel_c;
    wire       au_music_start, au_wav_search;
    wire [2:0] au_digit;
    // Treat a source change as a soft reset of the playback controller.  This
    // restarts WAV discovery for the newly selected source while preserving
    // boot_flow_done, so startup music is not replayed after the first boot.
    wire ctl_rst_soft = rst_soft_c & ~source_change_c;
    audio_startup_wav_ctl #(.THRESH_FRAMES(1), .WCNT_W(8), .AUDIO_TEST_MODE(AUDIO_TEST_MODE))
        u_ctl (
        .clk(clk_c), .rst_n(~rst_c), .rst_soft(ctl_rst_soft),
        .I_sd_init_done      (source_c ? 1'b1 : i_sd_init_done),
        .I_sample_enable     (i_audio_valid),    // 48k tick: ms 级延时计数
        .I_startup_done      (i_startup_done),   // Music done -> 搜 WAV
        .I_skip_boot_flow    (source_c),          // do not delay UDP WACKs
        .I_wav_found         (wav_found_c), .I_wav_error(wav_err_c),
        .I_wav_used          (wav_used_c), .I_wav_play_done(wav_play_done_w),
        .I_first_frame_ready (first_frame_d1),   // 兼容端口(不再影响启动)
        .I_no_bmp            (i_no_bmp),         // 兼容端口(不再影响启动)
        .O_audio_sel         (sel_c),
        .O_wav_play          (wav_play_w),
        .O_music_start       (au_music_start),
        .O_wav_search        (au_wav_search),
        .O_prefetch_start    (),
        .O_unlock_image      (unlock_w),
        .O_digit             (au_digit),
        .O_phase             (ctl_ph)
    );

    // WAV 预取启动: 仅在音频 FSM 进入 A_SEARCH(O_wav_search=1)后才启动一次。
    // A_WAIT_SD/A_DELAY/A_STARTUP_MUSIC 期间 O_wav_search=0 => WAV client 不申请 SD。
    reg  cph_c=0;
    always @(posedge clk_c or posedge rst_c_all) begin
        if (rst_c_all) cph_c <= 1'b0;
        else       cph_c <= au_wav_search;      // V27S2: 由 A_SEARCH 驱动(非相位!=0)
    end
    reg  ac_s0=0, ac_s1=0;
    wire wav_start_trig = ac_s1 | i_wav_start;  // i_wav_start 仅作可选外部触发(顶层接 0)
    always @(posedge clk_a or posedge rst_a_all) begin
        if (rst_a_all) begin ac_s0<=0; ac_s1<=0; started<=0; wav_start<=0; end
        else begin
            ac_s0 <= cph_c;  ac_s1 <= ac_s0;
            wav_start <= 1'b0;
            if (!started && wav_start_trig && (wav_st==4'd0)) begin
                wav_start <= 1'b1;
                started   <= 1'b1;
            end
        end
    end

    // 48k sample tick = 外部 audio_valid(video 域); 不新建第二 tick/clock
    wire wavc_v; wire signed [23:0] wavc_l, wavc_r;
    wav_fifo2_consumer u_wavc (
        .clk(clk_c), .rst(rst_c_all),
        .I_sample_enable(i_audio_valid), .I_enable(wav_play_w && !(PAUSE_ENABLE && i_pause)),
        .O_fifo_rd(f2_rd), .I_fifo_valid(f2_rdv), .I_fifo_data(f2_dout),
        .I_fifo_empty(f2_empty), .I_producer_done(wav_done_c),
        .O_pcm_valid(wavc_v), .O_left(wavc_l), .O_right(wavc_r),
        .O_play_done(wav_play_done_w), .O_gotframes(), .O_rd_pending()
    );
    assign o_wav_valid = wavc_v;
    assign o_wav_left  = wavc_l;
    assign o_wav_right = wavc_r;

    // audio_mux3 -> pcm_video_final; valid 对齐 video 域 48k audio_valid
    // mux 的 WAV 输入在无新帧时为 0(防陈旧重复); startup/tone 由上游按 tick 提供
    wire signed [23:0] wavmux_l = wavc_v ? wavc_l : 24'sd0;
    wire signed [23:0] wavmux_r = wavc_v ? wavc_r : 24'sd0;
    audio_mux3 u_mux (
        .I_sel(sel_c),
        .I_startup_left(i_startup_l), .I_startup_right(i_startup_r),
        .I_wav_left(wavmux_l),  .I_wav_right(wavmux_r),
        .I_tone_left(i_tone_l), .I_tone_right(i_tone_r),
        .O_left(o_pcm_left), .O_right(o_pcm_right)
    );
    assign o_pcm_valid = i_audio_valid;

    assign o_audio_sel    = sel_c;
    assign o_wav_play     = wav_play_w;
    assign o_unlock_image = unlock_w;
    assign o_ctl_phase    = ctl_ph;
    assign o_music_start  = au_music_start;   // V27S2: 启动 Startup Music one-shot
    assign o_wav_search   = au_wav_search;    // V27S2: A_SEARCH 电平
    assign o_audio_digit  = au_digit;         // V27S2: 0..4 音频启动 debug
    assign o_wav_ready    = !f2_empty;
    assign o_wav_found_c  = wav_found_c;
    assign o_wav_err_c    = wav_err_c;
    assign o_play_finished = wav_play_done_w;
    assign o_wav_done_c   = wav_done_c;
    assign o_fifo1_full   = f1_full;
    assign o_fifo1_empty  = f1_empty;
    assign o_fifo2_full   = f2_full;
    assign o_fifo2_empty  = f2_empty;
endmodule

// FAT32 WAV file source with byte-level backpressure. This module deliberately
// stops at the file-stream boundary so the WAV parser and PCM extractor can be
// shared with the Ethernet input path.
module tf_wav_file_source #(
    parameter integer DYNAMIC_NAME = 0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        I_start,
    input  wire [87:0] I_target_name,

    output wire        sd_sec_read,
    output wire [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,

    output wire [7:0]  file_byte,
    output wire        file_valid,
    input  wire        file_ready,
    output wire [31:0] file_size,
    output wire        file_end,
    output wire [15:0] wav_count, wav_index,
    output reg         file_found,
    output reg         error,
    output reg  [3:0]  error_code,
    output wire [3:0]  state
);
    localparam [3:0] S_IDLE         = 4'd0;
    localparam [3:0] S_LOC_START    = 4'd1;
    localparam [3:0] S_LOC_ARM      = 4'd2;
    localparam [3:0] S_LOC_WAIT     = 4'd3;
    localparam [3:0] S_STREAM_START = 4'd4;
    localparam [3:0] S_STREAM_ARM   = 4'd5;
    localparam [3:0] S_STREAM       = 4'd6;
    localparam [3:0] S_DONE         = 4'd7;
    localparam [3:0] S_ERROR        = 4'd8;

    reg [3:0] st;
    reg loc_start, stream_start;
    reg [31:0] first_cluster_r, file_size_r;

    wire loc_read, loc_done, loc_error, loc_found;
    wire [31:0] loc_addr, loc_size, loc_first_cluster;
    wire [3:0] loc_error_code;
    wire stream_read, stream_valid, stream_end, stream_done, stream_error;
    wire [31:0] stream_addr;
    wire [7:0] stream_byte;
    wire [3:0] stream_error_code;

    fat32_file_locator #(.DYNAMIC_NAME(DYNAMIC_NAME), .LAZY_CHAIN(1)) u_locator (
        .clk(clk), .rst(rst), .I_start(loc_start), .I_target_name(I_target_name),
        .sd_sec_read(loc_read), .sd_sec_read_addr(loc_addr),
        .sd_sec_read_data(sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .wav_count(wav_count), .wav_index(wav_index),
        .file_found(loc_found), .file_size(loc_size),
        .first_cluster(loc_first_cluster), .file_clusters(),
        .done(loc_done), .busy(), .error(loc_error),
        .error_code(loc_error_code), .state()
    );

    fat32_file_stream_reader u_stream (
        .clk(clk), .rst(rst), .I_start(stream_start),
        .I_file_first_cluster(first_cluster_r), .I_file_size(file_size_r),
        .sd_sec_read(stream_read), .sd_sec_read_addr(stream_addr),
        .sd_sec_read_data(sd_sec_read_data),
        .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end),
        .O_byte_valid(stream_valid), .O_byte_data(stream_byte),
        .O_byte_ready((st == S_STREAM) && file_ready),
        .O_file_end(stream_end), .O_done(stream_done), .O_busy(),
        .O_error(stream_error), .O_error_code(stream_error_code),
        .O_byte_cnt(), .O_state()
    );

    assign sd_sec_read = ((st == S_LOC_START) || (st == S_LOC_ARM) ||
                          (st == S_LOC_WAIT)) ? loc_read : stream_read;
    assign sd_sec_read_addr = ((st == S_LOC_START) || (st == S_LOC_ARM) ||
                               (st == S_LOC_WAIT)) ? loc_addr : stream_addr;
    assign file_byte = stream_byte;
    assign file_valid = (st == S_STREAM) && stream_valid;
    assign file_size = file_size_r;
    assign file_end = stream_end;
    assign state = st;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st <= S_IDLE;
            loc_start <= 1'b0;
            stream_start <= 1'b0;
            first_cluster_r <= 32'd0;
            file_size_r <= 32'd0;
            file_found <= 1'b0;
            error <= 1'b0;
            error_code <= 4'd0;
        end else begin
            loc_start <= 1'b0;
            stream_start <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (I_start) begin
                        file_found <= 1'b0;
                        error <= 1'b0;
                        error_code <= 4'd0;
                        st <= S_LOC_START;
                    end
                end
                S_LOC_START: begin
                    loc_start <= 1'b1;
                    st <= S_LOC_ARM;
                end
                // Give the locator one full cycle to clear a sticky done flag.
                S_LOC_ARM: st <= S_LOC_WAIT;
                S_LOC_WAIT: begin
                    if (loc_done) begin
                        if (loc_error || !loc_found) begin
                            error <= 1'b1;
                            error_code <= loc_error ? loc_error_code : 4'd5;
                            st <= S_ERROR;
                        end else begin
                            first_cluster_r <= loc_first_cluster;
                            file_size_r <= loc_size;
                            file_found <= 1'b1;
                            st <= S_STREAM_START;
                        end
                    end
                end
                S_STREAM_START: begin
                    stream_start <= 1'b1;
                    st <= S_STREAM_ARM;
                end
                // Give the reader one full cycle to clear sticky done/error flags.
                S_STREAM_ARM: st <= S_STREAM;
                S_STREAM: begin
                    if (stream_error) begin
                        error <= 1'b1;
                        error_code <= stream_error_code;
                        st <= S_ERROR;
                    end else if (stream_done) begin
                        st <= S_DONE;
                    end
                end
                S_DONE:  st <= S_IDLE;
                S_ERROR: st <= S_IDLE;
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
