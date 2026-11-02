
module d5_sd_card_bmp #(
    parameter integer FAST_SCAN = 0,
    parameter integer KEYS_DEBOUNCED = 0,
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [7:0]   SCAN_TARGET_COUNT = 32'd0,

    // G1-C3-D3-B: split the 2M x 32 SDRAM into two equal 1M-word buffers.
    parameter [20:0]  BUFFER_WORDS      = 21'd1048576
)(
    input                       clk,
    input                       rst,
    input touch_valid,input [3:0] touch_id,
    input                       key_next,
    input                       key_auto,
    
    // ===================== G1-C1 Fade transition handshake =====================
    // These two inputs are already synchronized into sd_card_clk
    // by the top-level CDC logic.
    input                       fade_midpoint_pulse,
    input                       fade_done_pulse,
    
    // Toggle events sent from sd_card_clk to video_clk.
    output reg                  fade_start_toggle,
    output reg                  image_ready_toggle,
    
    output [3:0]                state_code,
    // ===================== G1-B2 OSD status =====================
    output                      osd_auto_play,
    output [7:0]                osd_img_idx,
    output [7:0]                osd_img_count,

    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,

    // ===================== G1-C3-D3-B source / fit metadata =====================
    output [15:0]               adaptive_src_width,
    output [15:0]               adaptive_src_height,
    output [20:0]               adaptive_write_len,
    output [15:0]               adaptive_dst_width,
    output [15:0]               adaptive_dst_height,
    output [15:0]               adaptive_x_offset,
    output [15:0]               adaptive_y_offset,
    output                      adaptive_geometry_valid,
    
    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,
    output reg                  disp_buf_commit_toggle,
    input                       disp_buf_commit_ack_toggle,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    input wire sd_init_done,
    output wire sd_sec_read,
    output wire [31:0] sd_sec_read_addr,
    input wire [7:0] sd_sec_read_data,
    input wire sd_sec_read_data_valid, sd_sec_read_end,
    output wire O_scan_done, O_bmp_found
);

wire key_next_press;
wire touch_prev=touch_valid && touch_id==3;
wire manual_next=key_next_press || (touch_valid && touch_id==5);
wire manual_request=manual_next || touch_prev;
reg pending_previous;
always @(posedge clk or posedge rst) begin
 if(rst) pending_previous<=0;
 else if(manual_request) pending_previous<=touch_prev;
end
wire key_auto_press;

wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             bmp_ready;
wire [3:0]       bmp_state_code;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [7:0]       scan_found_total;

// ===================== G1-C3-D3-A adaptive fit handshake =====================
// New outputs from G1-C3-D2 d5_bmp_read.
wire [15:0]      scan_found_width;
wire [15:0]      scan_found_height;
wire [15:0]      load_image_width;
wire [15:0]      load_image_height;

// Keep d5_bmp_read's original write request internal.
// The external frame-write request will be released only after
// adaptive_image_fit has finished calculating geometry.
wire             bmp_write_req_raw;
wire             bmp_write_req_ack_gated;

reg              fit_start;
reg              fit_request_seen;
reg              fit_ready_latched;

wire             fit_busy;
wire             fit_done;
wire             fit_valid;
wire [15:0]      fit_dst_width;
wire [15:0]      fit_dst_height;
wire [15:0]      fit_x_offset;
wire [15:0]      fit_y_offset;

// ===================== G1-C3-D3-B dynamic source length / safety =====================
wire [31:0]      scan_width_ext;
wire [31:0]      scan_height_ext;
wire [31:0]      scan_pixel_count_full;
wire             scan_image_supported;

wire [31:0]      load_width_ext;
wire [31:0]      load_height_ext;
wire [31:0]      load_pixel_count_full;
wire             load_image_supported;

reg  [15:0]      adaptive_src_width_r;
reg  [15:0]      adaptive_src_height_r;
reg  [20:0]      adaptive_write_len_r;
reg  [15:0]      adaptive_dst_width_r;
reg  [15:0]      adaptive_dst_height_r;
reg  [15:0]      adaptive_x_offset_r;
reg  [15:0]      adaptive_y_offset_r;
reg              adaptive_geometry_valid_r;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
// This counter is cleared at CLK_FREQ_HZ-1 and on every disabled interval.
localparam integer AUTO_BITS=CLK_FREQ_HZ<1 ? 32:CLK_FREQ_HZ<2 ? 1:$clog2(CLK_FREQ_HZ);
reg [AUTO_BITS-1:0] auto_cnt;
reg [7:0]        img_found_count;
reg [7:0]        img_idx;              // 当前真正显示中的图片编号
reg target_committed;
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0] selected_sector;
reg [7:0] scan_limit;
reg [1:0] scan_mode; // 0: first image; 1: full catalogue; 2: ordinal lookup
reg catalogue_started, catalogue_complete, lookup_valid, scan_done_d, discovery_ready;
wire scan_finished=scan_done && !scan_done_d;
always @(posedge clk or posedge rst)
 if(rst) scan_done_d<=0; else scan_done_d<=scan_done;




reg              next_req_pending;
reg              load_busy;
// ===================== G1-C1 transition controller =====================

// One complete transaction is:
//
// request
// -> fade out
// -> black midpoint
// -> load BMP
// -> SDRAM frame commit
// -> image ready
// -> fade in
// -> fade done
reg              transition_active;

// Image selected at the moment the transition starts.
// Keep it fixed throughout the complete transaction.
reg [7:0]        transition_target_idx;

// midpoint_pulse may arrive before d5_bmp_read becomes ready.
// Latch it so the request can never be lost.
reg              fade_midpoint_seen;

// After SDRAM write completion, wait until the display-buffer
// commit ACK really returns before allowing fade-in.
reg              waiting_commit_ack;
// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;

reg [2:0]        commit_ack_sync;
wire             commit_pending;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;
reg              write_finish_seen;

wire auto_tick;
wire [7:0] next_from_current;

// Full-width products for source pixel counts.
assign scan_width_ext  = {16'd0, scan_found_width};
assign scan_height_ext = {16'd0, scan_found_height};
assign scan_pixel_count_full = scan_width_ext * scan_height_ext;

assign scan_image_supported =
    (scan_found_width  != 16'd0) &&
    (scan_found_height != 16'd0) &&
    (scan_pixel_count_full <= {11'd0, BUFFER_WORDS});

assign load_width_ext  = {16'd0, load_image_width};
assign load_height_ext = {16'd0, load_image_height};
assign load_pixel_count_full = load_width_ext * load_height_ext;

assign load_image_supported =
    (load_image_width  != 16'd0) &&
    (load_image_height != 16'd0) &&
    (load_pixel_count_full <= {11'd0, BUFFER_WORDS});

assign adaptive_src_width      = adaptive_src_width_r;
assign adaptive_src_height     = adaptive_src_height_r;
assign adaptive_write_len      = adaptive_write_len_r;
assign adaptive_dst_width      = adaptive_dst_width_r;
assign adaptive_dst_height     = adaptive_dst_height_r;
assign adaptive_x_offset       = adaptive_x_offset_r;
assign adaptive_y_offset       = adaptive_y_offset_r;
assign adaptive_geometry_valid = adaptive_geometry_valid_r;

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};

// ===================== G1-C3-D3-A adaptive fit gate =====================
// d5_bmp_read stops in ST_LOAD_WAIT while bmp_write_req_raw is high.
// Do not let the SDRAM frame-write side ACK that request until
// adaptive_image_fit has produced valid geometry.
assign write_req =
    bmp_write_req_raw &&
    fit_ready_latched &&
    load_image_supported;

assign bmp_write_req_ack_gated =
    write_req_ack &&
    fit_ready_latched &&
    load_image_supported;

assign auto_tick  = (auto_cnt == (CLK_FREQ_HZ - 1));
assign next_from_current = next_index_limited(img_idx, img_found_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign commit_pending = commit_ack_sync[2] != disp_buf_commit_toggle;

// ===================== HWDBG1 temporary seven-segment diagnosis =====================
// Keep the original d5_bmp_read states 0/1/2/3/4 while the source is active.
// After d5_bmp_read returns to IDLE, expose the outer transaction state:
//   5 = BMP source finished, waiting for SDRAM write_finish
//   6 = SDRAM write_finish arrived first, waiting for BMP source completion
//   7 = first display buffer has been committed and display_valid is high
assign state_code = display_valid ? 4'd7 :
                    ((bmp_state_code == 4'd1) && load_busy && source_done_seen && !write_finish_seen) ? 4'd5 :
                    ((bmp_state_code == 4'd1) && load_busy && write_finish_seen && !source_done_seen) ? 4'd6 :
                    bmp_state_code;

// ===================== G1-B2 OSD status outputs =====================
// These signals remain in the sd_card_clk domain.
// They will be synchronized into video_clk in the top-level module.
assign osd_auto_play = auto_play_en;

// D9: OSD reports the activated image, never the requested/building image.
assign osd_img_idx = img_idx;

assign osd_img_count = img_found_count;
always @(posedge clk or posedge rst) begin
    if (rst)
        commit_ack_sync <= 3'b000;
    else
        commit_ack_sync <= {commit_ack_sync[1:0], disp_buf_commit_ack_toggle};
end

// ===================== G1-C3-D3-A adaptive geometry calculator =====================
//
// Sequence for every BMP load:
//
// d5_bmp_read parses real width/height
//      ->
// bmp_write_req_raw rises and d5_bmp_read waits in ST_LOAD_WAIT
//      ->
// start adaptive_image_fit
//      ->
// wait fit_done + fit_valid
//      ->
// fit_ready_latched = 1
//      ->
// external write_req is finally released
//      ->
// frame-write side ACK returns to d5_bmp_read
//      ->
// BMP pixel streaming may begin
//
// This intentionally changes NO Fade / double-buffer / commit logic.

adaptive_image_fit u_adaptive_image_fit (
    .clk        (clk),
    .rst        (rst),

    .start      (fit_start),

    .src_width  (load_image_width),
    .src_height (load_image_height),

    .busy       (fit_busy),
    .done       (fit_done),
    .valid      (fit_valid),

    .dst_width  (fit_dst_width),
    .dst_height (fit_dst_height),

    .x_offset   (fit_x_offset),
    .y_offset   (fit_y_offset)
);


// Generate exactly one fit_start pulse for each d5_bmp_read write request.
// The fit outputs remain stable inside adaptive_image_fit after completion.

always @(posedge clk or posedge rst) begin

    if (rst) begin

        fit_start         <= 1'b0;
        fit_request_seen  <= 1'b0;
        fit_ready_latched <= 1'b0;

        adaptive_src_width_r      <= 16'd0;
        adaptive_src_height_r     <= 16'd0;
        adaptive_write_len_r      <= 21'd0;
        adaptive_dst_width_r      <= 16'd0;
        adaptive_dst_height_r     <= 16'd0;
        adaptive_x_offset_r       <= 16'd0;
        adaptive_y_offset_r       <= 16'd0;
        adaptive_geometry_valid_r <= 1'b0;

    end
    else if (!sd_init_done) begin

        fit_start         <= 1'b0;
        fit_request_seen  <= 1'b0;
        fit_ready_latched <= 1'b0;

        adaptive_src_width_r      <= 16'd0;
        adaptive_src_height_r     <= 16'd0;
        adaptive_write_len_r      <= 21'd0;
        adaptive_dst_width_r      <= 16'd0;
        adaptive_dst_height_r     <= 16'd0;
        adaptive_x_offset_r       <= 16'd0;
        adaptive_y_offset_r       <= 16'd0;
        adaptive_geometry_valid_r <= 1'b0;

    end
    else begin

        // one-clock pulse
        fit_start <= 1'b0;

        // No active d5_bmp_read write request:
        // prepare for the next image.
      if (!bmp_write_req_raw) begin

            fit_request_seen          <= 1'b0;
            fit_ready_latched         <= 1'b0;
            adaptive_geometry_valid_r <= 1'b0;
        
        end     
        else begin

            // First cycle of a new BMP write request.
            // load_image_width/height are already latched by d5_bmp_read.
            if (!fit_request_seen) begin

                fit_start         <= 1'b1;
                fit_request_seen  <= 1'b1;
                fit_ready_latched <= 1'b0;
                adaptive_geometry_valid_r <= 1'b0;

            end

            // Release the frame-write request only when the geometry is
            // valid and the raw source fits inside one SDRAM buffer.
            if (fit_done && fit_valid && load_image_supported) begin

                fit_ready_latched <= 1'b1;

                adaptive_src_width_r  <= load_image_width;
                adaptive_src_height_r <= load_image_height;
                adaptive_write_len_r  <= load_pixel_count_full[20:0];

                adaptive_dst_width_r  <= fit_dst_width;
                adaptive_dst_height_r <= fit_dst_height;
                adaptive_x_offset_r   <= fit_x_offset;
                adaptive_y_offset_r   <= fit_y_offset;

                adaptive_geometry_valid_r <= 1'b1;

            end

        end

    end

end


d5_key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (KEYS_DEBOUNCED ? 0 : 20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

d5_key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (KEYS_DEBOUNCED ? 0 : 20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

function [7:0] next_index_limited;
 input [7:0] cur,count;
 reg[7:0] incremented;
 begin incremented=cur+1'b1;next_index_limited=(count<=1 || incremented==count) ? 32'd0 : incremented; end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync         <= 3'b000;
        scan_start_pulse       <= 1'b0;
        load_start_pulse       <= 1'b0;
        load_sector            <= 32'd0;
        scan_kicked            <= 1'b0;
        first_image_committed  <= 1'b0;
        auto_play_en           <= 1'b0;
        auto_cnt               <= 32'd0;
        img_found_count        <= 3'd0;
        img_idx                <= 2'd0;
        target_committed <= 1'b0;
        pending_buf_idx        <= 2'd0;
        write_buf_idx          <= 2'd0;
        disp_buf_idx           <= 2'd0;
        disp_buf_commit_toggle <= 1'b0;

        selected_sector<=0;scan_limit<=1;scan_mode<=0;
        catalogue_started<=0;catalogue_complete<=0;lookup_valid<=0;discovery_ready<=0;
        next_req_pending       <= 1'b0;
        load_busy              <= 1'b0;
        source_done_seen       <= 1'b0;
        write_finish_seen      <= 1'b0;
        display_valid          <= 1'b0;

        // G1-C1
        fade_start_toggle      <= 1'b0;
        image_ready_toggle     <= 1'b0;
        transition_active      <= 1'b0;
        transition_target_idx  <= 2'd0;
        fade_midpoint_seen     <= 1'b0;
        waiting_commit_ack     <= 1'b0;

    end
    else begin

        // -----------------------------------------------------
        // Synchronize SDRAM write-finish toggle.
        // -----------------------------------------------------

        wrfin_tgl_sync    <= {
            wrfin_tgl_sync[1:0],
            write_finish_toggle
        };

        scan_start_pulse  <= 1'b0;
        load_start_pulse  <= 1'b0;


        // =====================================================
        // SD card not ready
        // =====================================================

        if (!sd_init_done) begin

            scan_kicked            <= 1'b0;
            first_image_committed  <= 1'b0;
            auto_play_en           <= 1'b0;
            auto_cnt               <= 32'd0;
            img_found_count        <= 3'd0;
            img_idx                <= 2'd0;
            target_committed <= 1'b0;
            pending_buf_idx        <= 2'd0;
            write_buf_idx          <= 2'd0;
            disp_buf_idx           <= 2'd0;

        selected_sector<=0;scan_limit<=1;scan_mode<=0;
        catalogue_started<=0;catalogue_complete<=0;lookup_valid<=0;discovery_ready<=0;
            next_req_pending       <= 1'b0;
            load_busy              <= 1'b0;
            source_done_seen       <= 1'b0;
            write_finish_seen      <= 1'b0;
            display_valid          <= 1'b0;

            // Abort any incomplete transition transaction.
            //
            // Do NOT force the event toggles back to zero here.
            // Only global reset initializes toggle phases.
            transition_active      <= 1'b0;
            transition_target_idx  <= 2'd0;
            fade_midpoint_seen     <= 1'b0;
            waiting_commit_ack     <= 1'b0;

        end
        else begin

            // =================================================
            // BMP scan result
            // =================================================

            if (scan_found_valid && scan_image_supported) begin
                if(scan_mode!=2) img_found_count<=scan_found_total;
                if(scan_mode==0) selected_sector<=scan_found_sector;
                if(scan_mode==2 && scan_found_total==scan_limit) begin
                    selected_sector<=scan_found_sector;
                    lookup_valid<=1;
                end
            end
            if(scan_finished && scan_mode==0) discovery_ready<=1;
            if(scan_finished && scan_mode==1) catalogue_complete<=1;
            // Preserve the current frame if a selected file disappears or a read fails.
            if(scan_finished && scan_mode==2 && !lookup_valid &&
               !(scan_found_valid && scan_found_total==scan_limit)) begin
                fade_midpoint_seen<=0;
                image_ready_toggle<=~image_ready_toggle;
            end

            // =================================================
            // BMP source-side completion
            //
            // bmp_ready only means source -> FIFO is finished.
            // Do NOT commit display buffer yet.
            // =================================================

            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            // HWDBG1 / robustness fix:
            // write_finish_pulse is a one-cycle CDC pulse.  Latch it independently
            // so it cannot be lost if it arrives before bmp_ready/source_done_seen.
            if (!load_busy)
                write_finish_seen <= 1'b0;
            else if (write_finish_pulse)
                write_finish_seen <= 1'b1;


            // =================================================
            // G1-C1: latch black midpoint event
            //
            // Do not depend on bmp_ready being high in exactly
            // the same clock cycle as the CDC pulse.
            // =================================================

            if (
                transition_active &&
                fade_midpoint_pulse
            ) begin

                fade_midpoint_seen <= 1'b1;

            end


            // =================================================
            // SDRAM whole-frame write completion
            // =================================================

            if (
                load_busy &&
                source_done_seen &&
                write_finish_seen
            ) begin

                load_busy         <= 1'b0;
                source_done_seen  <= 1'b0;
                write_finish_seen <= 1'b0;

                // Request display-buffer switch.
                disp_buf_idx <= pending_buf_idx;

                disp_buf_commit_toggle <=
                    ~disp_buf_commit_toggle;

                first_image_committed <= 1'b1;


                // ---------------------------------------------
                // Normal first-image loading:
                //
                // no Fade transaction exists yet, so preserve
                // original behavior.
                // ---------------------------------------------

                waiting_commit_ack <= 1'b1;

                // ---------------------------------------------
                // Fade transition:
                //
                // DO NOT release black screen yet.
                //
                // Wait until frame_read_write acknowledges
                // the display-buffer commit.
                // ---------------------------------------------


            end


            // =================================================
            // G1-C1: display-buffer commit ACK received
            //
            // commit_pending becomes:
            //
            //   0
            //
            // only after the new buffer selection has actually
            // been acknowledged by the display side.
            // =================================================

            if (
                waiting_commit_ack &&
                !commit_pending
            ) begin

                waiting_commit_ack <= 1'b0;

                // Now the new image is genuinely the committed
                // display image.
                // The displayed-image index changes only after the chosen
                // transition has completed. First-image boot has no animation.
                if(!transition_active) img_idx <= 32'd0;
                else target_committed<=1'b1;
                display_valid <= 1'b1;

                // Tell video_clk domain that it may release
                // black hold and begin Fade In.
                if (transition_active)
                    image_ready_toggle <= ~image_ready_toggle;

            end


            // =================================================
            // G1-C1: complete transaction only after Fade In
            // =================================================

            if (
                transition_active &&
                fade_done_pulse
            ) begin
                if(target_committed) img_idx <= transition_target_idx;

                transition_active  <= 1'b0;
                fade_midpoint_seen <= 1'b0;
                waiting_commit_ack <= 1'b0;

                // Restart AUTO timing only after Fade In has
                // completely finished.
                auto_cnt <= 32'd0;

            end


            // =================================================
            // Initial scan kickoff
            // =================================================

            if (!scan_kicked && bmp_ready) begin

                scan_start_pulse       <= 1'b1;
                scan_limit<=1;scan_mode<=0;catalogue_started<=0;catalogue_complete<=0;lookup_valid<=0;discovery_ready<=0;
                scan_kicked            <= 1'b1;
                first_image_committed  <= 1'b0;
                auto_play_en           <= 1'b0;
                auto_cnt               <= 32'd0;
                img_found_count        <= 3'd0;
                img_idx                <= 2'd0;
                target_committed <= 1'b0;
                pending_buf_idx        <= 2'd0;
                write_buf_idx          <= 2'd0;
                disp_buf_idx           <= 2'd0;
                next_req_pending       <= 1'b0;
                display_valid          <= 1'b0;
                load_busy              <= 1'b0;
                source_done_seen       <= 1'b0;
                write_finish_seen      <= 1'b0;

                transition_active      <= 1'b0;
                transition_target_idx  <= 2'd0;
                fade_midpoint_seen     <= 1'b0;
                waiting_commit_ack     <= 1'b0;

            end
            else begin

                if(first_image_committed && !catalogue_started && bmp_ready &&
                   !load_busy && !commit_pending && !transition_active) begin
                    scan_start_pulse<=1;scan_limit<=0;scan_mode<=1;
                    catalogue_started<=1;catalogue_complete<=0;
                end

                // =============================================
                // Physical K2:
                // manual "next image"
                //
                // Keep at most one queued request.
                // =============================================

                if (
                    manual_request &&
                    first_image_committed &&
                    (img_found_count > 3'd0)
                ) begin

                    next_req_pending <= 1'b1;

                end


                // =============================================
                // Physical K4:
                // AUTO playback ON / OFF
                // =============================================

                if (
                    (key_auto_press || (touch_valid && touch_id==4)) &&
                    scan_done &&
                    (img_found_count > 3'd1)
                ) begin

                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;

                end


                // =============================================
                // AUTO 1-second timer
                //
                // Critical:
                //
                // Do NOT count during Fade Out,
                // black hold, BMP load, commit wait,
                // or Fade In.
                // =============================================

                if (
                    scan_done &&
                    auto_play_en &&
                    display_valid &&
                    !load_busy &&
                    !commit_pending &&
                    !transition_active &&
                    !waiting_commit_ack &&
                    first_image_committed &&
                    (img_found_count > 3'd1)
                ) begin

                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <=
                            auto_cnt + 32'd1;

                end
                else begin

                    auto_cnt <= 32'd0;

                end


                // =============================================
                // First image
                //
                // Power-on first image keeps the original
                // direct-load behavior. There is nothing on
                // screen to fade out yet.
                // =============================================

                if (
                    scan_done &&
                    !first_image_committed &&
                    bmp_ready &&
                    !load_busy &&
                    (img_found_count != 3'd0)
                ) begin

                    target_committed <= 1'b0;
                    load_sector       <= selected_sector;
                    pending_buf_idx   <= 2'd0;
                    write_buf_idx     <= 2'd0;

                    load_start_pulse  <= 1'b1;
                    load_busy         <= 1'b1;

                    source_done_seen  <= 1'b0;
                    write_finish_seen <= 1'b0;
                    next_req_pending  <= 1'b0;
                    auto_cnt          <= 32'd0;

                end


                // =============================================
                // G1-C1:
                // Fade reached BLACK.
                //
                // NOW start loading the selected next image.
                // =============================================

                else if (
                    transition_active &&
                    lookup_valid &&
                    fade_midpoint_seen &&
                    bmp_ready &&
                    !load_busy &&
                    !commit_pending &&
                    !waiting_commit_ack
                ) begin

                    load_sector <=
                        selected_sector;

                    pending_buf_idx <=
                        next_buf_lut(
                            disp_buf_idx,
                            display_valid
                        );

                    write_buf_idx <=
                        next_buf_lut(
                            disp_buf_idx,
                            display_valid
                        );

                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;

                    source_done_seen  <= 1'b0;
                    write_finish_seen <= 1'b0;

                    // Consume the remembered midpoint event.
                    fade_midpoint_seen <= 1'b0;

                    auto_cnt <= 32'd0;

                end


                // =============================================
                // Physical K2 manual request
                //
                // OLD:
                //     immediately load BMP
                //
                // NEW:
                //     only start Fade Out
                // =============================================

                else if (
                    scan_done &&
                    catalogue_complete &&
                    bmp_ready &&
                    display_valid &&
                    !load_busy &&
                    !commit_pending &&
                    !transition_active &&
                    next_req_pending &&
                    (img_found_count != 3'd0)
                ) begin

                    transition_target_idx <=
                        pending_previous ? (img_idx==0 ? img_found_count-1'b1 : img_idx-1'b1) : next_from_current;
                    scan_start_pulse<=1;scan_mode<=2;lookup_valid<=0;
                    scan_limit<=pending_previous ? (img_idx==0 ? img_found_count : img_idx) : next_from_current+1'b1;

                    transition_active <= 1'b1;
                    target_committed<=1'b0;

                    fade_midpoint_seen <= 1'b0;

                    // SD -> VIDEO event.
                    fade_start_toggle <=
                        ~fade_start_toggle;

                    // Consume this request.
                    // Simultaneous consume and a new press retains one request.
                    next_req_pending <= manual_request;

                    auto_cnt <= 32'd0;

                end


                // =============================================
                // Physical K4 AUTO transition
                //
                // AUTO timeout now starts Fade Out rather than
                // directly starting BMP loading.
                // =============================================

                else if (
                    scan_done &&
                    catalogue_complete &&
                    bmp_ready &&
                    display_valid &&
                    !load_busy &&
                    !commit_pending &&
                    !transition_active &&
                    auto_play_en &&
                    auto_tick &&
                    (img_found_count > 3'd1)
                ) begin

                    transition_target_idx <=
                        next_from_current;
                    scan_start_pulse<=1;scan_mode<=2;lookup_valid<=0;scan_limit<=next_from_current+1'b1;

                    transition_active <= 1'b1;
                    target_committed<=1'b0;

                    fade_midpoint_seen <= 1'b0;

                    // SD -> VIDEO event.
                    fade_start_toggle <=
                        ~fade_start_toggle;

                    auto_cnt <= 32'd0;

                end

            end

        end

    end

end
// Fast FAT32 catalogue owns the port only during discovery; pixel loading
// still uses the proven bmp_read/SDRAM path. Raw-card fallback is unchanged.
wire raw_ready,raw_done,raw_found,raw_rd;
wire [31:0] raw_found_sector,raw_addr;
wire [7:0] raw_total;
wire [3:0] raw_state;
wire [15:0] raw_width,raw_height,fast_width,fast_height;
wire fast_busy,fast_done,fast_fallback,fast_found,fast_rd;
wire [31:0] fast_sector,fast_addr;
wire [7:0] fast_total;
reg fast_done_d;
always @(posedge clk or posedge rst)
 if(rst) fast_done_d<=0;else fast_done_d<=fast_done;
wire fast_start=FAST_SCAN && scan_start_pulse;
wire raw_start=FAST_SCAN ? (fast_done && !fast_done_d && fast_fallback) : scan_start_pulse;
wire use_fast=FAST_SCAN && !fast_fallback;
assign bmp_ready=raw_ready && !fast_busy && !fast_start;
assign scan_done=use_fast ? fast_done : raw_done;
assign scan_found_valid=use_fast ? fast_found : raw_found;
assign scan_found_sector=use_fast ? fast_sector : raw_found_sector;
assign scan_found_total=use_fast ? fast_total : raw_total;
assign sd_sec_read=fast_busy ? fast_rd : raw_rd;
assign sd_sec_read_addr=fast_busy ? fast_addr : raw_addr;
assign bmp_state_code=fast_busy ? 4'd2 : raw_state;

assign scan_found_width=use_fast ? fast_width : raw_width;
assign scan_found_height=use_fast ? fast_height : raw_height;
bmp_fat_scanner #(.MAX_PIXELS({11'd0,BUFFER_WORDS})) fast_scan(
 .clk(clk),.rst(rst),.start(fast_start),.bmp_width(bmp_width),.bmp_height(bmp_height),
 .found_width(fast_width),.found_height(fast_height),
 .target_count(scan_limit),.busy(fast_busy),.done(fast_done),.fallback(fast_fallback),
 .found(fast_found),.found_sector(fast_sector),.found_count(fast_total),
 .rd(fast_rd),.addr(fast_addr),.data(sd_sec_read_data),
 .valid(sd_sec_read_data_valid && fast_busy),.sector_end(sd_sec_read_end && fast_busy));
d5_bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (raw_ready),

    .scan_start             (raw_start),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (scan_limit),
    .scan_done              (raw_done),
    .scan_found_valid       (raw_found),
    .scan_found_sector      (raw_found_sector),
    .scan_found_total       (raw_total),
    .scan_found_width       (raw_width),
    .scan_found_height      (raw_height),

    .load_start             (load_start_pulse),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (raw_state),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),

    .load_image_width       (load_image_width),
    .load_image_height      (load_image_height),

    .write_req              (bmp_write_req_raw),
    .write_req_ack          (bmp_write_req_ack_gated),
    .sd_sec_read            (raw_rd),
    .sd_sec_read_addr       (raw_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid && !fast_busy),
    .sd_sec_read_end        (sd_sec_read_end && !fast_busy),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

assign O_scan_done = scan_kicked && discovery_ready;
assign O_bmp_found = (img_found_count != 0);


endmodule

module d5_key_press_debounce #(
    parameter integer FAST_SCAN = 0,
    parameter integer KEYS_DEBOUNCED = 0,
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = DEBOUNCE_MS==0 ? 1 : (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
localparam integer COUNT_BITS=DEBOUNCE_CYCLES<1 ? 32:DEBOUNCE_CYCLES<2 ? 1:$clog2(DEBOUNCE_CYCLES);
reg [COUNT_BITS-1:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
