`timescale 1ns/1ps

module osd_status_cdc (
    // Source status signals from sd_card_clk domain
    input             src_auto_play,
    input      [1:0]  src_img_idx,
    input      [2:0]  src_img_count,

    // Destination video clock domain
    input             video_clk,
    input             video_rst,
    input             video_vs,

    // Frame-boundary committed status for OSD
    output reg        video_auto_play,
    output reg [1:0]  video_img_idx,
    output reg [2:0]  video_img_count
);

    // =========================================================
    // Bundle the slow-changing OSD status.
    //
    // [5]   : auto_play
    // [4:3] : image index
    // [2:0] : image count
    // =========================================================
    wire [5:0] src_bundle;

    assign src_bundle = {
        src_auto_play,
        src_img_idx,
        src_img_count
    };

    // =========================================================
    // Multi-stage synchronization into video_clk domain
    // =========================================================
    reg [5:0] sync_meta;
    reg [5:0] sync_stage1;
    reg [5:0] sync_stage2;

    // Require the synchronized bundle to remain unchanged
    // before accepting it as a stable candidate.
    reg [5:0] sample_prev;
    reg [5:0] stable_bundle;

    // VS edge detector.
    // A rising VS edge occurs once per video frame.
    reg video_vs_d;

    wire video_frame_edge;

    assign video_frame_edge =
        video_vs && !video_vs_d;

    always @(posedge video_clk or posedge video_rst) begin
        if (video_rst) begin
            sync_meta    <= 6'd0;
            sync_stage1  <= 6'd0;
            sync_stage2  <= 6'd0;

            sample_prev  <= 6'd0;
            stable_bundle<= 6'd0;

            video_vs_d   <= 1'b0;

            video_auto_play <= 1'b0;
            video_img_idx   <= 2'd0;
            video_img_count <= 3'd0;
        end
        else begin
            // -------------------------------------------------
            // CDC synchronization
            // -------------------------------------------------
            sync_meta   <= src_bundle;
            sync_stage1 <= sync_meta;
            sync_stage2 <= sync_stage1;

            // -------------------------------------------------
            // Stability filter
            //
            // Only accept the bundle after it has appeared
            // identically on two consecutive video_clk cycles.
            // -------------------------------------------------
            sample_prev <= sync_stage2;

            if (sync_stage2 == sample_prev)
                stable_bundle <= sync_stage2;

            // -------------------------------------------------
            // Frame-boundary commit
            //
            // OSD status changes only once at a video-frame
            // boundary, preventing visible mid-frame changes.
            // -------------------------------------------------
            video_vs_d <= video_vs;

            if (video_frame_edge) begin
                video_auto_play <= stable_bundle[5];
                video_img_idx   <= stable_bundle[4:3];
                video_img_count <= stable_bundle[2:0];
            end
        end
    end

endmodule