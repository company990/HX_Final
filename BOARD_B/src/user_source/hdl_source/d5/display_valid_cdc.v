module display_valid_cdc (
    input  wire video_clk,
    input  wire video_rst,
    input  wire display_valid_src,
    input  wire video_vs,
    output reg  display_valid_pending,
    output reg  display_valid_active
);

reg display_valid_meta;
reg display_valid_sync;
reg video_vs_d;

wire frame_boundary = video_vs_d && !video_vs;

always @(posedge video_clk or posedge video_rst) begin
    if (video_rst) begin
        display_valid_meta    <= 1'b0;
        display_valid_sync    <= 1'b0;
        display_valid_pending <= 1'b0;
        display_valid_active  <= 1'b0;
        video_vs_d            <= 1'b0;
    end else begin
        display_valid_meta    <= display_valid_src;
        display_valid_sync    <= display_valid_meta;
        display_valid_pending <= display_valid_sync;
        video_vs_d            <= video_vs;

        // video_vs is active low. Commit only at the start of vertical sync,
        // while the video stream is in blanking.
        if (frame_boundary)
            display_valid_active <= display_valid_pending;
    end
end

endmodule
