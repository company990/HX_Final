module d5_bmp_read(

    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // G1-C3-D2:
    // 当 scan_found_valid=1 时，同时给出当前找到 BMP 的真实分辨率。
    output reg [15:0]           scan_found_width,
    output reg [15:0]           scan_found_height,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,

    // 兼容旧工程接口：
    // D2 开始不再用这两个输入限制 BMP 必须等于 640x480。
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    // 当前实际加载 BMP 的真实分辨率。
    // 在 ST_LOAD_HDR 通过合法性检查后锁存。
    output reg [15:0]           load_image_width,
    output reg [15:0]           load_image_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data
);

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;

reg [2:0]  state;
reg [9:0]  rd_cnt;

reg header_b_ok;
reg header_m_ok;

reg [31:0] file_len;
reg [31:0] pixel_offset;

reg [31:0] width;
reg [31:0] height;

reg planes_ok;
reg bits_ok;
reg compression_ok;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;

reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;

// =============================================================
// G1-C3-D2 row / padding control
//
// 24-bit BMP:
//     raw pixel bytes per row = width * 3
//     stored row stride        = (width*3 + 3) & ~3
//
// Example:
//     width=3
//     raw bytes = 9
//     stride    = 12
//     padding   = 3 bytes
//
// Padding bytes must NOT enter B/G/R assembly.
// =============================================================

// Valid width <= 65535: padded row length <= 196608, requiring 18 bits.
reg [17:0] row_pixel_bytes;
reg [17:0] row_stride;
reg [17:0] row_byte_cnt;
reg [15:0] row_index;

wire header_match;
wire bmp_data_valid;
wire pixel_region_active;

wire [17:0] row_pixel_bytes_calc;
wire [17:0] row_stride_calc;

wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;


// =============================================================
// Basic status
// =============================================================

assign ready = (state == ST_IDLE);


// =============================================================
// Generic 24-bit BMP header validation
//
// D2 difference from old code:
//   OLD: width/height had to equal external bmp_width/bmp_height.
//   NEW: accept arbitrary positive 16-bit width/height.
//
// Current supported BMP subset:
//   - "BM"
//   - positive width
//   - positive bottom-up height
//   - planes = 1
//   - 24 bits/pixel
//   - BI_RGB / compression = 0
//   - pixel_offset >= 54
//
// Top-down BMP (negative height) is deliberately rejected in D2.
// =============================================================

assign header_match =
                      header_b_ok &&
                      header_m_ok &&

                      (width[31:16]  == 16'd0) &&
                      (width[15:0]   != 16'd0) &&

                      (height[31:16] == 16'd0) &&
                      (height[15:0]  != 16'd0) &&

                      planes_ok &&
                      bits_ok &&
                      compression_ok &&

                      (pixel_offset  >= 32'd54) &&
                      (file_len      >  pixel_offset);


// =============================================================
// Row size calculation
//
// width*3 = width + width*2
// stride  = round_up(width*3, 4)
// =============================================================

assign row_pixel_bytes_calc =
                      width + (width << 1);

assign row_stride_calc =
                      (row_pixel_bytes_calc + 32'd3) &
                      32'hFFFF_FFFC;


// =============================================================
// Pixel-byte qualification
//
// bmp_len_cnt counts every byte read from the BMP file.
// row_byte_cnt counts bytes inside the current stored BMP row,
// including padding.
//
// Only bytes:
//     row_byte_cnt < row_pixel_bytes
// are real B/G/R bytes.
//
// Padding:
//     row_pixel_bytes <= row_byte_cnt < row_stride
// is discarded.
// =============================================================

assign pixel_region_active =
                      (sd_sec_read_data_valid == 1'b1) &&
                      (bmp_len_cnt >= pixel_offset) &&
                      (bmp_len_cnt <  file_len) &&
                      (row_index < load_image_height);

assign bmp_data_valid =
                      pixel_region_active &&
                      (row_byte_cnt < row_pixel_bytes);


// =============================================================
// Scan stepping
// =============================================================

assign file_sector_count =
                      (file_len == 32'd0) ?
                      32'd1 :
                      ((file_len + 32'd511) >> 9);

assign next_scan_sector_if_match =
                      scan_sector + file_sector_count;

assign next_scan_sector_if_miss =
                      scan_sector + 32'd1;


// =============================================================
// Header byte counter
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        rd_cnt <= 10'd0;

    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin

        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;

    end else begin

        rd_cnt <= 10'd0;

    end

end


// =============================================================
// BMP header parser
//
// BITMAPFILEHEADER:
//   0..1   signature
//   2..5   file size
//   10..13 pixel data offset
//
// BITMAPINFOHEADER:
//   18..21 width
//   22..25 height
//   26..27 planes
//   28..29 bit count
//   30..33 compression
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        header_b_ok <= 1'b0;
        header_m_ok <= 1'b0;

        file_len     <= 32'd0;
        pixel_offset <= 32'd54;

        width        <= 32'd0;
        height       <= 32'd0;

        planes_ok <= 1'b0;
        bits_ok <= 1'b0;
        compression_ok <= 1'b0;

    end else if (
        ((state == ST_SCAN) || (state == ST_LOAD_HDR)) &&
        sd_sec_read_data_valid
    ) begin

        case (rd_cnt)

            10'd0 : header_b_ok <= (sd_sec_read_data == 8'h42);
            10'd1 : header_m_ok <= (sd_sec_read_data == 8'h4d);

            10'd2 : file_len[7:0]   <= sd_sec_read_data;
            10'd3 : file_len[15:8]  <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0]   <= sd_sec_read_data;
            10'd11: pixel_offset[15:8]  <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            10'd18: width[7:0]   <= sd_sec_read_data;
            10'd19: width[15:8]  <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0]   <= sd_sec_read_data;
            10'd23: height[15:8]  <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd26: planes_ok <= (sd_sec_read_data == 8'd1);
            10'd27: planes_ok <= planes_ok && (sd_sec_read_data == 8'd0);

            10'd28: bits_ok <= (sd_sec_read_data == 8'd24);
            10'd29: bits_ok <= bits_ok && (sd_sec_read_data == 8'd0);

            10'd30: compression_ok <= (sd_sec_read_data == 8'd0);
            10'd31: compression_ok <= compression_ok && (sd_sec_read_data == 8'd0);
            10'd32: compression_ok <= compression_ok && (sd_sec_read_data == 8'd0);
            10'd33: compression_ok <= compression_ok && (sd_sec_read_data == 8'd0);

            default: ;

        endcase

    end

end


// =============================================================
// Raw BMP file byte counter
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        bmp_len_cnt <= 32'd0;

    end else if (state == ST_LOAD_DATA) begin

        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;

    end else begin

        bmp_len_cnt <= 32'd0;

    end

end


// =============================================================
// Row / padding byte counter
//
// Starts only when bmp_len_cnt reaches pixel_offset.
//
// Important:
//   row_byte_cnt advances for BOTH real pixel bytes and padding.
//   row_index increments after one complete stored BMP row.
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        row_byte_cnt <= 32'd0;
        row_index    <= 16'd0;

    end else if (state == ST_LOAD_DATA) begin

        if (
            sd_sec_read_data_valid &&
            (bmp_len_cnt >= pixel_offset) &&
            (bmp_len_cnt < file_len) &&
            (row_index < load_image_height)
        ) begin

            if (
                (row_stride != 32'd0) &&
                (row_byte_cnt == (row_stride - 32'd1))
            ) begin

                row_byte_cnt <= 32'd0;
                row_index    <= row_index + 16'd1;

            end else begin

                row_byte_cnt <= row_byte_cnt + 32'd1;

            end

        end

    end else begin

        row_byte_cnt <= 32'd0;
        row_index    <= 16'd0;

    end

end


// =============================================================
// B/G/R byte index
//
// Only real pixel bytes update bmp_byte_idx.
// Padding does not affect B/G/R grouping.
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        bmp_byte_idx <= 2'd0;

    end else if (state == ST_LOAD_DATA) begin

        if (bmp_data_valid)
            bmp_byte_idx <=
                (bmp_byte_idx == 2'd2) ?
                2'd0 :
                (bmp_byte_idx + 2'd1);

    end else begin

        bmp_byte_idx <= 2'd0;

    end

end


// =============================================================
// BMP B/G/R -> 24-bit pixel stream
//
// Existing byte order is preserved:
//   bmp_data[7:0]   = B
//   bmp_data[15:8]  = G
//   bmp_data[23:16] = R
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;

    end else if (state == ST_LOAD_DATA) begin

        if (bmp_data_valid) begin

            case (bmp_byte_idx)

                2'd0: begin

                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;

                end

                2'd1: begin

                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;

                end

                2'd2: begin

                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;

                end

                default: begin

                    bmp_data_wr_en <= 1'b0;

                end

            endcase

        end else begin

            bmp_data_wr_en <= 1'b0;

        end

    end else begin

        bmp_data_wr_en <= 1'b0;

    end

end


// =============================================================
// Main FSM
// =============================================================

always @(posedge clk or posedge rst) begin

    if (rst) begin

        state             <= ST_IDLE;
        state_code        <= 4'd0;

        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;

        write_req         <= 1'b0;

        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;

        scan_found_width  <= 16'd0;
        scan_found_height <= 16'd0;

        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;

        load_image_width  <= 16'd0;
        load_image_height <= 16'd0;

        row_pixel_bytes   <= 32'd0;
        row_stride        <= 32'd0;

    end else if (!sd_init_done) begin

        state             <= ST_IDLE;
        state_code        <= 4'd0;

        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;

        write_req         <= 1'b0;

        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;

        scan_found_width  <= 16'd0;
        scan_found_height <= 16'd0;

        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;

        load_image_width  <= 16'd0;
        load_image_height <= 16'd0;

        row_pixel_bytes   <= 32'd0;
        row_stride        <= 32'd0;

    end else begin

        // one-clock pulse
        scan_found_valid <= 1'b0;

        case (state)

            // =================================================
            // IDLE
            // =================================================

            ST_IDLE: begin

                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin

                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;

                    scan_sector      <= scan_start_sector;
                    sd_sec_read_addr <= scan_start_sector;

                    state            <= ST_SCAN;

                end else if (load_start) begin

                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;

                    // clear stale load geometry until this
                    // header is proven valid
                    load_image_width  <= 16'd0;
                    load_image_height <= 16'd0;

                    row_pixel_bytes <= 32'd0;
                    row_stride      <= 32'd0;

                    state <= ST_LOAD_HDR;

                end

            end


            // =================================================
            // SCAN
            // =================================================

            ST_SCAN: begin

                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin

                    sd_sec_read <= 1'b0;

                    if (header_match) begin

                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;

                        // D2: export real geometry together
                        // with scan_found_valid
                        scan_found_width  <= width[15:0];
                        scan_found_height <= height[15:0];

                        if (
                            (scan_found_total + 3'd1 >= scan_target_count) ||
                            (next_scan_sector_if_match > scan_max_sector)
                        ) begin

                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;

                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;

                        end else begin

                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;

                        end

                    end else begin

                        if (scan_sector >= scan_max_sector) begin

                            scan_done <= 1'b1;
                            state     <= ST_IDLE;

                        end else begin

                            sd_sec_read_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;

                        end

                    end

                end

            end


            // =================================================
            // LOAD HEADER
            // =================================================

            ST_LOAD_HDR: begin

                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin

                    sd_sec_read <= 1'b0;

                    if (header_match) begin

                        // D2: latch actual BMP dimensions
                        load_image_width  <= width[15:0];
                        load_image_height <= height[15:0];

                        // D2: latch row layout for padding filter
                        row_pixel_bytes <= row_pixel_bytes_calc;
                        row_stride      <= row_stride_calc;

                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;

                        state <= ST_LOAD_WAIT;

                    end else begin

                        load_image_width  <= 16'd0;
                        load_image_height <= 16'd0;

                        row_pixel_bytes <= 32'd0;
                        row_stride      <= 32'd0;

                        state <= ST_IDLE;

                    end

                end

            end


            // =================================================
            // WAIT FOR SDRAM WRITE PATH
            // =================================================

            ST_LOAD_WAIT: begin

                state_code <= 4'd3;

                if (write_req_ack) begin

                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;

                end

            end


            // =================================================
            // LOAD BMP DATA
            // =================================================

            ST_LOAD_DATA: begin

                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin

                    sd_sec_read <= 1'b0;

                    if (bmp_len_cnt >= file_len) begin

                        state <= ST_IDLE;

                    end else begin

                        sd_sec_read_addr <=
                            sd_sec_read_addr + 32'd1;

                    end

                end

            end


            // =================================================
            // RECOVERY
            // =================================================

            default: begin

                state       <= ST_IDLE;
                state_code  <= 4'd1;

                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

            end

        endcase

    end

end


// =============================================================
// Legacy interface note
//
// bmp_width / bmp_height are intentionally retained so the
// existing d5_sd_card_bmp named-port instantiation remains source
// compatible during D2.  They are no longer used as a fixed
// 640x480 acceptance filter.
// =============================================================

endmodule
