// FAT32 root locator: exact short name, or indexed WAV enumeration.
// A dynamic target with bits [87:16]==0 selects zero-based WAV index [15:0].
// Enumerate normal nonempty WAV files through their short aliases (LFN skipped).
// Out-of-range index falls back to the first WAV; count/index returned with found.
// Supports a volume at LBA0 or first FAT32 (0x0B/0x0C) primary MBR partition.
// 512-byte sectors; 1/2 FATs; power-of-two sectors per cluster (1..128).
// Streams directory and FAT bytes, follows fragmented chains, skips LFN entries.
// Errors: 1 chain guard, 2 sector size, 3 BPB, 4 root chain, 5 missing file,
//         6 invalid cluster, 7 truncated chain, 8 incomplete SD sector.
// LAZY_CHAIN=1 locates immediately; file stream reader validates as it plays.
module fat32_file_locator (
    input  wire clk,
    input  wire rst,
    input  wire I_start,
    input wire [87:0] I_target_name,

    // ---- SD 读口(复用) ----
    output reg        sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0] sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,

    // ---- 结果(保持到下一次 I_start) ----
    output reg [15:0] wav_count,
    output reg [15:0] wav_index,
    output reg         file_found,
    output reg  [31:0] file_size,
    output reg  [31:0] first_cluster,
    output reg  [15:0] file_clusters,
    output reg         done,
    output reg         busy,
    output reg         error,
    output reg  [3:0]  error_code,
    output reg  [3:0]  state
);
    // V27S7: 懒遍历开关
    //   0 = 兼容旧行为(默认): 目录命中后仍遍历整条文件 FAT 链统计 file_clusters 才 file_found
    //   1 = 懒遍历: 目录命中即 file_found(输出 first_cluster/file_size, file_clusters=0),
    //       不做启动前全链遍历; 链的合法性/EOC/file_size 边界由 fat32_file_stream_reader
    //       在按需读取(逐簇跟 FAT)时保证。
    parameter integer LAZY_CHAIN = 0;
    parameter integer DYNAMIC_NAME = 0;
    // DYNAMIC_NAME=2: board playlist uses suffix only; no full-name datapath.
    reg [87:0] target_name;
    reg [7:0] entry_first;

    localparam [31:0] EOC_MASK = 32'h0FFFFFF8;
    localparam [15:0] MAX_CHAIN = 16'd4096;    // 文件簇链上限(防坏链死循环)
    localparam [15:0] MAX_RDSEC = 16'd4096;    // root 目录最多扫多少扇区(防坏链)

    localparam [3:0] S_IDLE=4'd0, S_RD=4'd1, S_BOOT=4'd2, S_DIREND=4'd3,
                     S_FATDEC=4'd4, S_DONE=4'd5, S_ERR=4'd6;
    localparam [1:0] W_BOOT=2'd0, W_DIR=2'd1, W_FAT=2'd2;

    reg [3:0]  st;
    reg [1:0]  rd_who;
    reg        fat_kind;          // 1=文件簇链, 0=root 目录链
    // FAT32 at LBA0, or first FAT32 primary partition in an MBR.
    reg [31:0] volume_lba, partition_lba, part_shift;
    reg [7:0] part_type, sig0, sig1;
    reg [9:0]  bcnt;              // 扇区内字节 0..511

    // ---- BPB(收字节时抓取) ----
    reg [7:0]  b_11,b_12,b_13,b_14,b_15,b_16,b_36,b_37,b_38,b_39,b_44,b_45,b_46,b_47;
    reg [31:0] rsc, nfatx, fatsz32, rootc, fat1, data_base;
    reg [7:0]  spc;
    localparam S_ADDR_SUB=9, S_ADDR_SHIFT=10, S_ADDR_BASE=11, S_ADDR_SEND=12;
    reg [2:0] shift_left;
    reg [31:0] addr_calc;
    reg [7:0] addr_sector;
    wire [2:0] spc_shift = spc[7]?7:spc[6]?6:spc[5]?5:spc[4]?4:
                           spc[3]?3:spc[2]?2:spc[1]?1:0;
    task read_cluster;
        input [31:0] cluster;
        input [7:0] sector_in_cluster;
        begin addr_calc<=cluster; addr_sector<=sector_in_cluster; st<=S_ADDR_SUB; end
    endtask

    // ---- 目录项流式捕获 ----
    reg [87:0] nm;                // 11 字节短名移位寄存器(bit[7:0]=最后到的字节)
    wire indexed_wav = (DYNAMIC_NAME==2) || (DYNAMIC_NAME && target_name[87:16]==72'd0);
    wire nm_hit_music = (nm == target_name);
    wire wav_ext = ((nm[23:16] | 8'h20)==8'h77) &&
                   ((nm[15:8] | 8'h20)==8'h61) && ((nm[7:0] | 8'h20)==8'h76);
    reg ent_wav;
      // ★唯一命中条件(大写 MUSIC)
    reg [15:0] clhi, cllo;        // 起始簇高/低字(LE 组装)
    reg [31:0] fsz_r;             // size 移位寄存器
    reg        ent_hit;           // 本目录项(名字+属性)匹配
    reg        ent_zero;          // 本扇区出现过 0x00(目录结束)
    reg        hit_l;             // Selected file retained across all root sectors.

    // ---- FAT 抓取(依赖 c_cur, 故声明置于运行上下文之后) ----
    reg  [31:0] fat_word;

    // ---- 运行上下文 ----
    reg [31:0] c_cur;             // 当前簇(root 目录链 或 文件链)
    reg [7:0]  c_sec;             // 簇内扇区
    reg [15:0] cnt;               // 文件簇计数
    reg [12:0] dguard;            // root 目录扇区计数上限
    reg [31:0] acc_bytes;         // 文件链累计字节(校验覆盖 file_size)
    wire [31:0] bytes_per_cl = ({24'd0, spc} << 9);
    wire [31:0] fc_new       = ({clhi, cllo}) & 32'h0FFFFFFF;   // 目录项给出的起始簇
    wire [8:0]  fat_off      = ({2'b00, c_cur[6:0]} << 2);      // (cl & 0x7F)*4
    wire [31:0] next_w       = fat_word & 32'h0FFFFFFF;

    wire entry_valid = fc_new>=2 && fc_new<32'h0ffffff0 &&
                       {sd_sec_read_data,fsz_r[31:8]}!=0;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            target_name <= "MUSIC   WAV";
            wav_count<=0; wav_index<=0; ent_wav<=0; entry_first<=0;
            addr_calc<=0; addr_sector<=0; shift_left<=0;
            volume_lba<=0; partition_lba<=0; part_shift<=0; part_type<=0; sig0<=0; sig1<=0;
            st<=S_IDLE; rd_who<=W_BOOT; fat_kind<=1'b0; bcnt<=9'd0;
            sd_sec_read<=1'b0; sd_sec_read_addr<=32'd0;
            file_found<=1'b0; file_size<=32'd0; first_cluster<=32'd0; file_clusters<=16'd0;
            done<=1'b0; busy<=1'b0; error<=1'b0; error_code<=4'd0; state<=4'd0;
            b_11<=0;b_12<=0;b_13<=0;b_14<=0;b_15<=0;b_16<=0;
            b_36<=0;b_37<=0;b_38<=0;b_39<=0;b_44<=0;b_45<=0;b_46<=0;b_47<=0;
            rsc<=0; nfatx<=0; fatsz32<=0; rootc<=0; fat1<=0; data_base<=0; spc<=0;
            nm<=0; clhi<=0; cllo<=0; fsz_r<=0;
            ent_hit<=0; ent_zero<=0; hit_l<=0; fat_word<=0;
            c_cur<=0; c_sec<=0; cnt<=0; dguard<=0; acc_bytes<=0;
        end else begin
            state       <= st;
            sd_sec_read <= 1'b0;        // 默认单拍请求(arbiter 锁存 pending)

            if (st==S_RD && sd_sec_read_data_valid) begin
                case (bcnt)
                    450,466,482,498: part_type<=sd_sec_read_data;
                    454,455,456,470,471,472,486,487,488,502,503,504:
                        part_shift<={sd_sec_read_data,part_shift[31:8]};
                    457,473,489,505: begin
                        if (partition_lba==0 && (part_type==8'h0b || part_type==8'h0c))
                            partition_lba<={sd_sec_read_data,part_shift[31:8]};
                    end
                    510: sig0<=sd_sec_read_data;
                    511: sig1<=sd_sec_read_data;
                endcase
            end
            case (st)
                S_ADDR_SUB: begin addr_calc<=addr_calc-2; shift_left<=spc_shift; st<=S_ADDR_SHIFT; end
                // Prepare addresses outside byte transfers; at most seven extra clocks.
                S_ADDR_SHIFT: begin
                    if(shift_left==0) st<=S_ADDR_BASE;
                    else begin addr_calc<={addr_calc[30:0],1'b0};shift_left<=shift_left-1'b1;end
                end
                S_ADDR_BASE: begin addr_calc<=data_base+addr_calc; st<=S_ADDR_SEND; end
                S_ADDR_SEND: begin
                    sd_sec_read_addr<=addr_calc+addr_sector;
                    sd_sec_read<=1; bcnt<=0; st<=S_RD;
                end
                // ==================================================== IDLE
                S_IDLE: begin
                    if (I_start) begin
                        target_name <= DYNAMIC_NAME ? I_target_name : "MUSIC   WAV";
                        wav_count<=0; wav_index<=0; ent_wav<=0; entry_first<=0;
                        volume_lba<=0; partition_lba<=0;
                        file_found<=1'b0; file_size<=32'd0; first_cluster<=32'd0;
                        file_clusters<=16'd0;
                        error<=1'b0; error_code<=4'd0; done<=1'b0; busy<=1'b1;
                        hit_l<=1'b0; ent_hit<=1'b0; ent_zero<=1'b0; nm<=88'd0;
                        fat_word<=32'd0; dguard<=16'd0; acc_bytes<=32'd0;
                        rd_who<=W_BOOT; bcnt<=9'd0;
                        sd_sec_read_addr <= 32'd0;     // boot sector(逻辑扇区 0)
                        sd_sec_read      <= 1'b1;
                        st <= S_RD;
                    end
                end

                // ========================== 读一个 512B 扇区(纯流式, 边收边判)
                S_RD: begin
                    if (sd_sec_read_data_valid) begin
                        bcnt <= bcnt + 1'b1;
                        if (rd_who == W_BOOT) begin
                            case (bcnt)
                                9'd11: b_11 <= sd_sec_read_data;
                                9'd12: b_12 <= sd_sec_read_data;
                                9'd13: b_13 <= sd_sec_read_data;
                                9'd14: b_14 <= sd_sec_read_data;
                                9'd15: b_15 <= sd_sec_read_data;
                                9'd16: b_16 <= sd_sec_read_data;
                                9'd36: b_36 <= sd_sec_read_data;
                                9'd37: b_37 <= sd_sec_read_data;
                                9'd38: b_38 <= sd_sec_read_data;
                                9'd39: b_39 <= sd_sec_read_data;
                                9'd44: b_44 <= sd_sec_read_data;
                                9'd45: b_45 <= sd_sec_read_data;
                                9'd46: b_46 <= sd_sec_read_data;
                                9'd47: b_47 <= sd_sec_read_data;
                                default: ;
                            endcase
                        end else if (rd_who == W_DIR) begin
                            // 32B 目录项: 0..10 名字, 11 属性, 20/21 簇高, 26/27 簇低, 28..31 size
                            case (bcnt[4:0])
                                5'd0 : begin
                                           ent_zero <= ent_zero || (sd_sec_read_data == 8'h00);
                                           entry_first <= sd_sec_read_data;
                                           nm <= {nm[79:0], sd_sec_read_data};   // 名字第 0 字节
                                       end
                                5'd1 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd2 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd3 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd4 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd5 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd6 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd7 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd8 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd9 : nm <= {nm[79:0], sd_sec_read_data};
                                5'd10: nm <= {nm[79:0], sd_sec_read_data};
                                5'd11: begin
                                    ent_wav <= !ent_zero && entry_first!=8'he5 && wav_ext &&
                                               ((sd_sec_read_data & 8'h18)==0);
                                    ent_hit <= !ent_zero && entry_first!=8'he5 && nm_hit_music &&
                                               ((sd_sec_read_data & 8'h18)==0);
                                end
                                5'd20: clhi <= {sd_sec_read_data, clhi[15:8]};
                                5'd21: clhi <= {sd_sec_read_data, clhi[15:8]};
                                5'd26: cllo <= {sd_sec_read_data, cllo[15:8]};
                                5'd27: cllo <= {sd_sec_read_data, cllo[15:8]};
                                5'd28: fsz_r <= {sd_sec_read_data, fsz_r[31:8]};
                                5'd29: fsz_r <= {sd_sec_read_data, fsz_r[31:8]};
                                5'd30: fsz_r <= {sd_sec_read_data, fsz_r[31:8]};
                                default: ;
                            endcase
                            // 目录项最后一字节(第 31 字节)到齐 -> 提交命中结果
                            if (bcnt[4:0] == 5'd31) begin
                                ent_hit<=0; ent_wav<=0;
                                if(ent_wav && entry_valid && wav_count!=16'hffff)
                                    wav_count<=wav_count+1'b1;
                                if(entry_valid && (indexed_wav ?
                                    (ent_wav && wav_count!=16'hffff &&
                                      (wav_count==0 || wav_count==target_name[15:0])) :
                                    (ent_hit && !hit_l))) begin
                                    hit_l<=1;
                                    first_cluster<=fc_new;
                                    file_size<={sd_sec_read_data,fsz_r[31:8]};
                                    wav_index<=wav_count;
                                end
                            end
                        end else if (rd_who == W_FAT) begin
                            if ((bcnt >= fat_off) && (bcnt <= (fat_off + 9'd3)))
                                fat_word <= {sd_sec_read_data, fat_word[31:8]};
                        end
                    end

                    if (sd_sec_read_end) begin
                        if ((bcnt + sd_sec_read_data_valid) != 10'd512) begin
                            st<=S_ERR; error<=1; error_code<=4'd8;
                        end else if      (rd_who == W_BOOT) st <= S_BOOT;
                        else if (rd_who == W_DIR)  st <= S_DIREND;
                        else                       st <= S_FATDEC;
                    end
                end

                // ================================================== BPB 解析
                S_BOOT: begin
                    if ({b_12,b_11}!=16'h0200 && volume_lba==0 &&
                        partition_lba!=0 && sig0==8'h55 && sig1==8'haa) begin
                        volume_lba<=partition_lba;
                        sd_sec_read_addr<=partition_lba; sd_sec_read<=1;
                        bcnt<=0; rd_who<=W_BOOT; st<=S_RD;
                    end else if ({b_12, b_11} != 16'h0200) begin
                        st<=S_ERR; error<=1'b1; error_code<=4'd2;   // bytes/sector != 512
                    end else if (({b_39,b_38,b_37,b_36} == 32'd0) || (b_13 == 8'd0) ||
                                 ((b_13 & (b_13-1'b1)) != 0) || (b_16!=1 && b_16!=2) ||
                                 ({b_47,b_46,b_45,b_44} < 32'd2)) begin
                        st<=S_ERR; error<=1'b1; error_code<=4'd3;   // BPB 非法
                    end else begin
                        spc       <= b_13;                          // sectors per cluster
                        rsc       <= {16'd0, b_15, b_14};           // reserved sectors
                        nfatx     <= {24'd0, b_16};                 // num FATs
                        fatsz32   <= {b_39,b_38,b_37,b_36};         // FATSz32
                        rootc     <= {b_47,b_46,b_45,b_44};         // root cluster
                        fat1      <= volume_lba + {16'd0, b_15, b_14};           // FAT1 LBA
                        data_base <= volume_lba + {16'd0, b_15, b_14}
                                   + ({b_39,b_38,b_37,b_36} << (b_16==2));
                        c_cur  <= {b_47,b_46,b_45,b_44};
                        c_sec  <= 8'd0;
                        dguard <= 16'd0;
                        hit_l  <= 1'b0; ent_hit <= 1'b0; ent_zero <= 1'b0; nm <= 88'd0;
                        rd_who <= W_DIR; bcnt <= 9'd0;
                        // root 目录首扇区 = data_base + (rootc-2)*spc + 0
                        read_cluster({b_47,b_46,b_45,b_44},8'd0);
                    end
                end

                // ========================================= 目录扇区扫完的处理
                4'd13: begin
                    if (hit_l) begin
                        // 命中: (旧行为)开始文件簇链计数 / (V27S7 懒遍历)直接返回
                        // 注意: 必须用已锁存的 first_cluster(命中那一拍写入),
                        //       不能再用 clhi/cllo(fc_new)——它们会被后续目录项覆盖。
                        if (first_cluster < 32'd2) begin
                            st<=S_ERR; error<=1'b1; error_code<=4'd6;   // 非法 first_cluster(快速本地检查)
                        end else if (LAZY_CHAIN == 1) begin
                            // V27S7: 目录命中即出结果, 不在启动前遍历整条 FAT 链
                            //   (file_clusters=0 表示"未枚举"; 后级 stream reader 按需跟 FAT)
                            file_clusters <= 16'd0;
                            file_found    <= 1'b1;
                            st            <= S_DONE;
                        end else begin
                            c_cur     <= first_cluster;
                            cnt       <= 16'd1;
                            acc_bytes <= bytes_per_cl;
                            fat_kind  <= 1'b1;              // 文件链
                            fat_word  <= 32'd0;
                            rd_who    <= W_FAT; bcnt <= 9'd0;
                            sd_sec_read_addr <= fat1 + (first_cluster >> 7);
                            sd_sec_read <= 1'b1;
                            st <= S_RD;
                        end
                    end else begin
                        st<=S_ERR; error<=1; error_code<=4'd5;
                    end
                end

                S_DIREND: begin
                    if (ent_zero) begin
                        st<=4'd13;
                    end else if (dguard >= MAX_RDSEC) begin
                        st<=S_ERR; error<=1'b1; error_code<=4'd1;   // 目录链超长
                    end else begin
                        dguard <= dguard + 16'd1;
                        if (c_sec < (spc - 8'd1)) begin
                            // 本簇内下一扇区
                            c_sec <= c_sec + 8'd1;
                            ent_hit <= 1'b0; ent_zero <= 1'b0; nm <= 88'd0;
                            rd_who <= W_DIR; bcnt <= 9'd0;
                            read_cluster(c_cur,c_sec+8'd1);
                        end else begin
                            // 本簇扫完 -> 读 root 目录链下一簇
                            fat_kind <= 1'b0;               // root 目录链
                            fat_word <= 32'd0;
                            rd_who   <= W_FAT; bcnt <= 9'd0;
                            sd_sec_read_addr <= fat1 + (c_cur >> 7);
                            sd_sec_read <= 1'b1;
                            st <= S_RD;
                        end
                    end
                end

                // ================================================== FAT 结果处理
                S_FATDEC: begin
                    if (fat_kind == 1'b0) begin
                        // ---- root 目录链: 取下一簇继续扫 ----
                        if ((next_w >= EOC_MASK) || (next_w == 32'd0)) begin
                            st<=4'd13; // Root EOC: finalize full WAV enumeration
                        end else if (next_w < 32'd2) begin
                            st<=S_ERR; error<=1'b1; error_code<=4'd4;   // 目录链非法
                        end else begin
                            c_cur  <= next_w;
                            c_sec  <= 8'd0;
                            ent_hit <= 1'b0; ent_zero <= 1'b0; nm <= 88'd0;
                            rd_who <= W_DIR; bcnt <= 9'd0;
                            read_cluster(next_w,8'd0);
                        end
                    end else begin
                        // ---- 文件簇链: 计数直到 EOC ----
                        if (next_w >= EOC_MASK) begin
                            if (acc_bytes < file_size) begin
                                st<=S_ERR; error<=1'b1; error_code<=4'd7;   // 链未覆盖 size
                            end else begin
                                file_clusters <= cnt;
                                file_found    <= 1'b1;
                                st <= S_DONE;
                            end
                        end else if ((next_w < 32'd2) || (next_w == c_cur)) begin
                            st<=S_ERR; error<=1'b1; error_code<=4'd6;       // 链中非法 next
                        end else if (cnt >= MAX_CHAIN) begin
                            st<=S_ERR; error<=1'b1; error_code<=4'd1;       // 链超长
                        end else begin
                            cnt       <= cnt + 16'd1;
                            acc_bytes <= acc_bytes + bytes_per_cl;
                            c_cur     <= next_w;
                            fat_word  <= 32'd0;
                            rd_who    <= W_FAT; bcnt <= 9'd0;
                            sd_sec_read_addr <= fat1 + (next_w >> 7);
                            sd_sec_read <= 1'b1;
                            st <= S_RD;
                        end
                    end
                end

                // ======================================================= 终态
                S_DONE: begin
                    done <= 1'b1; busy <= 1'b0;
                    st   <= S_IDLE;
                end
                S_ERR: begin
                    done <= 1'b1; busy <= 1'b0;
                    st   <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
