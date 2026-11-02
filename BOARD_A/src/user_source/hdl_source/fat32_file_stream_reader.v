//=====================================================================

// fat32_file_stream_reader.v  (FAT32 Cluster Chain -> 连续文件 Byte Stream)

//

// 输入: I_file_first_cluster / I_file_size (来自 fat32_file_locator)

// 运行时读 BPB(不硬编码 cluster size/FAT 起/data 起)计算几何,

// 沿 FAT cluster chain(支持非连续: 200->205->210)逐簇/逐扇区读数据,

// 输出连续 file byte stream.

//

// 字节流: 电平 valid + 持有数据, transfer 在 (valid && ready) 的时钟沿发生;

//         consumer 可停(ready=0 不丢/不推进); 跨 sector/簇时 valid 拉低取数.

// file_size 控制: 恰输出 file_size 字节后 file_end=1 停止, 不吐 padding;

//                 file_size=0 -> 立即 done.

//

// 复用 SD 读口(sd_card_sec_read_write 读侧一致): sd_sec_read/addr/data/

//   data_valid/end(单块512B). 512B 单扇区 buffer 解耦 SD burst 与 consumer.

// 错误码: 2 截断(EOC 先于 file_size) / 3 坏簇(0x0FFFFFF6..F7) /

//         4 非法簇(<2) / 5 几何/读/guard.

//=====================================================================

module fat32_file_stream_reader #(parameter EXTERNAL_BPB=0, parameter FAST_OUTPUT=0) (

    input  wire clk,

    input  wire rst,

    input  wire I_start,

    input  wire [31:0] I_file_first_cluster,

    input  wire [31:0] I_file_size,



    output reg        sd_sec_read,

    output reg  [31:0] sd_sec_read_addr,

    input  wire [7:0] sd_sec_read_data,

    input  wire        sd_sec_read_data_valid,

    input  wire        sd_sec_read_end,



    output wire        O_byte_valid,

    output wire [7:0]  O_byte_data,

    input  wire        O_byte_ready,



    output reg         O_file_end,

    output reg         O_done,

    output reg         O_busy,

    output reg         O_error,

    output reg  [3:0]  O_error_code,

    output reg  [31:0] O_byte_cnt,

    output reg  [3:0]  O_state,
    output reg O_prefetch_valid,output reg [31:0] O_prefetch_addr,
    // EXTERNAL_BPB=1: geometry has been validated by this file's locator.
    // The source must hold these fields until O_done or reset.
    input I_geometry_valid,input [7:0] I_geometry_spc,
    input [31:0] I_geometry_fat_lba,I_geometry_data_lba

);

    localparam [31:0] EOC_MASK = 32'h0FFFFFF8;

    localparam [3:0] S_IDLE=0, S_RD=1, S_GEOM=2, S_DSTART=3,

                     S_OUT=4, S_FATD=5, S_FATC=6, S_DONE=7, S_ERR=8;



    // ---- 512B 扇区 buffer(1 个) ----

    reg [7:0]  sbuf [0:511];

    // FAT32 at LBA0, or first FAT32 primary partition in an MBR.
    reg [31:0] volume_lba, partition_lba, part_shift;
    reg [7:0] part_type, sig0, sig1;

    reg [9:0]  bcnt;



    // ---- 几何(运行时) ----

    reg [3:0]  st;
    reg [7:0] spc_local;
    wire [7:0] spc=EXTERNAL_BPB ? I_geometry_spc:spc_local;
    localparam S_ADDR_SUB=9, S_ADDR_SHIFT=10, S_ADDR_BASE=11, S_ADDR_SEND=12;
    reg [2:0] shift_left;
    reg [31:0] addr_calc;
    wire [7:0] addr_sector;
    wire [2:0] spc_shift = spc[7]?7:spc[6]?6:spc[5]?5:spc[4]?4:
                           spc[3]?3:spc[2]?2:spc[1]?1:0;
    task read_cluster;
        input [31:0] cluster;
        input [7:0] sector_in_cluster;
        begin addr_calc<=cluster;  st<=S_ADDR_SUB; end
    endtask
    reg [31:0] fat1_local, data_base_local;
    wire [31:0] fat1=EXTERNAL_BPB ? I_geometry_fat_lba:fat1_local;
    wire [31:0] data_base=EXTERNAL_BPB ? I_geometry_data_lba:data_base_local;



    // ---- 上下文 ----

    reg        rd_is_fat, rd_is_boot;
    reg [31:0] cl;

    reg [7:0]  si;

    reg [9:0]  bi;   // 0..512: 512 表示当前扇区 512 字节已全部送出

    reg [31:0] cnt;

    wire [31:0] next_cl;

    reg [16:0] guard; // Stops at 65536; maximum stored value is 65537.



    // ---- 输出电平寄存器 ----

    reg bv; reg [7:0] bd;
    reg [7:0] rdo;

    assign O_byte_valid = bv;

    assign O_byte_data  = FAST_OUTPUT ? rdo : bd;



    // ==================== sbuf -> TD 可推断 RAM(单写口 + 单读口, 同步读 1 拍) ====================

    // 旧 512x8 FF 阵列+动态多字节读 => 被综合成 ~4k FF + 5-6k LUT。改为:

    //   写口 = 独立 always(S_RD 收字节, 内容/地址 1 拍后写入, 与旧同步写等价);

    //   读口 = 独立 always 寄存输出 rdo <= sbuf[raddr](1 拍延迟);

    //   多字节读(几何/FAT entry)改为收字节时边抓; 字节流输出(S_OUT)用 raddr/rdo 流水。

    reg        ram_we;

    reg [8:0]  ram_waddr;

    reg [7:0]  ram_wdata;

    reg [8:0]  raddr;



    reg        rd_pend;                       // S_OUT: 读请求在途(2 级延迟后呈现)

    reg [1:0]  rdq;                           // 到呈现还需周期数(2->1->0)

    wire [8:0] fat_off  = ({2'b00, cl[6:0]} << 2);   // FAT entry 偏移 (cl&0x7F)*4, 9bit 不截断

    // Read-only one-sector FAT cache: neighbouring cluster links share
    // one FAT sector, even when file data clusters are fragmented.
    // Valid LBA matching is mandatory; no contiguous-cluster assumption.
    reg [31:0] fat_word;
    localparam S_FCACHE_WAIT=13,S_FCACHE_GET=14;
    reg [7:0] fat_cache[0:511];reg [7:0] fat_cache_q;
    reg [8:0] fat_cache_addr;reg [1:0] fat_cache_byte;
    reg fat_cache_valid;reg [31:0] fat_cache_lba;
    always @(posedge clk)begin
        if(FAST_OUTPUT && st==S_RD && rd_is_fat && sd_sec_read_data_valid && bcnt<512)
            fat_cache[bcnt[8:0]]<=sd_sec_read_data;
        fat_cache_q<=fat_cache[fat_cache_addr];
    end
    task fetch_fat_link;
        begin
            fat_word<=0;rd_is_fat<=1;
            if(FAST_OUTPUT && fat_cache_valid && fat_cache_lba==fat1+(cl>>7))begin
                fat_cache_addr<={cl[6:0],2'b00};fat_cache_byte<=0;st<=S_FCACHE_WAIT;
            end else begin
                sd_sec_read_addr<=fat1+(cl>>7);sd_sec_read<=1;bcnt<=0;st<=S_RD;
            end
        end
    endtask

    assign next_cl=fat_word & 32'h0fffffff;
    assign addr_sector=si;                      // 抓到的 FAT entry(字节序: LSB 先到=>低字节)

    reg [7:0]  g_11,g_12,g_13,g_14,g_15,g_16,g_36,g_37,g_38,g_39;  // boot 关键字节



    // 单写口(由 S_RD 收字节触发; ram_we=1 时 1 拍后写入)

    always @(posedge clk or posedge rst) begin

        if (rst) begin ram_we <= 1'b0; ram_waddr <= 9'd0; ram_wdata <= 8'd0; end

        else begin

            ram_we <= (st==S_RD) & sd_sec_read_data_valid;

            if (sd_sec_read_data_valid && (st==S_RD)) begin

                ram_waddr <= bcnt;

                ram_wdata <= sd_sec_read_data;

            end

        end

    end

    always @(posedge clk) begin

        if (ram_we) sbuf[ram_waddr] <= ram_wdata;

    end

    // 单读口(寄存输出, 1 拍延迟)

    always @(posedge clk) begin

        // Present the current byte and prefetch its successor only on a
        // real transfer. A stalled consumer repeatedly reads the same byte.
        rdo <= sbuf[FAST_OUTPUT && st==S_OUT ?
            (bi[8:0]+(bv && O_byte_ready ? 9'd1:9'd0)):raddr];

    end



    always @(posedge clk or posedge rst) begin : main

        integer o;

        if (rst) begin

            fat_cache_addr<=0;fat_cache_byte<=0;fat_cache_valid<=0;fat_cache_lba<=0;
            O_prefetch_valid<=0;O_prefetch_addr<=0;addr_calc<=0;  shift_left<=0;
            volume_lba<=0; partition_lba<=0; part_shift<=0; part_type<=0; sig0<=0; sig1<=0;
            st<=S_IDLE; bcnt<=0;

            sd_sec_read<=0; sd_sec_read_addr<=0;

            bv<=0; bd<=0; O_byte_cnt<=0;

            rd_pend<=0; rdq<=0; raddr<=9'd0; fat_word<=0;

            g_11<=0;g_12<=0;g_13<=0;g_14<=0;g_15<=0;g_16<=0;g_36<=0;g_37<=0;g_38<=0;g_39<=0;

            O_file_end<=0; O_done<=0; O_busy<=0; O_error<=0; O_error_code<=0;

            spc_local<=0; fat1_local<=0; data_base_local<=0;

            rd_is_fat<=0; rd_is_boot<=0;

            cl<=0; si<=0; bi<=0; cnt<=0;  guard<=0;

        end else begin

            O_prefetch_valid<=0;
            O_state <= st;
            if(I_start)fat_cache_valid<=0;
            if(FAST_OUTPUT && st==S_RD && rd_is_fat && sd_sec_read_end && bcnt+sd_sec_read_data_valid==512)begin
                fat_cache_valid<=1;fat_cache_lba<=sd_sec_read_addr;
            end

            sd_sec_read <= 1'b0;



            if (!EXTERNAL_BPB && st==S_RD && sd_sec_read_data_valid) begin
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
                S_IDLE: begin

                    // O_done/O_error 为粘性状态: 保持到下一次 I_start 才清除

                    if (I_start) begin

                        volume_lba<=0; partition_lba<=0;

                        O_done <= 1'b0; O_error <= 1'b0; O_error_code <= 4'd0;

                        O_busy <= 1'b1; O_file_end <= 1'b0;

                        cl <= I_file_first_cluster; cnt <= 0; O_byte_cnt <= 0;

                        si <= 0; bi <= 0; bv <= 0;

                        rd_is_fat<=0;bcnt<=0;
                        if(EXTERNAL_BPB)begin
                            rd_is_boot<=0;
                            if(I_geometry_valid && I_geometry_spc!=0 &&
                               (I_geometry_spc & (I_geometry_spc-1'b1))==0)
                                st<=S_DSTART;
                            else begin st<=S_ERR;O_error<=1;O_error_code<=5;end
                        end else begin
                            rd_is_boot<=1;sd_sec_read_addr<=0;
                            sd_sec_read<=1;st<=S_RD;
                        end

                    end

                end



                S_RD: begin

                    if (sd_sec_read_data_valid) begin

                        bcnt <= bcnt + 1'b1;

                        // boot/FAT 关键字段边收边抓(sbuf 已改 RAM, 不再读回多字节)

                        if (!EXTERNAL_BPB && rd_is_boot) begin

                            case (bcnt)

                                9'd11: g_11 <= sd_sec_read_data;

                                9'd12: g_12 <= sd_sec_read_data;

                                9'd13: g_13 <= sd_sec_read_data;

                                9'd14: g_14 <= sd_sec_read_data;

                                9'd15: g_15 <= sd_sec_read_data;

                                9'd16: g_16 <= sd_sec_read_data;

                                9'd36: g_36 <= sd_sec_read_data;

                                9'd37: g_37 <= sd_sec_read_data;

                                9'd38: g_38 <= sd_sec_read_data;

                                9'd39: g_39 <= sd_sec_read_data;

                                default: ;

                            endcase

                        end

                        if (rd_is_fat && (bcnt>=fat_off) && (bcnt <= (fat_off + 3'd3)))

                            fat_word <= {sd_sec_read_data, fat_word[31:8]};  // 新字节入高位, 先到(b0)留 LSB => {b3,b2,b1,b0}

                    end

                    if (sd_sec_read_end) begin

                        if ((bcnt + sd_sec_read_data_valid) != 10'd512) begin

                            st<=S_ERR; O_error<=1; O_error_code<=4'd5;

                        end else if (rd_is_boot) st <= S_GEOM;

                        else if (rd_is_fat)  st <= S_FATD;

                        else begin
                            st <= S_OUT;
                            // Hint only a verified next data sector inside this cluster/file.
                            if(si < spc-1 && I_file_size-cnt>512)begin
                                O_prefetch_valid<=1;O_prefetch_addr<=sd_sec_read_addr+1'b1;
                            end
                        end

                    end

                end



                S_GEOM: if(!EXTERNAL_BPB)begin

                    rd_is_boot <= 1'b0;

                    if ({g_12,g_11}!=16'h0200 && volume_lba==0 &&
                        partition_lba!=0 && sig0==8'h55 && sig1==8'haa) begin
                        volume_lba<=partition_lba; rd_is_boot<=1;
                        sd_sec_read_addr<=partition_lba; sd_sec_read<=1;
                        bcnt<=0; st<=S_RD;
                    end else if ({g_12, g_11} != 16'h0200 || g_13==0 ||
                        (g_13 & (g_13-1'b1))!=0 || (g_16!=1 && g_16!=2)) begin
                        st<=S_ERR; O_error<=1'b1; O_error_code<=4'd5;

                    end else begin

                        spc_local  <= g_13;

                        fat1_local <= volume_lba + {16'd0, g_15, g_14};

                        data_base_local <= volume_lba + {16'd0, g_15, g_14}

                                   + ({g_39,g_38,g_37,g_36} << (g_16==2));
                        st <= S_DSTART;

                    end

                end else begin st<=S_ERR;O_error<=1;O_error_code<=5;end



                S_DSTART: begin

                    if (I_file_size == 0) begin

                        O_file_end <= 1'b1;

                        st <= S_DONE;

                    end else if (cl < 2) begin

                        st<=S_ERR; O_error<=1'b1; O_error_code<=4'd4;

                    end else begin

                        rd_is_fat <= 1'b0;

                        read_cluster(cl,8'd0);
                    end

                end



                // ---- 输出(电平 valid); sbuf 读经 raddr/rdo 流水(寄存读, 2 级后呈现) ----

                S_OUT: if(FAST_OUTPUT)begin
                    if(cnt>=I_file_size)begin
                        O_file_end<=1;bv<=0;rd_pend<=0;rdq<=0;st<=S_DONE;
                    end else if(bi>=512)begin
                        bv<=0;rd_pend<=0;rdq<=0;
                        if(si<(spc-1))begin
                            si<=si+1'b1;bi<=0;rd_is_fat<=0;read_cluster(cl,si+8'd1);
                        end else begin
                            fetch_fat_link;
                        end
                    end else if(!bv)begin
                        // Synchronous RAM now contains sbuf[bi].
                        bv<=1;
                    end else if(O_byte_ready)begin
                        cnt<=cnt+1'b1;O_byte_cnt<=cnt+1'b1;bi<=bi+1'b1;
                        if(bi==511 || cnt+1'b1==I_file_size)bv<=0;
                    end
                end else begin

                    if (cnt >= I_file_size) begin

                        O_file_end <= 1'b1;

                        bv <= 0; rd_pend <= 1'b0; rdq <= 2'd0;

                        st <= S_DONE;

                    end else if (rd_pend) begin

                        // 等待 rdo 稳定(rdo 在请求后第 2 拍携带 sbuf[请求地址])

                        if (rdq == 2'd1) begin

                            bd <= rdo;

                            bv <= 1'b1;

                            rd_pend <= 1'b0;

                            rdq <= 2'd0;

                        end else begin

                            rdq <= rdq - 2'd1;

                        end

                    end else if (bv == 1'b0) begin

                        // 无待传字节: 若扇区用尽 -> 取下一扇区/簇; 否则发当前字节读请求

                        if (bi >= 10'd512) begin

                            rd_pend <= 1'b0; rdq <= 2'd0;

                            if (si < (spc - 1)) begin

                                si <= si + 1'b1; bi <= 0;

                                rd_is_fat <= 1'b0;

                                read_cluster(cl,si+8'd1);
                            end else begin

                                fetch_fat_link;

                            end

                        end else begin

                            raddr <= bi[8:0];

                            rd_pend <= 1'b1;

                            rdq <= 2'd2;           // 2 拍后 rdo=sbuf[bi] 并呈现

                        end

                    end else begin

                        // 有字节待传

                        if (O_byte_ready) begin

                            cnt <= cnt + 32'd1;

                            O_byte_cnt <= cnt + 32'd1;

                            bi <= bi + 1'b1;

                            bv <= 1'b0;          // 消费完, 下拍发下一字节读请求

                        end

                        // ready=0: 保持 bv/bd 不变(背压)

                    end

                end



                S_FCACHE_WAIT:st<=S_FCACHE_GET;
                S_FCACHE_GET:begin
                    fat_word<={fat_cache_q,fat_word[31:8]};
                    if(fat_cache_byte==3)st<=S_FATD;
                    else begin fat_cache_byte<=fat_cache_byte+1'b1;fat_cache_addr<=fat_cache_addr+1'b1;st<=S_FCACHE_WAIT;end
                end

                S_FATD: begin


                    st <= S_FATC;

                end



                S_FATC: begin

                    if (next_cl >= 32'h0FFFFFF6 && next_cl < EOC_MASK) begin

                        st<=S_ERR; O_error<=1'b1; O_error_code<=4'd3;

                    end else if (next_cl >= EOC_MASK) begin

                        if (cnt >= I_file_size) begin

                            O_file_end <= 1'b1;

                            st <= S_DONE;

                        end else begin

                            st<=S_ERR; O_error<=1'b1; O_error_code<=4'd2;

                        end

                    end else if (next_cl < 2) begin

                        st<=S_ERR; O_error<=1'b1; O_error_code<=4'd4;

                    end else begin

                        guard <= guard + 32'd1;

                        if (guard > 32'd65535) begin

                            st<=S_ERR; O_error<=1'b1; O_error_code<=4'd5;

                        end else begin

                            cl <= next_cl; si <= 0; bi <= 0;

                            rd_is_fat <= 1'b0;

                            read_cluster(next_cl,8'd0);
                        end

                    end

                end



                S_DONE: begin

                    O_done <= 1'b1; O_busy <= 1'b0;

                    st <= S_IDLE;

                end

                S_ERR: begin

                    O_done <= 1'b1; O_busy <= 1'b0;

                    st <= S_IDLE;

                end

                default: st<=S_IDLE;

            endcase

        end

    end

endmodule

// One-sector lookahead. Hints come only from validated sequential data sectors;
// FAT/boot/random accesses keep exact request addresses. No speculative EOF read.
module tf_video_sector_prefetch(
 input clk,rst,input req,input [31:0] addr,
 output wire [7:0] data,output wire valid,done,
 input hint,input [31:0] hint_addr,
 output reg sd_req,output reg [31:0] sd_addr,
 input [7:0] sd_data,input sd_valid,sd_done
);
 localparam IDLE=0,MISS=1,FILL=2,PRIME=3,REPLAY=4;
 reg [2:0] state;
 reg cached,pending; // Requester holds addr until done, as on the SD interface.
 reg [9:0] fill_count;reg [8:0] rd_addr,out_count;
 reg [7:0] ram[0:511],ram_out,out_data;reg out_valid,out_done;
 wire wanted=req || pending;
 wire [31:0] wanted_addr=addr;
 always @(posedge clk)begin
  if(state==FILL && sd_valid && fill_count<512)ram[fill_count[8:0]]<=sd_data;
  ram_out<=ram[rd_addr];
 end
 assign data=state==MISS?sd_data:out_data;
 assign valid=(state==MISS && sd_valid) || out_valid;
 assign done=(state==MISS && sd_done) || out_done;
 always @(posedge clk or posedge rst)begin
  if(rst)begin state<=IDLE;cached<=0;pending<=0;
   fill_count<=0;rd_addr<=0;out_count<=0;out_data<=0;out_valid<=0;out_done<=0;sd_req<=0;sd_addr<=0;end
  else begin
   sd_req<=0;out_valid<=0;out_done<=0;
   if(req && state!=IDLE)begin pending<=1;end
   case(state)
    IDLE:if(wanted)begin
     pending<=0;
     if(cached && wanted_addr==sd_addr)begin rd_addr<=0;out_count<=0;state<=PRIME;end
     else begin cached<=0;sd_addr<=wanted_addr;sd_req<=1;state<=MISS;end
    end else if(hint)begin
     cached<=0;sd_addr<=hint_addr;sd_req<=1;fill_count<=0;state<=FILL;
    end
    MISS:if(sd_done)state<=IDLE;
    FILL:begin
     if(sd_valid)fill_count<=fill_count+1'b1;
     if(sd_done)begin cached<=fill_count+sd_valid==512;state<=IDLE;end
    end
    PRIME:begin rd_addr<=1;state<=REPLAY;end
    REPLAY:begin
     out_data<=ram_out;out_valid<=1;rd_addr<=rd_addr+1'b1;out_count<=out_count+1'b1;
     if(out_count==511)begin out_done<=1;cached<=0;state<=IDLE;end
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule

