// Streaming validation flags replace fields used only in constant comparisons.

module bmp_fat_scanner #(parameter [31:0] MAX_PIXELS=1048576)(

 input clk,rst,start,

 input [15:0] bmp_width,bmp_height,

 input [2:0] target_count,

 output reg busy,done,fallback,found,

 output reg [31:0] found_sector,

 output reg [2:0] found_count,

 output reg [15:0] found_width,found_height,

 output reg rd,

 output reg [31:0] addr,

 input [7:0] data,input valid,sector_end

);

 localparam IDLE=0,READ=1,BOOT=2,DIR=3,HEADER=4,FAT=5,

            SUB=6,SHIFT=7,BASE=8,SEND=9,FINISH=10;

 reg [3:0] state;
 reg [2:0] shift_left;

 reg [1:0] kind;

 reg [9:0] pos;

 reg [31:0] volume,partition,part_shift;

 reg [7:0] part_type,sig0,sig1;

 reg [15:0] bps,reserved;

 reg [7:0] spc,nfats;

 reg [31:0] fatsz,root,fat_base,data_base;

 reg [31:0] dir_cluster,dir_sector,candidate_cluster,calc;

 reg [7:0] dir_offset;

 reg [4:0] resume_entry;

 reg [11:0] guard; // Stops before increment at 4095.

 reg name_live,ext_ok;

 reg [15:0] clhi,cllo;

 reg entry_size_ok;

 reg entry_ok,end_dir,candidate;

 reg [31:0] fat_word;

 reg magic_ok,bits_ok,planes_ok,compression_ok,dib_ok;

 reg [31:0] length,pixel_offset,width,height;

 wire [2:0] spc_shift=spc[7]?7:spc[6]?6:spc[5]?5:spc[4]?4:spc[3]?3:spc[2]?2:spc[1]?1:0;

 wire [31:0] next_cluster=fat_word & 32'h0fffffff;

 wire [8:0] fat_offset={dir_cluster[6:0],2'b00};



 wire [31:0] candidate_value={clhi,cllo}&32'h0fffffff;

 wire [31:0] pixels=width[15:0]*height[15:0];

 wire header_ok=magic_ok && width[31:16]==0 && width[15:0]!=0 &&

                 height[31:16]==0 && height[15:0]!=0 && planes_ok &&

                 pixels<=MAX_PIXELS && bits_ok && compression_ok && dib_ok && pixel_offset>=54 && length>pixel_offset;

 task request;

  input [31:0] sector;input [1:0] who;

  begin addr<=sector;kind<=who;pos<=0;rd<=1;state<=READ;end

 endtask

 task read_dir;

  begin candidate<=0;end_dir<=0;entry_ok<=0;request(dir_sector,1);end

 endtask

 always @(posedge clk or posedge rst) begin

  if(rst) begin

   state<=IDLE;shift_left<=0;busy<=0;done<=0;fallback<=0;found<=0;found_sector<=0;found_count<=0;

   rd<=0;addr<=0;pos<=0;kind<=0;volume<=0;partition<=0;part_shift<=0;part_type<=0;sig0<=0;sig1<=0;

   bps<=0;reserved<=0;spc<=0;nfats<=0;fatsz<=0;root<=0;fat_base<=0;data_base<=0;

   dir_cluster<=0;dir_sector<=0;candidate_cluster<=0;calc<=0;dir_offset<=0;resume_entry<=0;guard<=0;

   name_live<=0;ext_ok<=0;clhi<=0;cllo<=0;entry_size_ok<=0;entry_ok<=0;end_dir<=0;candidate<=0;fat_word<=0;

   found_width<=0;found_height<=0;planes_ok<=0;magic_ok<=0;bits_ok<=0;length<=0;pixel_offset<=0;width<=0;height<=0;compression_ok<=0;dib_ok<=0;

  end else begin

   rd<=0;found<=0;

   case(state)

    IDLE: if(start) begin

     busy<=1;done<=0;fallback<=0;found_count<=0;volume<=0;partition<=0;guard<=0;

     bps<=0;reserved<=0;spc<=0;nfats<=0;fatsz<=0;root<=0;

     request(0,0);

    end

    READ: begin

     if(valid) begin

      pos<=pos+1'b1;

      case(kind)

       0: case(pos)

        11:bps[7:0]<=data;12:bps[15:8]<=data;13:spc<=data;

        14:reserved[7:0]<=data;15:reserved[15:8]<=data;16:nfats<=data;

        36,37,38,39:fatsz<={data,fatsz[31:8]};

        44,45,46,47:root<={data,root[31:8]};

        450,466,482,498:part_type<=data;

        454,455,456,470,471,472,486,487,488,502,503,504:part_shift<={data,part_shift[31:8]};

        457,473,489,505:if(partition==0 && (part_type==8'h0b || part_type==8'h0c)) partition<={data,part_shift[31:8]};

        510:sig0<=data;511:sig1<=data;

       endcase

       1: begin

        case(pos[4:0])

         0:begin if(data==0) end_dir<=1; name_live<=data!=8'he5; end

         8:ext_ok<=(data|8'h20)==8'h62;

         9:ext_ok<=ext_ok && (data|8'h20)==8'h6d;

         10:ext_ok<=ext_ok && (data|8'h20)==8'h70;

         11:entry_ok<=!end_dir && !candidate && {1'b0,pos[8:5]}>=resume_entry &&

                       name_live && ext_ok && (data&8'h18)==0;

         20,21:clhi<={data,clhi[15:8]};

         26,27:cllo<={data,cllo[15:8]};

         28:entry_size_ok<=data>=54;

         29,30:entry_size_ok<=entry_size_ok || data!=0;

         31:begin

          entry_ok<=0;

          if(entry_ok && candidate_value>=2 && candidate_value<32'h0ffffff0 &&

             (entry_size_ok || data!=0)) begin

           candidate<=1;candidate_cluster<=candidate_value;

           resume_entry<={1'b0,pos[8:5]}+1'b1;

          end

         end

        endcase

       end

       2: case(pos)

        0:magic_ok<=data==8'h42;1:magic_ok<=magic_ok && data==8'h4d;

        2,3,4,5:length<={data,length[31:8]};

        10,11,12,13:pixel_offset<={data,pixel_offset[31:8]};

        14:dib_ok<=data>=40;15,16,17:dib_ok<=dib_ok || data!=0;

        18,19,20,21:width<={data,width[31:8]};

        22,23,24,25:height<={data,height[31:8]};

        26:planes_ok<=data==1;27:planes_ok<=planes_ok && data==0;

        28:bits_ok<=data==24;29:bits_ok<=bits_ok && data==0;

        30:compression_ok<=data==0;31,32,33:compression_ok<=compression_ok && data==0;

       endcase

       3:if(pos>=fat_offset && pos<({1'b0,fat_offset}+10'd4)) fat_word<={data,fat_word[31:8]};

      endcase

     end

     if(sector_end) begin

      if(pos+valid!=512) begin fallback<=0;state<=FINISH;end

      else case(kind) 0:state<=BOOT;1:state<=DIR;2:state<=HEADER;3:state<=FAT;endcase

     end

    end

    BOOT: begin

     if(bps==512 && reserved!=0 && (nfats==1 || nfats==2) && fatsz!=0 &&

        root>=2 && root<32'h0ffffff0 && spc!=0 && (spc&(spc-1'b1))==0) begin

      fat_base<=volume+reserved;data_base<=volume+reserved+(fatsz<<(nfats==2));

      dir_cluster<=root;dir_offset<=0;resume_entry<=0;

      calc<=root;kind<=1;state<=SUB;

     end else if(volume==0 && partition!=0 && sig0==8'h55 && sig1==8'haa) begin

      volume<=partition;request(partition,0);

     end else begin fallback<=1;state<=FINISH;end

    end

    DIR: begin

     if(candidate) begin calc<=candidate_cluster;kind<=2;state<=SUB;end

     else if(end_dir || guard==16'd4095) state<=FINISH;

     else begin

      guard<=guard+1'b1;resume_entry<=0;

      if(dir_offset<spc-1'b1) begin

       dir_offset<=dir_offset+1'b1;dir_sector<=dir_sector+1'b1;

       candidate<=0;end_dir<=0;request(dir_sector+1'b1,1);

      end else request(fat_base+(dir_cluster>>7),3);

     end

    end

    HEADER: begin

     if(header_ok) begin

      found_width<=width[15:0];found_height<=height[15:0];

      found<=1;found_sector<=addr;found_count<=found_count+1'b1;

      if(found_count+1'b1>=target_count || found_count==3) state<=FINISH;

      else read_dir;

     end else read_dir;

    end

    FAT: begin

     if(next_cluster<2 || next_cluster>=32'h0ffffff0 || next_cluster==dir_cluster) state<=FINISH;

     else begin dir_cluster<=next_cluster;dir_offset<=0;calc<=next_cluster;kind<=1;state<=SUB;end

    end

    SUB: begin calc<=calc-2;shift_left<=spc_shift;state<=SHIFT;end

    SHIFT: begin if(shift_left==0) state<=BASE;else begin calc<={calc[30:0],1'b0};shift_left<=shift_left-1'b1;end end

    BASE: begin calc<=data_base+calc;state<=SEND;end

    SEND: begin

     if(kind==1) begin dir_sector<=calc;candidate<=0;end_dir<=0;entry_ok<=0;end

     request(calc,kind);

    end

    FINISH: begin busy<=0;done<=1;state<=IDLE;end

    default:state<=IDLE;

   endcase

  end

 end

endmodule

