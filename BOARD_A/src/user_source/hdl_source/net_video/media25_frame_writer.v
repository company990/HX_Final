// Gray-pointer dual-clock FIFO; no reset on the storage array.
module media25_fifo(input wire wc,rc,rst,input wire[17:0] din,input wire push,
 output wire ready,output wire almost_full,output wire[17:0] dout,
 output reg valid,input wire pop);
 reg[9:0] wbin,rbin;
 // Keep registered Gray values at the SOURCE. Never allow an XOR/binary
 // counter to be absorbed into the asynchronous capture stage.
 reg[9:0] wgray /* synthesis keep="true" */;
 reg[9:0] rgray /* synthesis keep="true" */;
 reg[9:0] rg0 /* synthesis keep="true" */;
 reg[9:0] rg1 /* synthesis keep="true" */;
 reg[9:0] wg0 /* synthesis keep="true" */;
 reg[9:0] wg1 /* synthesis keep="true" */;wire[17:0] q;
 wire[9:0] wn=wbin+1'b1;
 wire full=((wn>>1)^wn)=={~rg1[9:8],rg1[7:0]};
 function[9:0] ungray;input[9:0] g;integer i;begin ungray[9]=g[9];for(i=8;i>=0;i=i-1)ungray[i]=ungray[i+1]^g[i];end endfunction
 assign ready=!full;
 // Pointer subtraction must wrap at 2*DEPTH, not at unsized integer width.
 wire[9:0] occupancy=wbin-ungray(rg1);
 assign almost_full=occupancy>=10'd448;
 wire fetch=!valid && rgray!=wg1;
 assign dout=q;
 // 512 individual RGB565 pixels + two flags fit one native 512x18 block.
 // Pixel capacity and high-water reserve match the old 256x34 FIFO, which
 // required two physical BRAMs. The MEM side repacks pairs before writing.
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(18),.DATA_WIDTH_B(18),
  .ADDR_WIDTH_A(9),.ADDR_WIDTH_B(9),.DATA_DEPTH_A(512),.DATA_DEPTH_B(512),
  .MODE("PDPW"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),
  .IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) storage(
  .dia(din),.addra(wbin[8:0]),.cea(push && ready),.ocea(1'b0),
  .clka(wc),.wea(push && ready),.rsta(1'b0),.bea(1'b0),.doa(),
  .dib(18'd0),.addrb(rbin[8:0]),.ceb(fetch),.oceb(1'b0),
  .clkb(rc),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(q));
 always @(posedge wc or posedge rst)
  if(rst)begin wbin<=0;wgray<=0;rg0<=0;rg1<=0;end
  else begin rg0<=rgray;rg1<=rg0;if(push&&ready)begin wbin<=wn;wgray<=(wn>>1)^wn;end end
 always @(posedge rc or posedge rst)
  if(rst)begin rbin<=0;wg1<=0;valid<=0;end
  else begin
   wg1<=wg0;
   if(fetch)valid<=1;
   if(valid&&pop)begin valid<=0;rbin<=rbin+1'b1;end
  end
 // The actual clocks are wc=25MHz, rc=50MHz. Capture the write Gray
 // pointer on falling rc, then resolve through wg1 on rising rc (10ns).
 // Register the read Gray pointer on falling rc after rbin has settled;
 // rg0/rg1 stay rising wc. Both cross-domain paths now have a real
 // half-cycle separation instead of relying on unfixable same-edge hold.
 // The RAM addresses and read/write acceptance remain rising-edge logic.
 always @(negedge rc or posedge rst)
  if(rst)begin wg0<=0;rgray<=0;end
  else begin wg0<=wgray;rgray<=(rbin>>1)^rbin;end
endmodule

// Native 640x480 image centered in the existing 640x480 double buffers.
// This retains the existing HDMI reader and avoids fourfold write bandwidth.
module media25_frame_writer #(
 parameter SOURCE_HOLDS=0, PIXEL_STREAM=0,
 parameter [20:0] BASE0=1048576,BASE1=1355776
)(input wire video_clk,mem_clk,rst,
 input wire[95:0] pixels,input wire[2:0] flags,input wire valid,output wire ready,
 input wire due_toggle,input wire allowed,audio_pause,other_busy,
 input wire init_done,refresh,busy,input wire front,frame_activation,
 output wire [1:0] credit_reason,
 output reg active,output reg displayed,output reg error,
 output wire claim,owns,output wire wr_en,output wire[20:0] wr_addr,
 output wire[31:0] wr_data,output reg commit,output reg target);
 reg[95:0] holding;reg[2:0] held_flags;reg[2:0] split_control;
 wire[2:0] split=PIXEL_STREAM?3'd0:split_control;
 wire fifo_ready,af,rv;wire[17:0] rd;wire pop;
 assign ready=PIXEL_STREAM ? (!af && fifo_ready) : SOURCE_HOLDS ? (split==6 && fifo_ready) : (split==0 && !af);
 assign credit_reason={af,(!af && split!=0)};
 wire push=PIXEL_STREAM ? (valid && ready) : split!=0;
 // SOURCE_HOLDS uses ordinary ready/valid: ACK only after six pixel writes.
 // The producer holds its group until that ACK, so no local group copy is
 // required. Default 0 captures a group and ACKs immediately as before.
 // The 448-entry high-water mark reserves all six pixel writes without stalling.
 reg [15:0] held_pixel;
 always @*begin
  case(split)
   1:held_pixel=pixels[15:0];2:held_pixel=pixels[31:16];3:held_pixel=pixels[47:32];
   4:held_pixel=pixels[63:48];5:held_pixel=pixels[79:64];default:held_pixel=pixels[95:80];
  endcase
 end
 wire[15:0] split_data=PIXEL_STREAM?pixels[15:0]:SOURCE_HOLDS?held_pixel:holding[15:0];
 wire[2:0] split_flags=(PIXEL_STREAM || SOURCE_HOLDS)?flags:held_flags;
 wire[17:0] din={(PIXEL_STREAM || split==1) && split_flags[0],split_flags[2],split_data};
 generate if(!PIXEL_STREAM && SOURCE_HOLDS)begin:held_source
  always @(posedge video_clk or posedge rst)
   if(rst)split_control<=0;
   else if(split==0 && valid && !af)split_control<=1;
   else if(push && fifo_ready)split_control<=split==6?0:split+1'b1;
 end else if(!PIXEL_STREAM)begin:captured_source
  always @(posedge video_clk or posedge rst)
   if(rst)begin holding<=0;held_flags<=0;split_control<=0;end
   else if(valid && ready)begin holding<=pixels;held_flags<=flags;split_control<=1;end
   else if(push && fifo_ready)begin holding<={16'd0,holding[95:16]};split_control<=split==6?0:split+1'b1;end
 end endgenerate
 media25_fifo fifo(video_clk,mem_clk,rst,din,push,fifo_ready,af,rd,rv,pop);
 localparam FILL=0,CLAIM=1,WRITE=2,TAIL=3,PRESENT=4,ACTIVATE=5;
 reg[2:0] state;reg[5:0] fill;reg[5:0] beat;reg[4:0] wait_count;
 wire[15:0] q;reg[20:0] address;reg[3:0] group;reg[8:0] row;
 reg d0 /* synthesis keep="true" */;
 reg d1 /* synthesis keep="true" */;
 reg seen_due,due;reg first_frame,first_word;
 wire last_group=group==9 && row==479;
 assign pop=state==FILL && rv && !error;
 assign claim=state==CLAIM || state==WRITE || state==TAIL;
 assign owns=state==WRITE || state==TAIL;
 assign wr_en=state==WRITE;
 // Advance the accepted pixel address directly; share the increment adder.
 assign wr_addr=address;
 wire [20:0] address_step=(state==WRITE)?21'd1:21'd0;
 wire[15:0] rgb=q;
 // Existing image words are {R8,G8,B8,8'b0}; expand by bit replication.
 assign wr_data={rgb[15:11],rgb[15:13],rgb[10:5],rgb[10:9],rgb[4:0],rgb[4:2],8'd0};
 wire cache_re=state==CLAIM || (state==WRITE && beat!=63);
 media25_pixel_cache cache(mem_clk,pop,cache_re,fill,rd[15:0],q);
 // synthesis translate_off
 wire [5:0] required_pixel_address=state==CLAIM?6'd0:(beat+6'd1);
 always @(posedge mem_clk)if(!rst && cache_re && fill!==required_pixel_address)
  $fatal(1,"Pixel cache read-ahead address differs from next SDRAM beat");
 // synthesis translate_on
 // Video25 -> memory50 event capture has a real half-cycle budget.
 // Stage2 and event consumption stay on the original rising memory edge.
 always @(negedge mem_clk or posedge rst)
  if(rst)d0<=0;else d0<=due_toggle;
 always @(posedge mem_clk or posedge rst)begin
  if(rst)begin
   active<=0;displayed<=0;error<=0;commit<=0;target<=0;state<=FILL;fill<=0;beat<=0;wait_count<=0;
   address<=0;group<=0;row<=0;d1<=0;seen_due<=0;due<=1;first_frame<=1;first_word<=1;
  end else begin
   d1<=d0;commit<=0;
   if(d1!=seen_due)begin seen_due<=d1;due<=1;end
   if(state==WRITE || (state==TAIL && wait_count==16 && !last_group && group==9))
    address<=address+address_step;
   case(state)
    FILL:if(pop)begin
     if(first_word)begin
      if(!rd[17])error<=1;
      active<=1;first_word<=0;
     end else if(rd[17])error<=1;
     if(fill==63)begin fill<=0;state<=CLAIM;wait_count<=0;end else fill<=fill+1'b1;
    end
    CLAIM:begin
     if(!allowed||audio_pause||other_busy||!init_done||refresh||busy)wait_count<=0;
     else if(wait_count==16)begin
      state<=WRITE;beat<=0;fill<=1;
      if(row==0 && group==0)begin target<=!front;address<=(front?BASE0:BASE1)+21'd0;end
     end else wait_count<=wait_count+1'b1;
    end
    WRITE:begin
     // Pixel0 was read in CLAIM. Read one following pixel per beat.
     if(beat!=63)fill<=fill+1'b1;
     if(beat==63)begin state<=TAIL;wait_count<=0;end else beat<=beat+1'b1;
    end
    TAIL:if(wait_count==16)begin
     if(last_group)state<=PRESENT;
     else begin
      // WRITE already advanced by 64 words; only the row gap is added above.
      if(group==9)begin group<=0;row<=row+1'b1;end else group<=group+1'b1;
      state<=FILL;
     end
    end else wait_count<=wait_count+1'b1;
    PRESENT:if(first_frame||due)begin commit<=1;due<=0;first_frame<=0;state<=ACTIVATE;end
    ACTIVATE:if(frame_activation && front==target)begin
     displayed<=1;first_word<=1;group<=0;row<=0;state<=FILL;
    end
    default:state<=FILL;
   endcase
  end
 end
endmodule

// Same full 64-pixel burst cache, now in independent 32K RAM resources.
// Clock enable preserves the original synchronous read-hold behavior.
module media25_burst_cache #(parameter COMPACT_PORTS=0)(input clk,we,input [4:0] wa,input [31:0] data,
 input re,input [4:0] ra,output [31:0] q);
`ifdef PRIORITY12_SIM
 reg [31:0] mem[0:31];reg [31:0] read_q;
 always @(posedge clk)begin if(we)mem[wa]<=data;if(re)read_q<=mem[ra];end
 assign q=read_q;
`else
 generate if(COMPACT_PORTS)begin:compact
 // FILL writes both halves; CLAIM/WRITE reads both halves. They never
 // overlap. A read of pair0 is always issued before the first SDRAM beat.
 wire [4:0] selected=we?wa:ra;
 wire [15:0] low_word,high_word;
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(6),.ADDR_WIDTH_B(6),
  .DATA_DEPTH_A(64),.DATA_DEPTH_B(64),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
  .dia(data[15:0]),.addra({selected,1'b0}),.cea(we||re),.ocea(1'b0),.clka(clk),.wea(we),.rsta(1'b0),.bea(1'b0),.doa(low_word),
  .dib(data[31:16]),.addrb({selected,1'b1}),.ceb(we||re),.oceb(1'b0),.clkb(clk),.web(we),.rstb(1'b0),.beb(1'b0),.dob(high_word));
 assign q={high_word,low_word};
 // synthesis translate_off
 always @(posedge clk)if(we && re)$fatal(1,"Burst cache fill/read phases collided");
 // synthesis translate_on
 end else begin:legacy
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(32),.DATA_WIDTH_B(32),.ADDR_WIDTH_A(5),.ADDR_WIDTH_B(5),
  .DATA_DEPTH_A(32),.DATA_DEPTH_B(32),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
  .dia(data),.addra(wa),.cea(we),.ocea(1'b0),.clka(clk),.wea(we),.rsta(1'b0),.bea(1'b0),.doa(),
  .dib(32'd0),.addrb(ra),.ceb(re),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(q));
 end endgenerate
`endif
endmodule

// Read and write phases are mutually exclusive. q while filling is not
// consumed. The first pixel is prefetched throughout CLAIM before wr_en.
module media25_pixel_cache(input clk,we,re,input [5:0] address,input [15:0] data,output [15:0] q);
 // synthesis translate_off
 always @(posedge clk)if(we && re)$fatal(1,"Pixel cache fill/read phases collided");
 // synthesis translate_on
`ifdef PRIORITY12_SIM
 reg [15:0] mem[0:63];reg [15:0] read_q;
 always @(posedge clk)begin if(we)mem[address]<=data;if(re)read_q<=mem[address];end
 assign q=read_q;
`else
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(16),.DATA_WIDTH_B(16),.ADDR_WIDTH_A(6),.ADDR_WIDTH_B(6),
  .DATA_DEPTH_A(64),.DATA_DEPTH_B(64),.MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) ram(
  .dia(data),.addra(address),.cea(we||re),.ocea(1'b0),.clka(clk),.wea(we),.rsta(1'b0),.bea(1'b0),.doa(q),
  .dib(16'd0),.addrb(6'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob());
`endif
endmodule
