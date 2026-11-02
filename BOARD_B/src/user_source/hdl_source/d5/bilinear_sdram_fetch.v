`timescale 1ns/1ps
// D9: 2048 RGB888 words, 16 direct-mapped sectors of 128 words.
// Sequential application runs match the V1 transport granularity. Sector tags
// make width/stride irrelevant: this also supports rows wider than the cache.
// All four neighbour addresses are immutable until response consumption.
// Only one fill is outstanding; return offset is its ordered tag FIFO.
module bilinear_sdram_fetch #(
 parameter ADDR_BITS=21, parameter MEM_DATA_BITS=32
)(
 input clk, input rst, input invalidate,
 input req_valid, output req_ready,
 input [ADDR_BITS-1:0] req_addr_p00,req_addr_p10,req_addr_p01,req_addr_p11,
 output resp_valid,input resp_ready,
 output [23:0] resp_pixel_p00,resp_pixel_p10,resp_pixel_p01,resp_pixel_p11,
 input Sdr_init_done,Sdr_init_ref_vld,Sdr_busy,App_wr_busy,
 output App_rd_en,output [ADDR_BITS-1:0] App_rd_addr,
 input Sdr_rd_en,input [MEM_DATA_BITS-1:0] Sdr_rd_dout,
 output fetch_busy,output reg [2:0] issued_count,returned_count,
 output reg unexpected_return_sticky
);
 localparam IDLE=0, LOOKUP=1, TEST=2, CAPTURE=3, FILL=4, DRAIN=5, RESPONSE=6;
 reg [2:0] state;
 reg [ADDR_BITS-1:0] addresses[0:3];
 reg [23:0] pixels[0:3];
 reg [1:0] slot;
 reg [15:0] valid;
 reg [ADDR_BITS-12:0] tags[0:15];
 reg [ADDR_BITS-12:0] tag_q;
 reg valid_q;
 reg [23:0] ram[0:2047];
 reg [23:0] ram_q;
 wire [ADDR_BITS-1:0] lookup_addr=addresses[slot];
 reg [ADDR_BITS-1:0] lookup_addr_q;
 reg [ADDR_BITS-1:0] fill_addr;
 reg [3:0] fill_index;
 reg [ADDR_BITS-12:0] fill_tag;
 reg [7:0] fill_issued,fill_returned;
 wire accept_return=Sdr_rd_en && (fill_returned<fill_issued) &&
                    ((state==FILL)||(state==DRAIN));
 assign req_ready=(state==IDLE)&&Sdr_init_done&&!App_wr_busy&&!invalidate;
 assign resp_valid=(state==RESPONSE);
 assign fetch_busy=(state!=IDLE);
 assign App_rd_en=(state==FILL);
 assign App_rd_addr=fill_addr;
 assign resp_pixel_p00=pixels[0];
 assign resp_pixel_p10=pixels[1];
 assign resp_pixel_p01=pixels[2];
 assign resp_pixel_p11=pixels[3];

 // Synchronous simple dual-port BRAM. No array reset or combinational read.
 always @(posedge clk) begin
   if(state==TEST) ram_q<=ram[lookup_addr_q[10:0]];
   if(accept_return)
     ram[{fill_index,fill_returned[6:0]}]<=Sdr_rd_dout[31:8];
 end
 integer k;
 always @(posedge clk or posedge rst) begin
   if(rst) begin
     state<=IDLE; valid<=0; slot<=0; tag_q<=0; valid_q<=0; lookup_addr_q<=0;
     fill_addr<=0; fill_index<=0; fill_tag<=0;
     fill_issued<=0; fill_returned<=0;
     issued_count<=0; returned_count<=0; unexpected_return_sticky<=0;
     for(k=0;k<4;k=k+1) begin addresses[k]<=0; pixels[k]<=0; end
   end else begin
     // Invalidate is legal only between completely drained transactions.
     // Do not cancel in-flight returns on an erroneous mid-request invalidate.
     if(invalidate) begin
       valid<=0;
       if(state!=IDLE) unexpected_return_sticky<=1;
     end
     if(Sdr_rd_en&&!accept_return) unexpected_return_sticky<=1;
     if(accept_return) begin
       fill_returned<=fill_returned+1'b1;
       if(fill_returned==127) begin
         tags[fill_index]<=fill_tag;
         valid[fill_index]<=1;
         state<=LOOKUP;
       end
     end
     case(state)
       IDLE: if(req_valid&&req_ready) begin
         addresses[0]<=req_addr_p00; addresses[1]<=req_addr_p10;
         addresses[2]<=req_addr_p01; addresses[3]<=req_addr_p11;
         slot<=0; issued_count<=4; returned_count<=0; state<=LOOKUP;
       end
       LOOKUP: begin
         // Capture the selected address alongside its tag and valid bit.
         // TEST no longer combines the four-neighbour mux with tag compare.
         lookup_addr_q<=lookup_addr;
         tag_q<=tags[lookup_addr[10:7]];
         valid_q<=valid[lookup_addr[10:7]];
         state<=TEST;
       end
       TEST: begin
         if(valid_q&&(tag_q==lookup_addr_q[ADDR_BITS-1:11])) state<=CAPTURE;
         else if (!App_wr_busy) begin
           // A pending miss waits here while audio owns the memory port.
           fill_addr<={lookup_addr_q[ADDR_BITS-1:7],7'b0};
           fill_index<=lookup_addr_q[10:7];
           fill_tag<=lookup_addr_q[ADDR_BITS-1:11];
           valid[lookup_addr_q[10:7]]<=0;
           fill_issued<=0; fill_returned<=0; state<=FILL;
         end
       end
       CAPTURE: begin
         pixels[slot]<=ram_q;
         returned_count<=returned_count+1'b1;
         if(slot==3) state<=RESPONSE;
         else begin slot<=slot+1'b1; state<=LOOKUP; end
       end
       FILL: begin
         fill_addr<=fill_addr+1'b1;
         fill_issued<=fill_issued+1'b1;
         if(fill_issued==127) state<=DRAIN;
       end
       DRAIN: begin end
       RESPONSE: if(resp_ready) state<=IDLE;
       default: state<=IDLE;
     endcase
   end
 end
endmodule
