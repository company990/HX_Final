`timescale 1ns/1ps
// Two sets x two ways x 128 RGB888 words. All source address bits retained.
// A four-neighbor query still returns four independently captured pixels.
// Two ways avoid vertical-neighbor aliasing at power-of-two source strides.
module bilinear_sdram_fetch #(
 parameter SOURCE_HOLDS=0,parameter ADDR_BITS=21,parameter MEM_DATA_BITS=32
)(
 input clk,rst,invalidate,input req_valid,output req_ready,
 input [ADDR_BITS-1:0] req_addr_p00,req_addr_p10,req_addr_p01,req_addr_p11,
 output resp_valid,input resp_ready,
 output [23:0] resp_pixel_p00,resp_pixel_p10,resp_pixel_p01,resp_pixel_p11,
 input Sdr_init_done,Sdr_init_ref_vld,Sdr_busy,App_wr_busy,
 output App_rd_en,output [ADDR_BITS-1:0] App_rd_addr,
 input Sdr_rd_en,input [MEM_DATA_BITS-1:0] Sdr_rd_dout,
 output memory_busy,fetch_busy,output reg [2:0] issued_count,returned_count,
 output reg unexpected_return_sticky
);
 localparam IDLE=0,LOOKUP=1,TEST=2,CAPTURE=3,FILL=4,DRAIN=5,RESPONSE=6;
 reg [2:0] state;reg [ADDR_BITS-1:0] addresses[0:3];reg [23:0] pixels[0:3];reg [1:0] slot;
 reg [3:0] valid;reg [ADDR_BITS-9:0] tags[0:3];reg [1:0] victim;
 reg [ADDR_BITS-9:0] tag0_q,tag1_q;reg v0_q,v1_q,hit_way;
 reg [ADDR_BITS-1:0] lookup_addr_q,fill_addr;reg fill_set,fill_way;
 reg [ADDR_BITS-9:0] fill_tag;reg [7:0] fill_issued,fill_returned;
 reg [23:0] ram[0:511] /* fehdl force_ram=1, ram_style="bram" */;
 reg [23:0] data_q;
 wire [ADDR_BITS-1:0] source_addr=slot==0 ? req_addr_p00:slot==1 ? req_addr_p10:slot==2 ? req_addr_p01:req_addr_p11;
 wire [ADDR_BITS-1:0] lookup_addr=SOURCE_HOLDS ? source_addr:addresses[slot];
 wire hit0=v0_q && tag0_q==lookup_addr_q[ADDR_BITS-1:8];
 wire hit1=v1_q && tag1_q==lookup_addr_q[ADDR_BITS-1:8];
 wire replacement=!v0_q ? 1'b0:!v1_q ? 1'b1:victim[lookup_addr_q[7]];
 wire accept_return=Sdr_rd_en && fill_returned<fill_issued && (state==FILL || state==DRAIN);
 assign req_ready=state==IDLE && Sdr_init_done && !App_wr_busy && !invalidate;
 assign resp_valid=state==RESPONSE;assign fetch_busy=state!=IDLE;
 assign memory_busy=state==FILL || state==DRAIN;
 assign App_rd_en=state==FILL;assign App_rd_addr=fill_addr;
 assign resp_pixel_p00=pixels[0];assign resp_pixel_p10=pixels[1];
 assign resp_pixel_p01=pixels[2];assign resp_pixel_p11=pixels[3];
 // A port alternates between refill writes and way-0 reads. B reads way 1.
 // There is no concurrent third memory port and no reset on the data array.
 always @(posedge clk)begin
  if(accept_return)ram[{fill_way,fill_set,fill_returned[6:0]}]<=Sdr_rd_dout[31:8];
  else if(state==TEST)data_q<=ram[{!hit0,lookup_addr_q[7:0]}];
 end
 integer k;
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   state<=IDLE;slot<=0;valid<=0;victim<=0;tag0_q<=0;tag1_q<=0;v0_q<=0;v1_q<=0;hit_way<=0;
   lookup_addr_q<=0;fill_addr<=0;fill_set<=0;fill_way<=0;fill_tag<=0;fill_issued<=0;fill_returned<=0;
   issued_count<=0;returned_count<=0;unexpected_return_sticky<=0;
   for(k=0;k<4;k=k+1)begin addresses[k]<=0;pixels[k]<=0;end
  end else begin
   if(invalidate)begin valid<=0;if(state!=IDLE)unexpected_return_sticky<=1;end
   if(Sdr_rd_en && !accept_return)unexpected_return_sticky<=1;
   if(accept_return)begin
    fill_returned<=fill_returned+1'b1;
    if(fill_returned==127)begin tags[{fill_way,fill_set}]<=fill_tag;valid[{fill_way,fill_set}]<=1;state<=LOOKUP;end
   end
   case(state)
    IDLE:if(req_valid && req_ready)begin
     addresses[0]<=req_addr_p00;addresses[1]<=req_addr_p10;addresses[2]<=req_addr_p01;addresses[3]<=req_addr_p11;
     slot<=0;issued_count<=4;returned_count<=0;state<=LOOKUP;
    end
    LOOKUP:begin
     lookup_addr_q<=lookup_addr;
     tag0_q<=tags[{1'b0,lookup_addr[7]}];tag1_q<=tags[{1'b1,lookup_addr[7]}];
     v0_q<=valid[{1'b0,lookup_addr[7]}];v1_q<=valid[{1'b1,lookup_addr[7]}];state<=TEST;
    end
    TEST:begin
     if(hit0 || hit1)begin hit_way<=!hit0;state<=CAPTURE;end
     else if(!App_wr_busy)begin
      fill_addr<={lookup_addr_q[ADDR_BITS-1:7],7'd0};fill_set<=lookup_addr_q[7];fill_way<=replacement;
      fill_tag<=lookup_addr_q[ADDR_BITS-1:8];valid[{replacement,lookup_addr_q[7]}]<=0;
      fill_issued<=0;fill_returned<=0;state<=FILL;
     end
    end
    CAPTURE:begin
     pixels[slot]<=data_q;victim[lookup_addr_q[7]]<=!hit_way;
     returned_count<=returned_count+1'b1;
     if(slot==3)state<=RESPONSE;else begin slot<=slot+1'b1;state<=LOOKUP;end
    end
    FILL:begin fill_addr<=fill_addr+1'b1;fill_issued<=fill_issued+1'b1;if(fill_issued==127)state<=DRAIN;end
    DRAIN:;
    RESPONSE:if(resp_ready)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
