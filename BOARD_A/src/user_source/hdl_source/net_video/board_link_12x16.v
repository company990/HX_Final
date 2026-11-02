// Eleven 12-bit SDR beats in both directions.
// MOSI: 88 payload + CRC8 + 36 zero padding bits.
// MISO: 120 payload + CRC8 + 4 zero padding bits. Padding is outside CRC.
// CRC-8/ATM, init zero, MSB first. No new clock domain.
module board_link_master_12x12 #(
 parameter HALF=4, HIGH_CYCLES=4, RX_POST=4, CS_SETUP_EXTRA=2
)(input wire clk,rst,start,input wire [87:0] payload,
 input wire [11:0] miso,output reg cs_n,sck,output wire [11:0] mosi,
 output reg busy,valid,output wire [119:0] received,output reg crc_error);
 reg [95:0] tx; reg [119:0] rx;
 reg [7:0] tc,rc;
 reg [3:0] beat;
 localparam TM=(HALF+CS_SETUP_EXTRA>HIGH_CYCLES)?HALF+CS_SETUP_EXTRA:HIGH_CYCLES;
 reg [$clog2(TM+1)-1:0] timer;
 reg first,last;
 reg [11:0] mi0,mi1;
 function [7:0] crc12;
 input [7:0] c; input [11:0] d; input final_lane;
 integer i; reg [7:0] t;
 begin t=c; for(i=11;i>=0;i=i-1) if(!final_lane || i>=8)
 t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00); crc12=t;end
 endfunction
 wire [11:0] lane=tx[95:84];
 assign mosi=(!busy || beat>7)?12'd0:((beat==7)?{lane[11:8],crc12(tc,lane,1'b1)}:lane);
 assign received=rx;
 wire sample_rx=busy && !sck && !first && timer==RX_POST-1;
 wire phase_end=timer==((sck?HIGH_CYCLES:(first?HALF+CS_SETUP_EXTRA:HALF))-1);
 always @(posedge clk or posedge rst)
 if(rst)begin mi0<=0;mi1<=0;end else begin mi0<=miso;mi1<=mi0;end
 always @(posedge clk)if(sample_rx && !last)rx<={rx[107:0],mi1};
 always @(posedge clk or posedge rst)begin
 if(rst)begin cs_n<=1;sck<=0;busy<=0;valid<=0;crc_error<=0;
 tx<=0;tc<=0;rc<=0;beat<=0;timer<=0;first<=0;last<=0;end
 else begin
 valid<=0;crc_error<=0;
 if(!busy)begin
 if(start)begin tx<={payload,8'd0};tc<=0;rc<=0;beat<=0;timer<=0;
 first<=1;last<=0;cs_n<=0;sck<=0;busy<=1;end
 end else begin
 if(sample_rx)begin
 if(!last)rc<=crc12(rc,mi1,1'b0);
 if(last)begin cs_n<=1;busy<=0;sck<=0;
 if(mi1[11:4]==rc && mi1[3:0]==0)valid<=1;else crc_error<=1;
 end
 end
 if(!(sample_rx && last))begin
 if(phase_end)begin timer<=0;
 if(!sck)begin sck<=1;first<=0;if(beat<7)tc<=crc12(tc,lane,1'b0);end
 else begin sck<=0;if(beat==10)last<=1;
 else begin beat<=beat+1'b1;tx<={tx[83:0],12'd0};end end
 end else timer<=timer+1'b1;
 end
 end
 end
 end
endmodule

module board_link_slave_12x12(
 input wire clk,rst,cs_n,sck,input wire [11:0] mosi,input wire [119:0] payload,
 output wire [11:0] miso,output reg valid,output reg [87:0] received,
 output reg crc_error,media_sent,status7a_sent,output wire frame_active);
 reg cs0,cs1,ck0,ck1,ckd,active,finished;
 reg [11:0] di0,di1; reg [95:0] rx;reg [131:0] tx;
 reg [7:0] tc,rc;reg [3:0] beat;
 reg loaded_media,loaded_status;
 wire rising=ck1&&!ckd,falling=!ck1&&ckd;
 function [7:0] crc12;
 input [7:0] c; input [11:0] d; input final_lane;
 integer i;reg[7:0]t;
 begin t=c;for(i=11;i>=0;i=i-1)if(!final_lane||i>=8)
 t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);crc12=t;end
 endfunction
 wire [11:0] lane=tx[131:120];
 assign miso=!active?12'd0:((beat==10)?{tc,4'd0}:lane);
 assign frame_active=active;
 always @(posedge clk or posedge rst)
 if(rst)begin cs0<=1;cs1<=1;ck0<=0;ck1<=0;di0<=0;di1<=0;end
 else begin cs0<=cs_n;cs1<=cs0;ck0<=sck;ck1<=ck0;di0<=mosi;di1<=di0;end
 always @(posedge clk or posedge rst)begin
 if(rst)begin ckd<=0;active<=0;finished<=0;valid<=0;received<=0;crc_error<=0;
 media_sent<=0;status7a_sent<=0;loaded_media<=0;loaded_status<=0;
 rx<=0;tx<=0;tc<=0;rc<=0;beat<=0;end
 else begin
 ckd<=ck1;valid<=0;crc_error<=0;media_sent<=0;status7a_sent<=0;
 if(cs1)begin
 media_sent<=active&&finished&&loaded_media;status7a_sent<=active&&finished&&loaded_status;
 active<=0;finished<=0;
 end else if(!active)begin
 active<=1;finished<=0;tx<={payload,12'd0};rx<=0;tc<=0;rc<=0;beat<=0;
 loaded_media<=payload[47:40]==8'h7b || (payload[47:40]==8'h7a&&payload[49]);
 loaded_status<=payload[47:40]==8'h7a;
 end else if(!finished)begin
 if(rising)begin
 if(beat<=7)begin rx<={rx[83:0],di1};rc<=crc12(rc,di1,beat==7);end
 // Do not update the final TX CRC: the last MISO lane must remain stable.
 if(beat<10)tc<=crc12(tc,lane,1'b0);
 if(beat==10)finished<=1;
 if(beat==7)begin
 if(di1[7:0]==crc12(rc,di1,1'b1))begin received<={rx[83:0],di1[11:8]};valid<=1;end
 else crc_error<=1;
 end
 end
 if(falling)begin beat<=beat+1'b1;tx<={tx[119:0],12'd0};end
 end
 end
 end
endmodule
