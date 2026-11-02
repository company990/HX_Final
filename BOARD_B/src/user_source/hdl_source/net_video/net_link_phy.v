// Shared 19-beat transport; integrated wiring uses 12 MOSI / 12 MISO.
// MOSI_LANES=8 retains the prior PHY for explicit compatibility tests.
// TX: 120 application bits + CRC8; remaining MOSI frame bits are padding.
// RX: 216 application bits + CRC8 + 4 padding bits.
// Both FPGAs use their EXISTING 125 MHz video PLL output. SCK is a strobe,
// never an internal clock. CRC-8/ATM, initial zero, MSB first; padding excluded.
module net_link_master_phy #(
 parameter HALF=4,HIGH_CYCLES=4,RX_POST=4,CS_SETUP_EXTRA=2,APP_HELD=0,SINK_SHIFT=0,CRC_PIPE=0,MOSI_LANES=8
)(input wire clk,rst,start,input wire [119:0] payload,
 input wire [11:0] miso,output reg cs_n,sck,output wire [MOSI_LANES-1:0] mosi,
 output reg busy,valid,output wire [215:0] received,output reg crc_error,input wire sink_shift);
 reg [119:0] captured_tx;reg [31:0] captured_header;reg [215:0] rx;
 // The integrated adapter already holds the 88-bit application request for
 // the complete transaction. Only changing transport metadata needs a copy.
 wire [119:0] tx=APP_HELD?{captured_header,payload[87:0]}:captured_tx;
 reg [7:0] tc,rc;reg [4:0] beat;
 localparam TM=(HALF+CS_SETUP_EXTRA>HIGH_CYCLES)?HALF+CS_SETUP_EXTRA:HIGH_CYCLES;
 reg [$clog2(TM+1)-1:0] timer;
 reg first,last;reg [11:0] mi0,mi1;
 localparam MOSI_DATA_BEATS=120/MOSI_LANES;
 function [7:0] crc_mosi;input [7:0] c;input [MOSI_LANES-1:0] d;
 integer i;reg [7:0] t;
 begin t=c;for(i=MOSI_LANES-1;i>=0;i=i-1)
  t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);crc_mosi=t;end endfunction
 // synthesis translate_off
 initial if(MOSI_LANES!=8 && MOSI_LANES!=12)$fatal(1,"MOSI_LANES must be 8 or 12");
 // synthesis translate_on
 function [7:0] c8;input [7:0] c,d;integer i;reg [7:0] t;
 begin t=c;for(i=7;i>=0;i=i-1)t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);c8=t;end endfunction
 function [7:0] c12;input [7:0] c;input [11:0] d;integer i;reg [7:0] t;
 begin t=c;for(i=11;i>=0;i=i-1)t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);c12=t;end endfunction
 reg [MOSI_LANES-1:0] tx_lane;
 // Fixed byte slices avoid turning an eight-bit word selector into a wide
 // arithmetic barrel shifter under the project's shifter_reg mapping policy.
 always @*begin
  if(MOSI_LANES==12)begin
   case(beat)
    0:tx_lane=tx[119:108];1:tx_lane=tx[107:96];2:tx_lane=tx[95:84];
    3:tx_lane=tx[83:72];4:tx_lane=tx[71:60];5:tx_lane=tx[59:48];
    6:tx_lane=tx[47:36];7:tx_lane=tx[35:24];8:tx_lane=tx[23:12];
    9:tx_lane=tx[11:0];default:tx_lane=0;
   endcase
  end else begin
  case(beat)
   0:tx_lane=tx[119:112];1:tx_lane=tx[111:104];2:tx_lane=tx[103:96];
   3:tx_lane=tx[95:88];4:tx_lane=tx[87:80];5:tx_lane=tx[79:72];
   6:tx_lane=tx[71:64];7:tx_lane=tx[63:56];8:tx_lane=tx[55:48];
   9:tx_lane=tx[47:40];10:tx_lane=tx[39:32];11:tx_lane=tx[31:24];
   12:tx_lane=tx[23:16];13:tx_lane=tx[15:8];14:tx_lane=tx[7:0];
   default:tx_lane=0;
  endcase
  end
 end
 // Byte selection is stable for HALF fast clocks before CRC update.
 // Only the integrated A endpoint enables this optional pipeline.
 reg [MOSI_LANES-1:0] crc_lane;
 always @(posedge clk)crc_lane<=tx_lane;
 // synthesis translate_off
 initial if(CRC_PIPE && HALF<2)$fatal(1,"CRC_PIPE requires HALF >= 2");
 // synthesis translate_on
 assign mosi=!busy?{MOSI_LANES{1'b0}}:beat<MOSI_DATA_BEATS?tx_lane:
  beat==MOSI_DATA_BEATS?{tc,{(MOSI_LANES-8){1'b0}}}:{MOSI_LANES{1'b0}};
 assign received=rx;
 wire sample_rx=busy && !sck && !first && timer==RX_POST-1;
 wire phase_end=timer==((sck?HIGH_CYCLES:(first?HALF+CS_SETUP_EXTRA:HALF))-1);
 // Payload/receive storage need no reset or per-beat load/shift mux. Every
 // CRC-accepted frame overwrites all 216 received bits before valid rises.
 always @(posedge clk)begin
  if(!busy && start)begin
   if(APP_HELD)captured_header<=payload[119:88];else captured_tx<=payload;
  end
  if(sample_rx && !last)rx<={rx[203:0],mi1};
  // The adapter requests this only for a CRC-accepted video packet while
  // PHY idle. Reuse the exact same shift wiring, zero-fill twelve inputs.
  else if(SINK_SHIFT && sink_shift)rx<={rx[203:0],12'd0};
 end
 always @(posedge clk or posedge rst)
 if(rst)begin mi0<=0;mi1<=0;end else begin mi0<=miso;mi1<=mi0;end
 always @(posedge clk or posedge rst)begin
  if(rst)begin cs_n<=1;sck<=0;busy<=0;valid<=0;crc_error<=0;
   tc<=0;rc<=0;beat<=0;timer<=0;first<=0;last<=0;end
  else begin
   valid<=0;crc_error<=0;
   if(!busy)begin
    if(start)begin tc<=0;rc<=0;beat<=0;timer<=0;
     first<=1;last<=0;cs_n<=0;sck<=0;busy<=1;end
   end else begin
    if(sample_rx)begin
     if(!last)begin rc<=c12(rc,mi1);end
     else begin
      cs_n<=1;busy<=0;sck<=0;
      if(mi1[11:4]==rc && mi1[3:0]==0)valid<=1;
      else crc_error<=1;
     end
    end
    if(!(sample_rx && last))begin
     if(phase_end)begin
      timer<=0;
      if(!sck)begin sck<=1;first<=0;if(beat<MOSI_DATA_BEATS)tc<=crc_mosi(tc,CRC_PIPE?crc_lane:tx_lane);end
      else begin
       sck<=0;
       if(beat==18)last<=1;
       else beat<=beat+1'b1;
      end
     end else timer<=timer+1'b1;
    end
   end
  end
 end
endmodule

module net_link_slave_phy #(parameter APP_HELD=0,MOSI_LANES=8)(
 input wire clk,rst,cs_n,sck,input wire [MOSI_LANES-1:0] mosi,
 input wire [215:0] payload,output wire [11:0] miso,
 output reg valid,output wire [119:0] received,output reg crc_error,
 output wire frame_active,output reg frame_sent,output reg frame_loaded,
 output reg [23:0] loaded_header,output wire payload_idle);
 reg cs0,cs1,ck0,ck1,ckd,active,finished,rx_good;
 reg [MOSI_LANES-1:0] di0,di1;reg [119:0] rx;reg [215:0] captured_tx;
 wire [215:0] tx=APP_HELD?payload:captured_tx;
 reg [7:0] tc,rc;reg [4:0] beat;
 wire rising=ck1&&!ckd,falling=!ck1&&ckd;
 localparam MOSI_DATA_BEATS=120/MOSI_LANES;
 function [7:0] crc_mosi;input [7:0] c;input [MOSI_LANES-1:0] d;
 integer i;reg [7:0] t;
 begin t=c;for(i=MOSI_LANES-1;i>=0;i=i-1)
  t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);crc_mosi=t;end endfunction
 // synthesis translate_off
 initial if(MOSI_LANES!=8 && MOSI_LANES!=12)$fatal(1,"MOSI_LANES must be 8 or 12");
 // synthesis translate_on
 function [7:0] c8;input [7:0] c,d;integer i;reg [7:0] t;
 begin t=c;for(i=7;i>=0;i=i-1)t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);c8=t;end endfunction
 function [7:0] c12;input [7:0] c;input [11:0] d;integer i;reg [7:0] t;
 begin t=c;for(i=11;i>=0;i=i-1)t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);c12=t;end endfunction
 reg [11:0] tx_lane;
 always @*begin
  case(beat)
   0:tx_lane=tx[215:204];1:tx_lane=tx[203:192];2:tx_lane=tx[191:180];
   3:tx_lane=tx[179:168];4:tx_lane=tx[167:156];5:tx_lane=tx[155:144];
   6:tx_lane=tx[143:132];7:tx_lane=tx[131:120];8:tx_lane=tx[119:108];
   9:tx_lane=tx[107:96];10:tx_lane=tx[95:84];11:tx_lane=tx[83:72];
   12:tx_lane=tx[71:60];13:tx_lane=tx[59:48];14:tx_lane=tx[47:36];
   15:tx_lane=tx[35:24];16:tx_lane=tx[23:12];17:tx_lane=tx[11:0];
   default:tx_lane=0;
  endcase
 end
 assign miso=!active?12'd0:beat==18?{tc,4'd0}:tx_lane;
 assign frame_active=active;
 // The adapter may refresh its held response only before synchronized CS
 // falls. This also protects the cycle that loads a transaction's header.
 assign payload_idle=cs1 && !active;
 // All fifteen received bytes are already held in rx through the valid
 // pulse and subsequent CDC capture, before the next transaction can write.
 assign received=rx;
 always @(posedge clk)begin
  if(!APP_HELD && !cs1 && !active)captured_tx<=payload;
  if(active && !finished && rising && beat<MOSI_DATA_BEATS)rx<={rx[119-MOSI_LANES:0],di1};
 end
 always @(posedge clk or posedge rst)
 if(rst)begin cs0<=1;cs1<=1;ck0<=0;ck1<=0;di0<=0;di1<=0;end
 else begin cs0<=cs_n;cs1<=cs0;ck0<=sck;ck1<=ck0;di0<=mosi;di1<=di0;end
 always @(posedge clk or posedge rst)begin
  if(rst)begin ckd<=0;active<=0;finished<=0;rx_good<=0;valid<=0;
   crc_error<=0;frame_sent<=0;frame_loaded<=0;loaded_header<=0;
   tc<=0;rc<=0;beat<=0;end
  else begin
   ckd<=ck1;valid<=0;crc_error<=0;frame_sent<=0;frame_loaded<=0;
   if(cs1)begin frame_sent<=active&&finished;active<=0;finished<=0;end
   else if(!active)begin
    active<=1;finished<=0;rx_good<=0;tc<=0;rc<=0;beat<=0;
    frame_loaded<=1;loaded_header<=payload[215:192];
   end else if(!finished)begin
    if(rising)begin
     if(beat<MOSI_DATA_BEATS)rc<=crc_mosi(rc,di1);
     if(beat==MOSI_DATA_BEATS)rx_good<=di1=={rc,{(MOSI_LANES-8){1'b0}}};
     if(beat<18)tc<=c12(tc,tx_lane);
     if(beat==18)begin
      finished<=1;
      if(rx_good)valid<=1;else crc_error<=1;
     end
    end
    if(falling && beat<18)beat<=beat+1'b1;
   end
  end
 end
endmodule

// Candidate-only serial media PHY; legacy PHY remains unchanged.
module net_link_stream_slave_phy #(parameter APP_HELD=0,MOSI_LANES=8)(
 input wire clk,rst,cs_n,sck,input wire [MOSI_LANES-1:0] mosi,
 input wire [215:0] payload,output wire [11:0] miso,
 output reg valid,output wire [119:0] received,output reg crc_error,
 output wire frame_active,output reg frame_sent,output reg frame_loaded,
 output reg [23:0] loaded_header,output wire payload_idle,
 input wire [11:0] media_word,output wire [3:0] media_read_index);
 reg cs0,cs1,ck0,ck1,ckd,active,finished,rx_good;
 reg [MOSI_LANES-1:0] di0,di1;reg [119:0] rx;reg [215:0] captured_tx;
 wire [215:0] tx=APP_HELD?payload:captured_tx;
 reg [7:0] tc,rc;reg [4:0] beat;
 wire rising=ck1&&!ckd,falling=!ck1&&ckd;
 localparam MOSI_DATA_BEATS=120/MOSI_LANES;
 function [7:0] crc_mosi;input [7:0] c;input [MOSI_LANES-1:0] d;
 integer i;reg [7:0] t;
 begin t=c;for(i=MOSI_LANES-1;i>=0;i=i-1)
  t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);crc_mosi=t;end endfunction
 // synthesis translate_off
 initial if(MOSI_LANES!=8 && MOSI_LANES!=12)$fatal(1,"MOSI_LANES must be 8 or 12");
 // synthesis translate_on
 function [7:0] c8;input [7:0] c,d;integer i;reg [7:0] t;
 begin t=c;for(i=7;i>=0;i=i-1)t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);c8=t;end endfunction
 function [7:0] c12;input [7:0] c;input [11:0] d;integer i;reg [7:0] t;
 begin t=c;for(i=11;i>=0;i=i-1)t={t[6:0],1'b0}^((t[7]^d[i])?8'h07:8'h00);c12=t;end endfunction
 reg [11:0] tx_lane;
 always @*begin
  case(beat)
   0:tx_lane=loaded_header[23:12];1:tx_lane=loaded_header[11:0];2:tx_lane=tx[191:180];
   3:tx_lane=tx[179:168];4:tx_lane=tx[167:156];5:tx_lane=tx[155:144];
   6:tx_lane=tx[143:132];7:tx_lane=tx[131:120];8:tx_lane=tx[119:108];
   9:tx_lane=tx[107:96];10:tx_lane=tx[95:84];11:tx_lane=tx[83:72];
   12:tx_lane=tx[71:60];13:tx_lane=tx[59:48];14:tx_lane=tx[47:36];
   15:tx_lane=tx[35:24];16:tx_lane=tx[23:12];17:tx_lane=tx[11:0];
   default:tx_lane=0;
  endcase
 end
 // Header words and status packets retain the original fixed slices.
 // Video words are read MSB first from the retained sixteen-word slot.
 assign media_read_index=beat<2 ? 4'd15:5'd17-beat;
 wire [11:0] wire_lane=(loaded_header[23:16]==8'h56 && beat>=2 && beat<18)?media_word:tx_lane;
 // The link has four fast clocks per half SCK. A lane is stable before
 // its rising edge; this register separates slice selection from CRC XORs.
 reg [11:0] crc_lane;
 always @(posedge clk)crc_lane<=wire_lane;
 assign miso=!active?12'd0:beat==18?{tc,4'd0}:wire_lane;
 assign frame_active=active;
 // The adapter may refresh its held response only before synchronized CS
 // falls. This also protects the cycle that loads a transaction's header.
 assign payload_idle=cs1 && !active;
 // All fifteen received bytes are already held in rx through the valid
 // pulse and subsequent CDC capture, before the next transaction can write.
 assign received=rx;
 always @(posedge clk)begin
  if(!APP_HELD && !cs1 && !active)captured_tx<=payload;
  if(active && !finished && rising && beat<MOSI_DATA_BEATS)rx<={rx[119-MOSI_LANES:0],di1};
 end
 always @(posedge clk or posedge rst)
 if(rst)begin cs0<=1;cs1<=1;ck0<=0;ck1<=0;di0<=0;di1<=0;end
 else begin cs0<=cs_n;cs1<=cs0;ck0<=sck;ck1<=ck0;di0<=mosi;di1<=di0;end
 always @(posedge clk or posedge rst)begin
  if(rst)begin ckd<=0;active<=0;finished<=0;rx_good<=0;valid<=0;
   crc_error<=0;frame_sent<=0;frame_loaded<=0;loaded_header<=0;
   tc<=0;rc<=0;beat<=0;end
  else begin
   ckd<=ck1;valid<=0;crc_error<=0;frame_sent<=0;frame_loaded<=0;
   if(cs1)begin frame_sent<=active&&finished;active<=0;finished<=0;end
   else if(!active)begin
    active<=1;finished<=0;rx_good<=0;tc<=0;rc<=0;beat<=0;
    frame_loaded<=1;loaded_header<=payload[215:192];
   end else if(!finished)begin
    if(rising)begin
     if(beat<MOSI_DATA_BEATS)rc<=crc_mosi(rc,di1);
     if(beat==MOSI_DATA_BEATS)rx_good<=di1=={rc,{(MOSI_LANES-8){1'b0}}};
     if(beat<18)tc<=c12(tc,crc_lane);
     if(beat==18)begin
      finished<=1;
      if(rx_good)valid<=1;else crc_error<=1;
     end
    end
    if(falling && beat<18)beat<=beat+1'b1;
   end
  end
 end
endmodule
