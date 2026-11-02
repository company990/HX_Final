// One owner for the proven FM controller: LCD/link commands and UART arbitrate
// before handshaking. Accepted and completed sequence IDs are distinct.
module fm_system_control #(parameter FM_ENABLE=1,SCAN_ENABLE=0,FM_GAIN_DEFAULT=2,DSP_ENABLE=0)(
 input clk,rst,legacy_net,legacy_mic,
 input link_valid,input [2:0] link_op,input [15:0] link_value,input [7:0] link_seq,
 output link_ready,
 input uart_valid,input [2:0] uart_op,input [15:0] uart_value,output uart_ready,
 output reg chip_valid,input chip_ready,input [3:0] chip_error,
 output reg [2:0] chip_op,output reg [15:0] chip_value,
 input [79:0] chip_status,input fm_powered,adc_fault,adc_clipped,overflow,
 output [1:0] audio_source,fft_source,output reg mic_monitor,
 output [127:0] status,output busy,
 input scan_busy,output reg scan_valid,output reg [15:0] scan_value,
 input scan_ready,scan_done,input [3:0] scan_error,
  output reg dsp_valid,output reg [15:0] dsp_value,input dsp_ready,dsp_done,input [3:0] dsp_error,
 output reg [2:0] fm_gain_shift,output reg rf_quiet_toggle);
 reg [2:0] state;
 reg remote;
 reg [7:0] ack_seq,done_seq,active_seq;
 reg [3:0] result_error;
 reg [1:0] source_config,fft_config;
 reg override_route,legacy_net_d;
 assign busy=state!=0;
 assign link_ready=state==0;
 assign uart_ready=state==0 && !link_valid;
 wire accept=(link_valid && link_ready) || (uart_valid && uart_ready);
 wire [2:0] op=link_valid ? link_op : uart_op;
 wire [15:0] value=link_valid ? link_value : uart_value;
 wire [1:0] desired_source=override_route ? source_config : legacy_net ? 3 : 0;
 assign audio_source=desired_source==1 && !mic_monitor ? 0 : desired_source;
 assign fft_source=override_route ? fft_config : legacy_mic ? 1 : legacy_net ? 3 : 0;
 assign status={ack_seq,done_seq,busy,result_error,audio_source,fft_source,
                mic_monitor,fm_powered,adc_fault,adc_clipped,overflow,override_route,
                (FM_ENABLE!=0),fm_gain_shift,rf_quiet_toggle,12'd0,chip_status};
 always @(posedge clk or posedge rst)begin
  if(rst)begin state<=0;remote<=0;ack_seq<=0;done_seq<=0;active_seq<=0;result_error<=0;
   chip_valid<=0;chip_op<=0;chip_value<=0;source_config<=0;fft_config<=0;
   mic_monitor<=0;override_route<=0;legacy_net_d<=0;scan_valid<=0;scan_value<=0;dsp_valid<=0;dsp_value<=0;fm_gain_shift<=FM_GAIN_DEFAULT;rf_quiet_toggle<=0;
  end else begin
   legacy_net_d<=legacy_net;
   if(legacy_net!=legacy_net_d)override_route<=0;
   case(state)
    0:if(accept)begin
     remote<=link_valid;active_seq<=link_seq;
     if(link_valid)ack_seq<=link_seq;
     if(op==7)begin
      if(DSP_ENABLE && value[15:4]==12'h400)begin dsp_valid<=1;dsp_value<=value;state<=5;end
      else if(value<=31)begin source_config<=value[1:0];fft_config<=value[3:2];mic_monitor<=value[4];override_route<=1;result_error<=0;if(link_valid)done_seq<=link_seq;end
      else if(value>=16'h1000 && value<=16'h1004)begin
       fm_gain_shift<=value[2:0];result_error<=0;if(link_valid)done_seq<=link_seq;
      end
      else if(value>=16'h1100 && value<=16'h1103 && FM_ENABLE)begin
       if(SCAN_ENABLE && scan_busy)begin result_error<=9;if(link_valid)done_seq<=link_seq;end
       else begin chip_valid<=1;chip_op<=7;chip_value<=16'h0100 | {14'd0,value[1:0]};state<=1;end
      end
      else if(value==16'h1201)begin
       if(SCAN_ENABLE && scan_busy)result_error<=9;
       else begin rf_quiet_toggle<=~rf_quiet_toggle;result_error<=0;end
       if(link_valid)done_seq<=link_seq;
      end
      else if(SCAN_ENABLE && FM_ENABLE && value[15])begin scan_valid<=1;scan_value<=value;state<=3;end
      else begin result_error<=6;if(link_valid)done_seq<=link_seq;end
     end else if(!FM_ENABLE)begin result_error<=7;if(link_valid)done_seq<=link_seq;end
     else if(SCAN_ENABLE && scan_busy)begin result_error<=9;if(link_valid)done_seq<=link_seq;end
     else begin chip_valid<=1;chip_op<=op;chip_value<=value;state<=1;end
    end
    1:if(chip_ready)begin chip_valid<=0;state<=2;end
    2:if(chip_ready)begin result_error<=chip_error;if(remote)done_seq<=active_seq;state<=0;end
    3:if(scan_ready)begin scan_valid<=0;state<=4;end
    4:if(scan_done)begin
     result_error<=scan_error;if(remote)done_seq<=active_seq;state<=0;
     if(scan_error==0 && scan_value[14:12]==3)begin source_config<=2;fft_config<=2;override_route<=1;end
    end
    5:if(dsp_ready)begin dsp_valid<=0;state<=6;end
    6:if(dsp_done)begin result_error<=dsp_error;if(remote)done_seq<=active_seq;state<=0;
     if(dsp_error==0 && (dsp_value[3:0]==0 || dsp_value[3:0]==1 || dsp_value[3:0]==12))begin source_config<=audio_source;fft_config<=audio_source;override_route<=1;end
    end
    default:begin dsp_valid<=0;chip_valid<=0;result_error<=7;state<=0;end
   endcase
  end
 end
endmodule
