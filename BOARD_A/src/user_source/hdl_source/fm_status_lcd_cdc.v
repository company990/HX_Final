// Read-only FM status. The production A clocks share the 50MHz reference:
// video_clk is divide-by-two, phase zero; LCD clk is the reference itself.
// A source rising edge therefore precedes the LCD falling capture by 10ns.
// Every data bit remains an ordinary, fully constrained STA path. This mode
// intentionally samples the latest coherent status, not an event queue.
// Commands continue through their existing handshake. Unrelated-clock reuse
// defaults to the original held-bus mailbox.
module fm_status_lcd_cdc #(parameter RELATED_CLOCKS=0)(
 input src_clk,dst_clk,rst,input[47:0] source_status,
 output sampled_valid,output[47:0] sampled_status);
 generate if(RELATED_CLOCKS)begin:related
  reg valid;reg[47:0] status;
  always @(negedge dst_clk or posedge rst)
   if(rst)begin valid<=0;status<=0;end
   else begin valid<=1;status<=source_status;end
  assign sampled_valid=valid;assign sampled_status=status;
 end else begin:unrelated
  board_link_mailbox #(.W(48)) mailbox(src_clk,dst_clk,rst,1'b1,
   source_status,sampled_valid,sampled_status);
 end endgenerate
endmodule
