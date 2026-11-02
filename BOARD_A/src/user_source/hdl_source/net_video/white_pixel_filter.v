// Selective horizontal RGB888 impulse suppression. No line RAM.
// ENABLE=0: exact combinational passthrough. Enabled: RGB/DE/VS delayed together.
module white_pixel_filter #(parameter ENABLE=0,parameter REGISTER_OUTPUT=1,parameter RUNTIME_ENABLE=0)(
 input wire clk,rst,input wire [23:0] in_rgb,input wire in_de,in_vs,
 output wire [23:0] out_rgb,output wire out_de,out_vs,input wire filter_enable);
generate if(ENABLE) begin:g_filter
 reg [23:0] l,c;
 reg ld,cd,cv,enabled_c;
 wire white_c=&{c[23:20],c[15:12],c[7:4]};
 // similar guarantees matching high nibbles. Left <192 therefore also
 // proves right <192; checking both duplicates the same condition.
 wire dark_lr=(l[23:16]<192)&&(l[15:8]<192)&&(l[7:0]<192);
 // Same upper nibble in each channel: cheap, conservative similarity test.
 wire similar=({l[23:20],l[15:12],l[7:4]}=={in_rgb[23:20],in_rgb[15:12],in_rgb[7:4]});
 wire replace_pixel=(!RUNTIME_ENABLE || enabled_c)&&ld&&cd&&in_de&&white_c&&dark_lr&&similar;
 // When replacing, neighbor high nibbles match. Their mean keeps that
 // nibble, so only the low nibbles need addition. Exact arithmetic identity.
 wire [4:0] r={1'b0,l[19:16]}+{1'b0,in_rgb[19:16]};
 wire [4:0] g={1'b0,l[11:8]}+{1'b0,in_rgb[11:8]};
 wire [4:0] b={1'b0,l[3:0]}+{1'b0,in_rgb[3:0]};
 wire [23:0] selected=replace_pixel?{l[23:20],r[4:1],l[15:12],g[4:1],l[7:4],b[4:1]}:c;
 always @(posedge clk or posedge rst) begin
  if(rst) begin l<=0;c<=0;ld<=0;cd<=0;cv<=0;enabled_c<=0;end
  else begin
   l<=c;ld<=cd;c<=in_rgb;cd<=in_de;cv<=in_vs;enabled_c<=filter_enable;

  end
 end
 if(REGISTER_OUTPUT)begin:g_out
  reg [23:0] q;reg qd,qv;
  always @(posedge clk or posedge rst)begin
   if(rst)begin q<=0;qd<=0;qv<=0;end
   else begin q<=selected;qd<=cd;qv<=cv;end
  end
  assign out_rgb=q;assign out_de=qd;assign out_vs=qv;
 end else begin:g_comb
  // Must feed an existing clocked consumer. Current in_rgb is right neighbor;
  // c is center from preceding cycle; l is left from two cycles earlier.
  assign out_rgb=selected;assign out_de=cd;assign out_vs=cv;
 end
end else begin:g_bypass
 assign out_rgb=in_rgb;assign out_de=in_de;assign out_vs=in_vs;
end endgenerate
endmodule
