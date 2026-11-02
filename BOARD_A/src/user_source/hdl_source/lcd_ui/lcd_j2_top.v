// Physical-only standalone top. UI event ports remain at the child instance
// for future application integration; no event pins are routed to the header.
// Confirmed NT35510 panel with the complete vendor initialization ROM.
module lcd_j2_top #(
 parameter integer PROFILE_CONFIRMED=1
)(
 input wire clk,rst_n,
 inout wire [15:0] lcd_d,
 output wire lcd_cs_n,lcd_rs,lcd_wr_n,lcd_rd_n,lcd_rst_n,lcd_bl,
 inout wire tp_scl,tp_sda,tp_int_n,
 output wire tp_rst_n
);
 lcd_touch_demo_top #(.PROFILE_CONFIRMED(PROFILE_CONFIRMED)) demo(
  .clk(clk),.rst_n(rst_n),.lcd_d(lcd_d),.lcd_cs_n(lcd_cs_n),.lcd_rs(lcd_rs),
  .lcd_wr_n(lcd_wr_n),.lcd_rd_n(lcd_rd_n),.lcd_rst_n(lcd_rst_n),.lcd_bl(lcd_bl),
  .tp_scl(tp_scl),.tp_sda(tp_sda),.tp_int_n(tp_int_n),.tp_rst_n(tp_rst_n),
  .action_valid(),.action_id(),.init_done(),.init_fault(),.touch_fault(),.touch_identified());
endmodule
