// Restart only the private audio datapath. Shared SD/SDRAM finish in flight.
// Name is latched at P and held across reset release and the entire playback.
module audio_uart_session #(parameter [87:0] INITIAL_NAME="MUSIC   WAV")(
 input clk_a,clk_b,clk_c,rst,input play_cmd,
 input [87:0] selected_name,input sd_idle,input mem_idle,
 output wire [87:0] active_name,output wire [87:0] current_name_c,output reg manual,
 output rst_a,rst_b,rst_c,output busy
);
 reg [87:0] name_c,name_a;
 assign active_name=name_a;
 assign current_name_c=name_c;
 // The board clocks share a PLL reference. Capture the held bundle on the
 // opposite A edge; STA checks the half-cycle path without a false path.
 always @(negedge clk_a or posedge rst)
  if(rst) name_a<=INITIAL_NAME;else name_a<=name_c;
 reg hold_reset;
 reg [5:0] count;
 reg sd0,sd1,mem0,mem1;
 reg [2:0] ar,br,cr;
// Release private async resets halfway between datapath rising edges.
 reg hold_a,hold_b;
 assign rst_a=ar[2];assign rst_b=br[2];assign rst_c=cr[2];
 assign busy=hold_reset;
 // Clocks A/B/C are PLL-derived from the board's common 50 MHz reference.
 // Time the C->A/B requests through the opposite edge, like the name bundle.
 // Only the global reset is asynchronous. Session reset is generated locally,
 // avoiding a video-clock signal on thousands of foreign async-reset pins.
 always @(negedge clk_a or posedge rst)
  if(rst) hold_a<=1;else hold_a<=hold_reset;
 always @(negedge clk_b or posedge rst)
  if(rst) hold_b<=1;else hold_b<=hold_reset;
 always @(negedge clk_a or posedge rst)
  if(rst) ar<=3'b111;else if(hold_a) ar<=3'b111;else ar<={ar[1:0],1'b0};
 always @(negedge clk_b or posedge rst)
  if(rst) br<=3'b111;else if(hold_b) br<=3'b111;else br<={br[1:0],1'b0};
 always @(negedge clk_c or posedge rst)
  if(rst) cr<=3'b111;else if(hold_reset) cr<=3'b111;else cr<={cr[1:0],1'b0};
 always @(posedge clk_c or posedge rst) begin
  if(rst) begin
   hold_reset<=0;count<=0;manual<=0;name_c<=INITIAL_NAME;
   sd0<=0;sd1<=0;mem0<=0;mem1<=0;
  end else begin
   sd0<=sd_idle;sd1<=sd0;mem0<=mem_idle;mem1<=mem0;
   if(play_cmd) begin
    name_c<=selected_name;manual<=1;hold_reset<=1;count<=0;
   end else if(hold_reset) begin
    if(count<32) count<=count+1'b1;
    else if(sd1 && mem1) hold_reset<=0;
   end
  end
 end
endmodule
