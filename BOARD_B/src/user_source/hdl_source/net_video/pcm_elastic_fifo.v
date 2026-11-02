// Single-clock elastic PCM buffer. Reuses the validated synchronous RAM queue.
// Count includes RAM, pending read and the stable output register: capacity is 2048.
module pcm_elastic_fifo (
 input wire clk, rst,
 input wire [31:0] in_data,
 input wire in_valid,
 output wire in_ready,
 output wire [31:0] out_data,
 output wire out_valid,
 input wire out_ready,
 output wire [11:0] level
);
 board_pcm_queue #(.ADDR_BITS(11), .PREFILL(1)) storage (
  .clk(clk), .rst(rst), .in_data(in_data), .in_valid(in_valid),
  .in_ready(in_ready), .stream_end(1'b0), .data(out_data),
  .valid(out_valid), .take(out_ready), .level(level)
 );
endmodule
