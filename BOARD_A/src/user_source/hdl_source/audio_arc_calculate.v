


// Fractional clock enable; average rate is exactly SAMPLE_HZ for CLK_HZ.
// No generated clock: downstream logic stays in the pixel clock domain.
module audio_sample_tick #(
 parameter CLK_HZ=25000000,parameter SAMPLE_HZ=48000
)(input wire clk,rst,output wire tick);
 function [31:0] gcd32;
  input [31:0] a,b;reg [31:0] x,y,t;integer i;
  begin x=a;y=b;
   for(i=0;i<64;i=i+1)if(y!=0)begin t=x%y;x=y;y=t;end
   gcd32=x==0?32'd1:x;
  end
 endfunction
 generate if(CLK_HZ>0 && SAMPLE_HZ>0 && SAMPLE_HZ<=CLK_HZ)begin:normalized
  // phase_original == DIV * phase on EVERY clock after reset.
  // At25MHz/48kHz: step6, period3125 instead of48000/25000000.
  localparam [31:0] DIV=gcd32(CLK_HZ,SAMPLE_HZ);
  localparam [31:0] PERIOD=CLK_HZ/DIV,STEP=SAMPLE_HZ/DIV,LIMIT=PERIOD-STEP;
  localparam integer BITS=PERIOD<=1?1:$clog2(PERIOD);
  reg [BITS-1:0] phase;
  assign tick=!rst && phase>=LIMIT;
  always @(posedge clk or posedge rst)
   if(rst)phase<=0;else if(tick)phase<=phase-LIMIT;else phase<=phase+STEP;
 end else begin:legacy
  // Preserve the exact original32bit semantics for invalid/special parameters.
  reg [31:0] phase;
  assign tick=!rst && phase>=CLK_HZ-SAMPLE_HZ;
  always @(posedge clk or posedge rst)
   if(rst)phase<=0;else if(tick)phase<=phase-(CLK_HZ-SAMPLE_HZ);else phase<=phase+SAMPLE_HZ;
 end endgenerate
endmodule

module audio_arc_calculate #(
    parameter ACR_N = 6144
    )(
    input wire       I_clk,
    input wire       I_rst,

    input wire       I_audio_valid,

    output reg       O_acr_valid,
    output reg[19:0] O_acr_cts,
    output reg[19:0] O_acr_n
);


    localparam AUDIO_DIV = ACR_N >> 7;

    reg[7:0]  S_audio_div_cnt;
    wire      S_audio_valid_div;
    reg[19:0] S_acr_cts_cnt;


    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst)
            S_audio_div_cnt <= 'd0;
        else
            if(I_audio_valid)
                begin
                    if(S_audio_div_cnt >= AUDIO_DIV-1)
                        S_audio_div_cnt <= 'd0;
                    else
                        S_audio_div_cnt <= S_audio_div_cnt + 'd1;
                end
            else
                S_audio_div_cnt <= S_audio_div_cnt;
    end


    assign S_audio_valid_div = I_audio_valid && (S_audio_div_cnt == AUDIO_DIV-1) ? 1'b1 : 1'b0;

    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst)
            S_acr_cts_cnt <= 'd0;
        else
            if(S_audio_valid_div)
                S_acr_cts_cnt <= 'd0;
            else
                S_acr_cts_cnt <= S_acr_cts_cnt + 'd1;
    end


    always @(posedge I_clk) begin
        if(S_audio_valid_div)
            begin
                O_acr_valid <= 1'b1;
                O_acr_cts   <= S_acr_cts_cnt + 'd1;
                O_acr_n     <= ACR_N;
            end
        else    
            begin
                O_acr_valid <= 1'b0;
                O_acr_cts   <= 'd0;
                O_acr_n     <= 'd0;
            end
    end

    
endmodule
