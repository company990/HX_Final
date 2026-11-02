// RASTER_BOUNDED requires the ordered, bounded destination scanner.
// The exact rational DDA cannot exceed the source endpoint in that mode.
`timescale 1ns/1ps
// Align-corners rational DDA, one shared address ALU and one row multiplier.
// Pixel preparation is a ready/advance transaction; its latency is internal.
module bilinear_src_coord_gen #(parameter DST_BITS=16,ADDR_BITS=21,RASTER_BOUNDED=0,BORROW_PHASE=0,FRACTION_BITS=16)(
 input clk,rst,cfg_start,input [ADDR_BITS-1:0] base_addr,
 input [15:0] src_width,src_height,dst_width,dst_height,input geometry_valid,
 output reg cfg_busy,cfg_done,cfg_valid,input frame_start,advance,inside_image,
 input [15:0] image_x,image_y,output reg sample_valid,
 output wire [15:0] src_x0,src_x1,src_y0,src_y1,frac_x,frac_y,
 output reg [ADDR_BITS-1:0] addr_p00,addr_p10,addr_p01,addr_p11);
 reg [ADDR_BITS-1:0] base;
 reg [15:0] width,xlast,ylast;
 reg [DST_BITS-1:0] dwlast,dhlast;
 localparam PHASE_BITS=16+FRACTION_BITS;
 reg [PHASE_BITS-1:0] xstep,ystep,xphase,yphase;
 // Only the explicitly configured consumer may discard these low bits.
 // Default16 preserves all public fractions. Q8 publishes zero low8 bits.
 wire [15:0] phase_fraction_x={xphase[FRACTION_BITS-1:0],{(16-FRACTION_BITS){1'b0}}};
 wire [15:0] phase_fraction_y={yphase[FRACTION_BITS-1:0],{(16-FRACTION_BITS){1'b0}}};
 // BORROW_PHASE is legal only with an ordered bounded scanner that holds
 // xphase/yphase until advance. Outputs are sampled only with sample_valid.
 reg [15:0] src_x0_q,src_x1_q,src_y0_q,src_y1_q,frac_x_q,frac_y_q;
 assign src_x0=BORROW_PHASE ? xphase[PHASE_BITS-1:FRACTION_BITS]:src_x0_q;
 assign src_x1=BORROW_PHASE ? (xphase[PHASE_BITS-1:FRACTION_BITS]==xlast ? xlast:xphase[PHASE_BITS-1:FRACTION_BITS]+16'd1):src_x1_q;
 assign src_y0=BORROW_PHASE ? yphase[PHASE_BITS-1:FRACTION_BITS]:src_y0_q;
 assign src_y1=BORROW_PHASE ? (yphase[PHASE_BITS-1:FRACTION_BITS]==ylast ? ylast:yphase[PHASE_BITS-1:FRACTION_BITS]+16'd1):src_y1_q;
 assign frac_x=BORROW_PHASE ? phase_fraction_x:frac_x_q;
 assign frac_y=BORROW_PHASE ? phase_fraction_y:frac_y_q;

 reg [DST_BITS-1:0] xr,yr,xrem,yrem,xthreshold,ythreshold;
 reg [2:0] cs;
 reg ds;
 wire axis_x=cs==2;
 wire [15:0] dimension=axis_x ? dwlast:dhlast;
 wire [31:0] dn=dimension==0 ? 32'd0:({16'd0,(axis_x ? xlast:ylast)} << FRACTION_BITS);
 wire [15:0] dd=dimension==0 ? 16'd1:dimension;
 wire db,done;wire [31:0] dq;wire [15:0] dr;
 u32_div_u16_seq #(.DEN_BITS(DST_BITS)) divider(clk,rst || cfg_start,ds,dn,dd,db,done,dq,dr);
 wire row_end=image_x==dwlast;
 wire [PHASE_BITS-1:0] phase_base=row_end ? yphase:xphase;
 wire [PHASE_BITS-1:0] phase_step=row_end ? ystep:xstep;
 wire [DST_BITS-1:0] remainder_base=row_end ? yrem:xrem;
 wire [DST_BITS-1:0] remainder_step=row_end ? yr:xr;
 wire [DST_BITS-1:0] threshold=row_end ? ythreshold:xthreshold;
 wire carry=remainder_step!=0 && remainder_base>=threshold;
 wire [PHASE_BITS-1:0] phase_next=phase_base+phase_step+carry;
 wire [DST_BITS-1:0] rem_next=remainder_base+(carry ? ~threshold:remainder_step)+carry;
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   base<=0;width<=0;xlast<=0;ylast<=0;dwlast<=0;dhlast<=0;
   xstep<=0;ystep<=0;xr<=0;yr<=0;xthreshold<=0;ythreshold<=0;
   cfg_busy<=0;cfg_done<=0;cfg_valid<=0;cs<=0;ds<=0;
  end else begin
   cfg_done<=0;ds<=0;
   if(cfg_start)begin
    base<=base_addr;width<=src_width;xlast<=src_width-1'b1;ylast<=src_height-1'b1;
    dwlast<=dst_width-1'b1;dhlast<=dst_height-1'b1;cfg_valid<=0;
    if(!geometry_valid || src_width==0 || src_height==0 || dst_width==0 || dst_height==0)begin cfg_busy<=0;cfg_done<=1;cs<=0;end
    else begin cfg_busy<=1;cs<=1;end
   end else case(cs)
    1:begin ds<=1;cs<=2;end
    2:if(done)begin xstep<=dq;xr<=dr;xthreshold<=dwlast-dr;cs<=3;end
    3:begin ds<=1;cs<=4;end
    4:if(done)begin ystep<=dq;yr<=dr;ythreshold<=dhlast-dr;cfg_valid<=1;cfg_done<=1;cfg_busy<=0;cs<=0;end
    default:;
   endcase
  end
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin xphase<=0;yphase<=0;xrem<=0;yrem<=0;end
  else if(cfg_start || frame_start)begin xphase<=0;yphase<=0;xrem<=0;yrem<=0;end
  else if(advance && inside_image && cfg_valid)begin
   if(row_end)begin
    xphase<=0;xrem<=0;
    if(image_y<dhlast)begin yphase<=phase_next;yrem<=rem_next;end
   end else begin xphase<=phase_next;xrem<=rem_next;end
  end
 end
 localparam WAIT=0,CLAMP=1,MULTIPLY=2,ROW0=3,ROW1=4,ADDRESS=5;
 reg [2:0] state;reg need;reg [1:0] address_index;
 reg [15:0] mem_y;reg y_has_next;
 (* dsp_style="yes" *) wire [31:0] row_full_product=width*mem_y;
 // Addresses are consumed only with sample_valid. During preparation,
 // p11 holds the row product and p00/p01 hold row bases. Visit x1 before
 // x0 so each base is overwritten only after its last use.
 wire [ADDR_BITS-1:0] product=addr_p11,row0=addr_p00,row1=addr_p01;
 reg [ADDR_BITS-1:0] operand_a,operand_b;reg subtract;
 always @*begin
  operand_a=address_index[1] ? row1:row0;
  operand_b=address_index[0] ? src_x1:src_x0;subtract=0;
  if(state==ROW0)begin operand_a=base;operand_b=product;end
  else if(state==ROW1)begin operand_a=row0;operand_b=y_has_next ? width:16'd0;subtract=1;end
 end
 wire [ADDR_BITS-1:0] address_sum=operand_a+(subtract ? ~operand_b:operand_b)+subtract;
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   state<=WAIT;need<=1;sample_valid<=0;address_index<=0;mem_y<=0;y_has_next<=0;
   src_x0_q<=0;src_x1_q<=0;src_y0_q<=0;src_y1_q<=0;frac_x_q<=0;frac_y_q<=0;
   addr_p00<=0;addr_p10<=0;addr_p01<=0;addr_p11<=0;
  end else if(cfg_start || frame_start || advance)begin state<=WAIT;need<=1;sample_valid<=0;end
  else case(state)
   WAIT:if(need && inside_image && cfg_valid && !cfg_busy)begin state<=CLAMP;need<=0;end
   CLAMP:begin
    src_x0_q<=(!RASTER_BOUNDED && xphase[PHASE_BITS-1:FRACTION_BITS]>xlast) ? xlast:xphase[PHASE_BITS-1:FRACTION_BITS];
    src_x1_q<=(RASTER_BOUNDED ? xphase[PHASE_BITS-1:FRACTION_BITS]==xlast : xphase[PHASE_BITS-1:FRACTION_BITS]>=xlast) ? xlast:xphase[PHASE_BITS-1:FRACTION_BITS]+16'd1;
    src_y0_q<=(!RASTER_BOUNDED && yphase[PHASE_BITS-1:FRACTION_BITS]>ylast) ? ylast:yphase[PHASE_BITS-1:FRACTION_BITS];
    src_y1_q<=(RASTER_BOUNDED ? yphase[PHASE_BITS-1:FRACTION_BITS]==ylast : yphase[PHASE_BITS-1:FRACTION_BITS]>=ylast) ? ylast:yphase[PHASE_BITS-1:FRACTION_BITS]+16'd1;
    frac_x_q<=(!RASTER_BOUNDED && xphase[PHASE_BITS-1:FRACTION_BITS]>xlast) ? 16'd0:phase_fraction_x;
    frac_y_q<=(!RASTER_BOUNDED && yphase[PHASE_BITS-1:FRACTION_BITS]>ylast) ? 16'd0:phase_fraction_y;
    mem_y<=(!RASTER_BOUNDED && yphase[PHASE_BITS-1:FRACTION_BITS]>ylast) ? 16'd0:ylast-yphase[PHASE_BITS-1:FRACTION_BITS];
    y_has_next<=yphase[PHASE_BITS-1:FRACTION_BITS]<ylast;state<=MULTIPLY;
   end
   MULTIPLY:begin addr_p11<=row_full_product[ADDR_BITS-1:0];state<=ROW0;end
   ROW0:begin addr_p00<=address_sum;state<=ROW1;end
   ROW1:begin addr_p01<=address_sum;address_index<=1;state<=ADDRESS;end
   ADDRESS:begin
    case(address_index)0:addr_p00<=address_sum;1:addr_p10<=address_sum;2:addr_p01<=address_sum;3:addr_p11<=address_sum;endcase
    if(address_index==2)begin sample_valid<=1;state<=WAIT;end else address_index<=address_index-1'b1;
   end
   default:state<=WAIT;
  endcase
 end
endmodule

module u32_div_u16_seq #(parameter DEN_BITS=16) (
    input             clk,
    input             rst,

    input             start,
    input      [31:0] numerator,
    input      [15:0] denominator,

    output reg        busy,
    output reg        done,

    output reg [31:0] quotient,
    output wire [15:0] remainder
);

    // The shift register contains remaining dividend bits followed by quotient bits.
    reg [DEN_BITS-1:0] divisor_reg;
    reg [DEN_BITS-1:0] remainder_reg;
    assign remainder={{(16-DEN_BITS){1'b0}},remainder_reg};

    reg [5:0] count;
    wire [DEN_BITS+1:0] remainder_difference={1'b0,remainder_reg,quotient[31]}-{2'b00,divisor_reg};

    reg [DEN_BITS:0] rem_next;
    reg [31:0] quot_next;

    always @(posedge clk or posedge rst) begin

        if (rst) begin

            busy <= 1'b0;
            done <= 1'b0;

            quotient  <= 32'd0;
            remainder_reg <= 16'd0;

            divisor_reg    <= 16'd1;

            count <= 6'd0;

        end
        else begin

            done <= 1'b0;

            if (start && !busy) begin

                if (denominator == 16'd0) begin

                    busy      <= 1'b0;
                    done      <= 1'b1;
                    quotient  <= 32'd0;
                    remainder_reg <= 16'd0;

                end
                else begin

                    busy <= 1'b1;

                    divisor_reg    <= denominator;

                    // The result ports double as the shift/remainder state.
                    // Their values are consumed only with done=1.
                    quotient  <= numerator;
                    remainder_reg <= 16'd0;

                    count <= 6'd0;
                end

            end
            else if (busy) begin

                rem_next =
                    {remainder_reg, quotient[31]};

                if (!remainder_difference[DEN_BITS+1]) begin

                    rem_next =
                        remainder_difference[DEN_BITS:0];

                    quot_next =
                        {quotient[30:0], 1'b1};

                end
                else begin

                    quot_next =
                        {quotient[30:0], 1'b0};

                end

                quotient  <= quot_next;
                remainder_reg <= rem_next[DEN_BITS-1:0];

                if (count == 6'd31) begin

                    busy <= 1'b0;
                    done <= 1'b1;

                end
                else begin

                    count <= count + 1'b1;
                end
            end
        end
    end

endmodule