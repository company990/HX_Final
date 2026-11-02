// FM/SCAN/DSP snapshots, one 9K true dual-port memory. Write both halves
// together; read in the inter-frame gap. Public completed/packet is atomic.
module status_word_assembler(input clk,rst,input incoming,input [1:0] kind,epoch,word_index,
 input [31:0] data,output reg completed,output reg [1:0] completed_kind,output [127:0] packet);
 reg [1:0] expected[0:2],generation[0:2];
 reg [31:0] first,second,third,last;reg [15:0] low_half;
 reg [5:0] read_address;reg [2:0] state;
 wire [15:0] read_data;
 wire ordered=kind<3 && expected[kind]!=0 && expected[kind]==word_index && generation[kind]==epoch;
 wire write_enable=incoming && kind<3 && (word_index==0 || word_index<3 && ordered);
 // EG4 true dual-port 9K supports nine bits per independent write port.
 // Serialize a 32-bit incoming word into two 16-bit writes, two bytes per
 // clock. Metadata arrives once per physical frame, hundreds of clocks apart.
 reg write_upper;reg [15:0] upper_data;reg [5:0] upper_address;
 always @(posedge clk or posedge rst)begin
  if(rst)begin write_upper<=0;upper_data<=0;upper_address<=0;end
  else begin
   write_upper<=write_enable;
   if(write_enable)begin upper_data<=data[31:16];upper_address<={kind,word_index,2'b10};end
  end
 end
 wire memory_write=write_enable || write_upper;
 wire [5:0] address_a=write_enable ? {kind,word_index,2'b00} : write_upper ? upper_address : read_address;
 wire [5:0] address_b={address_a[5:1],1'b1};
 wire [15:0] memory_data=write_enable ? data[15:0] : upper_data;
 wire [8:0] qa,qb;
 assign read_data={qb[7:0],qa[7:0]};
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(9),.DATA_WIDTH_B(9),.ADDR_WIDTH_A(6),.ADDR_WIDTH_B(6),
  .DATA_DEPTH_A(64),.DATA_DEPTH_B(64),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),.IMPLEMENT("9K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) storage(
  .dia({1'b0,memory_data[7:0]}),.addra(address_a),.cea(1'b1),.ocea(1'b0),.clka(clk),.wea(memory_write),.rsta(1'b0),.bea(1'b0),.doa(qa),
  .dib({1'b0,memory_data[15:8]}),.addrb(address_b),.ceb(1'b1),.oceb(1'b0),.clkb(clk),.web(memory_write),.rstb(1'b0),.beb(1'b0),.dob(qb));
 assign packet={last,third,second,first};
 integer k;
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   for(k=0;k<3;k=k+1)begin expected[k]<=0;generation[k]<=0;end
   first<=0;second<=0;third<=0;last<=0;low_half<=0;read_address<=0;state<=0;completed<=0;completed_kind<=0;
  end else begin
   completed<=0;
   case(state)
    1:begin read_address<={completed_kind,4'd2};state<=2;end
    2:begin low_half<=read_data;read_address<={completed_kind,4'd4};state<=3;end
    3:begin first<={read_data,low_half};read_address<={completed_kind,4'd6};state<=4;end
    4:begin low_half<=read_data;read_address<={completed_kind,4'd8};state<=5;end
    5:begin second<={read_data,low_half};read_address<={completed_kind,4'd10};state<=6;end
    6:begin low_half<=read_data;state<=7;end
    7:begin third<={read_data,low_half};completed<=1;state<=0;end
   endcase
   if(incoming && kind<3)begin
    if(word_index==0)begin generation[kind]<=epoch;expected[kind]<=1;end
    else if(ordered)begin
     if(word_index==3)begin
      expected[kind]<=0;completed_kind<=kind;last<=data;read_address<={kind,4'd0};state<=1;
     end else expected[kind]<=expected[kind]+1'b1;
    end else expected[kind]<=0;
   end
  end
 end
 // synthesis translate_off
 always @(posedge clk)if(!rst && incoming && (state!=0 || write_upper))$fatal(1,"Status frame gap shorter than seven clocks");
 // synthesis translate_on
endmodule
