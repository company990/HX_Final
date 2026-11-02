// EG4 adaptation of Anlogic APUG013 v1.7. MAC runs at 125 MHz; app_clk is
// independent (50 MHz in this project). Client FIFOs bridge both directions.
// SPEED: 2=1000 Mbps, 1=100 Mbps, 0=10 Mbps. Must match the PHY link.
// The board clock/PHY delay constraints belong in the integrating top level.
module eg4_udp_endpoint #(
    parameter integer AUTO_SPEED = 0,
    parameter [1:0] SPEED = 2'b10,
    parameter [31:0] LOCAL_IP = 32'hc0a8f001,
    parameter [47:0] LOCAL_MAC = 48'h020000000001,
    parameter [15:0] LOCAL_PORT = 16'd1
)(
    input wire clk125, app_clk, tx_clk_shift, rst,
    input wire rgmii_rxc, rgmii_rx_ctl,
    input wire [3:0] rgmii_rxd,
    output wire rgmii_txc, rgmii_tx_ctl,
    output wire [3:0] rgmii_txd,
    // Application ports, all synchronous to app_clk. RX has no backpressure.
    output wire [7:0] rx_data,
    output wire rx_valid,
    output wire [15:0] rx_length, rx_port,
    input wire tx_request, tx_valid,
    input wire [7:0] tx_data,
    input wire [15:0] tx_length, tx_dst_port,
    input wire [31:0] tx_dst_ip,
    output wire tx_ready, tx_ack,
    // PHY status outputs are in the receive clock domain, not clk125.
    output wire link_up, full_duplex,
    output wire [1:0] link_speed,
    output wire rx_overflow, tx_overflow, ip_error, arp_error
);
    wire mac_rx_clk, mac_rx_ce, mac_rx_valid, mac_rx_good, mac_rx_bad;
    wire [7:0] mac_rx_data, mac_tx_data;
    wire mac_tx_clk, mac_tx_ce, mac_tx_valid, mac_tx_ack;
    wire collision, retransmit;
    wire [7:0] fifo_rx_data, fifo_tx_data;
    wire fifo_rx_sof_n, fifo_rx_eof_n, fifo_rx_src_n, fifo_rx_dst_n;
    wire fifo_tx_sof_n, fifo_tx_eof_n, fifo_tx_src_n, fifo_tx_dst_n;
    wire [47:0] reversed_mac = {LOCAL_MAC[7:0], LOCAL_MAC[15:8],
        LOCAL_MAC[23:16], LOCAL_MAC[31:24], LOCAL_MAC[39:32], LOCAL_MAC[47:40]};
    // Match the board EG4 example: normal frames, no jumbo-frame mode.
    // RGMII idle status is independent of the MAC's configured speed. Sample
    // it in clk125 and require a stable valid link before updating the MAC.
    reg [2:0] status_s1=0, status_s2=0, status_last=0;
    reg [5:0] stable_count=0;
    reg [1:0] selected_speed=SPEED;
    always @(posedge clk125 or posedge rst) begin
        if(rst) begin
            status_s1<=0; status_s2<=0; status_last<=0;
            stable_count<=0; selected_speed<=SPEED;
        end else begin
            status_s1<={link_up,link_speed}; status_s2<=status_s1;
            status_last<=status_s2;
            if(status_s2!=status_last) stable_count<=0;
            else if(!(&stable_count)) stable_count<=stable_count+1'b1;
            if((&stable_count) && status_s2[2] && status_s2[1:0]!=2'b11)
                selected_speed<=status_s2[1:0];
        end
    end
    wire [1:0] configured_speed = AUTO_SPEED ? selected_speed : SPEED;
    wire [19:0] cfg = {1'b0,2'b00,configured_speed,8'b00000010,7'b0000010};
    temac_block #(.DEVICE("EG4"), .BANK_TYPE("HD")) u_mac (
        .reset(rst), .gtx_clk(clk125), .gtx_clk_90(tx_clk_shift),
        .rx_clk(mac_rx_clk), .rx_clk_en(mac_rx_ce), .rx_data(mac_rx_data),
        .rx_data_valid(mac_rx_valid), .rx_correct_frame(mac_rx_good),
        .rx_error_frame(mac_rx_bad), .rx_status_vector(), .rx_status_vld(),
        .tx_clk(mac_tx_clk), .tx_clk_en(mac_tx_ce), .tx_data(mac_tx_data),
        .tx_data_en(mac_tx_valid), .tx_rdy(mac_tx_ack), .tx_stop(1'b0),
        .tx_collision(collision), .tx_retransmit(retransmit), .tx_ifg_val(8'd0),
        .tx_status_vector(), .tx_status_vld(), .tri_speed(),
        .pause_req(1'b0), .pause_val(16'd0), .pause_source_addr(48'd0),
        .unicast_address(reversed_mac), .mac_cfg_vector(cfg),
        .s_axi_aclk(clk125), .s_axi_awaddr(8'd0), .s_axi_awvalid(1'b0),
        .s_axi_awready(), .s_axi_wdata(32'd0), .s_axi_wvalid(1'b0),
        .s_axi_wready(), .s_axi_bresp(), .s_axi_bvalid(), .s_axi_bready(1'b0),
        .s_axi_araddr(8'd0), .s_axi_arvalid(1'b0), .s_axi_arready(),
        .s_axi_rdata(), .s_axi_rresp(), .s_axi_rvalid(), .s_axi_rready(1'b0),
        .rgmii_txd(rgmii_txd), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txc(rgmii_txc),
        .rgmii_rxd(rgmii_rxd), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxc(rgmii_rxc),
        .inband_link_status(link_up), .inband_clock_speed(link_speed),
        .inband_duplex_status(full_duplex), .mdio_i(1'b1),
        .mdio_o(), .mdio_oen(), .mdio_clk()
    );
    rx_client_fifo #(.DEVICE("EG4"), .ADDR_WIDTH(11)) u_rx_fifo (
        .wr_clk(mac_rx_clk), .wr_sreset(rst), .wr_enable(mac_rx_ce),
        .rx_data(mac_rx_data), .rx_data_valid(mac_rx_valid),
        .rx_good_frame(mac_rx_good), .rx_bad_frame(mac_rx_bad), .overflow(rx_overflow),
        .rd_clk(app_clk), .rd_sreset(rst), .rd_data_out(fifo_rx_data),
        .rd_sof_n(fifo_rx_sof_n), .rd_eof_n(fifo_rx_eof_n),
        .rd_src_rdy_n(fifo_rx_src_n), .rd_dst_rdy_n(fifo_rx_dst_n), .rx_fifo_status()
    );
    tx_client_fifo #(.DEVICE("EG4"), .ADDR_WIDTH(11)) u_tx_fifo (
        .rd_clk(mac_tx_clk), .rd_sreset(rst), .rd_enable(mac_tx_ce),
        .tx_data(mac_tx_data), .tx_data_valid(mac_tx_valid), .tx_ack(mac_tx_ack),
        .tx_collision(collision), .tx_retransmit(retransmit), .overflow(tx_overflow),
        .wr_clk(app_clk), .wr_sreset(rst), .wr_data(fifo_tx_data),
        .wr_sof_n(fifo_tx_sof_n), .wr_eof_n(fifo_tx_eof_n),
        .wr_src_rdy_n(fifo_tx_src_n), .wr_dst_rdy_n(fifo_tx_dst_n), .wr_fifo_status()
    );
    // Validate the entire frame before the encrypted stack sees its SOF.
    wire [7:0] checked_rx_data;
    wire checked_rx_sof_n,checked_rx_eof_n,checked_rx_src_n,checked_rx_dst_n;
    wire guard_reject,stack_ip_error;
    eg4_ipv4_rx_guard #(.LOCAL_PORT(LOCAL_PORT)) u_rx_guard(
        .clk(app_clk),.rst(rst),.s_data(fifo_rx_data),
        .s_sof_n(fifo_rx_sof_n),.s_eof_n(fifo_rx_eof_n),
        .s_src_n(fifo_rx_src_n),.s_dst_n(fifo_rx_dst_n),
        .m_data(checked_rx_data),.m_sof_n(checked_rx_sof_n),.m_eof_n(checked_rx_eof_n),
        .m_src_n(checked_rx_src_n),.m_dst_n(checked_rx_dst_n),.reject(guard_reject));
    assign ip_error=stack_ip_error | guard_reject;
    udp_ip_protocol_stack #(.DEVICE("EG4"), .LOCAL_UDP_PORT_NUM(LOCAL_PORT),
        .LOCAL_IP_ADDRESS(LOCAL_IP), .LOCAL_MAC_ADDRESS(LOCAL_MAC)) u_udp (
        .udp_rx_clk(app_clk), .udp_tx_clk(app_clk), .reset(rst),
        .input_local_udp_port_num(16'd0), .input_local_udp_port_num_valid(1'b0),
        .input_local_ip_address(32'd0), .input_local_ip_address_valid(1'b0),
        .input_local_mac_address(48'd0), .input_local_mac_address_valid(1'b0),
        .udp2app_tx_ready(tx_ready), .udp2app_tx_ack(tx_ack),
        .app_tx_request(tx_request), .app_tx_data_valid(tx_valid), .app_tx_data(tx_data),
        .app_tx_data_length(tx_length), .app_tx_dst_port(tx_dst_port),
        .ip_tx_dst_address(tx_dst_ip), .app_rx_data_valid(rx_valid),
        .app_rx_data(rx_data), .app_rx_data_length(rx_length), .app_rx_port_num(rx_port),
        // The reference deliberately inverts ONLY RX source-ready here.
        .temac_rx_ready(checked_rx_dst_n), .temac_rx_valid(!checked_rx_src_n),
        .temac_rx_data(checked_rx_data), .temac_rx_sof(checked_rx_sof_n), .temac_rx_eof(checked_rx_eof_n),
        .temac_tx_ready(fifo_tx_dst_n), .temac_tx_valid(fifo_tx_src_n),
        .temac_tx_data(fifo_tx_data), .temac_tx_sof(fifo_tx_sof_n), .temac_tx_eof(fifo_tx_eof_n),
        .ip_rx_error(stack_ip_error), .arp_request_no_reply_error(arp_error)
    );
    // eg4_apm_sdpram.v binds the stack's public ICMP memory interface to two
    // EG4 BRAM9K blocks, retaining the FIFO depth and one-cycle read latency.
endmodule

// Store-and-forward guard, in app_clk only. Active-low LocalLink handshake.
// Max standard untagged Ethernet frame fits 2048 bytes. ARP/non-IPv4 frames
// retain their original bytes and are classified by the existing stack.
// IPv4 fragments are unsupported: reject them rather than bypass UDP checks.
module eg4_ipv4_rx_guard #(parameter [15:0] LOCAL_PORT=16'd1)(
 input wire clk,rst,input wire[7:0] s_data,
 input wire s_sof_n,s_eof_n,s_src_n,output wire s_dst_n,
 output wire[7:0] m_data,output wire m_sof_n,m_eof_n,m_src_n,
 input wire m_dst_n,output reg reject);
 localparam CAPTURE=0,CHECK=1,LOAD=2,SEND=3;
 reg[1:0] state;
 reg wr_bank,rd_bank;
 reg[1:0] bank_full;
 reg[11:0] bank_length[0:1];
 reg[1:0] tx_state;
 wire commit_frame,release_frame;
 wire[7:0] read_data;
 reg[11:0] count,length;
 reg[10:0] rd;
 reg bad;
 reg [15:0] ip_length,udp_length;
 reg ether_hi_ok,ether_ipv4,fragment_bad,udp_hi_ok,udp_port_ok,version_ok,udp_protocol;
 reg [7:0] word_hi;
 reg[6:0] header_bytes;
 reg[20:0] checksum_sum;
 wire[7:0] udp_start=8'd14+{1'b0,header_bytes};
 wire[16:0] folded={1'b0,checksum_sum[15:0]}+{12'd0,checksum_sum[20:16]};
 wire[15:0] checksum=folded[15:0]+folded[16];
 wire transfer=state==CAPTURE && !bank_full[wr_bank] && !s_src_n;
 wire read_enable=tx_state==LOAD || (tx_state==SEND && !m_dst_n);
 wire[10:0] read_address=tx_state==LOAD ? rd : rd+11'd1;
 wire ipv4=ether_ipv4;
 wire [16:0] ip_payload={1'b0,ip_length}-{10'd0,header_bytes};
 wire ip_bad=!version_ok || header_bytes<20 ||
             length<14+header_bytes || ip_payload[16] ||
             {1'b0,ip_length}+17'd14>{5'd0,length} ||
             checksum!=16'hffff || fragment_bad;
 // One payload subtraction proves both header fit and UDP size.
 wire udp_bad=udp_protocol && (ip_payload[16] || ip_payload[15:0]<8 ||
              udp_length!=ip_payload[15:0] || !udp_port_ok);
 assign commit_frame=state==CHECK && !(bad || length<14 || (ipv4 && (ip_bad || udp_bad)));
 assign release_frame=tx_state==SEND && !m_dst_n && !m_eof_n;
 assign s_dst_n=state!=CAPTURE || bank_full[wr_bank];
 assign m_src_n=tx_state!=SEND;
 assign m_data=read_data;
 assign m_sof_n=rd!=0;
 assign m_eof_n={1'b0,rd}!=bank_length[rd_bank]-1'b1;
 // Restore two independently owned 2048-byte banks. A native 32K block
 // holds both banks, releasing two 9K blocks instead of growing their count.
 // The reader starts only after full IPv4/UDP validation; bad frames never
 // become visible to the encrypted stack. Output is held during backpressure.
 wire ram_write=transfer && count<2048;
 EG_LOGIC_BRAM #(.DATA_WIDTH_A(8),.DATA_WIDTH_B(8),
  .ADDR_WIDTH_A(12),.ADDR_WIDTH_B(12),.DATA_DEPTH_A(4096),.DATA_DEPTH_B(4096),
  .MODE("PDPW"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE("SYNC"),
  .IMPLEMENT("32K"),.INIT_FILE("NONE"),.FILL_ALL("NONE")) frame_storage(
  .dia(s_data),.addra({wr_bank,count[10:0]}),.cea(ram_write),.ocea(1'b0),
  .clka(clk),.wea(ram_write),.rsta(1'b0),.bea(1'b0),.doa(),
  .dib(8'd0),.addrb({rd_bank,read_address}),.ceb(read_enable),.oceb(1'b0),
  .clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(read_data));
 always @(posedge clk)if(commit_frame)bank_length[wr_bank]<=length;
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   state<=CAPTURE;count<=0;length<=0;rd<=0;bad<=0;reject<=0;
   wr_bank<=0;rd_bank<=0;bank_full<=0;tx_state<=CAPTURE;
   ether_hi_ok<=0;ether_ipv4<=0;ip_length<=0;fragment_bad<=0;udp_hi_ok<=0;udp_port_ok<=0;udp_length<=0;
   version_ok<=0;udp_protocol<=0;word_hi<=0;header_bytes<=0;checksum_sum<=0;
  end else begin
   reject<=0;
   bank_full <= (bank_full | (commit_frame ? (2'b01 << wr_bank) : 2'b00))
                & ~(release_frame ? (2'b01 << rd_bank) : 2'b00);
   case(tx_state)
    CAPTURE:if(bank_full[rd_bank])begin rd<=0;tx_state<=LOAD;end
    LOAD:tx_state<=SEND;
    SEND:if(!m_dst_n)begin
     if(!m_eof_n)begin rd_bank<=~rd_bank;tx_state<=CAPTURE;end
     else rd<=rd+1'b1;
    end
    default:tx_state<=CAPTURE;
   endcase
   case(state)
    CAPTURE:if(transfer)begin
     if(count==0)begin
      bad<=s_sof_n;ether_hi_ok<=0;ether_ipv4<=0;ip_length<=0;fragment_bad<=0;
      udp_hi_ok<=0;udp_port_ok<=0;udp_length<=0;version_ok<=0;udp_protocol<=0;
      header_bytes<=0;checksum_sum<=0;
     end else if(!s_sof_n)bad<=1;
     if(count>=2048)bad<=1;
     if(count!=4095)count<=count+1'b1;
     case(count)
      12:ether_hi_ok<=s_data==8'h08;
      13:ether_ipv4<=ether_hi_ok && s_data==8'h00;
      14:begin version_ok<=s_data[7:4]==4;header_bytes<={1'b0,s_data[3:0],2'b00};end
      16:ip_length[15:8]<=s_data;
      17:ip_length[7:0]<=s_data;
      20:fragment_bad<=|(s_data & 8'hbf);
      21:fragment_bad<=fragment_bad || s_data!=0;
      23:udp_protocol<=s_data==17;
      default:;
     endcase
     if(count>=14 && count<udp_start)begin
      if(!count[0])word_hi<=s_data;
      else checksum_sum<=checksum_sum+{word_hi,s_data};
     end
     // First header byte precedes the newly decoded IHL register.
     if(count==14)word_hi<=s_data;
     if(count==udp_start+2)udp_hi_ok<=s_data==LOCAL_PORT[15:8];
     if(count==udp_start+3)udp_port_ok<=udp_hi_ok && s_data==LOCAL_PORT[7:0];
     if(count==udp_start+4)udp_length[15:8]<=s_data;
     if(count==udp_start+5)udp_length[7:0]<=s_data;
     if(!s_eof_n)begin length<=count+1'b1;state<=CHECK;end
    end
    CHECK:begin
     if(bad || length<14 || (ipv4 && (ip_bad || udp_bad)))begin
      reject<=1;count<=0;state<=CAPTURE;
     end else begin wr_bank<=~wr_bank;count<=0;state<=CAPTURE;end
    end
    default:state<=CAPTURE;
   endcase
  end
 end
endmodule
