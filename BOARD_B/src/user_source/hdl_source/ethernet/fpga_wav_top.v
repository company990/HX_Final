// Ethernet WAV front end for the EG4 project.
//
// pll_gen_2.v from the reference package targets PH1 primitives and its C3
// output is 25 MHz. The pin-compatible EG4 pll_gen.v is used for the 125 MHz
// RGMII clocks. app_clk is supplied by the existing 50 MHz system PLL so the
// UDP receiver and the downstream WAV consumer stay in one clock domain.
module fpga_wav_top #(
    parameter integer APP_CLK_HZ = 50000000,
    parameter integer AUTO_SPEED = 1,
    parameter [1:0] SPEED = 2'b10,
    parameter [31:0] LOCAL_IP = 32'hc0a8f001,
    parameter [47:0] LOCAL_MAC = 48'h020000000001,
    parameter [15:0] LOCAL_PORT = 16'd1,
    parameter [31:0] PC_IP = 32'hc0a8f002,
    parameter [15:0] PC_PORT = 16'd50000,
    parameter integer LOOPBACK_MODE = 0,
    parameter integer NET_VIDEO_ENABLE = 0
)(
    input  wire        ref_clk_50,
    input  wire        app_clk,
    input  wire        rst_n,
    input  wire        file_source_enable,

    input  wire        rgmii_rxc,
    input  wire        rgmii_rx_ctl,
    input  wire [3:0]  rgmii_rxd,
    output wire        rgmii_txc,
    output wire        rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd,
    output wire        phy_mdc,
    inout  wire        phy_mdio,

    output wire [7:0]  file_byte,
    output wire        file_valid,
    input  wire        file_ready,
    input  wire        file_error,
    output wire [31:0] file_size,
    output wire        file_end,
    output wire [31:0] next_offset,
    output wire        debug_rx_seen,
    output wire        debug_wack_sent,

    output wire        app_reset,
    output wire        phy_ready,
    output wire        phy_fault,
    output wire        link_up,
    output wire        full_duplex,
    output wire [1:0]  link_speed,
    output wire        rx_overflow,
    output wire        tx_overflow,
    output wire        ip_error,
    output wire        arp_error,
    output wire        phy_debug_valid,
    output wire [3:0]  phy_debug_step,phy_debug_reason,
    output wire [15:0] phy_debug_read,phy_debug_expected,
    output wire [3:0] wav_debug_state,wav_debug_checks,wav_debug_tx,
    output wire [15:0] wav_debug_length,wav_debug_count,

    input wire video_source_enable,
    output wire [7:0] video_byte,
    output wire video_valid,
    input wire video_ready,video_error,
    output wire [31:0] video_size,
    output wire video_end,video_active,
    output wire [3:0] video_slots,
    output wire video_obs_payload_byte,video_obs_commit_byte,
    output wire video_obs_drop_pkt,video_obs_commit_pkt
);
    wire clk125;
    wire tx_clk_shift;
    wire pll_locked;

    pll_gen u_net_pll (
        .refclk  (ref_clk_50),
        .reset   (!rst_n),
        .extlock (pll_locked),
        .clk0_out(clk125),
        .clk1_out(),
        .clk2_out(),
        .clk3_out(),
        .clk4_out(tx_clk_shift)
    );

    assign app_reset = !pll_locked;

    // The PHY supplies the RGMII RX delay. These input taps retain the routed
    // timing margin established by the existing Ethernet build.
    wire [3:0] rgmii_rxd_delayed;
    wire       rgmii_rx_ctl_delayed;
    genvar lane;
    generate
        for (lane=0; lane<4; lane=lane+1) begin : g_rx_delay
            EG_LOGIC_IDELAY #(.INDEL(20)) u_delay (
                .i(rgmii_rxd[lane]),
                .o(rgmii_rxd_delayed[lane])
            );
        end
    endgenerate
    EG_LOGIC_IDELAY #(.INDEL(20)) u_ctl_delay (
        .i(rgmii_rx_ctl),
        .o(rgmii_rx_ctl_delayed)
    );

    // The 90-degree FPGA TX clock provides TX skew, so PHY TX delay is off.
    // rtl8211e_init is the only MDIO master. The TEMAC management interface
    // remains disabled inside eg4_udp_endpoint, preventing two bus masters
    // from driving the same open-drain MDIO line.
    wire mdio_out;
    wire mdio_oe;
    rtl8211e_init #(
        .CLK_HZ          (APP_CLK_HZ),
        .PHY_ADDR        (5'd1),
        .ENABLE_RX_DELAY (1),
        .ENABLE_TX_DELAY (0)
    ) u_phy_init (
        .clk     (app_clk),
        .rst     (app_reset),
        .mdc     (phy_mdc),
        .mdio_out(mdio_out),
        .mdio_oe (mdio_oe),
        .mdio_in (phy_mdio),
        .ready   (phy_ready),
        .fault   (phy_fault),
        .debug_valid(phy_debug_valid),.debug_step(phy_debug_step),
        .debug_reason(phy_debug_reason),.debug_read(phy_debug_read),
        .debug_expected(phy_debug_expected)
    );
    assign phy_mdio = (mdio_oe && !mdio_out) ? 1'b0 : 1'bz;

    wire [7:0]  rx_data;
    wire        rx_valid;
    wire [15:0] rx_length;
    wire [15:0] rx_port_unused;
    wire        tx_request;
    wire        tx_valid;
    wire [7:0]  tx_data;
    wire [15:0] tx_length;
    wire        tx_ready;
    wire        tx_ack;

    eg4_udp_endpoint #(
        .AUTO_SPEED(AUTO_SPEED),
        .SPEED     (SPEED),
        .LOCAL_IP  (LOCAL_IP),
        .LOCAL_MAC (LOCAL_MAC),
        .LOCAL_PORT(LOCAL_PORT)
    ) u_endpoint (
        .clk125       (clk125),
        .app_clk      (app_clk),
        .tx_clk_shift (tx_clk_shift),
        .rst          (app_reset),
        .rgmii_rxc    (rgmii_rxc),
        .rgmii_rx_ctl (rgmii_rx_ctl_delayed),
        .rgmii_rxd    (rgmii_rxd_delayed),
        .rgmii_txc    (rgmii_txc),
        .rgmii_tx_ctl (rgmii_tx_ctl),
        .rgmii_txd    (rgmii_txd),
        .rx_data      (rx_data),
        .rx_valid     (rx_valid),
        .rx_length    (rx_length),
        .rx_port      (rx_port_unused),
        .tx_request   (tx_request),
        .tx_valid     (tx_valid),
        .tx_data      (tx_data),
        .tx_length    (tx_length),
        .tx_dst_port  (PC_PORT),
        .tx_dst_ip    (PC_IP),
        .tx_ready     (tx_ready),
        .tx_ack       (tx_ack),
        .link_up      (link_up),
        .full_duplex  (full_duplex),
        .link_speed   (link_speed),
        .rx_overflow  (rx_overflow),
        .tx_overflow  (tx_overflow),
        .ip_error     (ip_error),
        .arp_error    (arp_error)
    );

    assign wav_debug_tx={tx_request,tx_ack,tx_ready,tx_valid};
    generate
        if (LOOPBACK_MODE == 0 && NET_VIDEO_ENABLE != 0) begin : g_av_receiver
            udp_av_receiver u_av_receiver (
                .clk(app_clk),.rst(app_reset),
                .wav_enable(file_source_enable),.video_enable(video_source_enable),
                .rx_data(rx_data),.rx_valid(rx_valid),.rx_length(rx_length),
                .tx_ready(tx_ready),.tx_ack(tx_ack),.tx_request(tx_request),
                .tx_valid(tx_valid),.tx_data(tx_data),.tx_length(tx_length),
                .file_byte(file_byte),.file_valid(file_valid),.file_ready(file_ready),
                .file_error(file_error),.file_size(file_size),.file_end(file_end),
                .next_offset(next_offset),.debug_rx_seen(debug_rx_seen),
                .debug_wack_sent(debug_wack_sent),.wav_debug_state(wav_debug_state),
                .wav_debug_checks(wav_debug_checks),.wav_debug_length(wav_debug_length),
                .wav_debug_count(wav_debug_count),.video_byte(video_byte),
                .video_valid(video_valid),.video_ready(video_ready),.video_error(video_error),
                .video_size(video_size),.video_end(video_end),.video_active(video_active),
                .video_slots(video_slots),.video_obs_payload_byte(video_obs_payload_byte),
                .video_obs_commit_byte(video_obs_commit_byte),
                .video_obs_drop_pkt(video_obs_drop_pkt),.video_obs_commit_pkt(video_obs_commit_pkt)
            );
        end else if (LOOPBACK_MODE == 0) begin : g_wav_receiver
            udp_wav_receiver u_wav_receiver (
                .clk        (app_clk),
                .rst        (app_reset || !file_source_enable),
                .rx_data    (rx_data),
                .rx_valid   (rx_valid),
                .rx_length  (rx_length),
                .tx_ready   (tx_ready),
                .tx_ack     (tx_ack),
                .tx_request (tx_request),
                .tx_valid   (tx_valid),
                .tx_data    (tx_data),
                .tx_length  (tx_length),
                .file_byte  (file_byte),
                .file_valid (file_valid),
                .file_ready (file_ready),
                .file_error (file_error),
                .file_size  (file_size),
                .file_end   (file_end),
                .next_offset(next_offset),
                .debug_rx_seen(debug_rx_seen),
                .debug_wack_sent(debug_wack_sent),
                .debug_state(wav_debug_state),.debug_checks(wav_debug_checks),
                .debug_length(wav_debug_length),.debug_count(wav_debug_count)
            );
        end else begin : g_udp_loopback
            udp_loopback #(.DEVICE("EG4")) u_loopback (
                .app_rx_clk        (app_clk),
                .app_tx_clk        (app_clk),
                .reset             (app_reset),
                .app_rx_data       (rx_data),
                .app_rx_data_valid (rx_valid),
                .app_rx_data_length(rx_length),
                .udp_tx_ready      (tx_ready),
                .app_tx_ack        (tx_ack),
                .app_tx_data       (tx_data),
                .app_tx_data_request(tx_request),
                .app_tx_data_valid (tx_valid),
                .udp_data_length   (tx_length)
            );
            assign file_byte   = 8'd0;
            assign file_valid  = 1'b0;
            assign file_size   = 32'd0;
            assign file_end    = 1'b0;
            assign next_offset = 32'd0;
            assign debug_rx_seen = 1'b0;
            assign debug_wack_sent = 1'b0;
            assign wav_debug_state=0;assign wav_debug_checks=0;
            assign wav_debug_length=0;assign wav_debug_count=0;
        end
        if (LOOPBACK_MODE != 0 || NET_VIDEO_ENABLE == 0) begin : g_no_video
            assign video_byte=0;assign video_valid=0;assign video_size=0;
            assign video_end=0;assign video_active=0;assign video_slots=0;
            assign video_obs_payload_byte=0;assign video_obs_commit_byte=0;
            assign video_obs_drop_pkt=0;assign video_obs_commit_pkt=0;
        end
    endgenerate
endmodule
