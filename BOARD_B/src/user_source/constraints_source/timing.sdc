# Active BOARD_B clocks. Existing PLL ratios and phases are unchanged.
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
create_clock -name eth_rx_launch -period 8 -waveform {0 4}
create_clock -name eth_rxc -period 8 -waveform {2 6} [get_ports {eth_rxc}]
derive_clocks
rename_clock -name {video_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {mic_adc_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[2]}]

# Board link: asynchronous pins stop only at the first explicit two-FF stage.
set_false_path -from [get_ports {link_cs_n}]
set_false_path -from [get_ports {link_sck}]
set_false_path -from [get_ports {link_mosi}]

rename_clock -name {eth_clk125} -source [get_ports {clk}] -master_clock {clk} [get_pins {u_fpga_wav/u_net_pll/pll_inst.clkc[0]}]
rename_clock -name {eth_tx_shift_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {u_fpga_wav/u_net_pll/pll_inst.clkc[4]}]
create_generated_clock -name eth_tx_capture -master_clock eth_tx_shift_clk -source [get_pins {u_fpga_wav/u_net_pll/pll_inst.clkc[4]}] -edges {1 2 3} [get_ports {eth_txc}]

set_input_delay -clock eth_rx_launch -max 1.0 [get_ports {eth_rxd[*] eth_rx_ctl}]
set_input_delay -clock eth_rx_launch -min -1.0 [get_ports {eth_rxd[*] eth_rx_ctl}]
set_input_delay -clock eth_rx_launch -clock_fall -add_delay -max 1.0 [get_ports {eth_rxd[*] eth_rx_ctl}]
set_input_delay -clock eth_rx_launch -clock_fall -add_delay -min -1.0 [get_ports {eth_rxd[*] eth_rx_ctl}]
set_output_delay -clock eth_tx_capture -max 1.2 [get_ports {eth_txd[*] eth_tx_ctl}]
set_output_delay -clock eth_tx_capture -min -1.2 [get_ports {eth_txd[*] eth_tx_ctl}]
set_output_delay -clock eth_tx_capture -clock_fall -add_delay -max 1.2 [get_ports {eth_txd[*] eth_tx_ctl}]
set_output_delay -clock eth_tx_capture -clock_fall -add_delay -min -1.2 [get_ports {eth_txd[*] eth_tx_ctl}]

set_clock_groups -asynchronous \
    -group [get_clocks {eth_rxc eth_rx_launch}] \
    -group [get_clocks {clk eth_clk125 eth_tx_shift_clk eth_tx_capture video_clk mic_adc_clk}]
set_clock_groups -asynchronous \
    -group [get_clocks {eth_clk125 eth_tx_shift_clk eth_tx_capture}] \
    -group [get_clocks {clk video_clk mic_adc_clk}]


set_false_path -from [get_ports {eth_mdio sw3}]
