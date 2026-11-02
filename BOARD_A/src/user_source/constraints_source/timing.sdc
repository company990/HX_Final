
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# RTL8211E RXDLY is configured by rtl8211e_init. Keep both DDR edge
# relationships and allow 0.2 ns additional board skew on each side.
# Disabled Ethernet: create_clock -name eth_rx_launch -period 8 -waveform {0 4}
# Disabled Ethernet: create_clock -name eth_rxc -period 8 -waveform {2 6} [get_ports {eth_rxc}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# SW3 and MDIO enter explicit synchronizers. Exclude only their asynchronous
# pad-to-first-stage paths; the remaining synchronizer stages stay timed.
set_false_path -from [get_ports {sw3}]
# Disabled Ethernet: set_false_path -from [get_ports {eth_mdio}]

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]

# ------------------------------------------------------------
# 4. Pixel and serializer clocks run together at a fixed 1:5 ratio.
# Both come from video_pll: retain normal setup/hold analysis across them.
# Do not declare these clocks exclusive or asynchronous.
# ------------------------------------------------------------

# ------------------------------------------------------------
# 5. 音频 I2S 发生器/接收器跨域
#
#    域里生成 I2S，video_clk 域里做三拍同步采样。
#    这类路径不是普通同步时序路径，直接切掉更合理。
# ------------------------------------------------------------

# ------------------------------------------------------------
# 6. 可选：如果你的 frame_read_write 内部异步 FIFO 已经完全处理好了跨域，
#    且报告里总出现 video_clk / sd_card_clk / ext_mem_clk 之间的大量伪违例，
#    可以打开下面这组异步分组。
#
#    建议先不要开，先看报告；
#    只有确认这些跨域都只经过 FIFO / 同步器时再开。
# ------------------------------------------------------------
# set_clock_groups -asynchronous \
#     -group [get_clocks {sd_card_clk}] \
#     -group [get_clocks {ext_mem_clk}] \
#     -group [get_clocks {video_clk}] \
#     -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------

# UART selection integration CDC audit (2026-09-13).
# These are ONLY first-stage sampling registers of existing two-FF CDCs.
# Their source value may be captured on either destination edge; stage 2
# remains fully timed. Retain setup checks (including Gray bit delay/skew)
# and all other video->SD/memory data paths. Do not group clocks async.
# ac_s0 -> ac_s1: WAV start level (held until reset)
# read_req_d0 -> read_req_d1: frame request handshake level
# rgray_s1 -> rgray_s2: registered Gray read pointer
# rrs1 -> rrs2: other FIFO port reset release
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_audio_wav_top_path/ac_s0*}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_audio_wav_top_path/u_fifo2/rgray_s1*}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_audio_wav_top_path/u_fifo2/rrs1*}]

# Free video PLL output provides 12.5MHz for the hard ADC. Related to video_clk.
# Removed sensing instance on A: rename_clock -name {mic_adc_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[2]}]

# Disabled Ethernet: # Ethernet PLL outputs and RGMII source-synchronous interface.
# Disabled Ethernet: rename_clock -name {eth_clk125} -source [get_ports {clk}] -master_clock {clk} [get_pins {u_fpga_wav/u_net_pll/pll_inst.clkc[0]}]
# Disabled Ethernet: rename_clock -name {eth_tx_shift_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {u_fpga_wav/u_net_pll/pll_inst.clkc[4]}]
# Disabled Ethernet: create_generated_clock -name eth_tx_capture -master_clock eth_tx_shift_clk -source [get_pins {u_fpga_wav/u_net_pll/pll_inst.clkc[4]}] -edges {1 2 3} [get_ports {eth_txc}]
# Disabled Ethernet: 
# Disabled Ethernet: set_input_delay -clock eth_rx_launch -max 1.0 [get_ports {eth_rxd[*] eth_rx_ctl}]
# Disabled Ethernet: set_input_delay -clock eth_rx_launch -min -1.0 [get_ports {eth_rxd[*] eth_rx_ctl}]
# Disabled Ethernet: set_input_delay -clock eth_rx_launch -clock_fall -add_delay -max 1.0 [get_ports {eth_rxd[*] eth_rx_ctl}]
# Disabled Ethernet: set_input_delay -clock eth_rx_launch -clock_fall -add_delay -min -1.0 [get_ports {eth_rxd[*] eth_rx_ctl}]
# Disabled Ethernet: set_output_delay -clock eth_tx_capture -max 1.2 [get_ports {eth_txd[*] eth_tx_ctl}]
# Disabled Ethernet: set_output_delay -clock eth_tx_capture -min -1.2 [get_ports {eth_txd[*] eth_tx_ctl}]
# Disabled Ethernet: set_output_delay -clock eth_tx_capture -clock_fall -add_delay -max 1.2 [get_ports {eth_txd[*] eth_tx_ctl}]
# Disabled Ethernet: set_output_delay -clock eth_tx_capture -clock_fall -add_delay -min -1.2 [get_ports {eth_txd[*] eth_tx_ctl}]
# Disabled Ethernet: 
# Disabled Ethernet: set_clock_groups -asynchronous \
# Disabled Ethernet:     -group [get_clocks {eth_rxc eth_rx_launch}] \
# Disabled Ethernet:     -group [get_clocks {clk eth_clk125 eth_tx_shift_clk eth_tx_capture sd_card_clk ext_mem_clk video_clk hdmi_5x_clk mic_adc_clk}]
# Disabled Ethernet: set_clock_groups -asynchronous \
# Disabled Ethernet:     -group [get_clocks {eth_clk125 eth_tx_shift_clk eth_tx_capture}] \
# Disabled Ethernet:     -group [get_clocks {clk sd_card_clk ext_mem_clk video_clk hdmi_5x_clk mic_adc_clk}]
# Disabled Ethernet: 
# Default AUDIO_TEST_MODE=0 removes the legacy I2S tone clock.
# D5: exceptions end only at first CDC capture stages; successors stay timed.
set d5_capture [get_regs -nowarn {u_d5_visual/u_brightness_event_cdc/sync_ff[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/u_contrast_event_cdc/sync_ff[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/u_fade_start_cdc/sync_ff[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/u_image_ready_cdc/sync_ff[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/u_fade_midpoint_cdc/sync_ff[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/u_fade_done_cdc/sync_ff[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/u_display_valid_cdc/display_valid_meta}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/u_osd_status_cdc/sync_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/adjust_select_sync_sd[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/u_key_adjust_debounce/button_sync0}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/sd_card_bmp_m0/commit_ack_sync[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/sd_card_bmp_m0/wrfin_tgl_sync[0]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/sd_req_meta}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/src_w_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/src_h_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/dst_w_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/dst_h_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/xoff_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/yoff_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/geom_meta}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/write_idx_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/source_grant_meta_sd}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/commit_tgl_meta_mem}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/disp_idx_meta_mem[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/u_source_frame_write_d5/ack_meta}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/u_source_frame_write_d5/u_frame_fifo_write/write_req_d0}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/u_source_frame_write_d5/u_frame_fifo_write/write_len_d0[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/u_frame_display_read_d5/u_display_read_rfifo_bridge/fifo_empty_meta_mem}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/u_frame_display_read_d5/u_display_read_rfifo_bridge/boundary_meta_mem}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/u_frame_display_read_d5/u_display_read_rfifo_bridge/transaction_meta_mem}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/u_frame_display_read_d5/u_display_read_rfifo_bridge/start_meta_video}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/u_frame_display_read_d5/u_display_read_rfifo_bridge/transport_idle_meta_video}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
# Transition mode display and per-pixel read selection cross from video_clk.
# Constrain only their first metastability-capture stage as asynchronous.
set d5_capture [get_regs -nowarn {u_lcd_touch/transition_mode_meta[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/transition_mode_meta_mem[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/transition_counter_meta_mem[*]}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set d5_capture [get_regs -nowarn {u_d5_visual/d5_image_memory_path_m0/transition_busy_meta_mem}]
if {[llength $d5_capture] > 0} {set_false_path -to $d5_capture}
set_false_path -from [get_ports {adjust_select}]

# Audited CDC hold exceptions (2026-09-25).
# Use direct SDC commands; keep setup and all successor stages timed.
# These are existing two/three-stage synchronizer inputs from video_clk.
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/u_fade_done_cdc/sync_ff[0]}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/u_fade_midpoint_cdc/sync_ff[0]}]
# The former button_sync0 register was removed by the key-control revision.
# Removed sensing instance on A: set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {pir_startup_done_sync[0]}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/d5_image_memory_path_m0/u_frame_display_read_d5/u_display_read_rfifo_bridge/transaction_meta_mem}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/d5_image_memory_path_m0/u_frame_display_read_d5/u_display_read_rfifo_bridge/boundary_meta_mem}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/d5_image_memory_path_m0/u_frame_display_read_d5/u_display_read_rfifo_bridge/fifo_empty_meta_mem}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_lcd_touch/transition_mode_meta[*]}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/d5_image_memory_path_m0/transition_mode_meta_mem[*]}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/d5_image_memory_path_m0/transition_counter_meta_mem[*]}]
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/d5_image_memory_path_m0/transition_busy_meta_mem}]

# Board link: ONLY asynchronous pad to its first synchronizer stage.
set_false_path -from [get_ports {link_miso}]

# Image-clap toggle CDC: only the first capture stage may sample either cycle.
# Keep setup and the sync0 -> sync1 -> seen path fully timed.
set_false_path -hold -from [get_clocks {video_clk}] -to [get_regs -hier {u_d5_visual/u_clap_image_route/sync0}]
