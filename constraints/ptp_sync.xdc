# =============================================================================
# ptp_sync.xdc — Xilinx Vivado Timing Constraints
# Project: IEEE 1588 PTP Slave Clock Synchronization IP
#
# Adjust pin assignments and I/O standards for your specific board/PCB.
# All internal logic is single clock domain at 125 MHz.
# =============================================================================

# ─── Primary Clock ────────────────────────────────────────────────────────────
# 125 MHz system clock — adjust port name to match your wrapper's clock port
create_clock -name clk_125 \
             -period 8.000 \
             -waveform {0.000 4.000} \
             [get_ports clk]

# ─── Clock Uncertainty ────────────────────────────────────────────────────────
# Jitter budget for 125 MHz on-board oscillator
set_clock_uncertainty -setup 0.100 [get_clocks clk_125]
set_clock_uncertainty -hold  0.050 [get_clocks clk_125]

# ─── Input Delays ─────────────────────────────────────────────────────────────
# AXI4-Stream RX from MAC (assume 2 ns setup margin)
set_input_delay -clock clk_125 -max 2.0 [get_ports {rx_tdata[*]}]
set_input_delay -clock clk_125 -min 0.5 [get_ports {rx_tdata[*]}]
set_input_delay -clock clk_125 -max 2.0 [get_ports rx_tvalid]
set_input_delay -clock clk_125 -min 0.5 [get_ports rx_tvalid]
set_input_delay -clock clk_125 -max 2.0 [get_ports rx_tlast]
set_input_delay -clock clk_125 -min 0.5 [get_ports rx_tlast]

# SFD timestamp strobe — critical path, must meet tight setup
set_input_delay -clock clk_125 -max 1.0 [get_ports rx_tuser]
set_input_delay -clock clk_125 -min 0.2 [get_ports rx_tuser]

# PHY hardware timestamps (assumed registered at PHY boundary, 2 ns slack)
set_input_delay -clock clk_125 -max 2.0 [get_ports {phy_rx_timestamp[*]}]
set_input_delay -clock clk_125 -min 0.5 [get_ports {phy_rx_timestamp[*]}]
set_input_delay -clock clk_125 -max 2.0 [get_ports {phy_tx_timestamp[*]}]
set_input_delay -clock clk_125 -min 0.5 [get_ports {phy_tx_timestamp[*]}]
set_input_delay -clock clk_125 -max 2.0 [get_ports phy_tx_ts_valid]
set_input_delay -clock clk_125 -min 0.5 [get_ports phy_tx_ts_valid]

# TX backpressure
set_input_delay -clock clk_125 -max 2.0 [get_ports tx_tready]
set_input_delay -clock clk_125 -min 0.5 [get_ports tx_tready]

# Configuration ports (quasi-static, relax timing)
set_input_delay -clock clk_125 -max 4.0 [get_ports {cfg_*}]
set_input_delay -clock clk_125 -max 4.0 [get_ports {cfg_amt_table[*]}]
set_input_delay -clock clk_125 -max 4.0 [get_ports {cfg_amt_valid[*]}]

# ─── Output Delays ────────────────────────────────────────────────────────────
set_output_delay -clock clk_125 -max 2.0 [get_ports {tx_tdata[*]}]
set_output_delay -clock clk_125 -min 0.5 [get_ports {tx_tdata[*]}]
set_output_delay -clock clk_125 -max 2.0 [get_ports tx_tvalid]
set_output_delay -clock clk_125 -max 2.0 [get_ports tx_tlast]

# PPS output — relaxed, downstream logic should be tolerant of jitter
set_output_delay -clock clk_125 -max 3.0 [get_ports pps_out]
set_output_delay -clock clk_125 -min 0.5 [get_ports pps_out]

# Status/diagnostic outputs — relaxed (read by register file asynchronously)
set_output_delay -clock clk_125 -max 4.0 [get_ports {local_time_sec[*]}]
set_output_delay -clock clk_125 -max 4.0 [get_ports {local_time_ns[*]}]
set_output_delay -clock clk_125 -max 4.0 [get_ports clock_locked]
set_output_delay -clock clk_125 -max 4.0 [get_ports {offset_from_master[*]}]
set_output_delay -clock clk_125 -max 4.0 [get_ports {mean_path_delay[*]}]
set_output_delay -clock clk_125 -max 4.0 [get_ports {ptp_port_state[*]}]
set_output_delay -clock clk_125 -max 4.0 [get_ports grandmaster_changed]
set_output_delay -clock clk_125 -max 4.0 [get_ports {current_gm_id[*]}]
set_output_delay -clock clk_125 -max 4.0 [get_ports amt_violation]
set_output_delay -clock clk_125 -max 4.0 [get_ports forced_master_event]

# ─── False Paths ─────────────────────────────────────────────────────────────
# Async reset — multicycle is acceptable for reset distribution
set_false_path -from [get_ports rst_n]

# Configuration registers are quasi-static (written by CPU, not timing-critical)
set_multicycle_path -setup 4 -from [get_ports {cfg_*}]
set_multicycle_path -hold  3 -from [get_ports {cfg_*}]
set_multicycle_path -setup 4 -from [get_ports {cfg_amt_table[*]}]
set_multicycle_path -hold  3 -from [get_ports {cfg_amt_table[*]}]

# ─── I/O Standards (adjust for your PCB / voltage rail) ─────────────────────
# Uncomment and set appropriate standard:
# set_property IOSTANDARD LVCMOS33 [get_ports clk]
# set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
# set_property IOSTANDARD LVCMOS18 [get_ports {rx_tdata[*]}]
