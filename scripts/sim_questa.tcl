# =============================================================================
# sim_questa.tcl — Questa / ModelSim simulation script
# Usage: questa vsim -do scripts/sim_questa.tcl
#        OR from Questa console: do scripts/sim_questa.tcl
# =============================================================================

set REPO_ROOT [file normalize [file dirname [file normalize [info script]]]/..]

puts ""
puts "╔══════════════════════════════════════════════════════╗"
puts "║   IEEE 1588 PTP Slave Sync IP — Questa Simulation   ║"
puts "╚══════════════════════════════════════════════════════╝"
puts ""

# ─── Create work library ─────────────────────────────────────────────────────
if {[file exists work]} {
    vdel -lib work -all
}
vlib work
vmap work work

# ─── Compile RTL ─────────────────────────────────────────────────────────────
puts "Compiling RTL..."

vlog -sv -work work \
    +define+SIM \
    -timescale "1ns/1ps" \
    $REPO_ROOT/rtl/ptp_msg_parser.sv \
    $REPO_ROOT/rtl/ptp_delay_req_framer.sv \
    $REPO_ROOT/rtl/ptp_slave_sync_top.sv \
    $REPO_ROOT/rtl/ptp_sync_wrapper.sv

# ─── Compile Testbench ───────────────────────────────────────────────────────
puts "Compiling Testbench..."

vlog -sv -work work \
    -timescale "1ns/1ps" \
    $REPO_ROOT/tb/tb_ptp_sync_wrapper.sv

# ─── Simulate ────────────────────────────────────────────────────────────────
puts ""
puts "Starting simulation..."
puts "────────────────────────────────────────────────────────"

vsim -t 1ps \
     -lib work \
     work.tb_ptp_sync_wrapper \
     -voptargs="+acc" \
     -do "
        # Add wavegroups
        add wave -divider {System}
        add wave /tb_ptp_sync_wrapper/clk
        add wave /tb_ptp_sync_wrapper/rst_n

        add wave -divider {MAC RX}
        add wave /tb_ptp_sync_wrapper/rx_tdata
        add wave /tb_ptp_sync_wrapper/rx_tvalid
        add wave /tb_ptp_sync_wrapper/rx_tlast
        add wave /tb_ptp_sync_wrapper/rx_tuser

        add wave -divider {PTP Status}
        add wave /tb_ptp_sync_wrapper/ptp_port_state
        add wave /tb_ptp_sync_wrapper/clock_locked
        add wave /tb_ptp_sync_wrapper/grandmaster_changed
        add wave /tb_ptp_sync_wrapper/current_gm_id

        add wave -divider {Servo}
        add wave -radix decimal /tb_ptp_sync_wrapper/offset_from_master
        add wave /tb_ptp_sync_wrapper/mean_path_delay

        add wave -divider {Clock Output}
        add wave /tb_ptp_sync_wrapper/local_time_sec
        add wave /tb_ptp_sync_wrapper/local_time_ns
        add wave /tb_ptp_sync_wrapper/pps_out

        add wave -divider {Security}
        add wave /tb_ptp_sync_wrapper/amt_violation
        add wave /tb_ptp_sync_wrapper/forced_master_event

        # Run simulation
        run -all

        # Report
        echo ''
        echo '────────────────────────────────────────────────────────'
        echo 'Simulation complete. Check transcript for PASS/FAIL.'
        echo '────────────────────────────────────────────────────────'
     "
