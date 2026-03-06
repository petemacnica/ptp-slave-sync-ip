# =============================================================================
# synth_vivado.tcl — Xilinx Vivado synthesis script
# Usage: vivado -mode tcl -source scripts/synth_vivado.tcl
#        OR from Vivado Tcl console: source scripts/synth_vivado.tcl
#
# Default target: Xilinx Kintex UltraScale+ xcku5p-ffvb676-2-e
# Change PART and BOARD_PART below for your target device.
# =============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set REPO_ROOT  [file normalize $SCRIPT_DIR/..]

# ─── Target device ───────────────────────────────────────────────────────────
set PART       "xcku5p-ffvb676-2-e"   ;# Kintex UltraScale+
# set PART    "xc7a200tfbg676-2"      ;# Artix-7 200T (uncomment to change)
# set PART    "xc7z020clg484-1"       ;# Zynq-7020

set PROJECT    "ptp_slave_sync"
set OUTPUT_DIR "$REPO_ROOT/vivado_output"

puts ""
puts "╔══════════════════════════════════════════════════════╗"
puts "║   IEEE 1588 PTP Slave Sync IP — Vivado Synthesis    ║"
puts "╚══════════════════════════════════════════════════════╝"
puts ""
puts "Target part : $PART"
puts "Output dir  : $OUTPUT_DIR"
puts ""

# ─── Create project ──────────────────────────────────────────────────────────
file mkdir $OUTPUT_DIR
create_project $PROJECT $OUTPUT_DIR/$PROJECT -part $PART -force

set_property target_language SystemVerilog [current_project]

# ─── Add RTL sources ─────────────────────────────────────────────────────────
puts "Adding RTL sources..."

add_files -norecurse [list \
    $REPO_ROOT/rtl/ptp_msg_parser.sv       \
    $REPO_ROOT/rtl/ptp_delay_req_framer.sv \
    $REPO_ROOT/rtl/ptp_slave_sync_top.sv   \
    $REPO_ROOT/rtl/ptp_sync_wrapper.sv     \
]

set_property file_type SystemVerilog [get_files *.sv]
set_property top ptp_sync_wrapper [current_fileset]

# ─── Add constraints ─────────────────────────────────────────────────────────
add_files -fileset constrs_1 -norecurse \
    $REPO_ROOT/constraints/ptp_sync.xdc

# ─── Synthesis settings ──────────────────────────────────────────────────────
set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt    [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.KEEP_EQUIVALENT_REGISTERS 1  [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.RESOURCE_SHARING off         [get_runs synth_1]

# ─── Run synthesis ───────────────────────────────────────────────────────────
puts "Running synthesis..."
launch_runs synth_1 -jobs 4
wait_on_run synth_1

# ─── Check results ───────────────────────────────────────────────────────────
set synth_status [get_property STATUS [get_runs synth_1]]
if {[string match "*ERROR*" $synth_status]} {
    puts "ERROR: Synthesis failed — $synth_status"
    exit 1
}

puts ""
puts "Synthesis complete. Reporting utilisation..."

# ─── Reports ─────────────────────────────────────────────────────────────────
open_run synth_1 -name synth_1

report_utilization -file $OUTPUT_DIR/utilization.rpt
report_timing_summary -delay_type min_max \
    -report_unconstrained \
    -check_timing_verbose \
    -max_paths 10 \
    -input_pins \
    -file $OUTPUT_DIR/timing_summary.rpt
report_clock_interaction -file $OUTPUT_DIR/clock_interaction.rpt
report_cdc -file $OUTPUT_DIR/cdc.rpt

puts ""
puts "Reports written to: $OUTPUT_DIR"
puts ""
puts "╔══════════════════════════════════════════════════════╗"
puts "║   Synthesis complete. Check reports for WNS/TNS.    ║"
puts "╚══════════════════════════════════════════════════════╝"
