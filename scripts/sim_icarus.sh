#!/usr/bin/env bash
# =============================================================================
# sim_icarus.sh — Icarus Verilog simulation runner
# Usage: bash scripts/sim_icarus.sh [--waves] [--clean]
#
# Compiles all RTL and TB files using Icarus Verilog (iverilog) and runs
# the simulation. Writes VCD to tb/waves/ptp_sync_tb.vcd.
# =============================================================================

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WAVES_DIR="$REPO_ROOT/tb/waves"
SIM_BIN="$REPO_ROOT/tb/waves/sim_ptp"
VCD_OUT="$WAVES_DIR/ptp_sync_tb.vcd"

# Colours
GREEN='\033[0;32m'
RED='\033[0;31m'
CYAN='\033[0;36m'
DIM='\033[2m'
NC='\033[0m'

echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   IEEE 1588 PTP Slave Sync IP — Icarus Simulation   ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"
echo ""

# Parse arguments
CLEAN=0
for arg in "$@"; do
    case $arg in
        --clean) CLEAN=1 ;;
        --waves) echo -e "${DIM}VCD waveform will be written to: $VCD_OUT${NC}" ;;
    esac
done

if [[ $CLEAN -eq 1 ]]; then
    echo "Cleaning build artifacts..."
    rm -f "$SIM_BIN" "$VCD_OUT"
fi

# Check iverilog is available
if ! command -v iverilog &> /dev/null; then
    echo -e "${RED}ERROR: iverilog not found. Install with: sudo apt-get install iverilog${NC}"
    exit 1
fi

IVERILOG_VER=$(iverilog -V 2>&1 | head -1)
echo -e "${DIM}Simulator: $IVERILOG_VER${NC}"
echo ""

# Create output directory
mkdir -p "$WAVES_DIR"

# Source file lists
RTL_FILES=(
    "$REPO_ROOT/rtl/ptp_msg_parser.sv"
    "$REPO_ROOT/rtl/ptp_delay_req_framer.sv"
    "$REPO_ROOT/rtl/ptp_slave_sync_top.sv"
    "$REPO_ROOT/rtl/ptp_sync_wrapper.sv"
)

TB_FILES=(
    "$REPO_ROOT/tb/tb_ptp_sync_wrapper.sv"
)

echo "Compiling RTL + Testbench..."
echo -e "${DIM}Files:${NC}"
for f in "${RTL_FILES[@]}" "${TB_FILES[@]}"; do
    echo -e "${DIM}  $(basename $f)${NC}"
done
echo ""

# Compile
iverilog -g2012 \
    -Wall \
    -o "$SIM_BIN" \
    "${RTL_FILES[@]}" \
    "${TB_FILES[@]}" \
    2>&1

if [[ $? -ne 0 ]]; then
    echo -e "${RED}✗ Compilation FAILED${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Compilation successful${NC}"
echo ""
echo "Running simulation..."
echo "────────────────────────────────────────────────────────"

# Run simulation — capture output
SIM_OUTPUT=$(vvp "$SIM_BIN" 2>&1)
SIM_EXIT=$?

echo "$SIM_OUTPUT"
echo "────────────────────────────────────────────────────────"
echo ""

# Parse results
PASS_COUNT=$(echo "$SIM_OUTPUT" | grep -c "^PASS:" || true)
FAIL_COUNT=$(echo "$SIM_OUTPUT" | grep -c "^FAIL:" || true)
TIMEOUT=$(echo "$SIM_OUTPUT" | grep -c "TIMEOUT" || true)

echo "Results:"
echo -e "  ${GREEN}Passed: $PASS_COUNT${NC}"
if [[ $FAIL_COUNT -gt 0 ]]; then
    echo -e "  ${RED}Failed: $FAIL_COUNT${NC}"
fi
if [[ $TIMEOUT -gt 0 ]]; then
    echo -e "  ${RED}Timeout: $TIMEOUT${NC}"
fi
echo ""

if [[ -f "$VCD_OUT" ]]; then
    VCD_SIZE=$(du -sh "$VCD_OUT" | cut -f1)
    echo -e "Waveform: ${DIM}$VCD_OUT ($VCD_SIZE)${NC}"
    echo -e "${DIM}View with: gtkwave $VCD_OUT${NC}"
    echo ""
fi

if [[ $FAIL_COUNT -eq 0 && $TIMEOUT -eq 0 ]]; then
    echo -e "${GREEN}╔══════════════════════════════════╗${NC}"
    echo -e "${GREEN}║   ✓  ALL TESTS PASSED            ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════╝${NC}"
    exit 0
else
    echo -e "${RED}╔══════════════════════════════════╗${NC}"
    echo -e "${RED}║   ✗  SIMULATION FAILED           ║${NC}"
    echo -e "${RED}╚══════════════════════════════════╝${NC}"
    exit 1
fi
