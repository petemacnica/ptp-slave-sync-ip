# Contributing

Thank you for your interest in contributing to the IEEE 1588 PTP Slave Clock Synchronization IP.
Contributions are welcome — whether bug reports, RTL improvements, new testbench scenarios,
documentation fixes, or new synthesis targets.

---

## Getting Started

1. **Fork** the repository on GitHub
2. **Clone** your fork locally
3. Create a **feature branch**: `git checkout -b feature/your-feature-name`
4. Make your changes (see guidelines below)
5. Run the simulation to verify no regressions: `bash scripts/sim_icarus.sh`
6. **Push** your branch and open a **Pull Request**

---

## Code Style

### SystemVerilog RTL

- Use `always_ff`, `always_comb`, `always_latch` (never bare `always`)
- Use `logic` for all signals (not `reg` or `wire`)
- Use `'0`, `'1` for width-agnostic constants
- Every module must have a header comment block with: Module, Description, Author, Revision
- Parameter names in `UPPER_SNAKE_CASE`, signal names in `lower_snake_case`
- 4-space indentation, no tabs
- Limit line length to 100 characters
- All `always_ff` blocks must include an async reset (`negedge rst_n`)
- Use named port connections in instantiations (`.port_name(signal_name)`)

### Testbench

- Every new RTL feature must have at least one directed test in the testbench
- Each test must emit a `$display("PASS: ...")` or `$display("FAIL: ...")` and update `test_errors`
- Tests must be deterministic — no random seeds without a fixed default seed
- VCD dumping must not be removed (required for CI waveform inspection)

---

## Submitting RTL Changes

Before opening a PR:

- [ ] Simulation passes with zero errors (`test_errors == 0`)
- [ ] No `$fatal` or unintended `$error` in simulation output
- [ ] New parameters are documented in `README.md` parameter tables
- [ ] Update `CHANGELOG.md` under `[Unreleased]`
- [ ] Ensure synthesis does not infer unintended latches (check Vivado/Quartus warnings)

---

## Reporting Bugs

Open a GitHub Issue with the following:

1. **Summary** — one-sentence description
2. **Steps to reproduce** — minimal simulation stimulus or waveform
3. **Expected behavior** — what the spec says should happen
4. **Actual behavior** — what the IP does instead
5. **Environment** — simulator version, synthesis tool if applicable

---

## Roadmap Items (Good First Contributions)

| Item | Difficulty | Description |
|---|---|---|
| Configurable servo Kp/Ki | Easy | Expose servo gains as RTL parameters |
| Lock hysteresis counter | Easy | Require N consecutive |OFM|<1µs before asserting `clock_locked` |
| IPv6 UDP transport | Medium | Add IPv6 header parsing in `ptp_msg_parser` |
| P2P delay mechanism | Medium | Add PDelay_Req/Resp framer and parser |
| One-step mode | Medium | No Follow_Up — insert T1 directly into Sync |
| UVM testbench | Hard | Replace directed TB with UVM environment and coverage |
| FPGA reference design | Hard | Integrate with Xilinx Ethernet Subsystem on KC705/ZCU102 |

---

## License

By contributing, you agree that your contributions will be licensed under the project's
[MIT License](LICENSE).
