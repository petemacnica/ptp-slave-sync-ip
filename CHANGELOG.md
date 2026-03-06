# Changelog

All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.0] — 2024-04-21

### Added
- Initial release of IEEE 1588 PTP Slave Clock Synchronization IP
- `ptp_slave_sync_top`: Core engine with BMCA, port FSM (9 states), timestamp engine, PI servo, and 80-bit corrected PTP clock
- `ptp_msg_parser`: AXI4-Stream byte-stream PTP frame decoder supporting Ethernet II and IPv4/UDP transport, hardware RX timestamp latch on SFD
- `ptp_delay_req_framer`: IEEE 1588 Delay_Request TX frame builder with PHY hardware TX timestamp (T3) capture
- `ptp_sync_wrapper`: Top-level integration wrapper for SoC instantiation
- `tb_ptp_sync_wrapper`: Directed SystemVerilog testbench with 10 automated test scenarios and VCD output
- Full BMCA: 6-field cascaded comparison (Priority1, ClockClass, ClockAccuracy, ClockVariance, Priority2, ClockIdentity)
- Acceptable Master Table (AMT) security filter: 8-entry ClockIdentity whitelist, parameterisable depth
- Forced-master port security feature with event logging
- PTP domain filter (default 127, SMPTE ST 2059-2)
- PI servo controller with fixed-point Kp=2⁻⁴, Ki=2⁻⁸ and ±10 ms anti-windup integrator clamp
- Offset-from-master and mean-path-delay threshold alarms
- `clock_locked` output (|OFM| < 1 µs criterion)
- Pulse-per-second (`pps_out`) output
- SMPTE ST 2059-2 default profile parameters
- Vivado XDC timing constraints
- Icarus Verilog simulation script
- Questa/ModelSim simulation TCL script
- Vivado synthesis TCL script
- CI workflow via GitHub Actions (Icarus Verilog)
- Interactive HTML microarchitecture design document

### Protocol Compliance
- IEEE 1588-2008 §9.3.4 BMCA (Two-Step Ordinary/Slave Clock)
- SMPTE ST 2059-2 profile defaults
- AES67 / AESr16 compatible message rate ranges
- Ethernet II transport (EtherType 0x88F7), PTP multicast DA 01:1B:19:00:00:00

---

## [Unreleased]

### Planned
- One-step clock mode support
- Peer-to-peer (P2P) delay mechanism (PDelay_Req/Resp)
- Transparent Clock mode
- IPv6 UDP transport support
- Unicast negotiation (IEEE 1588 Annex D)
- Configurable servo gains via register interface
- Multi-domain support (simultaneous domain tracking)
- FPGA reference design for Xilinx KC705 / ZCU102
