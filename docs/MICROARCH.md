# Microarchitecture Reference

## Overview

This document describes the internal microarchitecture of the IEEE 1588 PTP Slave Clock
Synchronization IP. It supplements the top-level README with design-level detail
intended for RTL engineers integrating or modifying the core.

Macnica Americas, Inc.

Peter Mbua.
Dallas (Plano), TX.

---

## Clock Architecture

### Free-Running PTP Hardware Clock

```
clk (125 MHz)
    │
    ▼
┌─────────────────────────────────────┐
│  always_ff @(posedge clk)            │
│  hw_ns += 8  (8 ns per cycle)        │
│  if hw_ns >= 1_000_000_000:          │
│      hw_ns = 0                       │
│      hw_sec++                        │
└─────────────────────────────────────┘
    │
    ├── hw_sec[47:0]    (seconds, 48-bit)
    └── hw_ns[31:0]     (nanoseconds, 32-bit)
```

The free-running clock increments by `NS_INC = 10^9 / CLK_FREQ_HZ` each cycle.
At 125 MHz this is exactly 8 ns. The 80-bit timestamp format matches IEEE 1588-2008
Annex B: `{seconds[47:0], nanoseconds[31:0]}`.

### Servo-Corrected Clock

The corrected clock (`corr_sec`, `corr_ns`) increments identically to the hardware
clock but receives step corrections from the PI servo on each `measure_valid` strobe.
The correction is applied as a signed nanosecond adjustment with borrow/carry handling
at the second boundary.

---

## Timestamp Capture Pipeline

```
Ethernet Frame
      │
      ▼ rx_tuser=1 (SFD)
 phy_rx_timestamp ──────────────────────────────▶ T2 register
      │
      ▼ MSG_SYNC received, seq_id latched
 T2 valid, waiting for Follow_Up
      │
      ▼ MSG_FOLLOW_UP (seq_id matches)
 rx_precise_origin_ts ──────────────────────────▶ T1 register
      │
      ▼ (both T1, T2 valid) → trigger Delay_Req
 Delay_Req TX starts
      │
      ▼ phy_tx_ts_valid strobe
 phy_tx_timestamp ──────────────────────────────▶ T3 register
      │
      ▼ MSG_DELAY_RESP (seq_id + port_id match)
 rx_recv_ts ─────────────────────────────────────▶ T4 register
      │
      ▼ all 4 valid → measure_valid pulse
 Timestamp Engine: compute OFM, MPD
      │
      ▼
 PI Servo: apply correction to corr_ns/corr_sec
```

---

## BMCA Comparison Logic

The BMCA is implemented as a purely combinational `better` signal computed from
all six comparison fields. A registered update occurs on each valid Announce
message that passes domain, AMT, and forced-master filtering.

```
Announce message received
    │
    ├── Domain check  (rx_domain_num == cfg_domain_num?)
    ├── Forced-master (cfg_forced_master == 0?)
    ├── AMT check     (rx_announce_gm_id in cfg_amt_table?)
    │
    ▼ (passes all filters)
    │
    ├── BMCA comparison (6-field cascaded, lowest wins):
    │   1. Priority1
    │   2. ClockClass
    │   3. ClockAccuracy
    │   4. ClockVariance (OADEV)
    │   5. Priority2
    │   6. ClockIdentity (EUI-64 MAC-derived, tiebreaker)
    │
    ├── If BETTER → update bmca_* registers
    │              → if gm_id changed → grandmaster_changed strobe
    │
    └── Reset announce_timer to ANNOUNCE_TO_CNT
```

---

## Port State Machine Transitions

```
                 ┌─────────────────────────────────────┐
                 │                                     │
Power-on ──▶ INIT ──▶ LISTENING ──▶ UNCALIBRATED ──▶ SLAVE
                          ▲              │    ▲         │
                          │   announce   │    │         │
                          │   timeout    │    │ GM      │
                          └─────────────┘    │ changed │
                                             └─────────┘
                          ▲                            │
                          └────────────────────────────┘
                                 announce timeout

              ┌──────────┐
              │  PASSIVE │ (if better master exists on another port — BC use case)
              └──────────┘

              ┌──────────┐
              │  MASTER  │ (if cfg_forced_master and !cfg_slave_only)
              └──────────┘

              ┌──────────┐    ┌──────────┐
              │  FAULTY  │    │ DISABLED │ (reserved)
              └──────────┘    └──────────┘
```

---

## PI Servo Detail

```
On each measure_valid:

  OFM = offset_from_master (signed, nanoseconds)

  P_term = OFM >> KP_SHIFT            (KP_SHIFT=4, Kp=1/16)

  integrator += OFM
  integrator = clamp(integrator, -10_000_000, +10_000_000)  // ±10 ms anti-windup

  I_term = integrator >> KI_SHIFT     (KI_SHIFT=8, Ki=1/256)

  correction = -(P_term + I_term)     // negate: positive OFM → advance clock

  corr_ns += correction
  (handle borrow/carry at second boundary)

  clock_locked = (|OFM| < 1000 ns)   // 1 µs lock threshold
```

**Gain selection rationale:**
- At 8 Hz sync rate and typical LAN MPD of ~100–500 µs, Kp=1/16 gives a
  loop bandwidth of roughly 0.5 Hz — adequate for LAN stability.
- Ki=1/256 provides slow integral wind-up, correcting frequency offset over
  ~30–60 sync cycles (4–8 seconds) without creating servo oscillation.
- For noisier networks (e.g., non-PTP-aware intermediate switches), reduce
  Kp to 1/32 and Ki to 1/1024 via the RTL parameters.

---

## Security Architecture

```
Incoming Announce
        │
        ▼
   ┌─────────────┐  NO   ┌─────────────────────────┐
   │ Domain ==   │──────▶│ Silently discard         │
   │ cfg_domain? │       └─────────────────────────┘
   └──────┬──────┘
          │ YES
          ▼
   ┌──────────────────┐  YES  ┌─────────────────────────┐
   │ cfg_forced_      │──────▶│ forced_master_event=1    │
   │ master?          │       │ Discard                  │
   └──────┬───────────┘       └─────────────────────────┘
          │ NO
          ▼
   ┌──────────────────┐  NO   ┌─────────────────────────┐
   │ AMT empty OR     │──────▶│ amt_violation=1          │
   │ gm_id in table?  │       │ Discard                  │
   └──────┬───────────┘       └─────────────────────────┘
          │ YES
          ▼
   ┌──────────────────┐
   │  BMCA Processing │
   └──────────────────┘
```

---

## Timing Budget (Achievable Accuracy)

For a typical ProAV flat-fabric deployment (2-tier leaf-spine with Mellanox
Spectrum Boundary Clocks at leaf/ToR):

| Source of error | Typical contribution |
|---|---|
| PHY timestamp quantisation | ±4 ns (8 ns clock) |
| PHY timestamping asymmetry | ±2–5 ns (PHY-dependent) |
| Network PDV (PTP-aware BC) | ±1–10 ns |
| Servo steady-state residual | ±10–30 ns |
| **Total (RMS estimate)** | **±20–50 ns** |

With non-PTP-aware intermediate switches, PDV can increase to ±1–10 µs,
and a DSCP 46 "Expedited Forwarding" QoS marking should be applied to all
PTP packets at the switch level.
