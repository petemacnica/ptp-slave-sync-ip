if (rx_tuser)
    captured_hw_ts <= phy_rx_timestamp;
```

This runs **on every valid byte**, independent of FSM state. `rx_tuser` is the SFD (Start Frame Delimiter) strobe from the PHY timestamping unit — it pulses for exactly one cycle at the very first byte of the frame preamble. The moment it fires, the PHY's hardware timestamp is latched into `captured_hw_ts`. This is **T2** for the two-step exchange.

This happens before the FSM has even started processing the Ethernet header, which is why it's outside the `case` — you can't afford to miss it waiting for a state transition.

---

## Stage 2 — S_IDLE
```
Any valid byte arrives → reset all counters → go to S_ETH_HDR
```

`byte_cnt`, `buf_idx`, and the transport flags are all cleared here. The FSM is always in `S_IDLE` between frames and jumps to `S_ETH_HDR` on the first byte of any new frame.

---

## Stage 3 — S_ETH_HDR (14 bytes)

The FSM counts through the Ethernet header byte by byte. Nothing is stored — it's only watching for the EtherType at bytes 12 and 13.
```
byte 0–11  : ignored (DA[6] + SA[6])
byte 12    : ethertype_sr[15:8] ← high byte of EtherType
byte 13    : ethertype_sr[7:0]  ← low byte, decision made:

  0x88F7 → is_ptp_over_eth=1, go to S_PTP_HDR
  0x0800 → go to S_IPV4_HDR
  anything else → go to S_DROP
```

A safety check `if (rx_tlast) state <= S_IDLE` runs on every byte — if the frame ends early (runt frame), the FSM snaps back to IDLE regardless of state.

---

## Stage 4A — S_IPV4_HDR (IPv4 path only)

The parser assumes a **fixed 20-byte IPv4 header** (no options). It counts bytes until `byte_cnt == 33` (which is byte offset 33 from the start of the frame = byte 19 of the IPv4 header = the last IPv4 byte):
```
bytes 14–33 : IPv4 header, all ignored
byte 33     : transition to S_UDP_HDR
```

No IPv4 fields are parsed — the parser only needs to skip past the header to reach UDP.

---

## Stage 4B — S_UDP_HDR (IPv4 path only)

8-byte UDP header, again fully skipped:
```
bytes 34–41 : UDP header, all ignored
byte 41     : is_ptp_over_udp=1, transition to S_PTP_HDR
```

At this point both the Ethernet and IPv4/UDP paths have converged — both arrive at `S_PTP_HDR` with `buf_idx` reset to 0, ready to start collecting PTP bytes.

---

## Stage 5 — S_PTP_HDR (Core Parsing — up to 44 bytes)

This is where all the real work happens. Every incoming byte is written into `ptp_hdr_buf[buf_idx]` sequentially:
```
buf_idx 0  → ptp_hdr_buf[0]   : transportSpecific[7:4] | messageType[3:0]
buf_idx 1  → ptp_hdr_buf[1]   : reserved | versionPTP
buf_idx 2–3 → messageLength
buf_idx 4  → ptp_hdr_buf[4]   : domainNumber
buf_idx 5  → reserved
buf_idx 6–7 → flags
buf_idx 8–15 → correctionField (64-bit)
buf_idx 16–19 → reserved
buf_idx 20–27 → sourcePortIdentity (clockId[8 bytes])
buf_idx 28–29 → sequenceId
buf_idx 30  → controlField
buf_idx 31  → logMessageInterval
buf_idx 32–33 → (reserved / message body start)
buf_idx 34–43 → message body (originTimestamp or Announce body fields)
```

Collection stops at `buf_idx == 43` — the buffer is exactly 44 bytes deep. The decode triggers on **either** `rx_tlast` (frame ended) **or** `buf_idx == 43` (buffer full), whichever comes first.

---

## Stage 6 — Field Extraction (Triggered at End of S_PTP_HDR)

When the decode condition fires, all fields are extracted from `ptp_hdr_buf` in **a single registered assignment** — one clock edge latches everything simultaneously:

**Common header fields (all message types):**

| Output | Buffer bytes | Notes |
|---|---|---|
| `ptp_msg_type` | `[0][3:0]` | Low nibble only |
| `ptp_domain_num` | `[4]` | Full byte |
| `ptp_correction_field` | `[8:15]` | 64-bit concatenation |
| `ptp_src_port_id` | `[20:27]` | 8 bytes of ClockIdentity |
| `ptp_seq_id` | `[28:29]` | 16-bit |
| `ptp_hw_timestamp` | `captured_hw_ts` | Already latched at SFD |
| `ptp_origin_ts` | `[34:43]` | 80-bit, all message types |

**Then a `case` on `ptp_hdr_buf[0][3:0]` decodes message-specific body fields:**

### MSG_SYNC (0x0) and MSG_DELAY_REQ (0x1)
Nothing extra — `ptp_rx_valid` pulses for one cycle. The originTimestamp was already captured into `ptp_origin_ts` above (it's zero for Sync in two-step mode, which is expected).

### MSG_FOLLOW_UP (0x8)
```
ptp_precise_origin_ts ← buf[34:43]   (this is T1 — the master's TX timestamp)
ptp_rx_valid ← 1
```
The `preciseOriginTimestamp` sits at the same buffer offset as `originTimestamp` — the Sync body and Follow_Up body are both 10 bytes at offset 34, they just have different semantic meaning.

### MSG_DELAY_RESP (0x9)
```
ptp_recv_ts ← buf[34:43]   (T4 — when master received the Delay_Req)
ptp_rx_valid ← 1
```
Note: `requestingPortIdentity` would be at buf[44:53] — **beyond the 44-byte buffer**. This means `ptp_req_port_id` is never populated from the buffer. This is a known limitation in the current implementation.

### MSG_ANNOUNCE (0xB)
This is the most complex decode. The Announce body starts at offset 34 and contains:
```
buf[34:43] → originTimestamp (10 bytes, skipped for BMCA)
buf[44:45] → currentUtcOffset       ← BEYOND 44-byte buffer
buf[46]    → reserved
buf[47]    → grandmasterPriority1   ← BEYOND buffer
...
```

**This is a critical bug in the current implementation.** The Announce body fields the BMCA actually needs (Priority1, ClockClass, ClockAccuracy, ClockVariance, Priority2, gmIdentity) all live **beyond byte offset 43** — outside the buffer. The current code uses approximate offsets within the 44-byte window (buf[37]–buf[43]) which map to the originTimestamp bytes, not the actual BMCA fields. This is almost certainly the root cause of `GM_ID = 0x0000000000000000` that you saw — `ptp_announce_gm_id` is being assembled from the wrong buffer bytes.

---

## Stage 7 — S_DROP

Any non-PTP, non-IPv4 frame ends up here. The FSM does nothing except wait for `rx_tlast`, then returns to `S_IDLE`. No outputs are touched.

---

## The Buffer Sizing Bug — Root Cause of Your GM_ID Failure

The 44-byte buffer captures PTP header (34 bytes) + 10 bytes of body. That covers `originTimestamp` for Sync/Follow_Up/Delay_Resp — but the Announce message body is **30 bytes long**, requiring the buffer to extend to at least **64 bytes** (34 header + 30 body) to reach `gmIdentity`:
```
PTP header:           34 bytes  (offsets 0–33)
Announce body:
  originTimestamp:    10 bytes  (offsets 34–43)  ← buffer ends here ✗
  currentUtcOffset:    2 bytes  (offsets 44–45)
  reserved:            1 byte   (offset  46)
  gmPriority1:         1 byte   (offset  47)
  gmClockQuality:      4 bytes  (offsets 48–51)
  gmPriority2:         1 byte   (offset  52)
  gmIdentity:          8 bytes  (offsets 53–60)  ← what BMCA needs
  stepsRemoved:        2 bytes  (offsets 61–62)
  timeSource:          1 byte   (offset  63)
