// ============================================================================
// Macnica Americas
// 
// Module       : ptp_slave_sync_top
// Project      : IEEE 1588 PTP Slave Clock Synchronization IP
// Description  : Top-level PTP Boundary/Ordinary Clock Synchronization Logic
//                for ProAV Encoder/Decoder endpoints over Ethernet II networks.
//                Implements IEEE 1588-2008 PTP two-step slave synchronization
//                with hardware timestamping, BMCA filtering, and servo control.
//
// Architecture : Master–Slave Two-Step Mode
//                T1 → Sync_message (Master TX timestamp via Follow_Up)
//                T2 → Sync_message RX timestamp (local hardware)
//                T3 → Delay_Request TX timestamp (local hardware)
//                T4 → Delay_Response from Master
//
//                Clock Offset  = ((T2-T1) - (T4-T3)) / 2
//                Mean Path Dly = ((T2-T1) + (T4-T3)) / 2
//
// PTP Profile  : SMPTE ST 2059-2 (ProAV default), domain 127
//                Sync interval : -3 (125 ms / 8 Hz)
//                Announce int  : -2 (250 ms / 4 Hz)
//                Announce TO   : 3 (× announce interval)
//
// Target       : Mellanox Spectrum-compatible Boundary Clock endpoint
//                (Slave/Ordinary Clock instantiation in AV encoder/decoder)
//
// Author       : Peter Mbua (Plano, TX)
// Revision     : 1.0  (2026)
// ============================================================================

`timescale 1ns / 1ps
//`default_nettype none

module ptp_slave_sync_top #(
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    parameter int unsigned CLK_FREQ_HZ      = 125_000_000, // System clock (125 MHz)
    parameter int unsigned TIMESTAMP_W      = 80,          // 48b seconds + 32b nanoseconds
    parameter int unsigned NS_W             = 32,          // Nanosecond counter width
    parameter int unsigned SEC_W            = 48,          // Seconds counter width
    parameter int unsigned CORR_W           = 64,          // Correction field width (ns, Q32)
    parameter int unsigned SERVO_ACC_W      = 48,          // Servo accumulator width
    parameter int unsigned PTP_DOMAIN_NUM   = 127,         // SMPTE ST 2059-2 default domain
    parameter int unsigned ANNOUNCE_TIMEOUT = 3,           // Announce receipt timeout multiplier
    parameter int unsigned PRIORITY1_DEF    = 128,
    parameter int unsigned PRIORITY2_DEF    = 128,
    parameter logic [63:0] CLOCK_ID         = 64'hDEAD_BEEF_CAFE_0001, // 64-bit ClockIdentity (EUI-64)
    parameter int unsigned AMT_DEPTH        = 8,           // Acceptable Master Table entries
    parameter int unsigned MPD_THR_NS       = 1_000_000,   // Mean path delay threshold (1 ms)
    parameter int unsigned OFM_THR_NS       = 100_000,      // Offset-from-master threshold (100 µs)
    
    parameter int unsigned SIM_SPEEDUP      = 1  // set to 1000 in TB instantiation
) (
    // -------------------------------------------------------------------------
    // Clocks & Resets
    // -------------------------------------------------------------------------
    input  wire               clk,             // System clock (125 MHz)
    input  bit                rst_n,           // Active-low async reset

    // -------------------------------------------------------------------------
    // PTP Message RX Interface (from MAC/PHY layer parser)
    // -------------------------------------------------------------------------
    input  wire                 rx_ptp_valid,        // Strobed 1 cycle when PTP frame decoded
    input  logic [3:0]          rx_msg_type,         // PTP messageType field [3:0]
    input  logic [7:0]          rx_domain_num,       // domainNumber
    input  logic [63:0]         rx_src_port_id,      // sourcePortIdentity [79:16]
    input  logic [15:0]         rx_seq_id,           // sequenceId
    input  logic [CORR_W-1:0]   rx_correction_field, // correctionField (ns Q16.16)
    input  logic [79:0]         rx_origin_ts,        // originTimestamp (Sync/DelayReq)
    input  logic [79:0]         rx_hw_timestamp,     // Hardware RX timestamp from PHY
    // Follow_Up specific
    input  logic [79:0]         rx_precise_origin_ts,// preciseOriginTimestamp (Follow_Up)
    // Delay_Resp specific
    input  logic [79:0]         rx_recv_ts,          // receiveTimestamp (Delay_Resp)
    input  logic [63:0]         rx_req_port_id,      // requestingPortIdentity

    // -------------------------------------------------------------------------
    // PTP Message TX Interface (to MAC/PHY framer)
    // -------------------------------------------------------------------------
    output logic                tx_delay_req_valid,   // Request to transmit Delay_Req
    output logic [15:0]         tx_delay_req_seq_id,  // Delay_Req sequenceId
    output logic [79:0]         tx_delay_req_ts,      // Delay_Req originTimestamp (T3)
    input  logic                tx_delay_req_ack,     // TX framer accepted the request
    input  logic [79:0]         tx_hw_timestamp,      // HW TX timestamp feedback (T3 precise)

    // -------------------------------------------------------------------------
    // Announce Message Dataset Interface
    // -------------------------------------------------------------------------
    input  logic                rx_announce_valid,
    input  logic [7:0]          rx_announce_priority1,
    input  logic [7:0]          rx_announce_clock_class,
    input  logic [7:0]          rx_announce_clock_accuracy,
    input  logic [15:0]         rx_announce_clock_variance,
    input  logic [7:0]          rx_announce_priority2,
    input  logic [63:0]         rx_announce_gm_id,
    input  logic [15:0]         rx_announce_steps_removed,
    input  logic [7:0]          rx_announce_time_source,

    // -------------------------------------------------------------------------
    // Recovered / Adjusted Local Clock Output
    // -------------------------------------------------------------------------
    output logic [SEC_W-1:0]    local_time_sec,      // Corrected local seconds
    output logic [NS_W-1:0]     local_time_ns,       // Corrected local nanoseconds
    output logic                pps_out,             // Pulse-per-second output
    output logic                clock_locked,        // Servo locked to master

    // -------------------------------------------------------------------------
    // Servo / Debug Outputs
    // -------------------------------------------------------------------------
    output logic signed [31:0]  offset_from_master,  // OFM (ns), signed
    output logic [31:0]         mean_path_delay,     // MPD (ns), unsigned
    output logic                ofm_threshold_alarm, // OFM exceeded threshold
    output logic                mpd_threshold_alarm, // MPD exceeded threshold

    // -------------------------------------------------------------------------
    // BMCA / State Outputs
    // -------------------------------------------------------------------------
    output logic [2:0]          ptp_port_state,      // PTP port state machine output
    output logic                grandmaster_changed, // Strobe: new GM selected
    output logic [63:0]         current_gm_id,       // Current grandmaster ClockIdentity

    // -------------------------------------------------------------------------
    // Configuration / Status (register interface)
    // -------------------------------------------------------------------------
    input  logic                cfg_slave_only,      // Force slave-only mode
    input  logic                cfg_forced_master,   // Discard incoming Announce
    input  logic [7:0]          cfg_domain_num,      // Configurable domain override
    input  logic [AMT_DEPTH-1:0][63:0] cfg_amt_table, // Acceptable Master Table
    input  logic [AMT_DEPTH-1:0]       cfg_amt_valid, // AMT entry valid bits
    output logic                amt_violation,       // Announce from non-AMT source
    output logic                forced_master_event  // Announce discarded by forced-master
);

    // =========================================================================
    // Local Parameters & Derived Constants
    // =========================================================================
    localparam int unsigned NS_PER_SEC      = 1_000_000_000;
    localparam int unsigned NS_INC          = NS_PER_SEC / CLK_FREQ_HZ; // 8 ns @ 125 MHz
    localparam int unsigned SYNC_INTERVAL   = CLK_FREQ_HZ / (8 * SIM_SPEEDUP);  // 125ms @ 8Hz (logSyncInt=-3)
    localparam int unsigned ANNOUNCE_INTV   = CLK_FREQ_HZ / (4 * SIM_SPEEDUP);  // 250ms @ 4Hz (logAnnInt=-2)
    localparam int unsigned ANNOUNCE_TO_CNT = ANNOUNCE_TIMEOUT * ANNOUNCE_INTV;
    localparam int unsigned DELAYREQ_INTV   = SYNC_INTERVAL;     // logSyncInt

    // PTP messageType encoding (IEEE 1588-2008 Table 19)
    localparam logic [3:0] MSG_SYNC         = 4'h0;
    localparam logic [3:0] MSG_DELAY_REQ    = 4'h1;
    localparam logic [3:0] MSG_FOLLOW_UP    = 4'h8;
    localparam logic [3:0] MSG_DELAY_RESP   = 4'h9;
    localparam logic [3:0] MSG_ANNOUNCE     = 4'hB;

    // PTP Port State encoding
    localparam logic [2:0] STATE_INIT        = 3'd0;
    localparam logic [2:0] STATE_LISTENING   = 3'd1;
    localparam logic [2:0] STATE_UNCALIB     = 3'd2;
    localparam logic [2:0] STATE_SLAVE       = 3'd3;
    localparam logic [2:0] STATE_PASSIVE     = 3'd4;
    localparam logic [2:0] STATE_MASTER      = 3'd5;
    localparam logic [2:0] STATE_FAULTY      = 3'd6;
    localparam logic [2:0] STATE_DISABLED    = 3'd7;

    // =========================================================================
    // Internal Signals
    // =========================================================================

    // -------------------------------------------------------------------------
    // Free-running local hardware clock (before servo adjustment)
    // -------------------------------------------------------------------------
    logic [SEC_W-1:0]   hw_sec;
    logic [NS_W-1:0]    hw_ns;

    // -------------------------------------------------------------------------
    // Servo-corrected clock registers
    // -------------------------------------------------------------------------
    logic [SEC_W-1:0]   corr_sec;
    logic [NS_W-1:0]    corr_ns;
    logic               pps_r;

    // -------------------------------------------------------------------------
    // Timestamp capture registers
    // -------------------------------------------------------------------------
    logic [79:0]  t1_timestamp;   // Master TX time (from Follow_Up)
    logic [79:0]  t2_timestamp;   // Local RX time of Sync (hardware)
    logic [79:0]  t3_timestamp;   // Local TX time of Delay_Req (hardware)
    logic [79:0]  t4_timestamp;   // Master RX time of Delay_Req (from Delay_Resp)
    logic         t1_valid, t2_valid, t3_valid, t4_valid;
    logic [15:0]  sync_seq_id_latch;   // Latched sequenceId for Sync/FollowUp matching
    logic [15:0]  dreq_seq_id_latch;   // Latched sequenceId for DelayReq/Resp matching

    // -------------------------------------------------------------------------
    // Measurement pipeline
    // -------------------------------------------------------------------------
    logic signed [63:0]  t2_minus_t1;      // Forward delay + offset
    logic signed [63:0]  t4_minus_t3;      // Backward delay - offset
    logic signed [63:0]  offset_calc;      // Clock offset = ((T2-T1)-(T4-T3))/2
    logic signed [63:0]  mpd_calc;         // Mean path delay = ((T2-T1)+(T4-T3))/2
    logic                measure_valid;    // All four timestamps captured, ready to compute

    // -------------------------------------------------------------------------
    // Servo (PI controller)
    // -------------------------------------------------------------------------
    logic signed [SERVO_ACC_W-1:0]  servo_integrator;
    logic signed [31:0]             servo_p_term;
    logic signed [31:0]             servo_i_term;
    logic signed [31:0]             servo_correction; // ns per servo_update
    logic                           servo_update;

    // Servo gains (tunable — default for ProAV LAN environment)
    // Kp = 1/16, Ki = 1/256 (fixed-point scaled)
    localparam int signed KP_SHIFT = 4;   // Kp = 2^-4
    localparam int signed KI_SHIFT = 8;   // Ki = 2^-8

    // -------------------------------------------------------------------------
    // BMCA dataset
    // -------------------------------------------------------------------------
    logic [7:0]   bmca_priority1;
    logic [7:0]   bmca_clock_class;
    logic [7:0]   bmca_clock_accuracy;
    logic [15:0]  bmca_clock_variance;
    logic [7:0]   bmca_priority2;
    logic [63:0]  bmca_gm_id;
    logic [15:0]  bmca_steps_removed;
    logic         bmca_valid;
    logic [63:0]  current_gm_id_r;
    logic         gm_changed_r;

    // -------------------------------------------------------------------------
    // Announce timeout counter
    // -------------------------------------------------------------------------
    logic [31:0]  announce_timer;
    logic         announce_timeout;

    // -------------------------------------------------------------------------
    // Delay_Req rate control
    // -------------------------------------------------------------------------
    logic [31:0]  dreq_timer;
    logic         dreq_trigger;
    logic [15:0]  dreq_seq_id_cnt;

    // -------------------------------------------------------------------------
    // AMT check
    // -------------------------------------------------------------------------
    logic         amt_pass;
    logic         amt_viol_r;

    // -------------------------------------------------------------------------
    // Port state machine
    // -------------------------------------------------------------------------
    logic [2:0]   port_state;
    logic         forced_master_evt_r;

    // =========================================================================
    // 1. FREE-RUNNING HARDWARE CLOCK (PTP Clock)
    //    Increments at CLK_FREQ_HZ, wraps seconds at 1e9 ns
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hw_ns  <= '0;
            hw_sec <= '0;
        end else begin
            if (hw_ns >= NS_W'(NS_PER_SEC - NS_INC)) begin
                hw_ns  <= '0;
                hw_sec <= hw_sec + 1'b1;
            end else begin
                hw_ns  <= hw_ns + NS_W'(NS_INC);
            end
        end
    end

    // =========================================================================
    // 2. SERVO-CORRECTED CLOCK
    //    On servo_update, the corrected clock is adjusted by servo_correction
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            corr_ns  <= '0;
            corr_sec <= '0;
            pps_r    <= 1'b0;
        end else begin
            pps_r <= 1'b0;
            // Normal increment
            if (corr_ns >= NS_W'(NS_PER_SEC - NS_INC)) begin
                corr_ns  <= '0;
                corr_sec <= corr_sec + 1'b1;
                pps_r    <= 1'b1;
            end else begin
                corr_ns <= corr_ns + NS_W'(NS_INC);
            end

            // Apply servo step correction on servo_update
            if (servo_update && port_state == STATE_SLAVE) begin
                logic signed [NS_W:0] adjusted_ns;
                adjusted_ns = $signed({1'b0, corr_ns}) + servo_correction;
                if (adjusted_ns < 0) begin
                    // Borrow from seconds
                    corr_ns  <= NS_W'(NS_PER_SEC) + adjusted_ns[NS_W-1:0];
                    corr_sec <= corr_sec - 1'b1;
                end else if (adjusted_ns >= NS_W'(NS_PER_SEC)) begin
                    // Carry into seconds
                    corr_ns  <= adjusted_ns[NS_W-1:0] - NS_W'(NS_PER_SEC);
                    corr_sec <= corr_sec + 1'b1;
                end else begin
                    corr_ns  <= adjusted_ns[NS_W-1:0];
                end
            end
        end
    end

    assign local_time_sec = corr_sec;
    assign local_time_ns  = corr_ns;
    assign pps_out        = pps_r;

    // =========================================================================
    // 3. AMT (Acceptable Master Table) CHECK
    //    Filter incoming Announce messages by ClockIdentity whitelist
    // =========================================================================
    always_comb begin
        amt_pass = 1'b0;
        for (int i = 0; i < AMT_DEPTH; i++) begin
            if (cfg_amt_valid[i] && (cfg_amt_table[i] == rx_announce_gm_id))
                amt_pass = 1'b1;
        end
        // If AMT is empty (all invalid), pass all (open mode)
        if (cfg_amt_valid == '0) amt_pass = 1'b1;
    end

    // =========================================================================
    // 4. ANNOUNCE PROCESSING & BMCA DATASET UPDATE
    //    Receive Announce, check domain, AMT, forced-master, update BMCA state
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bmca_valid         <= 1'b0;
            bmca_priority1     <= 8'd255;
            bmca_clock_class   <= 8'd255;
            bmca_clock_accuracy<= 8'hFE;
            bmca_clock_variance<= 16'hFFFF;
            bmca_priority2     <= 8'd255;
            bmca_gm_id         <= '0;
            bmca_steps_removed <= '0;
            announce_timer     <= '0;
            current_gm_id_r    <= '0;
            gm_changed_r       <= 1'b0;
            amt_viol_r         <= 1'b0;
            forced_master_evt_r<= 1'b0;
        end else begin
            gm_changed_r       <= 1'b0;
            amt_viol_r         <= 1'b0;
            forced_master_evt_r<= 1'b0;

            // Announce timeout counter
            if (announce_timer > 0)
                announce_timer <= announce_timer - 1'b1;

            if (rx_announce_valid) begin
                // Domain filter
                if (rx_domain_num != cfg_domain_num) begin
                    // Silently discard — wrong domain
                end
                // Forced-master: discard and log
                else if (cfg_forced_master) begin
                    forced_master_evt_r <= 1'b1;
                end
                // AMT check
                else if (!amt_pass) begin
                    amt_viol_r <= 1'b1;
                end
                else begin
                    // BMCA comparison: update if incoming is "better"
                    // Criteria (lowest wins): Priority1, ClockClass,
                    //   ClockAccuracy, ClockVariance, Priority2, GrandmasterID
                    logic better;
                    better = (rx_announce_priority1 < bmca_priority1) ||
                             (rx_announce_priority1 == bmca_priority1 &&
                              rx_announce_clock_class < bmca_clock_class) ||
                             (rx_announce_priority1 == bmca_priority1 &&
                              rx_announce_clock_class == bmca_clock_class &&
                              rx_announce_clock_accuracy < bmca_clock_accuracy) ||
                             (rx_announce_priority1 == bmca_priority1 &&
                              rx_announce_clock_class == bmca_clock_class &&
                              rx_announce_clock_accuracy == bmca_clock_accuracy &&
                              rx_announce_clock_variance < bmca_clock_variance) ||
                             (rx_announce_priority1 == bmca_priority1 &&
                              rx_announce_clock_class == bmca_clock_class &&
                              rx_announce_clock_accuracy == bmca_clock_accuracy &&
                              rx_announce_clock_variance == bmca_clock_variance &&
                              rx_announce_priority2 < bmca_priority2) ||
                             (rx_announce_priority1 == bmca_priority1 &&
                              rx_announce_clock_class == bmca_clock_class &&
                              rx_announce_clock_accuracy == bmca_clock_accuracy &&
                              rx_announce_clock_variance == bmca_clock_variance &&
                              rx_announce_priority2 == bmca_priority2 &&
                              rx_announce_gm_id < bmca_gm_id) ||
                             !bmca_valid;

                    if (better) begin
                        bmca_priority1      <= rx_announce_priority1;
                        bmca_clock_class    <= rx_announce_clock_class;
                        bmca_clock_accuracy <= rx_announce_clock_accuracy;
                        bmca_clock_variance <= rx_announce_clock_variance;
                        bmca_priority2      <= rx_announce_priority2;
                        bmca_steps_removed  <= rx_announce_steps_removed;
                        bmca_valid          <= 1'b1;

                        if (rx_announce_gm_id != current_gm_id_r) begin
                            gm_changed_r    <= 1'b1;
                            current_gm_id_r <= rx_announce_gm_id;
                        end
                        bmca_gm_id <= rx_announce_gm_id;
                    end

                    // Reset announce timeout
                    announce_timer <= ANNOUNCE_TO_CNT;
                end
            end
        end
    end

    assign announce_timeout  = (announce_timer == '0) && bmca_valid;
    assign grandmaster_changed = gm_changed_r;
    assign current_gm_id     = current_gm_id_r;
    assign amt_violation     = amt_viol_r;
    assign forced_master_event = forced_master_evt_r;

    // =========================================================================
    // 5. PTP PORT STATE MACHINE
    //    Implements IEEE 1588-2008 state transitions for Slave/Ordinary Clock
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            port_state <= STATE_INIT;
        end else begin
            case (port_state)
                STATE_INIT: begin
                    port_state <= STATE_LISTENING;
                end

                STATE_LISTENING: begin
                    if (bmca_valid && !announce_timeout)
                        port_state <= STATE_UNCALIB;
                end

                STATE_UNCALIB: begin
                    // Transition to SLAVE after first valid OFM measurement
                    if (measure_valid)
                        port_state <= STATE_SLAVE;
                    if (announce_timeout)
                        port_state <= STATE_LISTENING;
                end

                STATE_SLAVE: begin
                    if (announce_timeout)
                        port_state <= STATE_LISTENING;
                    else if (gm_changed_r)
                        port_state <= STATE_UNCALIB;
                    else if (cfg_forced_master && !cfg_slave_only)
                        port_state <= STATE_MASTER;
                end

                STATE_PASSIVE: begin
                    if (announce_timeout)
                        port_state <= STATE_LISTENING;
                end

                STATE_MASTER: begin
                    if (!cfg_forced_master)
                        port_state <= STATE_LISTENING;
                end

                default: port_state <= STATE_INIT;
            endcase
        end
    end

    assign ptp_port_state = port_state;

    // =========================================================================
    // 6. TIMESTAMP CAPTURE — Two-Step Sync / Follow_Up
    //    T1: preciseOriginTimestamp from Follow_Up message
    //    T2: Hardware RX timestamp of Sync message
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t1_valid         <= 1'b0;
            t2_valid         <= 1'b0;
            t1_timestamp     <= '0;
            t2_timestamp     <= '0;
            sync_seq_id_latch<= '0;
        end else begin
            if (rx_ptp_valid && rx_domain_num == cfg_domain_num) begin
                // Sync: capture T2 (HW RX timestamp), latch seqId
                if (rx_msg_type == MSG_SYNC) begin
                    t2_timestamp      <= rx_hw_timestamp;
                    t2_valid          <= 1'b1;
                    sync_seq_id_latch <= rx_seq_id;
                    t1_valid          <= 1'b0; // Invalidate T1 until Follow_Up arrives
                end
                // Follow_Up: capture T1, validate seqId match
                if (rx_msg_type == MSG_FOLLOW_UP &&
                    rx_seq_id == sync_seq_id_latch && t2_valid) begin
                    t1_timestamp <= rx_precise_origin_ts;
                    t1_valid     <= 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // 7. DELAY_REQUEST GENERATION (rate-controlled, T3 capture)
    //    Triggered at logSyncInt interval (default = logSyncInt = -3 → 8 Hz)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dreq_timer        <= '0;
            dreq_trigger      <= 1'b0;
            dreq_seq_id_cnt   <= '0;
            t3_valid          <= 1'b0;
            t3_timestamp      <= '0;
            dreq_seq_id_latch <= '0;
        end else begin
            dreq_trigger <= 1'b0;

            // Delay_Req interval counter
            if (dreq_timer == '0) begin
                if (port_state == STATE_SLAVE || port_state == STATE_UNCALIB) begin
                    dreq_trigger    <= 1'b1;
                    dreq_seq_id_latch <= dreq_seq_id_cnt;
                    dreq_seq_id_cnt <= dreq_seq_id_cnt + 1'b1;
                    dreq_timer      <= DELAYREQ_INTV;
                    t3_valid        <= 1'b0;
                end
            end else begin
                dreq_timer <= dreq_timer - 1'b1;
            end

            // Capture T3 from TX hardware timestamp feedback
            if (tx_delay_req_ack) begin
                t3_timestamp <= tx_hw_timestamp;
                t3_valid     <= 1'b1;
            end
        end
    end

    assign tx_delay_req_valid  = dreq_trigger;
    assign tx_delay_req_seq_id = dreq_seq_id_latch;
    assign tx_delay_req_ts     = {corr_sec, corr_ns};

    // =========================================================================
    // 8. DELAY_RESPONSE CAPTURE (T4)
    //    Match Delay_Resp to outstanding Delay_Req by sequenceId + portId
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t4_valid     <= 1'b0;
            t4_timestamp <= '0;
        end else begin
            if (rx_ptp_valid && rx_msg_type == MSG_DELAY_RESP &&
                rx_domain_num == cfg_domain_num &&
                rx_seq_id == dreq_seq_id_latch &&
                rx_req_port_id == CLOCK_ID) begin
                t4_timestamp <= rx_recv_ts;
                t4_valid     <= 1'b1;
            end
        end
    end

    // =========================================================================
    // 9. MEASUREMENT ENGINE
    //    Computes offset_from_master and mean_path_delay when T1–T4 all valid
    //    IEEE 1588 equations:
    //      offset = ((T2-T1) - (T4-T3)) / 2
    //      MPD    = ((T2-T1) + (T4-T3)) / 2
    // =========================================================================

    // Convert 80-bit PTP timestamps {sec[47:0], ns[31:0]} → 64-bit nanosecond
    function automatic logic signed [63:0] ts_to_ns (input logic [79:0] ts);
        logic [SEC_W-1:0] secs;
        logic [NS_W-1:0]  nsec;
        secs = ts[79:32];
        nsec = ts[31:0];
        // Clamp seconds to avoid overflow: use only lower 32 bits of seconds
        return $signed({32'b0, secs[31:0]}) * $signed(64'd1_000_000_000) +
               $signed({32'b0, nsec});
    endfunction

    logic signed [63:0] t1_ns, t2_ns, t3_ns, t4_ns;
    logic               all_ts_valid;

    assign t1_ns = ts_to_ns(t1_timestamp);
    assign t2_ns = ts_to_ns(t2_timestamp);
    assign t3_ns = ts_to_ns(t3_timestamp);
    assign t4_ns = ts_to_ns(t4_timestamp);
    assign all_ts_valid = t1_valid & t2_valid & t3_valid & t4_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t2_minus_t1   <= '0;
            t4_minus_t3   <= '0;
            offset_calc   <= '0;
            mpd_calc      <= '0;
            measure_valid <= 1'b0;
        end else begin
            measure_valid <= 1'b0;
            if (all_ts_valid) begin
                t2_minus_t1 <= t2_ns - t1_ns;
                t4_minus_t3 <= t4_ns - t3_ns;
                offset_calc <= ((t2_ns - t1_ns) - (t4_ns - t3_ns)) >>> 1;
                mpd_calc    <= ((t2_ns - t1_ns) + (t4_ns - t3_ns)) >>> 1;
                measure_valid <= 1'b1;
                // Invalidate T3/T4 after consumption
            end
        end
    end

    assign offset_from_master = offset_calc[31:0];
    assign mean_path_delay    = mpd_calc[31:0];

    // Threshold alarms
    assign ofm_threshold_alarm = (measure_valid &&
                                  ($signed(offset_calc[31:0]) > $signed(32'(OFM_THR_NS)) ||
                                   $signed(offset_calc[31:0]) < -$signed(32'(OFM_THR_NS))));
    assign mpd_threshold_alarm = (measure_valid &&
                                  (mpd_calc[31:0] > 32'(MPD_THR_NS)));

    // =========================================================================
    // 10. SERVO CONTROLLER (PI)
    //     Adjusts the corrected clock frequency and phase to minimize OFM
    //     P-term: proportional to current offset
    //     I-term: accumulated integral to correct frequency error
    // =========================================================================
    logic signed [SERVO_ACC_W-1:0] new_integrator;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            servo_integrator <= '0;
            servo_p_term     <= '0;
            servo_i_term     <= '0;
            servo_correction <= '0;
            servo_update     <= 1'b0;
            clock_locked     <= 1'b0;
        end else begin
            servo_update <= 1'b0;

            if (measure_valid && port_state == STATE_SLAVE) begin
                // P term: offset >> KP_SHIFT
                servo_p_term <= $signed(offset_calc[31:0]) >>> KP_SHIFT;

                // I term: accumulate and clip
                //logic signed [SERVO_ACC_W-1:0] new_integrator;
                new_integrator = servo_integrator + $signed(offset_calc[31:0]);
                // Anti-windup clamp: ±10ms in nanoseconds
                if (new_integrator > $signed(SERVO_ACC_W'(10_000_000)))
                    servo_integrator <= $signed(SERVO_ACC_W'(10_000_000));
                else if (new_integrator < -$signed(SERVO_ACC_W'(10_000_000)))
                    servo_integrator <= -$signed(SERVO_ACC_W'(10_000_000));
                else
                    servo_integrator <= new_integrator;

                servo_i_term <= $signed(servo_integrator[31:0]) >>> KI_SHIFT;

                // Total correction: negate (subtract offset to correct)
                servo_correction <= -(servo_p_term + servo_i_term);
                servo_update     <= 1'b1;

                // Lock detection: |OFM| < 1 µs for 3 consecutive measurements
                // (simplified: single-sample check for lock indication)
                clock_locked <= ($signed(offset_calc[31:0]) < $signed(32'd1000)) &&
                                ($signed(offset_calc[31:0]) > -$signed(32'd1000));
            end

            if (port_state == STATE_LISTENING || port_state == STATE_INIT) begin
                servo_integrator <= '0;
                clock_locked     <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
// ============================================================================
// End of ptp_slave_sync_top.sv
// ============================================================================
