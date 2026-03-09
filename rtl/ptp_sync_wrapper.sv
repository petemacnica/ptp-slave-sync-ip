// ============================================================================
// Macnica Americas
// Module       : ptp_sync_wrapper
// Description  : Integration wrapper for the complete IEEE 1588 PTP Slave
//                Clock Synchronization IP. Connects:
//                  - ptp_msg_parser  (RX frame → decoded PTP fields + HW TS)
//                  - ptp_slave_sync_top (BMCA, port FSM, timestamp engine,
//                                        servo control, corrected clock)
//                  - ptp_delay_req_framer (Delay_Req TX frame builder)
//
//                This module is the single entity to instantiate in a ProAV
//                encoder or decoder SoC, connected to the Ethernet MAC.
//
// Usage        :
//   - Connect rx_tdata/rx_tvalid/rx_tlast/rx_tuser from MAC RX path
//   - Connect tx_tdata/tx_tvalid/tx_tlast/tx_tready to MAC TX path
//   - Connect phy_rx_timestamp and phy_tx_timestamp from PHY timestamping
//   - Drive cfg_ signals from a register file
//   - Read local_time_sec/ns, pps_out, clock_locked, offset_from_master
// 
// Author       : Peter Mbua (Plano, TX)
// Revision     : 1.0  (2026)
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module ptp_sync_wrapper #(
    parameter int unsigned CLK_FREQ_HZ  = 125_000_000,
    parameter logic [47:0] SRC_MAC      = 48'hDE_AD_BE_EF_00_01,
    parameter logic [63:0] CLOCK_ID     = 64'hDEAD_BEEF_CAFE_0001,
    parameter int unsigned AMT_DEPTH    = 8,
    parameter int unsigned PTP_DOMAIN   = 127
) (
    // System
    input  wire                clk,
    input  wire                rst_n,

    // MAC RX (AXI4-Stream)
    input  wire [7:0]          rx_tdata,
    input  wire logic          rx_tvalid,
    input  wire                rx_tlast,
    input  wire                rx_tuser,      // SFD pulse → HW RX timestamp

    // PHY Hardware Timestamps
    input  wire [79:0]         phy_rx_timestamp,
    input  wire [79:0]         phy_tx_timestamp,
    input  wire                phy_tx_ts_valid,

    // MAC TX (AXI4-Stream)
    output logic [7:0]          tx_tdata,
    output logic                tx_tvalid,
    output logic                tx_tlast,
    input  wire                 tx_tready,

    // Recovered Clock Outputs
    output logic [47:0]         local_time_sec,
    output logic [31:0]         local_time_ns,
    output logic                pps_out,
    output logic                clock_locked,

    // Servo Diagnostics
    output logic signed [31:0]  offset_from_master,
    output logic [31:0]         mean_path_delay,
    output logic                ofm_threshold_alarm,
    output logic                mpd_threshold_alarm,

    // PTP Status
    output logic [2:0]          ptp_port_state,
    output logic                grandmaster_changed,
    output logic [63:0]         current_gm_id,
    output logic                amt_violation,
    output logic                forced_master_event,

    // Configuration (from register file)
    input  wire                cfg_slave_only,
    input  wire                cfg_forced_master,
    input  wire [7:0]          cfg_domain_num,
    input  reg  [AMT_DEPTH-1:0][63:0] cfg_amt_table,
    input  wire [AMT_DEPTH-1:0]       cfg_amt_valid
);

    // =========================================================================
    // Internal connections
    // =========================================================================

    // Parser → Sync Top
    logic         parser_rx_valid;
    logic [3:0]   parser_msg_type;
    logic [7:0]   parser_domain_num;
    logic [63:0]  parser_src_port_id;
    logic [15:0]  parser_seq_id;
    logic [63:0]  parser_correction_field;
    logic [79:0]  parser_origin_ts;
    logic [79:0]  parser_hw_timestamp;
    logic [79:0]  parser_precise_origin_ts;
    logic [79:0]  parser_recv_ts;
    logic [63:0]  parser_req_port_id;
    logic         parser_announce_valid;
    logic [7:0]   parser_ann_prio1;
    logic [7:0]   parser_ann_clk_class;
    logic [7:0]   parser_ann_clk_acc;
    logic [15:0]  parser_ann_clk_var;
    logic [7:0]   parser_ann_prio2;
    logic [63:0]  parser_ann_gm_id;
    logic [15:0]  parser_ann_steps;
    logic [7:0]   parser_ann_ts;

    // Sync Top → Framer
    logic         dreq_valid;
    logic [15:0]  dreq_seq_id;
    logic [79:0]  dreq_ts;
    logic         dreq_ack;
    logic [79:0]  dreq_hw_ts;

    // =========================================================================
    // Submodule instantiations
    // =========================================================================

    ptp_msg_parser #(
        .DATA_W     (8),
        .TIMESTAMP_W(80)
    ) u_parser (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .rx_tdata                   (rx_tdata),
        .rx_tvalid                  (rx_tvalid),
        .rx_tlast                   (rx_tlast),
        .rx_tuser                   (rx_tuser),
        .phy_rx_timestamp           (phy_rx_timestamp),
        .ptp_rx_valid               (parser_rx_valid),
        .ptp_msg_type               (parser_msg_type),
        .ptp_domain_num             (parser_domain_num),
        .ptp_src_port_id            (parser_src_port_id),
        .ptp_seq_id                 (parser_seq_id),
        .ptp_correction_field       (parser_correction_field),
        .ptp_origin_ts              (parser_origin_ts),
        .ptp_hw_timestamp           (parser_hw_timestamp),
        .ptp_precise_origin_ts      (parser_precise_origin_ts),
        .ptp_recv_ts                (parser_recv_ts),
        .ptp_req_port_id            (parser_req_port_id),
        .ptp_announce_valid         (parser_announce_valid),
        .ptp_announce_priority1     (parser_ann_prio1),
        .ptp_announce_clock_class   (parser_ann_clk_class),
        .ptp_announce_clock_accuracy(parser_ann_clk_acc),
        .ptp_announce_clock_variance(parser_ann_clk_var),
        .ptp_announce_priority2     (parser_ann_prio2),
        .ptp_announce_gm_id         (parser_ann_gm_id),
        .ptp_announce_steps_removed (parser_ann_steps),
        .ptp_announce_time_source   (parser_ann_ts)
    );

    ptp_slave_sync_top #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .PTP_DOMAIN_NUM (PTP_DOMAIN),
        .CLOCK_ID       (CLOCK_ID),
        .AMT_DEPTH      (AMT_DEPTH)
    ) u_sync (
        .clk                        (clk),
        .rst_n                      (rst_n),
        // RX from parser
        .rx_ptp_valid               (parser_rx_valid),
        .rx_msg_type                (parser_msg_type),
        .rx_domain_num              (parser_domain_num),
        .rx_src_port_id             (parser_src_port_id),
        .rx_seq_id                  (parser_seq_id),
        .rx_correction_field        (parser_correction_field),
        .rx_origin_ts               (parser_origin_ts),
        .rx_hw_timestamp            (parser_hw_timestamp),
        .rx_precise_origin_ts       (parser_precise_origin_ts),
        .rx_recv_ts                 (parser_recv_ts),
        .rx_req_port_id             (parser_req_port_id),
        // TX Delay_Req
        .tx_delay_req_valid         (dreq_valid),
        .tx_delay_req_seq_id        (dreq_seq_id),
        .tx_delay_req_ts            (dreq_ts),
        .tx_delay_req_ack           (dreq_ack),
        .tx_hw_timestamp            (dreq_hw_ts),
        // Announce
        .rx_announce_valid          (parser_announce_valid),
        .rx_announce_priority1      (parser_ann_prio1),
        .rx_announce_clock_class    (parser_ann_clk_class),
        .rx_announce_clock_accuracy (parser_ann_clk_acc),
        .rx_announce_clock_variance (parser_ann_clk_var),
        .rx_announce_priority2      (parser_ann_prio2),
        .rx_announce_gm_id          (parser_ann_gm_id),
        .rx_announce_steps_removed  (parser_ann_steps),
        .rx_announce_time_source    (parser_ann_ts),
        // Clock outputs
        .local_time_sec             (local_time_sec),
        .local_time_ns              (local_time_ns),
        .pps_out                    (pps_out),
        .clock_locked               (clock_locked),
        // Servo
        .offset_from_master         (offset_from_master),
        .mean_path_delay            (mean_path_delay),
        .ofm_threshold_alarm        (ofm_threshold_alarm),
        .mpd_threshold_alarm        (mpd_threshold_alarm),
        // BMCA / state
        .ptp_port_state             (ptp_port_state),
        .grandmaster_changed        (grandmaster_changed),
        .current_gm_id              (current_gm_id),
        // Config
        .cfg_slave_only             (cfg_slave_only),
        .cfg_forced_master          (cfg_forced_master),
        .cfg_domain_num             (cfg_domain_num),
        .cfg_amt_table              (cfg_amt_table),
        .cfg_amt_valid              (cfg_amt_valid),
        .amt_violation              (amt_violation),
        .forced_master_event        (forced_master_event)
    );

    ptp_delay_req_framer #(
        .SRC_MAC    (SRC_MAC),
        .CLOCK_ID   (CLOCK_ID)
    ) u_framer (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .tx_delay_req_valid     (dreq_valid),
        .tx_delay_req_seq_id    (dreq_seq_id),
        .tx_delay_req_ts        (dreq_ts),
        .tx_tdata               (tx_tdata),
        .tx_tvalid              (tx_tvalid),
        .tx_tlast               (tx_tlast),
        .tx_tready              (tx_tready),
        .tx_ack                 (dreq_ack),
        .tx_hw_timestamp        (dreq_hw_ts),
        .phy_tx_timestamp       (phy_tx_timestamp),
        .phy_tx_ts_valid        (phy_tx_ts_valid)
    );

endmodule

`default_nettype wire
// ============================================================================
// End of ptp_sync_wrapper.sv
// ============================================================================
