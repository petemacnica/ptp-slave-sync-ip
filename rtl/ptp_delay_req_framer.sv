// ============================================================================
// Module       : ptp_delay_req_framer
// Description  : Generates IEEE 1588 Delay_Request PTP frames for transmission.
//                Frames a minimal PTP Delay_Req message over Ethernet_II
//                (EtherType 0x88F7), inserts originTimestamp and drives
//                AXI4-Stream TX toward the MAC.
//
//                On tx_delay_req_ack, the module drives tx_hw_timestamp with
//                the PHY-captured TX hardware timestamp (T3).
//
// Transport    : Ethernet_II (0x88F7), multicast PTP dest 01:1B:19:00:00:00
// Tx trigger   : tx_delay_req_valid strobe from ptp_slave_sync_top
// ============================================================================

`timescale 1ns / 1ps
//`default_nettype none

module ptp_delay_req_framer #(
    parameter logic [47:0] SRC_MAC     = 48'hDE_AD_BE_EF_00_01,
    parameter logic [47:0] PTP_MCAST   = 48'h01_1B_19_00_00_00, // PTP multicast DA
    parameter logic [63:0] CLOCK_ID    = 64'hDEAD_BEEF_CAFE_0001,
    parameter logic [15:0] PORT_NUM    = 16'd1,
    parameter int unsigned TIMESTAMP_W = 80
) (
    input  logic                    clk,
    input  logic                    rst_n,

    // Trigger from sync top
    input  logic                    tx_delay_req_valid,
    input  logic [15:0]             tx_delay_req_seq_id,
    input  logic [79:0]             tx_delay_req_ts,    // originTimestamp (T3 approx)

    // AXI4-Stream TX to MAC
    output logic [7:0]              tx_tdata,
    output logic                    tx_tvalid,
    output logic                    tx_tlast,
    input  logic                    tx_tready,

    // Handshake back to sync top
    output logic                    tx_ack,          // Accepted: frame queued
    output logic [TIMESTAMP_W-1:0]  tx_hw_timestamp, // PHY HW TX timestamp (T3)

    // PHY HW TX timestamp input
    input  logic [TIMESTAMP_W-1:0]  phy_tx_timestamp,
    input  logic                    phy_tx_ts_valid   // PHY signals TS is ready
);

    // =========================================================================
    // PTP Delay_Req frame layout (Ethernet II, no VLAN)
    // =========================================================================
    // Bytes  0– 5: DA (PTP multicast)
    // Bytes  6–11: SA (src MAC)
    // Bytes 12–13: EtherType 0x88F7
    // Bytes 14–14: transportSpecific[7:4]=0, messageType[3:0]=0x1 (Delay_Req)
    // Bytes 15–15: reserved[7:4]=0, versionPTP=0x2
    // Bytes 16–17: messageLength = 44
    // Byte  18:    domainNumber (127 = 0x7F)
    // Byte  19:    reserved
    // Bytes 20–21: flags = 0x0200 (twoStepFlag)
    // Bytes 22–29: correctionField = 0
    // Bytes 30–33: reserved
    // Bytes 34–43: sourcePortIdentity (clockId[8] + portNum[2])
    // Bytes 44–45: sequenceId
    // Byte  46:    controlField = 0x01 (Delay_Req)
    // Byte  47:    logMessageInterval = 0x7F
    // Bytes 48–57: originTimestamp (seconds[6] + nanoseconds[4])
    // Total: 58 bytes

    localparam int FRAME_LEN = 58;

    typedef enum logic [1:0] {
        S_IDLE,
        S_SEND,
        S_WAIT_TS
    } state_t;

    state_t             state;
    logic [7:0]         frame_buf [0:FRAME_LEN-1];
    logic [6:0]         byte_idx;
    logic               frame_ready;

    // =========================================================================
    // Build frame into buffer
    // =========================================================================
    task build_frame(input logic [15:0] seq_id, input logic [79:0] origin_ts);
        // DA
        frame_buf[0]  = PTP_MCAST[47:40];
        frame_buf[1]  = PTP_MCAST[39:32];
        frame_buf[2]  = PTP_MCAST[31:24];
        frame_buf[3]  = PTP_MCAST[23:16];
        frame_buf[4]  = PTP_MCAST[15:8];
        frame_buf[5]  = PTP_MCAST[7:0];
        // SA
        frame_buf[6]  = SRC_MAC[47:40];
        frame_buf[7]  = SRC_MAC[39:32];
        frame_buf[8]  = SRC_MAC[31:24];
        frame_buf[9]  = SRC_MAC[23:16];
        frame_buf[10] = SRC_MAC[15:8];
        frame_buf[11] = SRC_MAC[7:0];
        // EtherType
        frame_buf[12] = 8'h88;
        frame_buf[13] = 8'hF7;
        // messageType = Delay_Req (0x01)
        frame_buf[14] = 8'h01;
        // versionPTP = 2
        frame_buf[15] = 8'h02;
        // messageLength = 44 (PTP payload only, not including Eth header)
        frame_buf[16] = 8'h00;
        frame_buf[17] = 8'h2C;
        // domainNumber = 0x7F (127 = SMPTE ST 2059-2)
        frame_buf[18] = 8'h7F;
        frame_buf[19] = 8'h00;
        // flags: twoStepFlag
        frame_buf[20] = 8'h02;
        frame_buf[21] = 8'h00;
        // correctionField = 0
        for (int i = 22; i <= 29; i++) frame_buf[i] = 8'h00;
        // reserved
        for (int i = 30; i <= 33; i++) frame_buf[i] = 8'h00;
        // sourcePortIdentity: clockId (8 bytes) + portNum (2 bytes)
        frame_buf[34] = CLOCK_ID[63:56];
        frame_buf[35] = CLOCK_ID[55:48];
        frame_buf[36] = CLOCK_ID[47:40];
        frame_buf[37] = CLOCK_ID[39:32];
        frame_buf[38] = CLOCK_ID[31:24];
        frame_buf[39] = CLOCK_ID[23:16];
        frame_buf[40] = CLOCK_ID[15:8];
        frame_buf[41] = CLOCK_ID[7:0];
        frame_buf[42] = PORT_NUM[15:8];
        frame_buf[43] = PORT_NUM[7:0];
        // sequenceId
        frame_buf[44] = seq_id[15:8];
        frame_buf[45] = seq_id[7:0];
        // controlField = 0x01
        frame_buf[46] = 8'h01;
        // logMessageInterval = 0x7F (unspecified)
        frame_buf[47] = 8'h7F;
        // originTimestamp: 6 bytes seconds + 4 bytes nanoseconds
        frame_buf[48] = origin_ts[79:72];
        frame_buf[49] = origin_ts[71:64];
        frame_buf[50] = origin_ts[63:56];
        frame_buf[51] = origin_ts[55:48];
        frame_buf[52] = origin_ts[47:40];
        frame_buf[53] = origin_ts[39:32];
        frame_buf[54] = origin_ts[31:24];
        frame_buf[55] = origin_ts[23:16];
        frame_buf[56] = origin_ts[15:8];
        frame_buf[57] = origin_ts[7:0];
    endtask

    // =========================================================================
    // FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            tx_tvalid      <= 1'b0;
            tx_tlast       <= 1'b0;
            tx_tdata       <= '0;
            tx_ack         <= 1'b0;
            tx_hw_timestamp<= '0;
            byte_idx       <= '0;
            frame_ready    <= 1'b0;
        end else begin
            tx_ack <= 1'b0;

            case (state)
                S_IDLE: begin
                    tx_tvalid <= 1'b0;
                    tx_tlast  <= 1'b0;
                    if (tx_delay_req_valid) begin
                        build_frame(tx_delay_req_seq_id, tx_delay_req_ts);
                        byte_idx    <= '0;
                        frame_ready <= 1'b1;
                        tx_ack      <= 1'b1;
                        state       <= S_SEND;
                    end
                end

                S_SEND: begin
                    if (tx_tready || !tx_tvalid) begin
                        tx_tdata  <= frame_buf[byte_idx];
                        tx_tvalid <= 1'b1;
                        tx_tlast  <= (byte_idx == FRAME_LEN - 1);

                        if (byte_idx == FRAME_LEN - 1) begin
                            byte_idx    <= '0;
                            frame_ready <= 1'b0;
                            state       <= S_WAIT_TS;
                        end else begin
                            byte_idx <= byte_idx + 1'b1;
                        end
                    end
                end

                S_WAIT_TS: begin
                    tx_tvalid <= 1'b0;
                    tx_tlast  <= 1'b0;
                    // Wait for PHY to provide hardware TX timestamp
                    if (phy_tx_ts_valid) begin
                        tx_hw_timestamp <= phy_tx_timestamp;
                        state           <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
// ============================================================================
// End of ptp_delay_req_framer.sv
// ============================================================================
