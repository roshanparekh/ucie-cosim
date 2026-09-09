// cosim_monitor.sv
//
// Monitor to see sideband messages sent between the DUTs. Logs LTSM state
// changes through training, RDI and FDI bring-up, and the flits that get sent
// once the link is up.
//
// Signal-level probes are compiled out by default. Build with
// +define+COSIM_VERBOSE for the framing, deserializer and pattern detail.

module cosim_monitor (
    input wire        clk,
    input wire        rst_n,

    // madsim
    input wire [4:0]  mad_ltsm,
    input wire [3:0]  mad_rdi_state,
    input wire [3:0]  mad_fdi_state,
    input wire [511:0] mad_fdi_lp_data,
    input wire         mad_fdi_lp_valid,
    input wire         mad_fdi_pl_trdy,
    input wire [511:0] mad_fdi_pl_data,
    input wire         mad_fdi_pl_valid,
    input wire         mad_rdi_lp_valid,
    input wire         mad_rdi_pl_valid,
    // Where a TX flit stops inside UcieTL's adapter.
    input wire         ucie_fdi_pl_trdy,
    // The mainband receive framing check, and the two latches it sets.
    input wire         ucie_mb_rx_word_valid,
    input wire [31:0]  ucie_mb_rx_valid_word,
    input wire [31:0]  ucie_mb_rx_data_lane0,
    // Lane 0 before descrambling, and the LFSR word XORed into it. Both dies
    // run the same polynomial and seeds, so a bad payload is a phase question.
    input wire [31:0]  ucie_mb_rx_raw_lane0,
    input wire [31:0]  ucie_mb_descrambler_lane0,
    input wire         ucie_rx_aligned,
    input wire [4:0]   ucie_rx_align_offset,
    input wire [15:0]  ucie_rx_rearms,
    input wire [15:0]  ucie_rx_offset_changes,
    input wire [31:0]  ucie_mb_rx_raw_valid,
    input wire         ucie_mb_rx_raw_word_valid,
    input wire         ucie_mb_sticky_error,
    input wire [1:0]   ucie_stall_req_state,
    input wire        mad_trainerror,
    input wire        mad_inband_pres,

    // UcieTL link training
    input wire [3:0]  ucie_ltstate,        // LTState: RESET/SBINIT/MBINIT/...
    input wire [4:0]  ucie_ltsm_detail,    // LTSMState, the finer substate
    input wire [1:0]  ucie_sb_pat_cnt,     // needs to reach 2
    input wire        ucie_reset_min_wait,
    input wire [22:0] ucie_timeout_cnt,
    input wire        ucie_sb_rx_valid,
    input wire [63:0] ucie_sb_rx_word,
    input wire [3:0]  ucie_rdi_state,
    input wire [2:0]   adp_init_state,
    input wire         adp_cap_snt,
    input wire         adp_cap_rcv,
    input wire         adp_req_rcv,
    input wire         adp_rsp_rcv,
    input wire         adp_req_snt,
    input wire         adp_rsp_snt,
    // Adapter-level sideband messages. The only view of the adapter-to-adapter
    // exchange, since ucie_sb_rx_* taps the LTSM port, which never sees them.
    input wire [5:0]   adp_sb_rcv,
    input wire [5:0]   adp_sb_snt,
    input wire         adp_sb_rdy,
    input wire [3:0]   ucie_fdi_sts,
    input wire         ucie_fdi_tx_valid,
    input wire [511:0] ucie_fdi_tx_data,
    input wire         ucie_fdi_rx_valid,
    input wire [511:0] ucie_fdi_rx_data,
    input wire         ucie_rdi_tx_valid,
    input wire         ucie_rdi_rx_valid,
    input wire [511:0] ucie_rdi_rx_data,

    // bump activity
    input wire        a2b_sb_val,
    input wire        a2b_sb_data,
    input wire        ucie_sb_rx_clk,
    input wire        ucie_sb_rx_data_i,

    // Deserializer bit assembler, in the recovered forwarded clock domain.
    input wire [6:0]  des_bit_cnt,
    input wire        des_enq_valid,
    input wire        des_enq_ready,

    // Deserializer output, after the domain crossing.
    input wire        des_raw_valid,
    input wire [63:0] des_raw_word,

    // Link node output, after the parity check and the opcode priority queue.
    input wire        des_out_valid,
    input wire [63:0] des_out_word,
    input wire        b2a_sb_val,
    input wire        b2a_sb_data,

    // Mainband. First needed at MBINIT.REPAIRCLK.
    // madsim's mainband pattern detector, through the wrapper's ports.
    input wire [2:0]  mad_mb_clock_pass,
    input wire        mad_mb_rx_done,
    input wire        mad_mb_rx_error,
    input wire        mad_mb_rx_enable,
    input wire        mad_mb_tx_enable,
    input wire [1:0]  mad_mb_pattern_select,
    // madsim's sideband transmit arbitration. DriveTx grants ready by strict
    // priority (RDI, LTSM, CFG), so a persistent rdi_tx_valid starves the LTSM.
    input wire [63:0] mad_sb_ltsm_tx_msg,
    input wire        mad_sb_ltsm_tx_valid,
    input wire        mad_sb_ltsm_tx_ready,
    input wire [15:0] mad_mb_rx_chunk_ckp,
    input wire [15:0] mad_mb_rx_chunk_trk,
    input wire        mad_mb_rx_chunk_valid,
    input wire [15:0] mad_mb_rx_chunk_ckn,
    input wire [15:0] mad_mb_rx_chunk_vld,

    // The word ucieDigital hands the PHY, and what the PHY sees after the
    // controllerSel mux and crossing FIFO. Last point before the serializer.
    // UcieTL's own mainband pattern detection, reported as msgInfo of
    // {REPAIRCLK result resp}.
    // PatternReader internals. A lane passes when patternCounterReg reaches
    // errorThresholdReg (16) with iterDirtyReg clean.
    input wire [1:0]  rxpt_req_state,
    input wire [1:0]  rxpt_rsp_state,
    input wire [1:0]  pr_state,      // 0=sIdle 1=sDetect 2=sResult
    input wire        pr_rx_valid,   // per-word arrival handshake from the AFE
    input wire [15:0] pr_cnt0,
    input wire [15:0] pr_cnt1,
    input wire [15:0] pr_cnt2,
    input wire        pr_dirty2,
    input wire [15:0] pr_target,
    input wire        pr_consec,
    input wire [2:0]  pr_ptype,
    input wire        pr_counter_en,
    input wire        pr_resp_valid,
    input wire        pr_aggregate,
    input wire        pr_lane0,
    input wire        pr_lane1,
    input wire        pr_lane2,
    input wire        ucie_mb_rx_valid,
    input wire [31:0] ucie_mb_rx_trk,
    // The valid lane's deserialized word. REPAIRVAL trains it with an 8 UI
    // pattern, so the reference is a fixed 0x0F0F0F0F.
    input wire [31:0] ucie_mb_rx_vld,
    // Data lanes 0 and 1. REVERSALMB sends 1010 <laneId> 1010, so lane 0 reads
    // 0xa00a repeated and lane 1 0xa01a, up to word framing.
    input wire [31:0] ucie_mb_rx_d0,
    input wire [31:0] ucie_mb_rx_d1,

    input wire [31:0] rx_lfsr_ref0,
    input wire [31:0] rx_lane_d0,
    input wire [1:0]  txpt_req_ptype,
    input wire [1:0]  txpt_req_state,
    input wire [1:0]  mbt_txpt_ptype,
    input wire        mbt_txpt_start,
    input wire [7:0]  pw_cycle_count,
    input wire        pw_in_progress,
    input wire [1:0]  pw_pattern_type,
    input wire        dig_mb_tx_valid,
    input wire [31:0] dig_mb_tx_clkp,
    input wire [31:0] dig_mb_tx_clkn,
    input wire [31:0] dig_mb_tx_trk,
    input wire [31:0] dig_mb_tx_data0,
    input wire        phy_mb_tx_valid,
    input wire [31:0] phy_mb_tx_clkp,

    input wire [15:0] a2b_mb_data,
    input wire        a2b_mb_vld,
    input wire        a2b_mb_ckp,
    input wire        a2b_mb_ckn,
    input wire        a2b_mb_trk,
    input wire [15:0] b2a_mb_data,
    input wire        b2a_mb_vld,
    input wire        b2a_mb_ckp,
    input wire        b2a_mb_ckn,
    input wire        b2a_mb_trk,

    // UcieTL's SBINIT sub-FSM, since the outer ltState only says "SBINIT".
    // SBInitStateRequester is sPATTERN / sOUT_OF_RESET / sSBINIT_DONE_MSG.
    input wire [6:0]  des_max_bits,
    input wire        des_rxmode,
    input wire        des_reframe,
    input wire [2:0]  des_quiet,
    input wire        des_reset,
    input wire        des_fwclk,
    // MBINIT. Requester and responder are separate state machines.
    input wire [2:0]  mbi_req_state,
    input wire [2:0]  mbi_rsp_state,
    // Substate within a state. REPAIRCLK runs INIT(0), RESULT(1), DONE(2) as
    // three separate sideband exchanges, so the state alone cannot say which
    // of them is stuck.
    input wire [2:0]  mbi_req_sub,
    input wire [2:0]  mbi_rsp_sub,
    input wire        mbi_cal_done_reg,
    input wire        mbi_cal_done_in,
    input wire        plt_selfcal_start,
    input wire        plt_selfcal_done,
    input wire [1:0]  sbi_state,
    input wire [1:0]  sbi_detect_cnt,   // 2 means the partner's pattern was seen
    input wire [1:0]  sbi_four_cnt,     // 3 advances out of sPATTERN
    input wire        sbi_req_msg_sent,
    input wire        sbi_req_msg_recv,
    input wire        sbi_rsp_msg_sent,
    input wire        sbi_rsp_msg_recv,

    // Transmit handshake chain, requester outward. fourPatternCounter only
    // advances when the requester sees tx.ready, so these say who withholds it.
    input wire        req_tx_valid,
    input wire        req_tx_ready,
    input wire [63:0] req_tx_data,
    input wire        ln_tx_valid,
    input wire        ln_tx_ready,
    input wire [63:0] ln_tx_data,
    input wire        ser_in_valid,
    input wire        ser_in_ready,
    input wire [63:0] ser_in_data,

    // The rest of the chain, LTSM outward, through arbiters, skid buffers and the
    // switch. Any of them can withhold ready, and they fail differently.
    input wire        lt_tx_valid,
    input wire        lt_tx_ready,
    input wire [63:0] lt_tx_data,
    input wire [3:0]  lt_arb_chosen,     // which of the 12 clients is granted
    input wire        lib_bypass,        // layerInBuffer in pass-through
    input wire [63:0] lib_data,          // what it is holding when it is not
    input wire        sw_curr_valid,
    input wire        sw_curr_ready,
    input wire [63:0] sw_curr_data,
    input wire        sw_upper_valid,
    input wire        sw_lower_valid,
    input wire        lnskid_bypass,

    // Where an X enters. The switch routes on the message header, so an X there
    // strands every producer behind it. These follow the header to its source.
    input wire        rdic_tx_valid,
    input wire [63:0] rdic_tx_data,
    input wire        stxa_out_valid,
    input wire [63:0] stxa_out_data,
    input wire        stxq_deq_valid,
    input wire [63:0] stxq_deq_data,
    input wire        lyr_in_valid,
    input wire        lyr_in_ready,
    input wire [63:0] lyr_in_data,

    // The RDI side of the switch. An X on this valid poisons the switch's
    // lower-layer arbiter, which is what makes currLayer.from.ready X.
    input wire        rdin_rxout_valid,
    input wire        rdi_in_valid,
    input wire        lib_out_valid
);

  localparam longint unsigned SB_INIT_PATTERN = 64'h5555_5555_5555_5555;

  integer fh;
  longint unsigned cyc = 0;

  // Previous values, for edge detection.
  logic [4:0]  p_mad_ltsm;
  logic [3:0]  p_mad_rdi, p_ucie_ltstate, p_ucie_rdi;
  logic [4:0]  p_ucie_detail;
  logic [1:0]  p_sb_pat;
  logic        p_reset_min_wait, p_mad_trainerror;

  // Activity counters, so a quiet log still says whether anything moved.
  longint unsigned n_a2b_sb_bits  = 0;   // madsim sideband bits transmitted
  longint unsigned n_ucie_sb_words = 0;  // 64-bit words UcieTL's deser accepted
  longint unsigned n_ucie_sb_match = 0;  // of those, ones equal to the pattern
  longint unsigned n_b2a_sb_bits  = 0;   // UcieTL sideband bits coming back
  longint unsigned n_ucie_sbclk_edges = 0;
  longint unsigned n_des_words = 0;
  longint unsigned n_des_match = 0;
  longint unsigned n_enq_offer = 0;   // words the assembler completed
  longint unsigned n_enq_taken = 0;   // of those, ones the async FIFO accepted
  longint unsigned n_raw_words = 0;   // words out of the crossing
  longint unsigned n_raw_match = 0;

  // What UcieTL sends back, assembled as madsim's receiver frames it.
  // one sample per cycle in which the recovered valid is high, LSB first.
  // Knowing madsim times out is not the same as knowing what it was sent.
  logic [63:0] b2a_word = 64'b0;
  int unsigned b2a_bit_idx = 0;
  longint unsigned n_b2a_words = 0;
  logic [63:0] b2a_last = 64'hFFFF_FFFF_FFFF_FFFF;
  longint unsigned n_b2a_distinct = 0;

  // Mainband activity. Counted as transitions, because a lane that is parked
  // high is indistinguishable from one that is driven high if you only sample
  // the level.
  longint unsigned n_a2b_ck = 0, n_a2b_vld_hi = 0, n_a2b_dat_tog = 0, n_a2b_trk = 0;
  longint unsigned n_b2a_ck = 0, n_b2a_vld_hi = 0, n_b2a_dat_tog = 0, n_b2a_trk = 0;
  logic p_a2b_ckp = 1'b0, p_b2a_ckp = 1'b0, p_a2b_trk = 1'b0, p_b2a_trk = 1'b0;
  // madsim's REPAIRCLK result carries three pass bits, RCKP / RCKN / RTRK
  // (mb_init_repair_clk.h:26), and needs all three. CKN is a lane in its own
  // right, not a derived copy, so it has to be watched separately.
  longint unsigned n_a2b_ckn = 0, n_b2a_ckn = 0;
  logic p_a2b_ckn = 1'b0, p_b2a_ckn = 1'b0;
  // When the mainband clock ran matters as much as whether it ran. REPAIRCLK
  // interleaves sideband handshakes with mainband clock bursts, so a burst on
  // the wrong side of a handshake is missed entirely.
  longint unsigned t_a2b_ck_first = 0, t_a2b_ck_last = 0;
  longint unsigned t_b2a_ck_first = 0, t_b2a_ck_last = 0;
  logic [15:0] p_a2b_dat = 16'h0, p_b2a_dat = 16'h0;

  // SBINIT sub-FSM, previous values for edge detection.
  logic [2:0] p_mbi_req, p_mbi_rsp, p_mbi_rsub, p_mbi_ssub;
  // madsim's detector, previous values for edge detection.
  // Distinct clkp words the digital layer emits, so the log shows the pattern
  // rather than one snapshot.
  logic [31:0] p_dig_clkp = 32'hFFFF_FFFF;
  longint unsigned n_dig_clkp = 0;
  logic [31:0] p_phy_clkp = 32'hFFFF_FFFF;
  longint unsigned n_phy_clkp = 0;

  longint unsigned n_chunk_logged = 0;
  logic       p_chunk_valid = 1'b0;
  longint unsigned n_ucie_mb_rx = 0;
  longint unsigned n_ucie_mb_rx_nz = 0;
  logic mb_rx_seen_nz = 1'b0;
  longint unsigned n_ucie_mb_vld = 0;
  longint unsigned n_ucie_mb_d = 0;
  // Words per mainband transmit burst, and the quiet counter that ends it.
  longint unsigned n_ucie_tx_words = 0;
  longint unsigned n_lfsr_cmp = 0;
  logic [1:0] p_pr_state_lfsr = 2'd0;
  logic [7:0] pw_peak = 8'd0;
  logic       p_pw_busy = 1'b0;
  logic [15:0] ucie_tx_quiet = 16'hffff;
  logic p_ucie_txen_lvl = 1'b0;
  logic p_pr_valid = 1'b0;
  // Exchange ledger - the last message each direction carried, so a stall shows
  // who said what last rather than only which state each side is stuck in.
  logic [63:0]     last_tx_word = 64'h0;
  logic [63:0]     last_rx_word = 64'h0;
  longint unsigned last_tx_cyc = 0, last_rx_cyc = 0;
  longint unsigned n_tx_repeat = 0, n_rx_repeat = 0;
  logic [4:0]      last_tx_ltsm = 5'd0, last_rx_ltsm = 5'd0;

  logic p_mad_txen = 1'b0, p_ucie_txen = 1'b0;
  logic [2:0] p_mad_cp = 3'd0;
  logic       p_mad_rxen = 1'b0, p_mad_rxdone = 1'b0, p_mad_rxerr = 1'b0;
  logic       p_mbi_cdr, p_mbi_cdi, p_plt_start, p_plt_done;
  logic [1:0] p_sbi_state, p_sbi_detect, p_sbi_four;
  logic       p_sbi_rms, p_sbi_rmr, p_sbi_sms, p_sbi_smr;

  // Transmit handshake accounting.
  longint unsigned n_req_v = 0, n_req_r = 0, n_req_fire = 0;
  longint unsigned n_ln_v  = 0, n_ln_r  = 0, n_ln_fire  = 0;
  longint unsigned n_ser_v = 0, n_ser_r = 0, n_ser_fire = 0;
  longint unsigned n_lt_v = 0, n_lt_r = 0, n_lt_fire = 0;
  longint unsigned n_swc_v = 0, n_swc_r = 0, n_swu_v = 0, n_swl_v = 0;
  longint unsigned n_lib_skid = 0, n_lnskid_skid = 0;
  longint unsigned n_rdic_v = 0, n_rdic_x = 0;
  longint unsigned n_stxq_v = 0, n_lyr_v = 0, n_lyr_r = 0;
  logic reported_x = 1'b0;

  // Burst accounting. UcieTL's deserializer never re-syncs on the gap between
  // bursts, so any burst that is not exactly the packet length shifts every
  // later word permanently.
  int unsigned burst_len = 0;
  longint unsigned n_bursts = 0;
  logic p_a2b_val = 1'b0;
  longint unsigned n_reframe = 0;
  logic p_reframe = 1'b0;
  longint unsigned n_des_reset = 0;
  longint unsigned n_des_fwclk = 0;
  logic p_des_fwclk = 1'b0;
  logic reported_xsrc = 1'b0;

  // What madsim puts on the wire, LSB first, one sample per valid cycle. Reads
  // 0x5555555555555555 if the pattern is good and the corruption is in transit.
  logic [63:0] mad_word = 64'b0;
  int unsigned mad_bit_idx = 0;
  logic        mad_word_logged = 1'b0;

  function automatic string ltstate_name(input logic [3:0] s);
    case (s)
      4'd0: return "RESET";      4'd1: return "SBINIT";
      4'd2: return "MBINIT";     4'd3: return "MBTRAIN";
      4'd4: return "LINKINIT";   4'd5: return "ACTIVE";
      4'd6: return "PHYRETRAIN"; 4'd7: return "TRAINERROR";
      4'd8: return "L1_L2";      default: return "?";
    endcase
  endfunction

  // phy_spec.h:656 LtsmState, in full. The abbreviated version this replaces
  // had TRAINERROR at 25, which is PHYRETRAIN; a real TRAINERROR therefore
  // logged as the catch-all "MBINIT/MBTRAIN" and read as ordinary progress.
  function automatic string mad_ltsm_name(input logic [4:0] s);
    case (s)
      5'd0:  return "RESET";
      5'd1:  return "SBINIT";
      5'd2:  return "MBINIT_PARAM";
      5'd3:  return "MBINIT_CAL";
      5'd4:  return "MBINIT_REPAIRCLK";
      5'd5:  return "MBINIT_REPAIRVAL";
      5'd6:  return "MBINIT_REVERSALMB";
      5'd7:  return "MBINIT_REPAIRMB";
      5'd8:  return "MBTRAIN_VALVREF";
      5'd9:  return "MBTRAIN_DATAVREF";
      5'd10: return "MBTRAIN_SPEEDIDLE";
      5'd11: return "MBTRAIN_TXSELFCAL";
      5'd12: return "MBTRAIN_RXCLKCAL";
      5'd13: return "MBTRAIN_VALTRAINCENTER";
      5'd14: return "MBTRAIN_VALTRAINVREF";
      5'd15: return "MBTRAIN_DATATRAINCENTER1";
      5'd16: return "MBTRAIN_DATATRAINVREF";
      5'd17: return "MBTRAIN_RXDESKEW";
      5'd18: return "MBTRAIN_DATATRAINCENTER2";
      5'd19: return "MBTRAIN_LINKSPEED";
      5'd20: return "MBTRAIN_REPAIR";
      5'd21: return "LINKINIT";
      5'd22: return "ACTIVE";
      5'd23: return "L1";
      5'd24: return "L2";
      5'd25: return "PHYRETRAIN";
      5'd26: return "TRAINERROR";
      default: return "?";
    endcase
  endfunction

  function automatic string sbi_name(input logic [1:0] s);
    case (s)
      2'd0: return "sPATTERN";
      2'd1: return "sOUT_OF_RESET";
      2'd2: return "sSBINIT_DONE_MSG";
      default: return "?";
    endcase
  endfunction

  // ==========================================================================
  // Sideband message decoding
  // ==========================================================================
  // Field positions from phy_spec.h: msgcode [21:14], subcode [39:32], msgInfo
  // [55:40], opcode [4:0]. UcieTL packs the same way, so one decoder does both.
  // Subcodes are reused per state, so decoding takes the LTSM state as context.
  function automatic logic [7:0]  sb_msgcode(input logic [63:0] w);
    return w[21:14];
  endfunction
  function automatic logic [7:0]  sb_subcode(input logic [63:0] w);
    return w[39:32];
  endfunction
  function automatic logic [15:0] sb_msginfo(input logic [63:0] w);
    return w[55:40];
  endfunction

  // SBINIT carries its own msgcodes (phy_spec.h:262-266): 0x91 Out Of Reset,
  // which has no request/response pair, and 0x95/0x9A for Done. Everything
  // from MBINIT onwards uses the 0xA5/0xAA pair.
  function automatic bit sb_known(input logic [63:0] w);
    case (sb_msgcode(w))
      8'hA5, 8'hAA, 8'h91, 8'h95, 8'h9A, 8'hE5, 8'hEA,
      8'hB5, 8'hBA, 8'h85, 8'h8A:      return 1'b1;
      default:                           return 1'b0;
    endcase
  endfunction

  function automatic string sb_kind(input logic [63:0] w);
    case (sb_msgcode(w))
      8'hE5, 8'hB5, 8'h85: return "req";
      8'hEA, 8'hBA, 8'h8A: return "resp";
      8'hA5, 8'h95: return "req";
      8'hAA, 8'h9A: return "resp";
      8'h91:        return "";
      default:      return "?";
    endcase
  endfunction

  function automatic string sb_msg_name(input logic [4:0]  ltsm,
                                        input logic [63:0] w);
    logic [7:0] sub;
    sub = sb_subcode(w);
    if (w == SB_INIT_PATTERN) return "SBINIT clock pattern";
    // TrainError entry carries its own msgcodes (phy_spec.h:479-480) and can
    // arrive in any state, so it is named before the state-based decode --
    // which would otherwise read its subcode as whatever that state uses.
    if (sb_msgcode(w) == 8'hE5 || sb_msgcode(w) == 8'hEA)
      return "TRAINERROR.Entry";
    // MBTRAIN reuses one msgcode pair (0xB5 / 0xBA) across every substate and
    // distinguishes them purely by subcode (phy_spec.h:377-450), so decode it
    // from the subcode rather than from the LTSM state.
    if (sb_msgcode(w) == 8'hB5 || sb_msgcode(w) == 8'hBA)
      case (sb_subcode(w))
        8'h00:   return "MBTRAIN.ValVref.Start";
        8'h01:   return "MBTRAIN.ValVref.End";
        8'h02:   return "MBTRAIN.DataVref.Start";
        8'h03:   return "MBTRAIN.DataVref.End";
        8'h04:   return "MBTRAIN.SpeedIdle.Done";
        8'h05:   return "MBTRAIN.TxSelfCal.Done";
        8'h06:   return "MBTRAIN.RxClkCal.Start";
        8'h07:   return "MBTRAIN.RxClkCal.Done";
        8'h08:   return "MBTRAIN.ValTrainCenter.Start";
        8'h09:   return "MBTRAIN.ValTrainCenter.Done";
        8'h0A:   return "MBTRAIN.ValTrainVref.Start";
        8'h0B:   return "MBTRAIN.ValTrainVref.End";
        8'h0C:   return "MBTRAIN.DataTrainCenter1.Start";
        8'h0D:   return "MBTRAIN.DataTrainCenter1.End";
        8'h0E:   return "MBTRAIN.DataTrainVref.Start";
        8'h10:   return "MBTRAIN.DataTrainVref.End";
        8'h11:   return "MBTRAIN.RxDeskew.Start";
        8'h12:   return "MBTRAIN.RxDeskew.End";
        8'h13:   return "MBTRAIN.DataTrainCenter2.Start";
        8'h14:   return "MBTRAIN.DataTrainCenter2.End";
        8'h15:   return "MBTRAIN.LinkSpeed.Start";
        8'h16:   return "MBTRAIN.LinkSpeed.Error";
        default: return $sformatf("MBTRAIN.sub%02h", sb_subcode(w));
      endcase
    // D2C point-test messages, used inside MBINIT.REPAIRMB and MBTRAIN.
    if (sb_msgcode(w) == 8'h85 || sb_msgcode(w) == 8'h8A)
      case (sb_subcode(w))
        8'h01:   return "D2C.TxPointTest.Start";
        8'h02:   return "D2C.LfsrClearError";
        8'h03:   return "D2C.TxResults";
        8'h04:   return "D2C.TxPointTest.End";
        8'h07:   return "D2C.RxPointTest.Start";
        8'h08:   return "D2C.RxTxCount.Done";
        8'h09:   return "D2C.RxPointTest.End";
        default: return $sformatf("D2C.sub%02h", sb_subcode(w));
      endcase
    case (ltsm)
      5'd0, 5'd1: case (sub)                       // RESET / SBINIT
                    8'h00:   return "SBINIT.OutOfReset";
                    8'h01:   return "SBINIT.Done";
                    default: return $sformatf("SBINIT.sub%02h", sub);
                  endcase
      5'd2:       case (sub)                       // MBINIT.PARAM
                    8'h00:   return "MBINIT.PARAM.Config";
                    default: return $sformatf("MBINIT.PARAM.sub%02h", sub);
                  endcase
      5'd3:       case (sub)                       // MBINIT.CAL
                    8'h02:   return "MBINIT.CAL.Done";
                    default: return $sformatf("MBINIT.CAL.sub%02h", sub);
                  endcase
      5'd4:       case (sub)                       // MBINIT.REPAIRCLK
                    8'h03:   return "MBINIT.REPAIRCLK.Init";
                    8'h04:   return "MBINIT.REPAIRCLK.Result";
                    8'h08:   return "MBINIT.REPAIRCLK.Done";
                    default: return $sformatf("MBINIT.REPAIRCLK.sub%02h", sub);
                  endcase
      5'd5:       case (sub)                       // MBINIT.REPAIRVAL
                    8'h09:   return "MBINIT.REPAIRVAL.Init";
                    8'h0A:   return "MBINIT.REPAIRVAL.Result";
                    8'h0C:   return "MBINIT.REPAIRVAL.Done";
                    default: return $sformatf("MBINIT.REPAIRVAL.sub%02h", sub);
                  endcase
      5'd6:       case (sub)                       // MBINIT.REVERSALMB
                    8'h0D:   return "MBINIT.REVERSALMB.Init";
                    8'h0E:   return "MBINIT.REVERSALMB.ClearError";
                    8'h0F:   return "MBINIT.REVERSALMB.Result";
                    8'h10:   return "MBINIT.REVERSALMB.Done";
                    default: return $sformatf("MBINIT.REVERSALMB.sub%02h", sub);
                  endcase
      5'd7:       case (sub)                       // MBINIT.REPAIRMB
                    8'h11:   return "MBINIT.REPAIRMB.Start";
                    8'h13:   return "MBINIT.REPAIRMB.End";
                    8'h14:   return "MBINIT.REPAIRMB.ApplyDegrade";
                    default: return $sformatf("MBINIT.REPAIRMB.sub%02h", sub);
                  endcase
      default:    case (sub)                       // MBTRAIN: unique subcodes
                    8'h00:   return "MBTRAIN.VALVREF.Start";
                    8'h01:   return "MBTRAIN.VALVREF.End";
                    8'h02:   return "MBTRAIN.DATAVREF.Start";
                    8'h03:   return "MBTRAIN.DATAVREF.End";
                    8'h04:   return "MBTRAIN.SPEEDIDLE.Done";
                    8'h05:   return "MBTRAIN.TXSELFCAL.Done";
                    8'h06:   return "MBTRAIN.RXCLKCAL.Start";
                    8'h07:   return "MBTRAIN.RXCLKCAL.Done";
                    8'h08:   return "MBTRAIN.VALTRAINCENTER.Start";
                    8'h09:   return "MBTRAIN.VALTRAINCENTER.Done";
                    8'h0A:   return "MBTRAIN.VALTRAINVREF.Start";
                    8'h0B:   return "MBTRAIN.VALTRAINVREF.End";
                    8'h0E:   return "MBTRAIN.DATATRAINVREF.Start";
                    8'h10:   return "MBTRAIN.DATATRAINVREF.End";
                    8'h11:   return "MBTRAIN.RXDESKEW.Start";
                    8'h12:   return "MBTRAIN.RXDESKEW.End";
                    8'h15:   return "MBTRAIN.LINKSPEED.Start";
                    8'h16:   return "MBTRAIN.LINKSPEED.Error";
                    8'h17:   return "MBTRAIN.LINKSPEED.ExitToRepair";
                    8'h18:   return "MBTRAIN.LINKSPEED.ExitToSpeed";
                    8'h19:   return "MBTRAIN.LINKSPEED.Done";
                    8'h1F:   return "MBTRAIN.LINKSPEED.ExitToPhyRetrain";
                    default: return $sformatf("sub%02h", sub);
                  endcase
    endcase
  endfunction

  // One line per message, in the form the exchange is easiest to read in.
  function automatic string sb_line(input string dir, input logic [4:0] ltsm,
                                    input logic [63:0] w);
    if (w == SB_INIT_PATTERN)
      return $sformatf("%s  %s", dir, "SBINIT clock pattern");
    if (w == 64'h0)
      return $sformatf("%s  <idle, all zero>", dir);
    // Between messages the serializer shifts out whatever is left in its
    // register, so partial words appear on the wire. They carry no valid
    // msgcode; naming them would invent an exchange that never happened.
    if (!sb_known(w))
      return $sformatf("%s  <not a message: raw 0x%016h>", dir, w);
    if (sb_msginfo(w) != 16'h0)
      return $sformatf("%s  %s %s   info=0x%04h", dir,
                       sb_msg_name(ltsm, w), sb_kind(w), sb_msginfo(w));
    if (sb_kind(w) == "")
      return $sformatf("%s  %s", dir, sb_msg_name(ltsm, w));
    return $sformatf("%s  %s %s", dir, sb_msg_name(ltsm, w), sb_kind(w));
  endfunction

  task automatic ev(input string s);
    $fdisplay(fh, "[%12d] %s", cyc, s);
    $fflush(fh);
  endtask

  // Signal-level tracing. At full volume it buries the protocol exchange, so
  // it is off unless built with +define+COSIM_VERBOSE.
`ifdef COSIM_VERBOSE
  localparam bit VERBOSE = 1'b1;
`else
  localparam bit VERBOSE = 1'b0;
`endif

  task automatic evv(input string s);
    if (VERBOSE) begin
      $fdisplay(fh, "[%12d] %s", cyc, s);
      $fflush(fh);
    end
  endtask

  initial begin
    fh = 1;  // stdout, so the events land in run.log with the rest of the run
    $fdisplay(fh, "cycle        event");
    $fdisplay(fh, "------------ -----------------------------------------------");
  end

  always @(posedge clk) begin
    if (!rst_n) begin
      cyc              <= 0;
      p_mad_ltsm       <= 5'd0;
      p_mad_rdi        <= 4'd0;
      p_ucie_ltstate   <= 4'd0;
      p_ucie_detail    <= 5'd0;
      p_ucie_rdi       <= 4'd0;
      p_sb_pat         <= 2'd0;
      p_reset_min_wait <= 1'b0;
      p_mad_trainerror <= 1'b0;
      p_mbi_req        <= 3'd0;
      p_mbi_rsp        <= 3'd0;
      p_mbi_rsub       <= 3'd0;
      p_mbi_ssub       <= 3'd0;
      p_mbi_cdr        <= 1'b0;
      p_mbi_cdi        <= 1'b0;
      p_plt_start      <= 1'b0;
      p_plt_done       <= 1'b0;
      p_sbi_state      <= 2'd0;
      p_sbi_detect     <= 2'd0;
      p_sbi_four       <= 2'd0;
      p_sbi_rms        <= 1'b0;
      p_sbi_rmr        <= 1'b0;
      p_sbi_sms        <= 1'b0;
      p_sbi_smr        <= 1'b0;
    end else begin
      cyc <= cyc + 1;

      // --- bump activity ---------------------------------------------------
      if (a2b_sb_val)     n_a2b_sb_bits      <= n_a2b_sb_bits + 1;
      if (b2a_sb_val)     n_b2a_sb_bits      <= n_b2a_sb_bits + 1;
      if (ucie_sb_rx_clk) n_ucie_sbclk_edges <= n_ucie_sbclk_edges + 1;

      if (n_a2b_sb_bits == 0 && a2b_sb_val)
        ev("madsim began transmitting on the sideband");

      // --- what madsim itself transmits, assembled the same way ------------
      if (a2b_sb_val && !mad_word_logged) begin
        mad_word[mad_bit_idx] <= a2b_sb_data;
        if (mad_bit_idx == 63) begin
          mad_word_logged <= 1'b1;
          mad_bit_idx     <= 0;
        end else begin
          mad_bit_idx <= mad_bit_idx + 1;
        end
      end
      if (mad_bit_idx == 63 && a2b_sb_val && !mad_word_logged)
        evv($sformatf("madsim TX first 64 sideband bits = 0x%016h (expect 5555555555555555)",
                     {a2b_sb_data, mad_word[62:0]}));

      // --- stage 2: the bit assembler in the forwarded clock domain ---------
      // Sampled from the local clock. Both dies share one clock, so an offer
      // shorter than a period cannot occur.
      if (des_enq_valid) begin
        n_enq_offer <= n_enq_offer + 1;
        if (des_enq_ready) n_enq_taken <= n_enq_taken + 1;
        else if (n_enq_offer == n_enq_taken)
          evv("deserializer assembled a word but the async FIFO was not ready");
      end

      // --- stage 3: out of the domain crossing ------------------------------
      if (des_raw_valid) begin
        n_raw_words <= n_raw_words + 1;
        if (des_raw_word == SB_INIT_PATTERN) n_raw_match <= n_raw_match + 1;
        else if (n_raw_words < 6)
          evv($sformatf("deserializer out (pre-queue): 0x%016h (not the pattern)",
                       des_raw_word));
      end

      // --- stage 4: past the parity check and the priority queue ------------
      if (des_out_valid) begin
        n_des_words <= n_des_words + 1;
        if (des_out_word == SB_INIT_PATTERN) begin
          n_des_match <= n_des_match + 1;
          if (n_des_match < 3)
            evv($sformatf("link node out: SBINIT pattern (match %0d)", n_des_match + 1));
        end else if (n_des_words < 6) begin
          evv($sformatf("link node out: 0x%016h (not the pattern)", des_out_word));
        end
      end

      // --- stage 5: what actually reaches the LTSM --------------------------
      if (ucie_sb_rx_valid) begin
        n_ucie_sb_words <= n_ucie_sb_words + 1;
        if (ucie_sb_rx_word == SB_INIT_PATTERN) n_ucie_sb_match <= n_ucie_sb_match + 1;
        if (ucie_sb_rx_word !== last_rx_word) begin
          last_rx_word <= ucie_sb_rx_word;
          last_rx_cyc  <= cyc;
          last_rx_ltsm <= mad_ltsm;
          n_rx_repeat  <= 0;
          // The clock pattern arrives thousands of times; announce the first
          // few and then stay quiet about it.
          if (ucie_sb_rx_word != SB_INIT_PATTERN || n_ucie_sb_match < 4)
            ev(sb_line("madsim -> ucie  :", mad_ltsm, ucie_sb_rx_word));
        end else begin
          n_rx_repeat <= n_rx_repeat + 1;
          last_rx_cyc <= cyc;
        end
      end

      if (des_reset) n_des_reset <= n_des_reset + 1;
      if (des_fwclk && !p_des_fwclk) n_des_fwclk <= n_des_fwclk + 1;
      p_des_fwclk <= des_fwclk;

      // --- deserializer re-framing ------------------------------------------
      if (des_reframe) n_reframe <= n_reframe + 1;
      if (des_reframe && !p_reframe && n_reframe < 12)
        evv($sformatf("deserializer reframe asserted, bit counter = %0d, maxBits = %0d, quiet = %0d",
                     des_bit_cnt, des_max_bits, des_quiet));
      p_reframe <= des_reframe;

      // --- madsim burst structure -------------------------------------------
      if (a2b_sb_val) begin
        burst_len <= burst_len + 1;
        // Bursts before sbReset releases are uninformative, because the deserializer is
        // held in reset so its counter reads 0 regardless. The interesting
        // window is the handful of bursts either side of the release.
        if (!p_a2b_val && n_bursts >= 46 && n_bursts < 78)
          evv($sformatf("madsim burst %0d starts, deserializer bit counter = %0d, maxBits = %0d, rxMode = %0d%s",
                       n_bursts, des_bit_cnt, des_max_bits, des_rxmode,
                       ((des_bit_cnt & 6'h3F) == 0) ? "" : "   <-- MISFRAMED, not on a 64-bit chunk boundary"));
      end else if (p_a2b_val) begin
        n_bursts <= n_bursts + 1;
        if (n_bursts >= 46 && n_bursts < 78)
          evv($sformatf("madsim burst %0d ended, length %0d bits%s",
                       n_bursts, burst_len,
                       (burst_len == 64) ? "" : "   <-- NOT 64"));
        burst_len <= 0;
      end
      p_a2b_val <= a2b_sb_val;

      // --- transmit handshake chain -----------------------------------------
      if (req_tx_valid) n_req_v <= n_req_v + 1;
      if (req_tx_ready) n_req_r <= n_req_r + 1;
      if (req_tx_valid && req_tx_ready) begin
        n_req_fire <= n_req_fire + 1;
        if (n_req_fire < 6)
          evv($sformatf("sbInit requester tx FIRE, data 0x%016h", req_tx_data));
      end
      if (ln_tx_valid) n_ln_v <= n_ln_v + 1;
      if (ln_tx_ready) n_ln_r <= n_ln_r + 1;
      if (ln_tx_valid && ln_tx_ready) begin
        n_ln_fire <= n_ln_fire + 1;
        if (n_ln_fire < 6)
          evv($sformatf("link node txIn FIRE, data 0x%016h", ln_tx_data));
      end
      if (ser_in_valid) n_ser_v <= n_ser_v + 1;
      if (ser_in_ready) n_ser_r <= n_ser_r + 1;
      if (ser_in_valid && ser_in_ready) begin
        n_ser_fire <= n_ser_fire + 1;
        if (n_ser_fire < 6)
          evv($sformatf("serializer in FIRE, data 0x%016h", ser_in_data));
      end

      if (lt_tx_valid) n_lt_v <= n_lt_v + 1;
      if (lt_tx_ready) n_lt_r <= n_lt_r + 1;
      if (lt_tx_valid && lt_tx_ready) n_lt_fire <= n_lt_fire + 1;
      if (sw_curr_valid)  n_swc_v <= n_swc_v + 1;
      if (sw_curr_ready)  n_swc_r <= n_swc_r + 1;
      if (sw_upper_valid) n_swu_v <= n_swu_v + 1;
      if (sw_lower_valid) n_swl_v <= n_swl_v + 1;
      if (!lib_bypass)    n_lib_skid <= n_lib_skid + 1;
      if (!lnskid_bypass) n_lnskid_skid <= n_lnskid_skid + 1;

      if (rdic_tx_valid === 1'b1) n_rdic_v <= n_rdic_v + 1;
      if (rdic_tx_valid === 1'bx) n_rdic_x <= n_rdic_x + 1;
      if (stxq_deq_valid) n_stxq_v <= n_stxq_v + 1;
      if (lyr_in_valid)   n_lyr_v  <= n_lyr_v + 1;
      if (lyr_in_ready)   n_lyr_r  <= n_lyr_r + 1;

      // First signal in the receive chain to go X. An X on a valid propagates,
      // so every consumer downstream computes an undefined ready.
      if (!reported_xsrc &&
          ((rdi_in_valid === 1'bx) || (rdin_rxout_valid === 1'bx) ||
           (sw_upper_valid === 1'bx) || (lib_bypass === 1'bx) ||
           (sw_curr_ready === 1'bx))) begin
        reported_xsrc <= 1'b1;
        ev($sformatf("first X on a control signal: rdi.in.valid=%0b rdiIntfNode.rxOut.valid=%0b switch.upper.from.valid=%0b layerInBuffer.bypassReg=%0b layerInBuffer.out.valid=%0b switch.curr.from.ready=%0b",
                     rdi_in_valid, rdin_rxout_valid, sw_upper_valid,
                     lib_bypass, lib_out_valid, sw_curr_ready));
      end

      // Catch the first stalled header and say where the X came from, rather
      // than only reporting the value that happens to be there at $finish.
      if (!reported_x && sw_curr_valid && (^sw_curr_data === 1'bx)) begin
        reported_x <= 1'b1;
        ev($sformatf("switch header is X. curr=0x%016h  lyrIn=0x%016h(v%0b r%0b)  stxq=0x%016h(v%0b)  arbOut=0x%016h(v%0b)  rdiCtrl=0x%016h(v%0b)  ltsm=0x%016h(v%0b)",
                     sw_curr_data, lyr_in_data, lyr_in_valid, lyr_in_ready,
                     stxq_deq_data, stxq_deq_valid,
                     stxa_out_data, stxa_out_valid,
                     rdic_tx_data, rdic_tx_valid,
                     lt_tx_data, lt_tx_valid));
      end

      // --- what UcieTL sends back -------------------------------------------
      if (b2a_sb_val) begin
        if (b2a_bit_idx == 63) begin
          b2a_bit_idx <= 0;
          n_b2a_words <= n_b2a_words + 1;
          // Consecutive repeats are counted, not printed, because the exchanger
          // re-sends an unanswered message every few thousand cycles, and
          // printing each one buries the exchange in duplicates.
          if ({b2a_sb_data, b2a_word[62:0]} !== last_tx_word) begin
            last_tx_word <= {b2a_sb_data, b2a_word[62:0]};
            last_tx_cyc  <= cyc;
            last_tx_ltsm <= mad_ltsm;
            n_tx_repeat  <= 0;
            n_b2a_distinct <= n_b2a_distinct + 1;
            ev(sb_line("ucie  -> madsim :", mad_ltsm,
                       {b2a_sb_data, b2a_word[62:0]}));
          end else begin
            n_tx_repeat <= n_tx_repeat + 1;
            last_tx_cyc <= cyc;
            if (n_tx_repeat == 0)
              ev($sformatf("ucie  -> madsim :   (re-sending the above; further repeats counted, not logged)"));
          end
        end else begin
          b2a_word[b2a_bit_idx] <= b2a_sb_data;
          b2a_bit_idx <= b2a_bit_idx + 1;
        end
      end

      // --- mainband activity ------------------------------------------------
      if (a2b_mb_ckp !== p_a2b_ckp) begin
        n_a2b_ck <= n_a2b_ck + 1;
        if (n_a2b_ck == 0) t_a2b_ck_first <= cyc;
        t_a2b_ck_last <= cyc;
      end
      if (b2a_mb_ckp !== p_b2a_ckp) begin
        n_b2a_ck <= n_b2a_ck + 1;
        if (n_b2a_ck == 0) t_b2a_ck_first <= cyc;
        t_b2a_ck_last <= cyc;
      end
      if (a2b_mb_ckn !== p_a2b_ckn) n_a2b_ckn <= n_a2b_ckn + 1;
      if (b2a_mb_ckn !== p_b2a_ckn) n_b2a_ckn <= n_b2a_ckn + 1;
      p_a2b_ckn <= a2b_mb_ckn; p_b2a_ckn <= b2a_mb_ckn;
      if (a2b_mb_trk !== p_a2b_trk) n_a2b_trk <= n_a2b_trk + 1;
      if (b2a_mb_trk !== p_b2a_trk) n_b2a_trk <= n_b2a_trk + 1;
      if (a2b_mb_data !== p_a2b_dat) n_a2b_dat_tog <= n_a2b_dat_tog + 1;
      if (b2a_mb_data !== p_b2a_dat) n_b2a_dat_tog <= n_b2a_dat_tog + 1;
      if (a2b_mb_vld) n_a2b_vld_hi <= n_a2b_vld_hi + 1;
      if (b2a_mb_vld) n_b2a_vld_hi <= n_b2a_vld_hi + 1;
      p_a2b_ckp <= a2b_mb_ckp; p_b2a_ckp <= b2a_mb_ckp;
      p_a2b_trk <= a2b_mb_trk; p_b2a_trk <= b2a_mb_trk;
      p_a2b_dat <= a2b_mb_data; p_b2a_dat <= b2a_mb_data;

      // --- what ucieDigital hands the PHY -----------------------------------
      if (dig_mb_tx_valid && (dig_mb_tx_clkp !== p_dig_clkp)) begin
        p_dig_clkp <= dig_mb_tx_clkp;
        if (n_dig_clkp < 8) begin
          n_dig_clkp <= n_dig_clkp + 1;
          evv($sformatf("ucieDigital -> phy: clkP=0x%08h clkN=0x%08h trk=0x%08h data0=0x%08h",
                       dig_mb_tx_clkp, dig_mb_tx_clkn, dig_mb_tx_trk, dig_mb_tx_data0));
        end
      end
      if (phy_mb_tx_valid && (phy_mb_tx_clkp !== p_phy_clkp)) begin
        p_phy_clkp <= phy_mb_tx_clkp;
        if (n_phy_clkp < 8) begin
          n_phy_clkp <= n_phy_clkp + 1;
          evv($sformatf("phy tx input: clkP=0x%08h", phy_mb_tx_clkp));
        end
      end

      // --- UcieTL's own pattern detection ------------------------------------
      // Every word once the first non-zero one arrives, idle included, since
      // filtering idle out makes a bursty stream look continuous.
      if (ucie_mb_rx_valid) begin
        n_ucie_mb_rx <= n_ucie_mb_rx + 1;
        if (ucie_mb_rx_trk != 32'h0) mb_rx_seen_nz <= 1'b1;
        if ((mb_rx_seen_nz || ucie_mb_rx_trk != 32'h0) && n_ucie_mb_rx_nz < 40) begin
          n_ucie_mb_rx_nz <= n_ucie_mb_rx_nz + 1;
          evv($sformatf("ucie mainband rx word %0d: trk=0x%08h vld=0x%08h%s",
                       n_ucie_mb_rx_nz, ucie_mb_rx_trk, ucie_mb_rx_vld,
                       (ucie_mb_rx_trk == 32'h0) ? "   <- trk IDLE" : ""));
        end
        // The valid lane during REPAIRVAL, logged on its own so the VALTRAIN
        // window is visible even though trk is quiet by then.
        if ((ucie_mb_rx_d0 != 32'h0 || ucie_mb_rx_d1 != 32'h0) && n_ucie_mb_d < 12) begin
          n_ucie_mb_d <= n_ucie_mb_d + 1;
          evv($sformatf("ucie data-lane word %0d: d0=0x%08h d1=0x%08h (PERLANEID reference 0xa00aa00a / 0xa01aa01a)",
                       n_ucie_mb_d, ucie_mb_rx_d0, ucie_mb_rx_d1));
        end
        if (ucie_mb_rx_vld != 32'h0 && n_ucie_mb_vld < 24) begin
          n_ucie_mb_vld <= n_ucie_mb_vld + 1;
          evv($sformatf("ucie valid-lane word %0d: vld=0x%08h (VALTRAIN reference 0x0f0f0f0f)",
                       n_ucie_mb_vld, ucie_mb_rx_vld));
        end
      end
      if (pr_resp_valid && !p_pr_valid)
        evv($sformatf("ucie patternReader resp: aggregate=%0b lanes=%0b%0b%0b  type=%0d consec=%0b target=%0d counts=%0d/%0d/%0d",
                     pr_aggregate, pr_lane2, pr_lane1, pr_lane0,
                     pr_ptype, pr_consec, pr_target, pr_cnt0, pr_cnt1, pr_cnt2));
      p_pr_valid <= pr_resp_valid;

      // --- mainband activity, as events -------------------------------------
      // Each die's view of whether it is driving or watching the mainband.
      if (mad_mb_tx_enable && !p_mad_txen) begin
        evv($sformatf("madsim -> ucie  :   MAINBAND pattern %0d transmitting", mad_mb_pattern_select));
        p_mad_txen <= 1'b1;
      end
      if (!mad_mb_tx_enable && p_mad_txen) begin
        evv("madsim -> ucie  :   MAINBAND pattern transmit ended");
        p_mad_txen <= 1'b0;
      end
      // dig_mb_tx_valid pulses once per serializer word, not once per burst, so
      // count words and call the burst over only after a long quiet period.
      // Word count for this burst, and during an LFSR compare the received word
      // against the reference. Recapture each detect window, not just the first.
      if (pr_state == 2'd1 && p_pr_state_lfsr != 2'd1) n_lfsr_cmp <= 0;
      p_pr_state_lfsr <= pr_state;
      if (pr_counter_en && pr_ptype == 3'd3 && n_lfsr_cmp < 140 &&
          (rx_lane_d0 != 32'h0)) begin
        n_lfsr_cmp <= n_lfsr_cmp + 1;
        evv($sformatf("ucie LFSR word %0d: d0=0x%08h ref=0x%08h",
                     n_lfsr_cmp, rx_lane_d0, rx_lfsr_ref0));
      end
      if (mbt_txpt_start)
        evv($sformatf("ucie txPtTest start pulse: MBTrainSM offers type=%0d, requester holds type=%0d (state %0d)",
                     mbt_txpt_ptype, txpt_req_ptype, txpt_req_state));
      if (pw_in_progress && pw_cycle_count > pw_peak) pw_peak <= pw_cycle_count;
      if (!pw_in_progress && p_pw_busy) begin
        evv($sformatf("ucie patternWriter burst done: type=%0d emitted %0d words",
                     pw_pattern_type, pw_peak + 1));
        pw_peak <= 8'd0;
      end
      p_pw_busy <= pw_in_progress;

      if (dig_mb_tx_valid && !p_ucie_txen_lvl) n_ucie_tx_words <= n_ucie_tx_words + 1;
      p_ucie_txen_lvl <= dig_mb_tx_valid;

      if (dig_mb_tx_valid) ucie_tx_quiet <= 0;
      else if (ucie_tx_quiet != 16'hffff) ucie_tx_quiet <= ucie_tx_quiet + 1;

      if (dig_mb_tx_valid && !p_ucie_txen) begin
        evv($sformatf("ucie  -> madsim :   MAINBAND pattern transmitting  clkP=0x%08h trk=0x%08h",
                     dig_mb_tx_clkp, dig_mb_tx_trk));
        p_ucie_txen <= 1'b1;
      end
      if (p_ucie_txen && ucie_tx_quiet == 16'd256) begin
        evv($sformatf("ucie  -> madsim :   MAINBAND pattern transmit ended, %0d words",
                     n_ucie_tx_words));
        p_ucie_txen <= 1'b0;
        n_ucie_tx_words <= 0;
      end

      // --- madsim's pattern detector ----------------------------------------
      // What madsim received in each chunk, off phy_top's public mb_rx_chunk.
      // Which third it expected is private to Mainband, so not checkable here.
      if (mad_mb_rx_chunk_valid && !p_chunk_valid && n_chunk_logged < 12) begin
        n_chunk_logged <= n_chunk_logged + 1;
        evv($sformatf("mad.mb rx chunk: vld=0x%04h ckp=0x%04h ckn=0x%04h trk=0x%04h",
                     mad_mb_rx_chunk_vld, mad_mb_rx_chunk_ckp,
                     mad_mb_rx_chunk_ckn, mad_mb_rx_chunk_trk));
      end
      p_chunk_valid <= mad_mb_rx_chunk_valid;



      if (mad_mb_clock_pass !== p_mad_cp) begin
        evv($sformatf("mad.mb clock_pass %0d -> %0d  (ckp=%0b ckn=%0b trk=%0b, all three needed)",
                     p_mad_cp, mad_mb_clock_pass, mad_mb_clock_pass[0],
                     mad_mb_clock_pass[1], mad_mb_clock_pass[2]));
        p_mad_cp <= mad_mb_clock_pass;
      end
      if (mad_mb_rx_enable !== p_mad_rxen) begin
        evv($sformatf("mad.mb pattern rx_enable = %0b (select %0d)",
                     mad_mb_rx_enable, mad_mb_pattern_select));
        p_mad_rxen <= mad_mb_rx_enable;
      end
      if (mad_mb_rx_done !== p_mad_rxdone) begin
        evv($sformatf("mad.mb pattern rx_done = %0b", mad_mb_rx_done));
        p_mad_rxdone <= mad_mb_rx_done;
      end
      if (mad_mb_rx_error !== p_mad_rxerr) begin
        evv($sformatf("mad.mb pattern rx_error = %0b", mad_mb_rx_error));
        p_mad_rxerr <= mad_mb_rx_error;
      end

      // --- UcieTL's MBINIT sub-FSMs -----------------------------------------
      if (mbi_req_state !== p_mbi_req) begin
        evv($sformatf("ucie.mbInit.requester state %0d -> %0d", p_mbi_req, mbi_req_state));
        p_mbi_req <= mbi_req_state;
      end
      if (mbi_rsp_state !== p_mbi_rsp) begin
        evv($sformatf("ucie.mbInit.responder state %0d -> %0d", p_mbi_rsp, mbi_rsp_state));
        p_mbi_rsp <= mbi_rsp_state;
      end
      if (mbi_req_sub !== p_mbi_rsub) begin
        evv($sformatf("ucie.mbInit.requester substate %0d -> %0d", p_mbi_rsub, mbi_req_sub));
        p_mbi_rsub <= mbi_req_sub;
      end
      if (mbi_rsp_sub !== p_mbi_ssub) begin
        evv($sformatf("ucie.mbInit.responder substate %0d -> %0d", p_mbi_ssub, mbi_rsp_sub));
        p_mbi_ssub <= mbi_rsp_sub;
      end
      if (mbi_cal_done_reg !== p_mbi_cdr) begin
        evv($sformatf("ucie.mbInit.mbInitCalDoneReg = %0b", mbi_cal_done_reg));
        p_mbi_cdr <= mbi_cal_done_reg;
      end
      if (mbi_cal_done_in !== p_mbi_cdi) begin
        evv($sformatf("ucie.mbInit.io_mbInitCalDone = %0b", mbi_cal_done_in));
        p_mbi_cdi <= mbi_cal_done_in;
      end
      if (plt_selfcal_start !== p_plt_start) begin
        evv($sformatf("ucie.phyLaneTrainer.mbInit.selfCalStart = %0b", plt_selfcal_start));
        p_plt_start <= plt_selfcal_start;
      end
      if (plt_selfcal_done !== p_plt_done) begin
        evv($sformatf("ucie.phyLaneTrainer.mbInit.selfCalDone = %0b", plt_selfcal_done));
        p_plt_done <= plt_selfcal_done;
      end

      // --- UcieTL's SBINIT sub-FSM ------------------------------------------
      if (sbi_state !== p_sbi_state) begin
        evv($sformatf("ucie.sbInit.requester %s -> %s",
                     sbi_name(p_sbi_state), sbi_name(sbi_state)));
        p_sbi_state <= sbi_state;
      end
      if (sbi_detect_cnt !== p_sbi_detect) begin
        evv($sformatf("ucie.sbInit.detectPatternCounter %0d -> %0d%s",
                     p_sbi_detect, sbi_detect_cnt,
                     (sbi_detect_cnt == 2'd2) ? "  (partner pattern detected)" : ""));
        p_sbi_detect <= sbi_detect_cnt;
      end
      if (sbi_four_cnt !== p_sbi_four) begin
        evv($sformatf("ucie.sbInit.fourPatternCounter %0d -> %0d", p_sbi_four, sbi_four_cnt));
        p_sbi_four <= sbi_four_cnt;
      end
      if (sbi_req_msg_sent !== p_sbi_rms) begin
        evv($sformatf("ucie.sbInit.requester.msgSent = %0b", sbi_req_msg_sent));
        p_sbi_rms <= sbi_req_msg_sent;
      end
      if (sbi_req_msg_recv !== p_sbi_rmr) begin
        evv($sformatf("ucie.sbInit.requester.msgReceived = %0b", sbi_req_msg_recv));
        p_sbi_rmr <= sbi_req_msg_recv;
      end
      if (sbi_rsp_msg_sent !== p_sbi_sms) begin
        evv($sformatf("ucie.sbInit.responder.msgSent = %0b", sbi_rsp_msg_sent));
        p_sbi_sms <= sbi_rsp_msg_sent;
      end
      if (sbi_rsp_msg_recv !== p_sbi_smr) begin
        evv($sformatf("ucie.sbInit.responder.msgReceived = %0b", sbi_rsp_msg_recv));
        p_sbi_smr <= sbi_rsp_msg_recv;
      end

      // --- heartbeat --------------------------------------------------------
      // A run spends millions of cycles waiting for resetMinWait with nothing
      // else to say. Without this the log looks hung rather than patient.
      if (cyc % 500000 == 0 && cyc != 0)
        evv($sformatf("... mad.ltsm=%0d ucie.ltState=%0d patCnt=%0d resetMinWait=%0b (%0d)  sb: %0d bits in, %0d words assembled",
                     mad_ltsm, ucie_ltstate, ucie_sb_pat_cnt, ucie_reset_min_wait,
                     ucie_timeout_cnt, n_a2b_sb_bits, n_enq_offer));
      if (cyc % 500000 == 0 && cyc != 0)
        evv($sformatf("      sbInit: %s detect=%0d four=%0d  req(sent=%0b recv=%0b) rsp(sent=%0b recv=%0b)  b2a %0d words, %0d distinct",
                     sbi_name(sbi_state), sbi_detect_cnt, sbi_four_cnt,
                     sbi_req_msg_sent, sbi_req_msg_recv,
                     sbi_rsp_msg_sent, sbi_rsp_msg_recv,
                     n_b2a_words, n_b2a_distinct));

      // --- state transitions ------------------------------------------------
      if (mad_ltsm !== p_mad_ltsm) begin
        ev($sformatf("mad.ltsm    %0d -> %0d (%s)",
                     p_mad_ltsm, mad_ltsm, mad_ltsm_name(mad_ltsm)));
        p_mad_ltsm <= mad_ltsm;
      end
      if (mad_rdi_state !== p_mad_rdi) begin
        ev($sformatf("mad.rdi     %0d -> %0d", p_mad_rdi, mad_rdi_state));
        p_mad_rdi <= mad_rdi_state;
      end
      if (mad_trainerror && !p_mad_trainerror) begin
        ev("mad.TRAINERROR asserted");
        p_mad_trainerror <= 1'b1;
      end

      if (ucie_ltstate !== p_ucie_ltstate) begin
        ev($sformatf("ucie.ltState %0d -> %0d (%s)",
                     p_ucie_ltstate, ucie_ltstate, ltstate_name(ucie_ltstate)));
        p_ucie_ltstate <= ucie_ltstate;
      end
      if (ucie_ltsm_detail !== p_ucie_detail) begin
        evv($sformatf("ucie.ltsmDetail %0d -> %0d", p_ucie_detail, ucie_ltsm_detail));
        p_ucie_detail <= ucie_ltsm_detail;
      end
      if (ucie_rdi_state !== p_ucie_rdi) begin
        ev($sformatf("ucie.rdi    %0d -> %0d", p_ucie_rdi, ucie_rdi_state));
        p_ucie_rdi <= ucie_rdi_state;
      end

      // --- the RESET exit conditions ---------------------------------------
      if (ucie_sb_pat_cnt !== p_sb_pat) begin
        evv($sformatf("ucie.sbInitPatternCounter %0d -> %0d%s",
                     p_sb_pat, ucie_sb_pat_cnt,
                     (ucie_sb_pat_cnt == 2'd2) ? "  (remote trigger armed)" : ""));
        p_sb_pat <= ucie_sb_pat_cnt;
      end
      if (ucie_reset_min_wait && !p_reset_min_wait) begin
        evv($sformatf("ucie.resetMinWait satisfied after %0d cycles in RESET", ucie_timeout_cnt));
        p_reset_min_wait <= 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------------
  // UcieTL's mainband clock lane, captured at UI resolution
  // ---------------------------------------------------------------------
  // Two UI per cycle, so nothing sampled once per clock sees the transmitted
  // word. These sample mid-half-period and assemble the UI stream the way
  // madsim's MbRx does, to check the CLKREPAIR word and its rotation.
  localparam [47:0] CLKREPAIR_EXPECTED = 48'h0000_5555_5555;
  // A quarter period samples the middle of each half period, where the lane is
  // settled. The monitor is not synthesised, so the delay is legitimate here.
  localparam realtime CLK_QUARTER = 312ps;

  // Clock lane and track lane both, since ucieDigital drives identical words
  // on each, so a difference between them is a transmit-side fault.
  logic        ui_pos, ui_neg;
  logic        tk_pos, tk_neg;
  logic [47:0] tk_window = 48'd0;
  logic [47:0] ui_window = 48'd0;
  longint unsigned ui_count = 0;
  longint unsigned n_ui_logged = 0;

  always @(posedge clk) begin
    #(CLK_QUARTER) ui_pos = b2a_mb_ckp;
    tk_pos = b2a_mb_trk;
  end

  always @(negedge clk) begin
    #(CLK_QUARTER) ui_neg = b2a_mb_ckp;
    tk_neg = b2a_mb_trk;
    if (rst_n) begin
      tk_window = {tk_neg, tk_pos, tk_window[47:2]};
      // Two UI per cycle, oldest toward the LSB.
      ui_window = {ui_neg, ui_pos, ui_window[47:2]};
      ui_count  = ui_count + 2;
      // Log a few complete windows once the burst is under way, plus the
      // rotation that would match, which names the offset directly.
      if (ui_count >= 48 && (ui_count % 48 == 0) && (|ui_window) &&
          n_ui_logged < 6) begin
        n_ui_logged = n_ui_logged + 1;
        evv($sformatf("ucie mainband 48 UI: ckp=0x%012h trk=0x%012h %s (expect 0x%012h)",
                     ui_window, tk_window,
                     (ui_window == tk_window) ? "IDENTICAL on the wire"
                                              : "DIFFER on the wire",
                     CLKREPAIR_EXPECTED));
      end
    end
  end

  // ---------------------------------------------------------------------
  // Edge-triggered counters
  // ---------------------------------------------------------------------
  // Sampling a sideband-clock signal on that clock lands in one phase and reads
  // as a constant, and `reframe` self-clears within a delta. These count edges.
  longint unsigned n_reframe_edges = 0;
  longint unsigned n_des_bit_edges = 0;

  always @(posedge des_reframe) n_reframe_edges <= n_reframe_edges + 1;

  // True transition counts. Both dies carry two UI per cycle, so sampling once
  // per cycle undercounts and cannot tell an idle lane from a full-rate one.
  longint unsigned e_a2b_ckp = 0, e_a2b_ckn = 0, e_a2b_trk = 0, e_a2b_dat = 0;
  longint unsigned e_b2a_ckp = 0, e_b2a_ckn = 0, e_b2a_trk = 0, e_b2a_dat = 0;
  always @(a2b_mb_ckp)  e_a2b_ckp <= e_a2b_ckp + 1;
  always @(a2b_mb_ckn)  e_a2b_ckn <= e_a2b_ckn + 1;
  always @(a2b_mb_trk)  e_a2b_trk <= e_a2b_trk + 1;
  always @(a2b_mb_data) e_a2b_dat <= e_a2b_dat + 1;
  always @(b2a_mb_ckp)  e_b2a_ckp <= e_b2a_ckp + 1;
  always @(b2a_mb_ckn)  e_b2a_ckn <= e_b2a_ckn + 1;
  always @(b2a_mb_trk)  e_b2a_trk <= e_b2a_trk + 1;
  always @(b2a_mb_data) e_b2a_dat <= e_b2a_dat + 1;

  // The deserializer samples a bit on each FALLING edge of the forwarded clock,
  // so this is the number of bits it actually took in.
  always @(negedge des_fwclk) n_des_bit_edges <= n_des_bit_edges + 1;

  // ---------------------------------------------------------------------
  // Stage 1 measured the same way stage 2 measures it
  // ---------------------------------------------------------------------
  // Assembles the shim's output by the deserializer's own rule, so the two can
  // be compared to locate a rotation.
  logic [63:0] eye_word = 64'b0;
  int unsigned eye_idx = 0;
  int unsigned eye_words = 0;

  always @(negedge ucie_sb_rx_clk) begin
    if (rst_n) begin
      if (eye_idx == 63) begin
        if (eye_words < 4)
          if (VERBOSE) begin
            $fdisplay(fh, "[%12d] shim output as the deserializer would frame it = 0x%016h",
                      cyc, {ucie_sb_rx_data_i, eye_word[62:0]});
            $fflush(fh);
          end
        eye_words <= eye_words + 1;
        eye_idx   <= 0;
      end else begin
        eye_word[eye_idx] <= ucie_sb_rx_data_i;
        eye_idx <= eye_idx + 1;
      end
    end
  end

  // ---------------------------------------------------------------------
  // madsim's sideband transmit path
  // ---------------------------------------------------------------------
  logic       p_ltsm_starved  = 1'b0;
  // Offers versus acknowledged handoffs. sent_req_/sent_resp_ are set when
  // SendMsg is called, not on handoff, so a message the sideband does not take
  // is never re-offered and the LTSM still believes it went out.
  logic [63:0] p_ltsm_msg = 64'h0;
  int unsigned n_ltsm_lost = 0;
  int unsigned n_ltsm_offers = 0;
  int unsigned n_ltsm_handoffs = 0;
  logic        p_ltsm_offer = 1'b0;

  always @(posedge clk) if (rst_n) begin

    if (mad_sb_ltsm_tx_valid && p_ltsm_offer &&
        (mad_sb_ltsm_tx_msg !== p_ltsm_msg) && !mad_sb_ltsm_tx_ready) begin
      n_ltsm_lost <= n_ltsm_lost + 1;
      $fdisplay(fh, "[%12d] mad.sideband MESSAGE LOST: 0x%016h overwritten by 0x%016h before handoff (%s -> %s)",
                cyc, p_ltsm_msg, mad_sb_ltsm_tx_msg,
                sb_msg_name(mad_ltsm, p_ltsm_msg), sb_msg_name(mad_ltsm, mad_sb_ltsm_tx_msg));
      $fflush(fh);
    end
    p_ltsm_msg <= mad_sb_ltsm_tx_msg;
    if (mad_sb_ltsm_tx_valid && !p_ltsm_offer)
      n_ltsm_offers <= n_ltsm_offers + 1;
    if (mad_sb_ltsm_tx_valid && mad_sb_ltsm_tx_ready)
      n_ltsm_handoffs <= n_ltsm_handoffs + 1;
    p_ltsm_offer <= mad_sb_ltsm_tx_valid;

    p_ltsm_starved  <= (mad_sb_ltsm_tx_valid && !mad_sb_ltsm_tx_ready);
  end

  // High-water marks for the reader's consecutive counters. A counter that
  // climbs and is then reset by a dirty word looks identical at the end of the
  // window to one that never counted at all; only the peak separates them.
  logic [15:0] pr_cnt0_hi = 16'd0, pr_cnt1_hi = 16'd0, pr_cnt2_hi = 16'd0;
  int unsigned pr_dirty_events = 0;
  logic p_pr_dirty2 = 1'b0;
  // Cycles the reader spent able to count, and words offered while it could.
  // Separating these says whether the window was open, whether words arrived
  // inside it, and whether the two ever overlapped.
  int unsigned pr_detect_cycles = 0;
  int unsigned pr_rxvalid_cycles = 0;
  int unsigned pr_counting_cycles = 0;
  int unsigned pr_rxvalid_in_detect = 0;

  always @(posedge clk) if (rst_n) begin
    if (pr_cnt0 > pr_cnt0_hi) pr_cnt0_hi <= pr_cnt0;
    if (pr_cnt1 > pr_cnt1_hi) pr_cnt1_hi <= pr_cnt1;
    if (pr_cnt2 > pr_cnt2_hi) pr_cnt2_hi <= pr_cnt2;
    if (pr_dirty2 && !p_pr_dirty2) pr_dirty_events <= pr_dirty_events + 1;
    p_pr_dirty2 <= pr_dirty2;

    if (pr_state == 2'd1)  pr_detect_cycles <= pr_detect_cycles + 1;
    if (pr_rx_valid)       pr_rxvalid_cycles <= pr_rxvalid_cycles + 1;
    if (pr_counter_en)     pr_counting_cycles <= pr_counting_cycles + 1;
    if (pr_state == 2'd1 && pr_rx_valid)
      pr_rxvalid_in_detect <= pr_rxvalid_in_detect + 1;
  end

  // ---------------------------------------------------------------------
  // Bring-up milestones
  // ---------------------------------------------------------------------
  // Each layer reaching ACTIVE, on its own line. RDI and FDI encode reset=0,
  // active=1 (interfaces/Types.scala); LTSM ACTIVE is 22 on madsim, 5 here.
  logic mad_ltsm_act = 1'b0, ucie_ltsm_act = 1'b0;
  logic mad_rdi_act  = 1'b0, ucie_rdi_act  = 1'b0;
  logic mad_fdi_act  = 1'b0, ucie_fdi_act  = 1'b0;
  logic data_ready   = 1'b0;
  logic train_done   = 1'b0;

  task automatic milestone(input string who, input string what);
    $fdisplay(fh, "");
    $fdisplay(fh, "[%12d]  *** %s %s ***", cyc, who, what);
    $fflush(fh);
  endtask

  always @(posedge clk) if (rst_n) begin
    if (!mad_ltsm_act && mad_ltsm == 5'd22) begin
      mad_ltsm_act <= 1'b1; milestone("madsim", "LTSM ACTIVE");
    end
    if (!ucie_ltsm_act && ucie_ltstate == 5'd5) begin
      ucie_ltsm_act <= 1'b1; milestone("ucieTL", "LTSM ACTIVE");
    end
    if (!train_done && mad_ltsm_act && ucie_ltsm_act) begin
      train_done <= 1'b1; milestone("BOTH DIES", "TRAINING DONE (LTSM ACTIVE)");
    end
    if (!mad_rdi_act && mad_rdi_state == 4'h1) begin
      mad_rdi_act <= 1'b1; milestone("madsim", "RDI ACTIVE");
    end
    if (!ucie_rdi_act && ucie_rdi_state == 4'h1) begin
      ucie_rdi_act <= 1'b1; milestone("ucieTL", "RDI ACTIVE");
    end
    if (!mad_fdi_act && mad_fdi_state == 4'h1) begin
      mad_fdi_act <= 1'b1; milestone("madsim", "FDI ACTIVE");
    end
    if (!ucie_fdi_act && ucie_fdi_sts == 4'h1) begin
      ucie_fdi_act <= 1'b1; milestone("ucieTL", "FDI ACTIVE");
    end
    if (!data_ready && mad_ltsm_act && ucie_ltsm_act &&
        mad_rdi_act && ucie_rdi_act && mad_fdi_act && ucie_fdi_act) begin
      data_ready <= 1'b1;
      milestone("BOTH DIES", "READY TO SEND DATA (LTSM + RDI + FDI all ACTIVE)");
    end
  end

  function automatic string adp_init_name(input logic [2:0] st);
    case (st)
      3'd0: return "RESET";
      3'd1: return "SB_INIT";
      3'd2: return "PARAM_EXCH";
      3'd3: return "FDI_BRINGUP";
      3'd4: return "DONE";
      default: return $sformatf("state%0d", st);
    endcase
  endfunction

  logic [2:0] p_adp_init = 3'd0;
  logic [7:0] p_adp_flags = 8'd0;
  always @(posedge clk) if (rst_n) begin
    if (adp_init_state !== p_adp_init) begin
      $fdisplay(fh, "[%12d] ucie.adapter linkInit %s -> %s",
                cyc, adp_init_name(p_adp_init), adp_init_name(adp_init_state));
      $fflush(fh);
    end
    if ({adp_cap_snt,adp_cap_rcv,adp_req_snt,adp_req_rcv,adp_rsp_snt,adp_rsp_rcv} !== p_adp_flags[5:0]) begin
      evv($sformatf("ucie.adapter %s: ADV_CAP snt=%0b rcv=%0b | REQ_ACTIVE snt=%0b rcv=%0b | RSP_ACTIVE snt=%0b rcv=%0b",
                adp_init_name(adp_init_state),
                adp_cap_snt, adp_cap_rcv, adp_req_snt, adp_req_rcv, adp_rsp_snt, adp_rsp_rcv));
    end
    p_adp_init <= adp_init_state;
    p_adp_flags[5:0] <= {adp_cap_snt,adp_cap_rcv,adp_req_snt,adp_req_rcv,adp_rsp_snt,adp_rsp_rcv};
  end

  // SideBandMessage encodings, from
  // ucie/scala/src/uciedigital/interfaces/Types.scala.
  function automatic string adp_sb_name(input logic [5:0] m);
    case (m)
      6'h00: return "NOP";
      6'h01: return "REQ_ACTIVE";
      6'h11: return "RSP_ACTIVE";
      6'h24: return "ADV_CAP";
      6'h09: return "REQ_LINKRESET";
      6'h19: return "RSP_LINKRESET";
      6'h0a: return "REQ_DISABLED";
      6'h1a: return "RSP_DISABLED";
      default: return $sformatf("msg0x%02h", m);
    endcase
  endfunction

  // A message is only acted on in the substate that expects it; anything
  // earlier is dropped. Log arrival against the substate so that is visible.
  logic [5:0] p_adp_rcv = 6'h0;
  logic [5:0] p_adp_snt = 6'h0;
  always @(posedge clk) if (rst_n) begin
    if (adp_sb_rcv !== 6'h0 && adp_sb_rcv !== p_adp_rcv)
      ev($sformatf("ucie.adapter  <- %s  (linkInit %s)",
                   adp_sb_name(adp_sb_rcv), adp_init_name(adp_init_state)));
    if (adp_sb_snt !== 6'h0 && adp_sb_snt !== p_adp_snt)
      ev($sformatf("ucie.adapter  -> %s  (linkInit %s, sideband rdy=%0b)",
                   adp_sb_name(adp_sb_snt), adp_init_name(adp_init_state),
                   adp_sb_rdy));
    p_adp_rcv <= adp_sb_rcv;
    p_adp_snt <= adp_sb_snt;
  end


  // ---------------------------------------------------------------------
  // Stall detection, to stop as soon as nothing is advancing
  // ---------------------------------------------------------------------
  // Finishes early once every progress signal has been still for a while.
  // Retry traffic does not count, since the exchanger re-sends forever while the link
  // sits in one substate. Disable with +stall_cycles=0.
  int unsigned stall_limit = 300000;

  wire [127:0] progress_sig = {
    mad_ltsm, mad_rdi_state, mad_fdi_state, mad_trainerror, mad_inband_pres,
    ucie_ltstate, ucie_ltsm_detail, ucie_rdi_state,
    mbi_req_state, mbi_req_sub, mbi_rsp_state, mbi_rsp_sub,
    sbi_state, sbi_detect_cnt, sbi_four_cnt,
    n_ucie_sb_words[31:0], n_enq_offer[31:0]
  };

  logic [127:0] p_progress_sig = 128'h0;
  longint unsigned stall_count = 0;
  logic stall_reported = 1'b0;

  initial void'($value$plusargs("stall_cycles=%d", stall_limit));

  always @(posedge clk) begin
    if (!rst_n) begin
      stall_count    <= 0;
      p_progress_sig <= 128'h0;
    end else if (progress_sig !== p_progress_sig) begin
      p_progress_sig <= progress_sig;
      stall_count    <= 0;
    end else if (stall_limit != 0 && !stall_reported) begin
      if (stall_count >= stall_limit) begin
        stall_reported <= 1'b1;
        ev($sformatf("STALLED: no progress for %0d cycles. madsim ltsm=%s, ucieTL ltState=%s",
                     stall_limit, mad_ltsm_name(mad_ltsm), ltstate_name(ucie_ltstate)));
        $display("cosim: STALLED at cycle %0d, no progress for %0d cycles -- finishing early",
                 cyc, stall_limit);
        $finish;
      end else begin
        stall_count <= stall_count + 1;
      end
    end
  end

  // ---------------------------------------------------------------------
  // Data plane. Counts flits at four boundaries per die, so one that goes
  // missing names the stage it stopped at.
  //   FDI lp  protocol -> adapter    RDI lp  adapter -> logical PHY
  //   RDI pl  logical PHY -> adapter FDI pl  adapter -> protocol
  // ---------------------------------------------------------------------
  int unsigned n_ucie_fdi_tx = 0, n_ucie_rdi_tx = 0;
  int unsigned n_ucie_rdi_rx = 0, n_ucie_fdi_rx = 0;
  int unsigned n_mad_fdi_tx  = 0, n_mad_rdi_tx  = 0;
  int unsigned n_mad_rdi_rx  = 0, n_mad_fdi_rx  = 0;

  logic [511:0] first_ucie_fdi_tx, last_ucie_fdi_tx;
  logic [511:0] first_mad_fdi_rx,  last_mad_fdi_rx;
  logic [511:0] first_mad_fdi_tx,  last_mad_fdi_tx;
  logic [511:0] first_ucie_fdi_rx, last_ucie_fdi_rx;
  int unsigned  cyc_ucie_fdi_tx = 0, cyc_mad_fdi_rx = 0;
  int unsigned  cyc_mad_fdi_tx  = 0, cyc_ucie_fdi_rx = 0;

  always @(posedge clk) if (rst_n) begin
    if (ucie_fdi_tx_valid) begin
      if (n_ucie_fdi_tx == 0) begin
        first_ucie_fdi_tx <= ucie_fdi_tx_data;
        cyc_ucie_fdi_tx   <= cyc;
        milestone("ucieTL", "started sending data");
      end
      n_ucie_fdi_tx <= n_ucie_fdi_tx + 1;
      last_ucie_fdi_tx <= ucie_fdi_tx_data;
    end
    if (ucie_rdi_tx_valid) n_ucie_rdi_tx <= n_ucie_rdi_tx + 1;
    if (ucie_rdi_rx_valid) begin
      if (n_ucie_rdi_rx < 6)
        evv($sformatf("ucie rdi rx flit %0d: 0x%0128h",
                     n_ucie_rdi_rx, ucie_rdi_rx_data));
      n_ucie_rdi_rx <= n_ucie_rdi_rx + 1;
    end
    if (ucie_fdi_rx_valid) begin
      if (n_ucie_fdi_rx == 0) begin
        first_ucie_fdi_rx <= ucie_fdi_rx_data;
        cyc_ucie_fdi_rx   <= cyc;
        milestone("ucieTL", "started receiving data");
      end
      n_ucie_fdi_rx <= n_ucie_fdi_rx + 1;
      last_ucie_fdi_rx <= ucie_fdi_rx_data;
    end

    if (mad_fdi_lp_valid) begin
      if (n_mad_fdi_tx == 0) begin
        first_mad_fdi_tx <= mad_fdi_lp_data;
        cyc_mad_fdi_tx   <= cyc;
        milestone("madsim", "started sending data");
      end
      n_mad_fdi_tx <= n_mad_fdi_tx + 1;
      last_mad_fdi_tx <= mad_fdi_lp_data;
    end
    if (mad_rdi_lp_valid) n_mad_rdi_tx <= n_mad_rdi_tx + 1;
    if (mad_rdi_pl_valid) n_mad_rdi_rx <= n_mad_rdi_rx + 1;
    if (mad_fdi_pl_valid) begin
      if (n_mad_fdi_rx == 0) begin
        first_mad_fdi_rx <= mad_fdi_pl_data;
        cyc_mad_fdi_rx   <= cyc;
        milestone("madsim", "started receiving data");
      end
      n_mad_fdi_rx <= n_mad_fdi_rx + 1;
      last_mad_fdi_rx <= mad_fdi_pl_data;
    end
  end

  // A valid-lane word that is not the framing pattern latches stickyError,
  // which stalls RDI and suppresses pl_valid for the rest of the run. The word
  // itself is logged alongside the error.
  logic p_sticky = 1'b0;
  logic p_rx_aligned = 1'b0;
  logic [15:0] p_rx_offset_changes = 16'd0;
  logic [31:0] p_raw_valid = 32'h0;
  int unsigned n_raw_valid_logged = 0;
  logic [1:0] p_stall_req_state = 2'd0;
  int unsigned n_mb_rx_words = 0;
  int unsigned n_mb_rx_nonidle = 0;
  always @(posedge clk) if (rst_n) begin
    if (ucie_mb_rx_word_valid) begin
      if (n_mb_rx_words < 4)
        evv($sformatf("ucie mainband rx word %0d: valid lane 0x%08h (framing pattern is 0x0f0f0f0f)",
                     n_mb_rx_words, ucie_mb_rx_valid_word));
      n_mb_rx_words <= n_mb_rx_words + 1;
      // The interesting words are the ones where the partner asserted the
      // valid lane at all. An idle link gives 0x00000000 for thousands of
      // words, so log the first few non-idle ones instead of the first few.
      if (ucie_mb_rx_valid_word != 32'h0 && n_mb_rx_nonidle < 8) begin
        evv($sformatf("ucie mainband rx NON-IDLE valid 0x%08h  lane0 raw 0x%08h  lfsr 0x%08h  descrambled 0x%08h",
                     ucie_mb_rx_valid_word, ucie_mb_rx_raw_lane0,
                     ucie_mb_descrambler_lane0, ucie_mb_rx_data_lane0));
        n_mb_rx_nonidle <= n_mb_rx_nonidle + 1;
      end
    end
    // Raw and aligned valid word side by side, for the first stretch of
    // non-idle traffic. A framing error can only be read against the word the
    // aligner was given and the offset it applied.
    if (ucie_mb_rx_raw_word_valid && n_raw_valid_logged < 40 &&
        (ucie_mb_rx_raw_valid != 32'h0 || p_raw_valid != 32'h0)) begin
      evv($sformatf("ucie mb rx word: raw valid 0x%08h -> aligned 0x%08h (offset %0d, rearms %0d)",
                   ucie_mb_rx_raw_valid, ucie_mb_rx_valid_word,
                   ucie_rx_align_offset, ucie_rx_rearms));
      n_raw_valid_logged <= n_raw_valid_logged + 1;
    end
    if (ucie_mb_rx_raw_word_valid) p_raw_valid <= ucie_mb_rx_raw_valid;

    if (ucie_rx_offset_changes !== p_rx_offset_changes)
      ev($sformatf("ucie mainband rx: word boundary moved to %0d UI (change %0d of %0d measurements)",
                   ucie_rx_align_offset, ucie_rx_offset_changes, ucie_rx_rearms));
    if (ucie_rx_aligned && !p_rx_aligned)
      ev($sformatf("ucie mainband rx: locked to the partner's word boundary, %0d UI into this die's word",
                   ucie_rx_align_offset));
    if (ucie_mb_sticky_error && !p_sticky)
      ev($sformatf("ucie mainband FRAMING ERROR latched: valid lane 0x%08h, expected 0x0f0f0f0f -- RDI stall and pl_valid suppression are now permanent",
                   ucie_mb_rx_valid_word));
    if (ucie_stall_req_state !== p_stall_req_state)
      evv($sformatf("ucie rdi stallRequester %0d -> %0d (0=IDLE 1=WAIT_ACK 2=STALLED 3=WAIT_DEASSERT)",
                   p_stall_req_state, ucie_stall_req_state));
    p_sticky <= ucie_mb_sticky_error;
    p_rx_aligned <= ucie_rx_aligned;
    p_rx_offset_changes <= ucie_rx_offset_changes;
    p_stall_req_state <= ucie_stall_req_state;
  end

  // ---------------------------------------------------------------------
  // Payload checking at RDI and FDI, the two boundaries where a flit means the
  // same thing on both dies. Above FDI the stacks diverge, with UcieTL framing
  // TileLink, madsim frames AXI.
  // ---------------------------------------------------------------------
  int unsigned n_mad2ucie_ok = 0, n_mad2ucie_bad = 0;

  // ucie -> madsim has no per-flit counter, so it is checked as a stream, in
  // order and bit exact. Recorded on the FDI transfer (lpValid && plTrdy),
  // since one flit sits on lpValid for many cycles.
  // Payloads logged per direction, from variables.mk. -1 logs all of them.
`ifndef COSIM_SHOW_FLITS
  `define COSIM_SHOW_FLITS 20
`endif
  localparam int SHOW_FLITS = `COSIM_SHOW_FLITS;
  localparam bit SHOW_ALL   = (SHOW_FLITS < 0);

  // Field positions in ucie -> madsim flits, from Cat(UcieTXA, 0.U). Display
  // only. The check is the full 512-bit compare.
  //
  // Note there is no per-message marker at this boundary. The framer offers a
  // beat every cycle from the held manager-port wires, and its tl_valid bit is
  // managerTl.a.fire, a pulse that has dropped by the time the flit reaches
  // FDI. So the flit counts below are transport volume, not message counts --
  // the beat count comes from the driver, which reports it itself.
  localparam int TL_ADDR_HI = 361, TL_ADDR_LO = 298;
  localparam int TL_DATA_HI = 265, TL_DATA_LO = 10;
  logic [255:0] shown_data = 256'd0;
  int unsigned  n_shown = 0;

  localparam int TXQ = 1024;
  logic [511:0] txq [TXQ];
  int unsigned txq_wr = 0, txq_rd = 0;
  bit          txq_synced = 0;
  int unsigned n_ucie2mad_ok = 0, n_ucie2mad_bad = 0, n_txq_lapped = 0;

  always @(posedge clk) if (rst_n) begin
    // One entry per flit the adapter actually took from the protocol layer.
    if (ucie_fdi_tx_valid && ucie_fdi_pl_trdy) begin
      txq[txq_wr % TXQ] <= ucie_fdi_tx_data;
      txq_wr <= txq_wr + 1;
    end

    if (mad_fdi_pl_valid) begin
      if (!txq_synced) begin
        // Align the streams on the first flit; order is exact from there on.
        for (int i = 0; i < TXQ; i++) begin
          if (i < txq_wr && txq[i % TXQ] === mad_fdi_pl_data) begin
            txq_rd <= i + 1;
            txq_synced <= 1'b1;
            n_ucie2mad_ok <= n_ucie2mad_ok + 1;
            ev($sformatf("ucie -> madsim : stream aligned at flit %0d, checking in order",
                         i));
            break;
          end
        end
      end else begin
        if (txq[txq_rd % TXQ] === mad_fdi_pl_data) begin
          // Only the beats that carry a TileLink message are worth showing.
          // The same beat is framed onto the wire more than once, so show a
          // payload only when it changes.
          if (mad_fdi_pl_data[TL_DATA_HI:TL_DATA_LO] !== shown_data
              && mad_fdi_pl_data[TL_ADDR_HI:TL_ADDR_LO] != 0) begin
            shown_data <= mad_fdi_pl_data[TL_DATA_HI:TL_DATA_LO];
            if (SHOW_ALL || n_shown < SHOW_FLITS) begin
              n_shown <= n_shown + 1;
              ev($sformatf("ucie -> madsim ok: addr=%04h data=%064h",
                           mad_fdi_pl_data[TL_ADDR_LO+15 -: 16],
                           mad_fdi_pl_data[TL_DATA_HI:TL_DATA_LO]));
              if (!SHOW_ALL && n_shown == SHOW_FLITS - 1)
                ev($sformatf("ucie -> madsim : first %0d shown, the rest are checked but not logged",
                             SHOW_FLITS));
            end
          end
          n_ucie2mad_ok <= n_ucie2mad_ok + 1;
        end else begin
          if (n_ucie2mad_bad < 4)
            ev($sformatf("ucie -> madsim flit %0d WRONG: got 0x%0128h, expected 0x%0128h",
                         txq_rd, mad_fdi_pl_data, txq[txq_rd % TXQ]));
          n_ucie2mad_bad <= n_ucie2mad_bad + 1;
        end
        txq_rd <= txq_rd + 1;
      end
    end
  end

  // madsim -> ucie, same stream check in the other direction. The payload is
  // random, so the expectation is what madsim actually handed down rather than
  // anything derived from a field of the received flit.
  localparam int RXQ = 1024;
  logic [511:0] rxq [RXQ];
  int unsigned rxq_wr = 0, rxq_rd = 0;
  bit          rxq_synced = 0;
  int unsigned n_mad2ucie_ok = 0, n_mad2ucie_bad = 0, n_rxq_lapped = 0;

  always @(posedge clk) if (rst_n) begin
    // One entry per flit the adapter actually took, not per cycle lp_valid is
    // held. Without the trdy qualifier the same flit is recorded many times.
    if (mad_fdi_lp_valid && mad_fdi_pl_trdy) begin
      rxq[rxq_wr % RXQ] <= mad_fdi_lp_data;
      rxq_wr <= rxq_wr + 1;
      if (rxq_synced && (rxq_wr - rxq_rd) >= RXQ) n_rxq_lapped <= n_rxq_lapped + 1;
    end

    if (ucie_fdi_rx_valid) begin
      if (!rxq_synced) begin
        for (int i = 0; i < RXQ; i++) begin
          if (i < rxq_wr && rxq[i % RXQ] === ucie_fdi_rx_data) begin
            rxq_rd <= i + 1;
            rxq_synced <= 1'b1;
            n_mad2ucie_ok <= n_mad2ucie_ok + 1;
            ev($sformatf("madsim -> ucie : stream aligned at flit %0d, checking in order",
                         i));
            break;
          end
        end
      end else begin
        if (rxq[rxq_rd % RXQ] === ucie_fdi_rx_data) begin
          // BuildRawFlit fields, shown to make the payload readable. The check
          // itself is the full 512-bit compare above.
          if (SHOW_ALL || n_mad2ucie_ok < SHOW_FLITS) begin
            ev($sformatf("madsim -> ucie flit %0d ok: addr=%08h data=%016h",
                         rxq_rd, ucie_fdi_rx_data[39:8], ucie_fdi_rx_data[103:40]));
            if (!SHOW_ALL && n_mad2ucie_ok == SHOW_FLITS - 1)
              ev($sformatf("madsim -> ucie : first %0d shown, the rest are checked but not logged",
                           SHOW_FLITS));
          end
          n_mad2ucie_ok <= n_mad2ucie_ok + 1;
        end else begin
          if (n_mad2ucie_bad < 4)
            ev($sformatf("madsim -> ucie flit %0d WRONG: got 0x%0128h, expected 0x%0128h",
                         rxq_rd, ucie_fdi_rx_data, rxq[rxq_rd % RXQ]));
          n_mad2ucie_bad <= n_mad2ucie_bad + 1;
        end
        rxq_rd <= rxq_rd + 1;
      end
    end
  end

  task automatic print_data_plane();
    $fdisplay(fh, "  ucie -> madsim : %0d flits sent, %0d checked in order, %0d wrong",
              txq_wr, n_ucie2mad_ok, n_ucie2mad_bad);
    $fdisplay(fh, "  madsim -> ucie : %0d flits sent, %0d checked in order, %0d wrong",
              rxq_wr, n_mad2ucie_ok, n_mad2ucie_bad);
    if (n_txq_lapped != 0 || n_rxq_lapped != 0)
      $fdisplay(fh, "  WARNING: a flit queue lapped (%0d / %0d), so these counts are not trustworthy",
                n_txq_lapped, n_rxq_lapped);
  endtask

  // ---------------------------------------------------------------------
  // End of run, where each die stopped, then the payload check.
  // ---------------------------------------------------------------------
  final begin
    $fdisplay(fh, "");
    $fdisplay(fh, "---- end of run, cycle %0d ----", cyc);
    print_data_plane();
  end

endmodule
