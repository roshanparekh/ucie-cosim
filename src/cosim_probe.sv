// cosim_probe.sv
//
// Wires cosim_monitor into the co-simulation. Everything the monitor watches is
// listed here as a `bind`, so probes can be added or dropped without touching
// either die's RTL.
//
// A bind's port expressions elaborate in the target module's scope, so the
// hierarchical paths below resolve as if written inside ucie_cosim_top. madsim
// signals come through the wrapper's ports instead, since a Verilog
// hierarchical reference cannot reach into the SystemC kernel.

bind ucie_cosim_top cosim_monitor u_mon (
  .clk   (clk),
  .rst_n (rst_n),

  // madsim, through the wrapper's observation ports.
  .mad_ltsm        (mad_ltsm_state),
  .mad_rdi_state   (mad_rdi_state),
  .mad_fdi_state   (mad_fdi_state),
  .mad_fdi_lp_data (mad_fdi_lp_data),
  .mad_fdi_lp_valid(mad_fdi_lp_valid),
  .mad_fdi_pl_trdy (mad_fdi_pl_trdy),
  .mad_fdi_pl_data (mad_fdi_pl_data),
  .mad_fdi_pl_valid(mad_fdi_pl_valid),
  .mad_rdi_lp_valid(mad_rdi_lp_valid),
  .mad_rdi_pl_valid(mad_rdi_pl_valid),
  .ucie_fdi_pl_trdy       (u_ucie.ucieDigitalLazy_d2dAdapter.io_fdi_plTrdy),
  .ucie_mb_rx_word_valid  (u_ucie.ucieDigitalLazy_logicalPhy.mainbandLaneController.io_mbLanes_rx_valid),
  .ucie_mb_rx_valid_word  (u_ucie.ucieDigitalLazy_logicalPhy.mainbandLaneController.io_mbLanes_rx_bits_valid),
  .ucie_mb_rx_data_lane0  (u_ucie.ucieDigitalLazy_logicalPhy.mainbandLaneController.io_mbLanes_rx_bits_data_0),
  .ucie_mb_rx_raw_lane0   (u_ucie.ucieDigitalLazy_logicalPhy.io_analog_mainband_rx_bits_data_0),
  .ucie_mb_descrambler_lane0 (u_ucie.ucieDigitalLazy_logicalPhy._descrambler_io_lfsrOutput_0),
  .ucie_rx_aligned       (u_ucie.ucieDigitalLazy_logicalPhy.rxAligner.aligned),
  .ucie_rx_align_offset  (u_ucie.ucieDigitalLazy_logicalPhy.rxAligner.alignOffset),
  .ucie_rx_rearms        (u_ucie.ucieDigitalLazy_logicalPhy.rxAligner.rearmCount),
  .ucie_rx_offset_changes(u_ucie.ucieDigitalLazy_logicalPhy.rxAligner.offsetChangeCount),
  .ucie_mb_rx_raw_valid  (u_ucie.ucieDigitalLazy_logicalPhy.io_analog_mainband_rx_bits_valid),
  .ucie_mb_rx_raw_word_valid (u_ucie.ucieDigitalLazy_logicalPhy.rxAligner.io_in_valid),
  .ucie_mb_sticky_error   (u_ucie.ucieDigitalLazy_logicalPhy.mainbandLaneController.stickyError),
  .ucie_stall_req_state   (u_ucie.ucieDigitalLazy_logicalPhy.rdiController.stallRequester.currentState),
  .mad_trainerror  (mad_trainerror),
  .mad_inband_pres (mad_inband_pres),

  // UcieTL, by hierarchical reference. No ports added to the generated RTL; the
  // paths fail the build if the hierarchy changes.
  .ucie_ltstate        (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.currentState),
  .ucie_ltsm_detail    (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.io_currentState),
  .ucie_sb_pat_cnt     (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitPatternCounter),
  .ucie_reset_min_wait (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.resetMinWait),
  .ucie_timeout_cnt    (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.timeoutCounter),
  .ucie_sb_rx_valid    (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.io_sbLaneIo_rx_valid),
  .ucie_sb_rx_word     (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.io_sbLaneIo_rx_bits_data[63:0]),
  .ucie_rdi_state      (ucie_rdi_state),
  // FDI sits between the protocol layer and the adapter, RDI between the
  // adapter and the logical PHY. Both carry the same 512-bit flit.
  // The adapter's link-init handshake. FDI reaches ACTIVE only after ADV_CAP
  // both ways, then REQ_ACTIVE / RSP_ACTIVE both ways.
  .adp_init_state  (u_ucie.ucieDigitalLazy_d2dAdapter.linkManager.linkInitStateReg),
  .adp_cap_snt     (u_ucie.ucieDigitalLazy_d2dAdapter.linkManager.paramExchSbMsgSntFlag),
  .adp_cap_rcv     (u_ucie.ucieDigitalLazy_d2dAdapter.linkManager.paramExchSbMsgRcvFlag),
  .adp_req_rcv     (u_ucie.ucieDigitalLazy_d2dAdapter.linkManager.activeSbMsgReqRcvFlag),
  .adp_rsp_rcv     (u_ucie.ucieDigitalLazy_d2dAdapter.linkManager.activeSbMsgRspRcvFlag),
  .adp_req_snt     (u_ucie.ucieDigitalLazy_d2dAdapter.linkManager.activeSbMsgExtReqReg),
  .adp_rsp_snt     (u_ucie.ucieDigitalLazy_d2dAdapter.linkManager.activeSbMsgExtRspReg),
  // Decoded adapter-level messages, off D2DSidebandModule. The LTSM tap cannot
  // see these, because the switch routes D2D-addressed messages to the adapter.
  .adp_sb_rcv      (u_ucie.ucieDigitalLazy_d2dAdapter._d2dSideband_io_sb_rcv),
  .adp_sb_snt      (u_ucie.ucieDigitalLazy_d2dAdapter._linkManager_io_sb_snd),
  .adp_sb_rdy      (u_ucie.ucieDigitalLazy_d2dAdapter._d2dSideband_io_sb_rdy),
  .ucie_fdi_sts        (u_ucie._ucieDigitalLazy_d2dAdapter_io_fdi_plStateSts),
  .ucie_fdi_tx_valid   (u_ucie._ucieDigitalLazy_protocolLayer_io_fdi_lpValid),
  .ucie_fdi_tx_data    (u_ucie._ucieDigitalLazy_protocolLayer_io_fdi_lpData),
  .ucie_fdi_rx_valid   (u_ucie._ucieDigitalLazy_d2dAdapter_io_fdi_plValid),
  .ucie_fdi_rx_data    (u_ucie._ucieDigitalLazy_d2dAdapter_io_fdi_plData),
  .ucie_rdi_tx_valid   (u_ucie._ucieDigitalLazy_d2dAdapter_io_rdi_lpValid),
  .ucie_rdi_rx_valid   (u_ucie._ucieDigitalLazy_logicalPhy_io_rdi_plValid),
  .ucie_rdi_rx_data    (u_ucie._ucieDigitalLazy_logicalPhy_io_rdi_plData),

  // The bumps themselves, to separate nothing sent from sent and misread.
  .a2b_sb_val      (a2b_sb_val),
  .a2b_sb_data     (a2b_sb_data),
  .ucie_sb_rx_clk    (ucie_sb_rx_clk),
  .ucie_sb_rx_data_i (ucie_sb_rx_data),

  // Stages 2 to 4 walk the receive path inward, so a failure points at one
  // stage. The bit assembler in the recovered clock domain, then after the
  // async FIFO into the digital domain, then after parity and the opcode queue.
  .des_bit_cnt   (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.fwBitCounter),
  .des_enq_valid (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.rxQueue_io_enq_valid),
  .des_enq_ready (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.rxQueue_io_enq_ready),

  .des_raw_valid (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.io_out_valid),
  .des_raw_word  (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.io_out_bits[63:0]),

  .des_out_valid (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.io_rxOut_valid),
  .des_out_word  (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.io_rxOut_bits[63:0]),
  .b2a_sb_val      (b2a_sb_val),
  .b2a_sb_data     (b2a_sb_data),

  // UcieTL's SBINIT sub-FSM, since the outer ltState only reports "SBINIT":
  // trading clock patterns, waiting on Out Of Reset, or waiting on Done.
  // maxBits comes from the opcode at bit 5, so a misframed word picks the
  // wrong packet length and compounds the error.
  .des_max_bits  (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.maxBits),
  .des_reset     (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.reset),
  .des_fwclk     (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.io_in_fwClock),
  .des_reframe   (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.accumHold),
  .des_quiet     (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.fwQuietCycles),
  .des_rxmode    (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.deserializer.io_ctrl_rxMode),

  // Mainband bumps, straight off the harness nets.
  // madsim's own view of mainband pattern detection. clock_pass is what it
  // reports as msgInfo[2:0] of the REPAIRCLK result.
  .mad_mb_clock_pass     (mad_mb_clock_pass),
  .mad_mb_rx_done        (mad_mb_rx_done),
  .mad_mb_rx_error       (mad_mb_rx_error),
  .mad_mb_rx_enable      (mad_mb_rx_enable),
  .mad_mb_tx_enable      (mad_mb_tx_enable),
  .mad_mb_pattern_select (mad_mb_pattern_select),
  .mad_sb_ltsm_tx_msg   (mad_sb_ltsm_tx_msg),
  .mad_sb_ltsm_tx_valid (mad_sb_ltsm_tx_valid),
  .mad_sb_ltsm_tx_ready (mad_sb_ltsm_tx_ready),
  .mad_mb_rx_chunk_ckp   (mad_mb_rx_chunk_ckp),
  .mad_mb_rx_chunk_trk   (mad_mb_rx_chunk_trk),
  .mad_mb_rx_chunk_valid (mad_mb_rx_chunk_valid),
  .mad_mb_rx_chunk_ckn   (mad_mb_rx_chunk_ckn),
  .mad_mb_rx_chunk_vld   (mad_mb_rx_chunk_vld),

  // The word leaving ucieDigital, and what the PHY sees after the controllerSel
  // mux and txTestFifo.
  // UcieTL's own detection result, and the received track word it compares.
  // Why a lane passes or fails. patternCounterReg must reach errorThresholdReg
  // (16) with iterDirtyReg clean, and counterEn/detectPipeValid gate counting.
  // The RX D2C point test runs as a requester and a responder. Both dies run
  // both.
  .rxpt_req_state   (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.rxPtTestRequester.currentState),
  .rxpt_rsp_state   (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.rxPtTestResponder.currentState),
  .pr_state         (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.state),
  .pr_rx_valid      (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.io_mbRxValid),
  .pr_cnt0          (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.patternCounterReg_0),
  .pr_cnt1          (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.patternCounterReg_1),
  .pr_cnt2          (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.patternCounterReg_2),
  .pr_dirty2        (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.iterDirtyReg_2),
  .pr_target        (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.errorThresholdReg),
  .pr_consec        (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.doConsecutiveCountReg),
  .pr_ptype         (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.patternTypeReg),
  .pr_counter_en    (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.counterEn),
  .pr_resp_valid    (u_ucie.ucieDigitalLazy_logicalPhy._patternReader_io_interfaceIo_resp_valid),
  .pr_aggregate     (u_ucie.ucieDigitalLazy_logicalPhy._patternReader_io_interfaceIo_resp_bits_aggregateStatus),
  .pr_lane0         (u_ucie.ucieDigitalLazy_logicalPhy._patternReader_io_interfaceIo_resp_bits_perLaneStatusBits_0),
  .pr_lane1         (u_ucie.ucieDigitalLazy_logicalPhy._patternReader_io_interfaceIo_resp_bits_perLaneStatusBits_1),
  .pr_lane2         (u_ucie.ucieDigitalLazy_logicalPhy._patternReader_io_interfaceIo_resp_bits_perLaneStatusBits_2),
  .ucie_mb_rx_valid (u_ucie.ucieDigitalLazy_logicalPhy.io_analog_mainband_rx_valid),
  .ucie_mb_rx_trk   (u_ucie.ucieDigitalLazy_logicalPhy.io_analog_mainband_rx_bits_trk),
  .ucie_mb_rx_vld   (u_ucie.ucieDigitalLazy_logicalPhy.io_analog_mainband_rx_bits_valid),
  .ucie_mb_rx_d0    (u_ucie.ucieDigitalLazy_logicalPhy.io_analog_mainband_rx_bits_data_0),
  .ucie_mb_rx_d1    (u_ucie.ucieDigitalLazy_logicalPhy.io_analog_mainband_rx_bits_data_1),

  // PatternWriter's burst counter, the serializer words actually emitted, which
  // is what the partner's detect window sees.
  // What the TX point-test requester latched, and what MBTRAIN is offering it.
  // Both sides of the lane 0 LFSR comparison, word received and reference.
  .rx_lfsr_ref0    (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.io_rxLfsrCtrl_pattern_0),
  .rx_lane_d0      (u_ucie.ucieDigitalLazy_logicalPhy.patternReader.io_mbRxLaneIo_data_0),
  .txpt_req_ptype  (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.txPtTestRequester.patternTypeReg),
  .txpt_req_state  (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.txPtTestRequester.currentState),
  .mbt_txpt_ptype  (u_ucie.ucieDigitalLazy_logicalPhy.ltsm._mbTrainSM_io_txPtTestReqIntfIo_patternType),
  .mbt_txpt_start  (u_ucie.ucieDigitalLazy_logicalPhy.ltsm._mbTrainSM_io_txPtTestReqIntfIo_start),
  .pw_cycle_count  (u_ucie.ucieDigitalLazy_logicalPhy.patternWriter.cycleCount),
  .pw_in_progress  (u_ucie.ucieDigitalLazy_logicalPhy.patternWriter.inProgress),
  .pw_pattern_type (u_ucie.ucieDigitalLazy_logicalPhy.patternWriter.patternTypeReg),
  .dig_mb_tx_valid (u_ucie._ucieDigitalLazy_logicalPhy_io_analog_mainband_tx_valid),
  .dig_mb_tx_clkp  (u_ucie._ucieDigitalLazy_logicalPhy_io_analog_mainband_tx_bits_clkP),
  .dig_mb_tx_clkn  (u_ucie._ucieDigitalLazy_logicalPhy_io_analog_mainband_tx_bits_clkN),
  .dig_mb_tx_trk   (u_ucie._ucieDigitalLazy_logicalPhy_io_analog_mainband_tx_bits_trk),
  .dig_mb_tx_data0 (u_ucie._ucieDigitalLazy_logicalPhy_io_analog_mainband_tx_bits_data_0),
  .phy_mb_tx_valid (u_ucie._txTestFifo_io_deq_valid),
  .phy_mb_tx_clkp  (u_ucie._txTestFifo_io_deq_bits_clkp),

  .a2b_mb_data (a2b_mb_data), .a2b_mb_vld (a2b_mb_vld),
  .a2b_mb_ckp  (a2b_mb_ckp),  .a2b_mb_ckn (a2b_mb_ckn), .a2b_mb_trk (a2b_mb_trk),
  .b2a_mb_data (b2a_mb_data), .b2a_mb_vld (b2a_mb_vld),
  .b2a_mb_ckp  (b2a_mb_ckp),  .b2a_mb_ckn (b2a_mb_ckn), .b2a_mb_trk (b2a_mb_trk),

  .mbi_req_state     (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.mbInitSM.requester.currentState),
  .mbi_rsp_state     (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.mbInitSM.responder.currentState),
  .mbi_req_sub       (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.mbInitSM.requester.substateReg),
  .mbi_rsp_sub       (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.mbInitSM.responder.substateReg),
  .mbi_cal_done_reg  (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.mbInitSM.requester.mbInitCalDoneReg),
  .mbi_cal_done_in   (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.mbInitSM.requester.io_mbInitCalDone),
  .plt_selfcal_start (u_ucie.ucieDigitalLazy_logicalPhy.phyLaneTrainer.io_phyTrainIo_mbInit_selfCalStart),
  .plt_selfcal_done  (u_ucie.ucieDigitalLazy_logicalPhy.phyLaneTrainer.io_phyTrainIo_mbInit_selfCalDone),

  .sbi_state        (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.requester.currentState),
  .sbi_detect_cnt   (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.requester.detectPatternCounter),
  .sbi_four_cnt     (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.requester.fourPatternCounter),
  .sbi_req_msg_sent (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.requester.msgSent),
  .sbi_req_msg_recv (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.requester.msgReceived),
  .sbi_rsp_msg_sent (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.responder.msgSent),
  .sbi_rsp_msg_recv (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.responder.msgReceived),

  // fourPatternCounter is gated on tx_ready, so without ready the sub-FSM stays
  // in sPATTERN regardless of the receive path.
  .req_tx_valid (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.requester.io_sbLaneIo_tx_valid),
  .req_tx_ready (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.requester.io_sbLaneIo_tx_ready),
  .req_tx_data  (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.sbInitSM.requester.io_sbLaneIo_tx_bits_data[63:0]),

  .ln_tx_valid  (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.io_txIn_valid),
  .ln_tx_ready  (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.io_txIn_ready),
  .ln_tx_data   (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.io_txIn_bits[63:0]),

  .ser_in_valid (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.serializer.io_in_valid),
  .ser_in_ready (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.serializer.io_in_ready),
  .ser_in_data  (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.serializer.io_in_bits[63:0]),

  // The full transmit chain, requester -> arbiter -> skid -> switch -> link
  // node -> serializer -> bump. The switch routes on the header, not a port.
  // currToLower is bits(58) alone, so a word with bit 58 clear is steered
  // upward, and a block there backpressures everything behind it.
  .lt_tx_valid   (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.io_sbLaneIo_tx_valid),
  .lt_tx_ready   (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.io_sbLaneIo_tx_ready),
  .lt_tx_data    (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.io_sbLaneIo_tx_bits_data[63:0]),
  .lt_arb_chosen (u_ucie.ucieDigitalLazy_logicalPhy.ltsm.txArbiter.io_chosen_choice),

  .lib_bypass    (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.layerInBuffer.bypassReg),
  .lib_data      (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.layerInBuffer.dataReg[63:0]),

  .sw_curr_valid (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.switch.io_currLayer_from_valid),
  .sw_curr_ready (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.switch.io_currLayer_from_ready),
  .sw_curr_data  (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.switch.io_currLayer_from_bits[63:0]),
  .sw_upper_valid(u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.switch.io_upperLayer_from_valid),
  .sw_lower_valid(u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.switch.io_lowerLayer_to_valid),

  .lnskid_bypass (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.linkNode.skidBuffer.bypassReg),

  // Tracing an X in the switch header to its source. The LTSM is one of two
  // producers, merged with the RDI controller by a 2-way arbiter and a queue.
  .rdic_tx_valid  (u_ucie.ucieDigitalLazy_logicalPhy.rdiController.io_sbLaneIo_tx_valid),
  .rdic_tx_data   (u_ucie.ucieDigitalLazy_logicalPhy.rdiController.io_sbLaneIo_tx_bits_data[63:0]),
  .stxa_out_valid (u_ucie.ucieDigitalLazy_logicalPhy.sidebandTxArbiter.io_out_valid),
  .stxa_out_data  (u_ucie.ucieDigitalLazy_logicalPhy.sidebandTxArbiter.io_out_bits[63:0]),
  .stxq_deq_valid (u_ucie.ucieDigitalLazy_logicalPhy.sidebandTxQueue.io_deq_valid),
  .stxq_deq_data  (u_ucie.ucieDigitalLazy_logicalPhy.sidebandTxQueue.io_deq_bits[63:0]),
  .lyr_in_valid   (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.io_layer_in_valid),
  .lyr_in_ready   (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.io_layer_in_ready),
  .lyr_in_data    (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.io_layer_in_bits[63:0]),

  // The RDI ingress into the switch's upper port. LogPhySidebandChannel.scala:141
  // wires rdiIntfNode.rxOut straight to switch.upperLayer.from, so an X here
  // lands directly in the arbiter that also serves the link training traffic.
  .rdin_rxout_valid (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.rdiIntfNode.io_rxOut_valid),
  .rdi_in_valid     (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.io_rdi_in_valid),
  .lib_out_valid    (u_ucie.ucieDigitalLazy_logicalPhy.logPhySidebandChannel.layerInBuffer.io_out_valid)
);
