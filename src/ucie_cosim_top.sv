// ucie_cosim_top.sv
//
// Two UCIe dies facing each other across the bump interface.
//
//     madsim_die (SystemC, via VCS syscan)  <-->  UcieTL (elaborated RTL)
//
// madsim is die A and self-starts out of reset. UcieTL is die B and wakes on
// the sideband pattern die A sends, the same order UcieTL's own two-die
// testbench uses. Neither side needs a software trigger to start training.
//
// Both dies run from one shared clock. madsim has no fixed frequency ratios
// and crosses its domains through async FIFOs, and UcieTL clocks its sideband
// and mainband from a single ucieClk, so one clock is what gives the sideband
// a defined phase relationship.
//
// This file connects the dies, drives TileLink traffic into UcieTL and brings
// the interesting buses out as named nets. Checking happens in cosim_monitor,
// bound in by cosim_probe.

`timescale 1ps/1ps

// Set by the Makefile from variables.mk; the fallbacks are for a standalone
// compile. 0 parallel, 1 ucie_first, 2 madsim_first.
`ifndef COSIM_UCIE_TX_MSGS
  `define COSIM_UCIE_TX_MSGS 2000
`endif
`ifndef COSIM_MADSIM_TX_MSGS
  `define COSIM_MADSIM_TX_MSGS 2000
`endif
`ifndef COSIM_TRAFFIC_MODE
  `define COSIM_TRAFFIC_MODE 0
`endif
`ifndef COSIM_RANDOM_SEED
  `define COSIM_RANDOM_SEED 1
`endif

module ucie_cosim_top;

  // ---------------------------------------------------------------------
  // Clock and reset
  // ---------------------------------------------------------------------
  // 800 MHz, UcieTL's nominal sideband and mainband rate, and a rate madsim
  // reaches ACTIVE at with all three of its domains tied together. Comes from
  // CLK_PERIOD_PS in the Makefile so the Verilog and the SystemC wrapper cannot
  // disagree; halved as an integer, hence the even-period rule.
`ifndef COSIM_CLK_PERIOD_PS
  `define COSIM_CLK_PERIOD_PS 1250
`endif
  localparam realtime CLK_HALF_PERIOD = (`COSIM_CLK_PERIOD_PS / 2) * 1ps;

  // UcieTL holds RESET for 3.2M cycles before it will accept a trigger, and
  // its per-substate timeout is 6.4M. Training therefore cannot complete in
  // less than a few million cycles whatever else is true.
  localparam int RESET_CYCLES = 64;

  // A plusarg, not a define, so changing it does not force a relink.
  //
  //   ./simv_cosim +run_cycles=30000
  //
  int unsigned run_cycles = 8_000_000;

  // Starts high so every posedge lands on a whole multiple of the period.
  // matchlib requires that of its reference sc_clock, so the Verilog side is
  // the one that has to line up. Half a period of skew shows up as
  // CONNECTIONS-113, not as anything clock-shaped.
  logic clk = 1'b1;
  always #CLK_HALF_PERIOD clk = ~clk;

  // Do not divide this for the mainband. Both dies carry two UI per cycle, so a
  // half-rate lane clock puts UcieTL at half madsim's rate.
  //   logic mb_lane_clk = 1'b1;
  //   always @(negedge clk) mb_lane_clk <= ~mb_lane_clk;

  logic rst_n = 1'b0;     // madsim convention, active low
  wire  rst   = ~rst_n;   // UcieTL convention, active high

  initial begin
    rst_n = 1'b0;
    repeat (RESET_CYCLES) @(posedge clk);
    rst_n = 1'b1;
  end

  // ---------------------------------------------------------------------
  // Bump nets. Named for the direction they travel, A is madsim, B is UcieTL.
  // ---------------------------------------------------------------------
  wire [15:0] a2b_mb_data;
  wire        a2b_mb_vld, a2b_mb_ckp, a2b_mb_ckn, a2b_mb_trk;

  // madsim's mainband pattern detector state, observed.
  wire [2:0]  mad_mb_clock_pass;
  wire        mad_mb_rx_done, mad_mb_rx_error;
  wire        mad_mb_rx_enable, mad_mb_tx_enable;
  wire [1:0]  mad_mb_pattern_select;
  wire [63:0] mad_sb_ltsm_tx_msg;
  wire        mad_sb_ltsm_tx_valid, mad_sb_ltsm_tx_ready;
  wire [15:0] mad_mb_rx_chunk_ckp, mad_mb_rx_chunk_trk;
  wire        mad_mb_rx_chunk_valid;
  wire [15:0] mad_mb_rx_chunk_ckn, mad_mb_rx_chunk_vld;
  // UcieTL's mainband outputs go straight to madsim's bumps. Nothing is
  // interposed. Both dies drive two UI per clock with the same phase
  // convention, so an adapter here would only hide a real phase bug.
  wire [15:0] b2a_mb_data;
  wire        b2a_mb_vld, b2a_mb_ckp, b2a_mb_ckn, b2a_mb_trk;

  // Sideband as each side wants to see it. The two are not interchangeable,
  // since madsim carries a level valid and UcieTL a gated forwarded clock.
  wire a2b_sb_data, a2b_sb_val;      // madsim drives
  wire b2a_sb_data, b2a_sb_val;      // shim drives, madsim samples
  wire ucie_sb_rx_data, ucie_sb_rx_clk;   // shim drives, UcieTL samples
  wire ucie_sb_tx_data, ucie_sb_tx_clk;   // UcieTL drives

  // ---------------------------------------------------------------------
  // Observation nets. Not required for the connection, only so the buses
  // have stable names at this level.
  // ---------------------------------------------------------------------
  wire [511:0] mad_fdi_lp_data, mad_fdi_pl_data;
  wire         mad_rdi_lp_valid, mad_rdi_pl_valid;
  wire         mad_fdi_lp_valid, mad_fdi_pl_valid;
  wire         mad_fdi_pl_trdy;
  wire [4:0]   mad_ltsm_state;
  wire [3:0]   mad_rdi_state, mad_fdi_state;
  wire         mad_trainerror, mad_inband_pres;

  // TileLink register-write driver. Declared before the instance so the port
  // connections below bind to these rather than to implicit nets.
  logic         tl_a_valid = 1'b0;
  logic [21:0]  tl_a_address = 22'b0;
  logic [31:0]  tl_a_mask = 32'b0;
  logic [255:0] tl_a_data = 256'b0;
  wire          tl_a_ready;
  wire          tl_d_valid;
  logic         tl_write_done = 1'b0;

  // Protocol traffic out of UcieTL.
  // auto_manager_in_a is the manager port UcieTL forwards across the link. An
  // accepted beat is framed as Cat(ucieManagerTxA, 0) and put on FDI as a raw
  // flit. Note the framer offers a beat every cycle even when held at zero.
  logic         mgr_a_valid = 1'b0;
  logic [15:0]  mgr_a_address = 16'b0;
  logic [31:0]  mgr_a_mask = 32'b0;
  logic [255:0] mgr_a_data = 256'b0;
  logic [7:0]   mgr_a_source = 8'b0;
  wire          mgr_a_ready;
  logic         mgr_traffic_done = 1'b0;
  // A plusarg so the credit behaviour can be demonstrated and then taken out of
  // the way without a rebuild between the two runs.
  int unsigned  credit_flow   = 1;

  // Payload source. The receive counter madsim_first waits on is further down,
  // after the FDI aliases it reads.
  logic [63:0]  tx_rng = 64'd0;
  logic [255:0] mgr_data = 256'd0;
  int unsigned  n_ucie_fdi_rx_flits = 0;


  // ---------------------------------------------------------------------
  // Die A: madsim, wrapped for the Verilog boundary
  // ---------------------------------------------------------------------
  // syscan's generated madsim_die.v declares its ports as 4-state wires mapped
  // onto C++ bool, so VCS squashes x and z to 0 there and notes it once per
  // port. That is inside the generated shell and not reachable from here.
  // Report an x on any pin feeding madsim instead, so the squash is not silent.
  always @(posedge clk) if (rst_n) begin
    if ($isunknown({b2a_mb_data, b2a_mb_vld, b2a_mb_ckp, b2a_mb_ckn,
                    b2a_mb_trk, b2a_sb_data, b2a_sb_val}))
      $display("[%t] cosim: X on a madsim input pin, squashed to 0", $time);
  end

  madsim_die u_mad (
    .clk   (clk),
    .rst_n (rst_n),

    .mb_tx_data_0 (a2b_mb_data[0]),   .mb_tx_data_1 (a2b_mb_data[1]),
    .mb_tx_data_2 (a2b_mb_data[2]),   .mb_tx_data_3 (a2b_mb_data[3]),
    .mb_tx_data_4 (a2b_mb_data[4]),   .mb_tx_data_5 (a2b_mb_data[5]),
    .mb_tx_data_6 (a2b_mb_data[6]),   .mb_tx_data_7 (a2b_mb_data[7]),
    .mb_tx_data_8 (a2b_mb_data[8]),   .mb_tx_data_9 (a2b_mb_data[9]),
    .mb_tx_data_10(a2b_mb_data[10]),  .mb_tx_data_11(a2b_mb_data[11]),
    .mb_tx_data_12(a2b_mb_data[12]),  .mb_tx_data_13(a2b_mb_data[13]),
    .mb_tx_data_14(a2b_mb_data[14]),  .mb_tx_data_15(a2b_mb_data[15]),
    .mb_tx_vld (a2b_mb_vld),
    .mb_tx_ckp (a2b_mb_ckp),
    .mb_tx_ckn (a2b_mb_ckn),
    .mb_tx_trk (a2b_mb_trk),

    .mb_rx_data_0 (b2a_mb_data[0]),   .mb_rx_data_1 (b2a_mb_data[1]),
    .mb_rx_data_2 (b2a_mb_data[2]),   .mb_rx_data_3 (b2a_mb_data[3]),
    .mb_rx_data_4 (b2a_mb_data[4]),   .mb_rx_data_5 (b2a_mb_data[5]),
    .mb_rx_data_6 (b2a_mb_data[6]),   .mb_rx_data_7 (b2a_mb_data[7]),
    .mb_rx_data_8 (b2a_mb_data[8]),   .mb_rx_data_9 (b2a_mb_data[9]),
    .mb_rx_data_10(b2a_mb_data[10]),  .mb_rx_data_11(b2a_mb_data[11]),
    .mb_rx_data_12(b2a_mb_data[12]),  .mb_rx_data_13(b2a_mb_data[13]),
    .mb_rx_data_14(b2a_mb_data[14]),  .mb_rx_data_15(b2a_mb_data[15]),
    .mb_rx_vld (b2a_mb_vld),
    .mb_rx_ckp (b2a_mb_ckp),
    .mb_rx_ckn (b2a_mb_ckn),
    .mb_rx_trk (b2a_mb_trk),

    .sb_tx_data (a2b_sb_data),
    .sb_tx_val  (a2b_sb_val),
    .sb_rx_data (b2a_sb_data),
    .sb_rx_val  (b2a_sb_val),

    .obs_rdi_lp_valid(mad_rdi_lp_valid),
    .obs_rdi_pl_valid(mad_rdi_pl_valid),
    .obs_fdi_lp_data (mad_fdi_lp_data),
    .obs_fdi_lp_valid(mad_fdi_lp_valid),
    .obs_fdi_pl_data (mad_fdi_pl_data),
    .obs_fdi_pl_valid(mad_fdi_pl_valid),
    .obs_fdi_pl_trdy (mad_fdi_pl_trdy),
    .obs_ltsm_state  (mad_ltsm_state),
    .obs_rdi_state   (mad_rdi_state),
    .obs_fdi_state   (mad_fdi_state),
    .obs_trainerror  (mad_trainerror),
    .obs_inband_pres (mad_inband_pres),

    // madsim's mainband pattern detector. Ports rather than probes, since syscan
    // exposes only ports of the SystemC shell, so nothing inside it can be
    // reached by hierarchical reference.
    .obs_mb_clock_pass     (mad_mb_clock_pass),
    .obs_mb_rx_done        (mad_mb_rx_done),
    .obs_mb_rx_error       (mad_mb_rx_error),
    .obs_mb_rx_enable      (mad_mb_rx_enable),
    .obs_mb_tx_enable      (mad_mb_tx_enable),
    .obs_mb_pattern_select (mad_mb_pattern_select),
    .obs_sb_ltsm_tx_msg    (mad_sb_ltsm_tx_msg),
    .obs_sb_ltsm_tx_valid  (mad_sb_ltsm_tx_valid),
    .obs_sb_ltsm_tx_ready  (mad_sb_ltsm_tx_ready),
    .obs_mb_rx_chunk_ckp   (mad_mb_rx_chunk_ckp),
    .obs_mb_rx_chunk_trk   (mad_mb_rx_chunk_trk),
    .obs_mb_rx_chunk_valid (mad_mb_rx_chunk_valid),
    .obs_mb_rx_chunk_ckn   (mad_mb_rx_chunk_ckn),
    .obs_mb_rx_chunk_vld   (mad_mb_rx_chunk_vld)
  );

  // ---------------------------------------------------------------------
  // Sideband shim. The only place the two conventions are reconciled.
  // ---------------------------------------------------------------------
  ucie_sb_shim #(.RX_ALIGN(0)) u_sb_shim (
    .sb_clk (clk),
    .rst_n  (rst_n),

    .mad_sb_tx_data (a2b_sb_data),
    .mad_sb_tx_val  (a2b_sb_val),
    .mad_sb_rx_data (b2a_sb_data),
    .mad_sb_rx_val  (b2a_sb_val),

    .ucie_sb_rx_data (ucie_sb_rx_data),
    .ucie_sb_rx_clk  (ucie_sb_rx_clk),
    .ucie_sb_tx_data (ucie_sb_tx_data),
    .ucie_sb_tx_clk  (ucie_sb_tx_clk)
  );

  // ---------------------------------------------------------------------
  // Die B: UcieTL. Mainband crosses straight over, both sides being 16
  // serial lanes with a forwarded differential clock, valid and track.
  // ---------------------------------------------------------------------
  UcieTL u_ucie (
    .auto_digital_clock_in_clock (clk),
    .auto_digital_clock_in_reset (rst),

    // Mainband in, from madsim's TX
    .io_phy_rxData_0 (a2b_mb_data[0]),   .io_phy_rxData_1 (a2b_mb_data[1]),
    .io_phy_rxData_2 (a2b_mb_data[2]),   .io_phy_rxData_3 (a2b_mb_data[3]),
    .io_phy_rxData_4 (a2b_mb_data[4]),   .io_phy_rxData_5 (a2b_mb_data[5]),
    .io_phy_rxData_6 (a2b_mb_data[6]),   .io_phy_rxData_7 (a2b_mb_data[7]),
    .io_phy_rxData_8 (a2b_mb_data[8]),   .io_phy_rxData_9 (a2b_mb_data[9]),
    .io_phy_rxData_10(a2b_mb_data[10]),  .io_phy_rxData_11(a2b_mb_data[11]),
    .io_phy_rxData_12(a2b_mb_data[12]),  .io_phy_rxData_13(a2b_mb_data[13]),
    .io_phy_rxData_14(a2b_mb_data[14]),  .io_phy_rxData_15(a2b_mb_data[15]),
    .io_phy_rxValid (a2b_mb_vld),
    .io_phy_rxTrack (a2b_mb_trk),
    // Mainband receive clock. The shared harness clock, not the partner's
    // clock lane.
    //
    // clock_receiver.v is `assign Vout = Vin`, a pass-through with no flywheel,
    // so it stops ticking through REPAIRCLK's 16 UI low phase and the word
    // framing drifts. Both dies share one clock here, so sample on that. The
    // partner's clock lane is still driven and observed.
    // MB_RX_CLK_FROM_PARTNER reverts this.
`ifdef MB_RX_CLK_FROM_PARTNER
    .io_phy_rxClkP  (a2b_mb_ckp),
    .io_phy_rxClkN  (a2b_mb_ckn),
`else
    .io_phy_rxClkP  (clk),
    .io_phy_rxClkN  (~clk),
`endif

    // Mainband out, to madsim's RX
    .io_phy_txData_0 (b2a_mb_data[0]),   .io_phy_txData_1 (b2a_mb_data[1]),
    .io_phy_txData_2 (b2a_mb_data[2]),   .io_phy_txData_3 (b2a_mb_data[3]),
    .io_phy_txData_4 (b2a_mb_data[4]),   .io_phy_txData_5 (b2a_mb_data[5]),
    .io_phy_txData_6 (b2a_mb_data[6]),   .io_phy_txData_7 (b2a_mb_data[7]),
    .io_phy_txData_8 (b2a_mb_data[8]),   .io_phy_txData_9 (b2a_mb_data[9]),
    .io_phy_txData_10 (b2a_mb_data[10]),  .io_phy_txData_11 (b2a_mb_data[11]),
    .io_phy_txData_12 (b2a_mb_data[12]),  .io_phy_txData_13 (b2a_mb_data[13]),
    .io_phy_txData_14 (b2a_mb_data[14]),  .io_phy_txData_15 (b2a_mb_data[15]),
    .io_phy_txValid (b2a_mb_vld),
    .io_phy_txTrack (b2a_mb_trk),
    .io_phy_txClkP  (b2a_mb_ckp),
    .io_phy_txClkN  (b2a_mb_ckn),

    // Sideband, through the shim
    .io_phy_sbRxClk  (ucie_sb_rx_clk),
    .io_phy_sbRxData (ucie_sb_rx_data),
    .io_phy_sbTxClk  (ucie_sb_tx_clk),
    .io_phy_sbTxData (ucie_sb_tx_data),

    // The analog clocking bypasses. Driven from the same clock so the digital
    // model has a running clock wherever the real design would take one from
    // the clocking tile.
    // Mainband lane clock at half the digital clock, so the dies agree on the
    // UI rate. madsim's mainband port carries one UI per cycle; UcieTL's
    // tx_lane.v is a DDR serializer emitting two per lane clock, so at the full
    // rate every other UI is dropped.
  
    // clocking_tile routes BypassClk only to the mainband lane clocks, so the
    // digital domain and the whole sideband stay at full rate.
    .io_phy_bypassClk        (clk),
    .io_phy_digitalBypassClk (clk),

    // Debug outputs, observed by hierarchical reference rather than wired.
    .io_debug_txClk   (), .io_debug_rxClk  (), .io_debug_rxData (),
    .io_debug_clkMux  (), .io_debug_txData (),

    // TileLink. Quiescent for now, since this revision establishes the link and does
    // not drive protocol traffic. Every input is held inactive and every
    // output left open rather than tied, so nothing is accidentally consumed.
    .auto_regs_in_a_valid        (tl_a_valid),
    .auto_regs_in_a_bits_opcode  (3'd0),          // PutFullData
    .auto_regs_in_a_bits_size    (3'd3),          // 2^3 = 8 bytes
    .auto_regs_in_a_bits_source  (1'b0),
    .auto_regs_in_a_bits_address (tl_a_address),
    .auto_regs_in_a_bits_mask    (tl_a_mask),
    .auto_regs_in_a_bits_data    (tl_a_data),
    .auto_regs_in_d_ready        (1'b1),
    .auto_regs_in_a_ready        (tl_a_ready),
    .auto_regs_in_d_valid        (tl_d_valid),
    .auto_regs_in_d_bits_opcode  (),
    .auto_regs_in_d_bits_size    (),
    .auto_regs_in_d_bits_source  (),
    .auto_regs_in_d_bits_data    (),

    .auto_client_out_a_ready         (1'b1),
    .auto_client_out_d_valid         (1'b0),
    .auto_client_out_d_bits_opcode   (3'b0),
    .auto_client_out_d_bits_param    (2'b0),
    .auto_client_out_d_bits_size     (3'b0),
    .auto_client_out_d_bits_source   (8'b0),
    .auto_client_out_d_bits_sink     (1'b0),
    .auto_client_out_d_bits_denied   (1'b0),
    .auto_client_out_d_bits_data     (256'b0),
    .auto_client_out_d_bits_corrupt  (1'b0),
    .auto_client_out_a_valid         (),
    .auto_client_out_a_bits_opcode   (),
    .auto_client_out_a_bits_param    (),
    .auto_client_out_a_bits_size     (),
    .auto_client_out_a_bits_source   (),
    .auto_client_out_a_bits_address  (),
    .auto_client_out_a_bits_mask     (),
    .auto_client_out_a_bits_data     (),
    .auto_client_out_a_bits_corrupt  (),
    .auto_client_out_d_ready         (),

    .auto_manager_in_a_valid        (mgr_a_valid),
    .auto_manager_in_a_bits_opcode  (3'd0),        // PutFullData
    .auto_manager_in_a_bits_param   (3'b0),
    .auto_manager_in_a_bits_size    (3'd5),        // 2^5 = 32 bytes, one beat
    .auto_manager_in_a_bits_source  (mgr_a_source),
    .auto_manager_in_a_bits_address (mgr_a_address),
    .auto_manager_in_a_bits_mask    (mgr_a_mask),
    .auto_manager_in_a_bits_data    (mgr_a_data),
    .auto_manager_in_a_bits_corrupt (1'b0),
    .auto_manager_in_d_ready        (1'b1),
    .auto_manager_in_a_ready        (mgr_a_ready),
    .auto_manager_in_d_valid        (),
    .auto_manager_in_d_bits_opcode  (),
    .auto_manager_in_d_bits_param   (),
    .auto_manager_in_d_bits_size    (),
    .auto_manager_in_d_bits_source  (),
    .auto_manager_in_d_bits_sink    (),
    .auto_manager_in_d_bits_denied  (),
    .auto_manager_in_d_bits_data    (),
    .auto_manager_in_d_bits_corrupt ()
  );

  // ---------------------------------------------------------------------
  // UcieTL's RDI and FDI, reached by hierarchical reference.
  // ---------------------------------------------------------------------
  // Aliased rather than ported, so the elaborated UcieTL stays byte-identical
  // to what the Chisel emits. The paths also fail the build if the hierarchy
  // changes shape.
  wire [511:0] ucie_rdi_lp_data  = u_ucie.ucieDigitalLazy_d2dAdapter.io_rdi_lpData;
  wire [511:0] ucie_rdi_pl_data  = u_ucie.ucieDigitalLazy_d2dAdapter.io_rdi_plData;
  wire         ucie_rdi_lp_valid = u_ucie.ucieDigitalLazy_d2dAdapter.io_rdi_lpValid;
  wire         ucie_rdi_lp_irdy  = u_ucie.ucieDigitalLazy_d2dAdapter.io_rdi_lpIrdy;
  wire         ucie_rdi_pl_valid = u_ucie.ucieDigitalLazy_d2dAdapter.io_rdi_plValid;
  wire         ucie_rdi_pl_trdy  = u_ucie.ucieDigitalLazy_d2dAdapter.io_rdi_plTrdy;

  wire [511:0] ucie_fdi_lp_data  = u_ucie.ucieDigitalLazy_d2dAdapter.io_fdi_lpData;
  wire [511:0] ucie_fdi_pl_data  = u_ucie.ucieDigitalLazy_d2dAdapter.io_fdi_plData;
  wire         ucie_fdi_lp_valid = u_ucie.ucieDigitalLazy_d2dAdapter.io_fdi_lpValid;
  wire         ucie_fdi_pl_valid = u_ucie.ucieDigitalLazy_d2dAdapter.io_fdi_plValid;

  // Flits arriving from madsim, so madsim_first knows when to start sending.
  always @(posedge clk) if (rst_n && ucie_fdi_pl_valid)
    n_ucie_fdi_rx_flits <= n_ucie_fdi_rx_flits + 1;
  wire  [3:0]  ucie_rdi_state    = u_ucie.ucieDigitalLazy_d2dAdapter.io_rdi_plStateSts;
  wire  [3:0]  ucie_fdi_state    = u_ucie.ucieDigitalLazy_d2dAdapter.io_fdi_plStateSts;
  wire         ucie_inband_pres  = u_ucie.ucieDigitalLazy_d2dAdapter.io_rdi_plInbandPres;

  // ---------------------------------------------------------------------
  // One register write, handing both bands to the UCIe stack
  // ---------------------------------------------------------------------
  // controllerSel resets to `phytest`, which routes PhyTest's serializer to the
  // bumps. Receive is unconditional, so the UCIe stack can receive but not
  // answer until this is written.
  //
  // Offset from the generated header. Regenerate with
  //   ./mill runMain edu.berkeley.cs.uciedigital.tilelink.GenUcieHeader <path>
  // Register map, from scala/src/tilelink/Codegen.scala via GenUcieHeader. The
  // names and values match that header exactly so the two can be diffed.
  localparam [21:0] UCIE_CONTROLLER_SEL      = 22'h37d8;
  localparam        UCIE_CONTROLLER_SEL_UCIE = 1;
  // Credit flow on the TileLink A and D channels. Enabled by default, which is
  // wrong against this partner, because madsim returns no credits, so the A channel
  // spends its initial allowance of 63 beats and stops. Writing 0 selects
  // CreditCounter's no-flow mode.
  localparam [21:0] UCIE_CREDIT_FLOW_ENABLE  = 22'h37f0;

  localparam [21:0] UCIE_TXCTL               = 22'h03e0;
  localparam [21:0] UCIE_TXCTL_TILE_OFS      = 22'h0000;
  localparam [21:0] UCIE_TXCTL_WIDTH         = 22'h0118;
  localparam [21:0] UCIE_RXCTL               = 22'h1ad8;
  localparam [21:0] UCIE_RXCTL_ZEN_OFS       = 22'h0000;
  localparam [21:0] UCIE_RXCTL_ZCTL_OFS      = 22'h0008;
  localparam [21:0] UCIE_RXCTL_SHUFFLER_OFS  = 22'h0038;
  localparam [21:0] UCIE_RXCTL_SHUFFLER_STEP = 22'h0008;
  localparam [21:0] UCIE_RXCTL_WIDTH         = 22'h0148;
  localparam [21:0] UCIE_DEBUG_DRIVERCTL     = 22'h3678;
  localparam [21:0] UCIE_DEBUG_TXCTL_TILE    = 22'h3698;

  localparam [63:0] UCIE_ENABLE_TX_CTL       = 64'h0001_fff0_0000_0000;
  localparam [63:0] UCIE_ENABLE_DRIVER_CTL   = 64'h0000_0000_0000_3ffe;
  localparam int    UCIE_NUM_LANES           = 21;

  // pwrGood is deliberately not written. It resets true so the link can train
  // without software, so touching it would only risk clearing it.

  // Minimal TileLink-UL PutFullData. The bus is 32 bytes wide, so an 8-byte
  // write is placed in the lane its address selects.
  // Both channels are guarded with a timeout, since an unanswered write would
  // otherwise hang this initial block and training would never start.
  task automatic tl_write64(input [21:0] addr, input [63:0] value);
    int unsigned lane;
    int unsigned guard;
    begin
      lane = addr[4:3];                       // which 8-byte slot in the beat
      tl_a_address <= addr;
      tl_a_mask    <= 32'hFF << (lane * 8);
      tl_a_data    <= {192'b0, value} << (lane * 64);
      tl_a_valid   <= 1'b1;

      // Valid has to stay asserted through to a cycle where ready is high.
      guard = 0;
      forever begin
        @(posedge clk);
        if (tl_a_ready) break;
        guard = guard + 1;
        if (guard > 2000) begin
          $display("cosim: FATAL TileLink A channel stalled writing 0x%05h", addr);
          $fatal(1);
        end
      end
      tl_a_valid <= 1'b0;

      // d_ready is tied high, so the response is taken the moment it appears,
      // which can be the same cycle the request was accepted. Test before
      // waiting rather than after, or that response is missed and this hangs.
      guard = 0;
      while (!tl_d_valid) begin
        @(posedge clk);
        guard = guard + 1;
        if (guard > 2000) begin
          $display("cosim: FATAL TileLink D channel stalled writing 0x%05h", addr);
          $fatal(1);
        end
      end
      @(posedge clk);
    end
  endtask

  // PHY bring-up, normally done by software. tx_lane.v holds its driver off in
  // the reset state, so every mainband lane drives 0 until the segments are
  // enabled. Mirrors setup_ucie() in the generated ucie.h, minus the
  // PhyTest-only parts. 21 lanes = 16 data, valid, track, clock pair.
  initial begin
    wait (rst_n === 1'b1);
    repeat (16) @(posedge clk);

    for (int lane = 0; lane < UCIE_NUM_LANES; lane++) begin
      tl_write64(UCIE_TXCTL + lane * UCIE_TXCTL_WIDTH + UCIE_TXCTL_TILE_OFS,
                 UCIE_ENABLE_TX_CTL);
      tl_write64(UCIE_RXCTL + lane * UCIE_RXCTL_WIDTH + UCIE_RXCTL_ZEN_OFS, 64'h1);
      tl_write64(UCIE_RXCTL + lane * UCIE_RXCTL_WIDTH + UCIE_RXCTL_ZCTL_OFS, 64'h0);
    end

    for (int i = 0; i < 4; i++)
      tl_write64(UCIE_DEBUG_DRIVERCTL + i * 8, UCIE_ENABLE_DRIVER_CTL);

    tl_write64(UCIE_DEBUG_TXCTL_TILE, UCIE_ENABLE_TX_CTL);

    tl_write64(UCIE_CONTROLLER_SEL, UCIE_CONTROLLER_SEL_UCIE);

    void'($value$plusargs("credit_flow=%d", credit_flow));
    if (credit_flow == 0) begin
      tl_write64(UCIE_CREDIT_FLOW_ENABLE, 64'h0);
      $display("cosim: TileLink credit flow DISABLED (partner returns no credits)");
    end else begin
      $display("cosim: TileLink credit flow left enabled (default)");
    end

    tl_write_done <= 1'b1;
    $display("cosim: PHY tiles enabled on %0d lanes, controllerSel <- ucie",
             UCIE_NUM_LANES);
  end

  // ---------------------------------------------------------------------
  // Protocol traffic, once the link is up
  // ---------------------------------------------------------------------
  // TileLink PutFullData beats with recognisable payloads, so a flit at the
  // partner can be matched to the beat that produced it. Waits for both dies to
  // report FDI ACTIVE, since both protocol layers drop beats before that. No
  // D-channel response is expected, because madsim is not a TileLink agent.

  // Returns 0 if the beat was accepted, 1 if the A channel never granted ready.
  // Bounded rather than a forever loop, because running out of credits is one of the
  // things this is meant to demonstrate, and a hang reports nothing.
  localparam int MGR_A_TIMEOUT = 20000;

  task automatic mgr_send(input [15:0] addr, input [255:0] data,
                          output bit stalled);
    int unsigned waited;
    begin
      stalled = 1'b0;
      waited  = 0;
      @(posedge clk);
      mgr_a_address <= addr;
      mgr_a_data    <= data;
      mgr_a_mask    <= {32{1'b1}};
      mgr_a_valid   <= 1'b1;
      forever begin
        @(posedge clk);
        if (mgr_a_ready) break;
        waited = waited + 1;
        if (waited > MGR_A_TIMEOUT) begin
          stalled = 1'b1;
          break;
        end
      end
      mgr_a_valid <= 1'b0;
      if (!stalled) mgr_a_source <= mgr_a_source + 8'd1;
    end
  endtask

  int unsigned n_mgr_beats_sent = 0;

  initial begin : mgr_traffic
    wait (rst_n === 1'b1);
    wait (tl_write_done === 1'b1);
    wait (ucie_fdi_state === 4'h1 && mad_fdi_state === 4'h1);
    repeat (8) @(posedge clk);

    // madsim_first holds off until every madsim message has arrived.
    if (`COSIM_TRAFFIC_MODE == 2)
      wait (n_ucie_fdi_rx_flits >= `COSIM_MADSIM_TX_MSGS);

    // Reproducible from COSIM_RANDOM_SEED, and independent of the partner's
    // stream so the two do not correlate.
    tx_rng = {32'd0, 32'h9e37_79b9} ^ {32'd0, 32'(`COSIM_RANDOM_SEED)};

    for (int i = 0; i < `COSIM_UCIE_TX_MSGS; i++) begin
      bit stalled;
      // Fill the whole 256-bit payload, not just the low word, so the data
      // lanes and the scrambler see real transition density.
      for (int w = 0; w < 8; w++) begin
        tx_rng ^= tx_rng << 13;  tx_rng ^= tx_rng >> 7;  tx_rng ^= tx_rng << 17;
        mgr_data[w*32 +: 32] = tx_rng[63:32];
      end
      // Always non-zero, so a flit carrying a message is distinguishable from
      // the framer's idle beats, which leave the address field at 0.
      mgr_send(16'h1000 | (tx_rng[15:0] & 16'h0ff0), mgr_data, stalled);
      if (stalled) begin
        $display("cosim: TileLink A channel stopped granting ready after %0d beats", i);
        $display("       (credit_flow=%0d; with it enabled the partner returns no", credit_flow);
        $display("        credits, so only the initial allowance can be spent)");
        n_mgr_beats_sent = i;
        mgr_traffic_done <= 1'b1;
        disable mgr_traffic;
      end
      n_mgr_beats_sent = i + 1;
    end

    mgr_traffic_done <= 1'b1;
    $display("cosim: sent %0d TileLink beats into UcieTL's manager port",
             n_mgr_beats_sent);
  end

  // ---------------------------------------------------------------------
  // Run control and waveform dump
  // ---------------------------------------------------------------------
  initial begin
`ifdef DUMP_FSDB
    $fsdbDumpfile(`DUMP_FILE);
    $fsdbDumpvars(0, ucie_cosim_top);
`endif
    void'($value$plusargs("run_cycles=%d", run_cycles));
    repeat (run_cycles) @(posedge clk);
    $display("cosim: ran %0d cycles", run_cycles);
    $display("  madsim : ltsm=%0d rdi=%0d fdi=%0d inband=%0b trainerror=%0b",
             mad_ltsm_state, mad_rdi_state, mad_fdi_state,
             mad_inband_pres, mad_trainerror);
    $display("  ucieTL : rdi=%0d fdi=%0d inband=%0b",
             ucie_rdi_state, ucie_fdi_state, ucie_inband_pres);
    $finish;
  end

endmodule
