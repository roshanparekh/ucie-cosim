#ifndef MADSIM_DIE_H
#define MADSIM_DIE_H
//
// madsim_die.h - SystemC wrapper that makes madsim's ucie_top instantiable from
// SystemVerilog through VCS syscan. The file name must match the module name;
// syscan resolves the module from a header of the same name.
//
// It does three things a raw ucie_top cannot do at a Verilog boundary.
//
//   1. Binds rva_in / rva_out. These are matchlib Connections channels, which
//      are transaction level and have no Verilog representation. This wrapper
//      drives them, so madsim's protocol traffic is generated here.
//
//   2. Converts the lane bus and status ports from ac_int to bool and sc_uint,
//      which syscan can bridge. The 16 mainband lanes stay 16 separate bools
//      because UcieTL presents them that way.
//
//   3. Brings the RDI and FDI buses out as observation ports. They are internal
//      sc_signals, and a Verilog hierarchical reference cannot reach into the
//      SystemC kernel. Outputs only; nothing drives them.
//
// One shared clock feeds ucie_top's die, mainband and sideband domains. madsim
// has no hardcoded frequency ratios and its cross-domain FIFOs are async, so
// this is legal, confirmed by reaching LTSM_ACTIVE with all three tied together
// at 1 GHz and at 800 MHz. Reset is active low, as madsim expects.

#include <systemc.h>
#include "ucie_top.h"

// Traffic and timing parameters. The Makefile defines all of these on the
// syscan command line and the matching +define+ on the vcs one, so the two
// sides cannot drift. The fallbacks below only apply to a standalone compile.
#ifndef COSIM_CLK_PERIOD_PS
#define COSIM_CLK_PERIOD_PS 1250
#endif
#ifndef COSIM_MADSIM_TX_MSGS
#define COSIM_MADSIM_TX_MSGS 2000
#endif
#ifndef COSIM_UCIE_TX_MSGS
#define COSIM_UCIE_TX_MSGS 2000
#endif
// 0 parallel, 1 ucie_first, 2 madsim_first.
#ifndef COSIM_TRAFFIC_MODE
#define COSIM_TRAFFIC_MODE 0
#endif
#ifndef COSIM_RANDOM_SEED
#define COSIM_RANDOM_SEED 1
#endif

SC_MODULE(madsim_die) {
  // ---- clock and reset ----------------------------------------------------
  sc_in<bool> clk;
  sc_in<bool> rst_n;

  // ---- mainband, toward the partner die -----------------------------------
  sc_out<bool> mb_tx_data_0,  mb_tx_data_1,  mb_tx_data_2,  mb_tx_data_3;
  sc_out<bool> mb_tx_data_4,  mb_tx_data_5,  mb_tx_data_6,  mb_tx_data_7;
  sc_out<bool> mb_tx_data_8,  mb_tx_data_9,  mb_tx_data_10, mb_tx_data_11;
  sc_out<bool> mb_tx_data_12, mb_tx_data_13, mb_tx_data_14, mb_tx_data_15;
  sc_out<bool> mb_tx_vld;
  sc_out<bool> mb_tx_ckp;
  sc_out<bool> mb_tx_ckn;
  sc_out<bool> mb_tx_trk;

  // ---- mainband, from the partner die --------------------------------------
  sc_in<bool> mb_rx_data_0,  mb_rx_data_1,  mb_rx_data_2,  mb_rx_data_3;
  sc_in<bool> mb_rx_data_4,  mb_rx_data_5,  mb_rx_data_6,  mb_rx_data_7;
  sc_in<bool> mb_rx_data_8,  mb_rx_data_9,  mb_rx_data_10, mb_rx_data_11;
  sc_in<bool> mb_rx_data_12, mb_rx_data_13, mb_rx_data_14, mb_rx_data_15;
  sc_in<bool> mb_rx_vld;
  sc_in<bool> mb_rx_ckp;
  sc_in<bool> mb_rx_ckn;
  sc_in<bool> mb_rx_trk;

  // ---- sideband ------------------------------------------------------------
  // Data plus a LEVEL valid. The shim converts this to and from UcieTL's
  // gated forwarded clock; see src/ucie_sb_shim.sv.
  sc_out<bool> sb_tx_data;
  sc_out<bool> sb_tx_val;
  sc_in<bool>  sb_rx_data;
  sc_in<bool>  sb_rx_val;

  // ---- observation only ----------------------------------------------------
  // RDI, the adapter to PHY boundary.
  sc_out<bool>        obs_rdi_lp_valid;
  sc_out<sc_bv<512> > obs_rdi_pl_data;   // from the PHY
  sc_out<bool>        obs_rdi_pl_valid;

  // FDI, the protocol layer boundary.
  sc_out<sc_bv<512> > obs_fdi_lp_data;   // protocol layer output
  sc_out<bool>        obs_fdi_lp_valid;
  sc_out<sc_bv<512> > obs_fdi_pl_data;   // protocol layer input
  sc_out<bool>        obs_fdi_pl_valid;
  sc_out<bool>        obs_fdi_pl_trdy;   // qualifies lp_valid into a transfer

  // Link state, for telling "still training" from "wedged".
  sc_out<sc_uint<5> > obs_ltsm_state;
  sc_out<sc_uint<4> > obs_rdi_state;
  // madsim's own protocol layer request into its adapter. Its adapter gates
  // REQ_ACTIVE on this being STATE_REQ_ACTIVE, exactly as UcieTL's does.
  sc_out<sc_uint<4> > obs_fdi_lp_req;

  sc_out<sc_uint<4> > obs_fdi_state;
  sc_out<bool>        obs_trainerror;
  sc_out<bool>        obs_inband_pres;

  // madsim's mainband pattern detector. These must be ports, since syscan gives
  // Verilog only a shell module of the declared ports. clock_pass decides
  // MBINIT.REPAIRCLK, and the partner needs all three bits set.
  sc_out<sc_uint<3> >  obs_mb_clock_pass;
  sc_out<bool>         obs_mb_rx_done;
  sc_out<bool>         obs_mb_rx_error;
  sc_out<bool>         obs_mb_rx_enable;
  sc_out<bool>         obs_mb_tx_enable;
  sc_out<sc_uint<2> >  obs_mb_pattern_select;

  // Sideband transmit arbitration. PhyLtsm::Run skips runLTSM while tx_stalled,
  // and DriveTx grants ready by strict priority (RDI, LTSM, CFG), so a
  // persistent rdi_sb_tx_valid stops the state machine outright.
  // The message the LTSM is offering. A change while valid stays high with no
  // handoff means it was overwritten before the sideband took it. Low 64 bits
  // are the header.
  sc_out<sc_uint<64> > obs_sb_ltsm_tx_msg;

  sc_out<bool>         obs_sb_ltsm_tx_valid;
  sc_out<bool>         obs_sb_ltsm_tx_ready;

  // The received chunk's forwarded-clock fields, off phy_top's mb_rx_chunk.
  // kClockRepairPattern in three 16 UI chunks is 0x5555, 0x5555, 0x0000, so ckp
  // says whether the dies agree on where a chunk starts.
  sc_out<sc_uint<16> > obs_mb_rx_chunk_ckp;
  sc_out<sc_uint<16> > obs_mb_rx_chunk_trk;
  sc_out<bool>         obs_mb_rx_chunk_valid;
  // ckn and valid as well, so a field-boundary error in this extraction is
  // distinguishable from a genuine lane-to-lane difference.
  sc_out<sc_uint<16> > obs_mb_rx_chunk_ckn;
  sc_out<sc_uint<16> > obs_mb_rx_chunk_vld;

  // ---- the wrapped design --------------------------------------------------
  ucie_top dut;

  // Flits received from the partner, counted in the observe method below.
  uint64_t rx_flit_count = 0;

  // Connections finds its clock by walking the tree for an sc_clock, and the
  // Verilog clock arrives as a plain sc_in<bool>, so the lookup fails with
  // CONNECTIONS-111. This exists only to be found; end_of_elaboration aliases
  // its posedge to the port's, and Verilog still drives the design.
  sc_clock conn_ref_clk;

  // ac_int-typed nets that ucie_top actually binds to.
  sc_signal<phy_spec::LaneBits> mb_tx_data_int;
  sc_signal<phy_spec::LaneBits> mb_rx_data_int;

  sc_signal<NVUINTW(4)> st_fdi_state;
  sc_signal<bool>       st_fdi_inband_pres;
  sc_signal<NVUINTW(4)> st_rdi_state;
  sc_signal<bool>       st_rdi_inband_pres;
  sc_signal<NVUINTW(4)> st_protocol;
  sc_signal<NVUINTW(4)> st_protocol_flitfmt;
  sc_signal<bool>       st_protocol_vld;
  sc_signal<NVUINTW(3)> st_speedmode;
  sc_signal<NVUINTW(3)> st_lnk_cfg;
  sc_signal<bool>       st_phyinrecenter;
  sc_signal<bool>       st_trainerror;
  sc_signal<NVUINTW(5)> st_ltsm_state;

  // Protocol-side channels, bound so elaboration succeeds.
  Connections::Combinational<spec::Axi::SlaveToRVA::Write> chan_rva_in;
  Connections::Combinational<spec::Axi::SlaveToRVA::Read>  chan_rva_out;
  Connections::Out<spec::Axi::SlaveToRVA::Write> rva_in_src;
  Connections::In<spec::Axi::SlaveToRVA::Read>   rva_out_sink;

  // ac_int<512> to sc_bv<512>, bit by bit. ac_int has no bulk accessor wide
  // enough and to_uint64 would silently truncate.
  static void wide_to_bv(const ucie::DataBus_t& in, sc_bv<512>& out) {
    for (int i = 0; i < 512; ++i) out[i] = in[i] ? true : false;
  }

  void split_tx_lanes() {
    const phy_spec::LaneBits d = mb_tx_data_int.read();
    mb_tx_data_0.write(d[0]);   mb_tx_data_1.write(d[1]);
    mb_tx_data_2.write(d[2]);   mb_tx_data_3.write(d[3]);
    mb_tx_data_4.write(d[4]);   mb_tx_data_5.write(d[5]);
    mb_tx_data_6.write(d[6]);   mb_tx_data_7.write(d[7]);
    mb_tx_data_8.write(d[8]);   mb_tx_data_9.write(d[9]);
    mb_tx_data_10.write(d[10]); mb_tx_data_11.write(d[11]);
    mb_tx_data_12.write(d[12]); mb_tx_data_13.write(d[13]);
    mb_tx_data_14.write(d[14]); mb_tx_data_15.write(d[15]);
  }

  void merge_rx_lanes() {
    phy_spec::LaneBits d = 0;
    d[0]  = mb_rx_data_0.read();  d[1]  = mb_rx_data_1.read();
    d[2]  = mb_rx_data_2.read();  d[3]  = mb_rx_data_3.read();
    d[4]  = mb_rx_data_4.read();  d[5]  = mb_rx_data_5.read();
    d[6]  = mb_rx_data_6.read();  d[7]  = mb_rx_data_7.read();
    d[8]  = mb_rx_data_8.read();  d[9]  = mb_rx_data_9.read();
    d[10] = mb_rx_data_10.read(); d[11] = mb_rx_data_11.read();
    d[12] = mb_rx_data_12.read(); d[13] = mb_rx_data_13.read();
    d[14] = mb_rx_data_14.read(); d[15] = mb_rx_data_15.read();
    mb_rx_data_int.write(d);
  }

  void drive_observation() {
    sc_bv<512> bv;
    wide_to_bv(dut.rdi_pl_data.read(), bv); obs_rdi_pl_data.write(bv);
    wide_to_bv(dut.fdi_lp_data.read(), bv); obs_fdi_lp_data.write(bv);
    wide_to_bv(dut.fdi_pl_data.read(), bv); obs_fdi_pl_data.write(bv);

    obs_rdi_lp_valid.write(dut.rdi_lp_valid.read());
    obs_rdi_pl_valid.write(dut.rdi_pl_valid.read());

    obs_fdi_lp_valid.write(dut.fdi_lp_valid.read());
    obs_fdi_pl_valid.write(dut.fdi_pl_valid.read());
    obs_fdi_pl_trdy.write(dut.fdi_pl_trdy.read());
    // Flits arriving from the partner, so ucie_first knows when to start.
    if (dut.fdi_pl_valid.read()) rx_flit_count++;

    obs_ltsm_state.write((unsigned)st_ltsm_state.read().to_uint64());
    obs_rdi_state.write((unsigned)st_rdi_state.read().to_uint64());
    obs_fdi_state.write((unsigned)st_fdi_state.read().to_uint64());
    obs_trainerror.write(st_trainerror.read());
    obs_inband_pres.write(st_rdi_inband_pres.read());

    obs_mb_clock_pass.write(
        (unsigned)dut.phy.mb_pattern_rx_clock_pass.read().to_uint64());
    obs_mb_rx_done.write(dut.phy.mb_pattern_rx_done.read());
    obs_mb_rx_error.write(dut.phy.mb_pattern_rx_error.read());
    obs_mb_rx_enable.write(dut.phy.mb_pattern_rx_enable.read());
    obs_mb_tx_enable.write(dut.phy.mb_pattern_tx_enable.read());
    obs_mb_pattern_select.write(
        (unsigned)dut.phy.mb_pattern_select.read().to_uint64());

    obs_sb_ltsm_tx_msg.write(
        (sc_dt::uint64)(dut.phy.ltsm_sb_tx_msg.read()
                            .slc<64>(0).to_uint64()));

    obs_sb_ltsm_tx_valid.write(dut.phy.ltsm_sb_tx_valid.read());
    obs_sb_ltsm_tx_ready.write(dut.phy.ltsm_sb_tx_ready.read());

    // Field positions from phy_spec.h: data occupies the low
    // kNumLanes*kUiPerChunk bits, then valid, ckp, ckn, trk each kUiPerChunk.
    {
      const NVUINTW(phy_spec::kMbChunkWidth) chunk = dut.phy.mb_rx_chunk.read();
      unsigned vld = 0, ckp = 0, ckn = 0, trk = 0;
      for (int i = 0; i < phy_spec::kUiPerChunk; ++i) {
        if (chunk[phy_spec::kMbChunkValidPos + i]) vld |= (1u << i);
        if (chunk[phy_spec::kMbChunkCkpPos + i])   ckp |= (1u << i);
        if (chunk[phy_spec::kMbChunkCknPos + i])   ckn |= (1u << i);
        if (chunk[phy_spec::kMbChunkTrkPos + i])   trk |= (1u << i);
      }
      obs_mb_rx_chunk_ckp.write(ckp);
      obs_mb_rx_chunk_trk.write(trk);
      obs_mb_rx_chunk_ckn.write(ckn);
      obs_mb_rx_chunk_vld.write(vld);
    }
    obs_mb_rx_chunk_valid.write(dut.phy.mb_rx_chunk_valid.read());
  }

  // Traffic out of madsim's protocol layer. PlTxEngine packs each AXI write
  // into a raw flit (BuildRawFlit) as [7:0] counter, [39:8] addr, [103:40]
  // data. Address and data are random, so the monitor compares against what
  // was sent rather than deriving it from the counter.
  //
  // It drops flits until its link FSM reaches ACTIVE, so the push waits for
  // FDI. Connections ports still need one reset even if nothing is pushed;
  // COSIM_MADSIM_TX_MSGS = 0 keeps the protocol side quiescent.
  void drive_protocol_side() {
    rva_in_src.Reset();

    // xorshift64*, so the stream is reproducible from COSIM_RANDOM_SEED and
    // does not depend on the host's rand().
    uint64_t rng = 0x9e3779b97f4a7c15ull + (uint64_t)COSIM_RANDOM_SEED;

    for (int i = 0; i < COSIM_MADSIM_TX_MSGS; ++i) {
      while ((unsigned)st_fdi_state.read().to_uint64() !=
             (unsigned)ucie::STATE_STS_ACTIVE) {
        wait();
      }

      // ucie_first holds off until every UcieTL message has been received.
      if (COSIM_TRAFFIC_MODE == 1) {
        while (rx_flit_count < (uint64_t)COSIM_UCIE_TX_MSGS) wait();
      }

      rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
      const uint64_t r1 = rng * 0x2545f4914f6cdd1dull;
      rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
      const uint64_t r2 = rng * 0x2545f4914f6cdd1dull;

      spec::Axi::SlaveToRVA::Write req;
      // 32 bits of address, 64 of data. BuildRawFlit takes addr at [39:8].
      req.addr  = (uint32_t)r1;
      req.data  = r2;
      req.wstrb = 0xff;  // all bytes of the 64-bit word
      req.rw    = 1;  // write
      rva_in_src.Push(req);
    }

    while (true) wait();
  }

  // The read-response channel has no consumer, so drain it. A full channel would
  // back-pressure madsim's receive engine and stall its RX path.
  void sink_protocol_side() {
    rva_out_sink.Reset();
    while (true) {
      spec::Axi::SlaveToRVA::Read resp;
      rva_out_sink.PopNB(resp);
      wait();
    }
  }

  // Registered after port binding, because the alias stores the address of the
  // port's event, and before start_of_simulation, which is when Connections
  // resolves aliases.
  void end_of_elaboration() {
    Connections::get_sim_clk().add_clock_alias(conn_ref_clk.posedge_event(),
                                               clk->posedge_event());
  }

  SC_CTOR(madsim_die)
      : dut("dut"),
        conn_ref_clk("conn_ref_clk", sc_time(COSIM_CLK_PERIOD_PS, SC_PS),
                     0.5, SC_ZERO_TIME, true),
        chan_rva_in("chan_rva_in"),
        chan_rva_out("chan_rva_out"),
        rva_in_src("rva_in_src"),
        rva_out_sink("rva_out_sink") {

    // One clock into all three madsim domains.
    dut.clk(clk);
    dut.sb_clk(clk);
    dut.mb_clk(clk);
    dut.rst(rst_n);

    dut.rva_in(chan_rva_in);
    dut.rva_out(chan_rva_out);
    rva_in_src(chan_rva_in);
    rva_out_sink(chan_rva_out);

    dut.mb_tx_data(mb_tx_data_int);
    dut.mb_tx_vld(mb_tx_vld);
    dut.mb_tx_ckp(mb_tx_ckp);
    dut.mb_tx_ckn(mb_tx_ckn);
    dut.mb_tx_trk(mb_tx_trk);

    dut.mb_rx_data(mb_rx_data_int);
    dut.mb_rx_vld(mb_rx_vld);
    dut.mb_rx_ckp(mb_rx_ckp);
    dut.mb_rx_ckn(mb_rx_ckn);
    dut.mb_rx_trk(mb_rx_trk);

    dut.sb_tx_data(sb_tx_data);
    dut.sb_tx_val(sb_tx_val);
    dut.sb_rx_data(sb_rx_data);
    dut.sb_rx_val(sb_rx_val);

    dut.status_fdi_state(st_fdi_state);
    dut.status_fdi_inband_pres(st_fdi_inband_pres);
    dut.status_rdi_state(st_rdi_state);
    dut.status_rdi_inband_pres(st_rdi_inband_pres);
    dut.status_protocol(st_protocol);
    dut.status_protocol_flitfmt(st_protocol_flitfmt);
    dut.status_protocol_vld(st_protocol_vld);
    dut.status_speedmode(st_speedmode);
    dut.status_lnk_cfg(st_lnk_cfg);
    dut.status_phyinrecenter(st_phyinrecenter);
    dut.status_trainerror(st_trainerror);
    dut.status_ltsm_state(st_ltsm_state);

    SC_METHOD(split_tx_lanes);
    sensitive << mb_tx_data_int;

    SC_METHOD(merge_rx_lanes);
    sensitive << mb_rx_data_0  << mb_rx_data_1  << mb_rx_data_2  << mb_rx_data_3
              << mb_rx_data_4  << mb_rx_data_5  << mb_rx_data_6  << mb_rx_data_7
              << mb_rx_data_8  << mb_rx_data_9  << mb_rx_data_10 << mb_rx_data_11
              << mb_rx_data_12 << mb_rx_data_13 << mb_rx_data_14 << mb_rx_data_15;

    SC_METHOD(drive_observation);
    sensitive << clk.pos();

    SC_THREAD(drive_protocol_side);
    sensitive << clk.pos();

    SC_THREAD(sink_protocol_side);
    sensitive << clk.pos();
  }
};

#endif  // MADSIM_DIE_H
