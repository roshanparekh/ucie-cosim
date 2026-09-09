// ucie_sb_shim.sv
//
// Sideband adapter between the madsim die and UcieTL. Same data at the same
// rate; only the second wire differs. madsim uses a level valid, high for the
// 64 cycles of a burst and low for the 32 cycle gap. UcieTL uses a gated
// forwarded clock that toggles only while a packet is shifting.
//
// Both dies share one sideband clock. Every flop here runs on the falling
// edge. madsim drives on the rising edge, and through syscan the SystemC
// update has no defined order against a Verilog always_ff.
//
// RX_ALIGN adds one sideband cycle to the UcieTL-to-madsim pair.

module ucie_sb_shim #(
    parameter int RX_ALIGN = 0
) (
    input  wire sb_clk,     // shared sideband clock, drives both dies
    input  wire rst_n,      // active low

    // ---- madsim side -------------------------------------------------------
    input  wire mad_sb_tx_data,   // madsim drives, level valid qualified
    input  wire mad_sb_tx_val,
    output wire mad_sb_rx_data,   // madsim samples on posedge sb_clk
    output wire mad_sb_rx_val,

    // ---- UcieTL side -------------------------------------------------------
    output wire ucie_sb_rx_data,  // into UcieTL io_phy_sbRxData
    output wire ucie_sb_rx_clk,   // into UcieTL io_phy_sbRxClk
    input  wire ucie_sb_tx_data,  // from UcieTL io_phy_sbTxData
    input  wire ucie_sb_tx_clk    // from UcieTL io_phy_sbTxClk
);

  // =========================================================================
  // madsim to UcieTL. Level valid becomes a gated forwarded clock.
  // =========================================================================
  // Register the pair on the falling edge, mid-eye. A plain
  // `sb_clk & mad_sb_tx_val` gate does not work. madsim writes sb_tx_val false
  // then true within a cycle (SbTx.h:82, :208) and syscan does not collapse the
  // two writes, so the valid dips every rising edge and adds a clock edge.
  // Reset folds into the enable, not the clock. Nothing downstream re-frames,
  // so one stray edge shifts every word after it.
  logic mad_gate, mad_data_n;
  always_ff @(negedge sb_clk or negedge rst_n) begin
    if (!rst_n) begin
      mad_gate   <= 1'b0;
      mad_data_n <= 1'b0;
    end else begin
      mad_gate   <= mad_sb_tx_val;
      mad_data_n <= mad_sb_tx_data;
    end
  end

  assign ucie_sb_rx_clk = sb_clk & mad_gate;

  // Hold the bit flat for the whole period, the way UcieTL's sb_driver does,
  // so the sampling falling edge lands mid-plateau.
  //
  //   negedge N    mad_data_n <= bit N, mad_gate <= val(N)
  //   posedge N+1  mad_data_p <= bit N, gate open
  //   negedge N+1  UcieTL samples bit N
  //
  // A 64 cycle burst of bits 0..63 comes out as pulses in cycles 1..64.
  logic mad_data_p;
  always_ff @(posedge sb_clk or negedge rst_n) begin
    if (!rst_n) mad_data_p <= 1'b0;
    else        mad_data_p <= mad_data_n;
  end

  assign ucie_sb_rx_data = mad_data_p;

  // =========================================================================
  // UcieTL to madsim. Gated forwarded clock becomes a level valid.
  // =========================================================================
  // ucie_sb_tx_clk falls at the same instant sb_clk does, so sampling it there
  // races the AND inside sb_driver and can drop the enable entirely. Its rising
  // edge is clean, so toggle on that and read the toggle half a period later;
  // comparing against the delayed copy gives back the level madsim wants.
  logic tx_tog;
  always_ff @(posedge ucie_sb_tx_clk or negedge rst_n) begin
    if (!rst_n) tx_tog <= 1'b0;
    else        tx_tog <= ~tx_tog;
  end

  // Data holds for the whole period, so the falling edge is mid-eye here too.
  logic tx_tog_q, tx_val_neg, tx_dat_neg;
  always_ff @(negedge sb_clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_tog_q   <= 1'b0;
      tx_val_neg <= 1'b0;
      tx_dat_neg <= 1'b0;
    end else begin
      tx_tog_q   <= tx_tog;
      tx_val_neg <= tx_tog ^ tx_tog_q;
      tx_dat_neg <= ucie_sb_tx_data;
    end
  end

  generate
    if (RX_ALIGN == 0) begin : g_align0
      assign mad_sb_rx_val  = tx_val_neg;
      assign mad_sb_rx_data = tx_dat_neg;
    end else begin : g_align1
      // One more full cycle, on the falling edge again so the pair is settled
      // for madsim's rising-edge sampler.
      logic tx_val_r, tx_dat_r;
      always_ff @(negedge sb_clk or negedge rst_n) begin
        if (!rst_n) begin
          tx_val_r <= 1'b0;
          tx_dat_r <= 1'b0;
        end else begin
          tx_val_r <= tx_val_neg;
          tx_dat_r <= tx_dat_neg;
        end
      end
      assign mad_sb_rx_val  = tx_val_r;
      assign mad_sb_rx_data = tx_dat_r;
    end
  endgenerate

endmodule
