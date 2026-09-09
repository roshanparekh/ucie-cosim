# Knobs for the co-simulation. This is the only file you should need to edit;
# everything else in the Makefile is machinery. Any of these can also be
# overridden on the command line, e.g. `make run CYCLES=30000`.

# ---------------------------------------------------------------------------
# Run length
# ---------------------------------------------------------------------------
# Simulation cycles before the run ends on its own.
CYCLES ?= 8000000

# Finish early once nothing has advanced for this many cycles. 0 disables.
STALL  ?= 300000

# Signal-level tracing in the event log. Off by default so the log reads as the
# protocol exchange; 1 brings back the framing, deserializer and pattern probes.
VERBOSE ?= 0

# Payloads logged per direction; -1 logs every one. The rest are still checked,
# just not printed.
SHOW_FLITS ?= 20

# ---------------------------------------------------------------------------
# Traffic
# ---------------------------------------------------------------------------
# How many messages each die sends. 0 leaves that direction idle.
UCIE_TX_MSGS   ?= 2000
MADSIM_TX_MSGS ?= 2000

# When each die sends.
#   parallel      both start as soon as their FDI is ACTIVE
#   ucie_first    madsim waits until it has received every UcieTL message
#   madsim_first  UcieTL waits until it has received every madsim message
TRAFFIC_MODE ?= parallel

# Address and data are random. This seeds both dies, so a failing run repeats.
RANDOM_SEED ?= 1

# Leave TileLink A/D credit flow enabled. madsim is not a TileLink agent and
# returns no credits, so with this on the A channel spends its initial
# allowance of 63 beats and stops. 0 selects CreditCounter's no-flow mode.
CREDIT_FLOW ?= 0

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
# Sideband and die clock. The Verilog toggles every half period, so keep this
# even. Reaches both the SystemC wrapper and the Verilog, so they cannot drift.
CLK_PERIOD_PS ?= 1250

# Cycles UcieTL sits in RESET before it will look at the partner. Read at
# elaboration and baked into the RTL, so `make elab` is needed after a change.
# The Chisel default is 1,600,000, longer than a debug run and hard to tell
# from a hang.
UCIE_RESET_MIN_WAIT_CYCLES ?= 20000

# ---------------------------------------------------------------------------
# Paths, if the tree is laid out differently
# ---------------------------------------------------------------------------
MADSIM    ?= $(ROOT)/dut/madsim
UCIE      ?= $(ROOT)/dut/ucie
UCIE_RTL  ?= $(UCIE)/scala/build/UcieTL_should_generate_valid_SystemVerilog
UCIE_VSRC ?= $(UCIE)/scala/resources/vsrc
RESULTS   ?= $(ROOT)/results
