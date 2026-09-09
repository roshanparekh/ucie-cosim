# UcieTL + madsim co-simulation

Two UCIe dies wired together at the bump interface and trained against each
other:

    UcieTL (Berkeley, Chisel)  <-->  madsim (Stanford, SystemC)

UcieTL is die B and wakes on the sideband pattern its partner sends; madsim is
die A and self-starts out of reset. Both run from one clock, since UcieTL
clocks its sideband and mainband together and madsim has no fixed frequency
ratios. Sideband data and forwarded clocks now connect directly between
the dies, without the sideband shim.

Once the link is up, both dies push flits across it and a monitor checks them
at RDI and at the protocol-layer boundary.

## Layout

    variables.mk   knobs -- run length, traffic, clock. The file to edit.
    Makefile       build and run
    env/           tool paths and environment setup
    src/           testbench, SystemC wrapper, monitor, probe
    dut/ucie       Berkeley Chisel UCIe implementation
    dut/madsim     Stanford SystemC UCIe implementation
    build/         syscan and vcs output
    results/       run logs and waveforms

Neither die is copied or modified to build. The elaborated UcieTL RTL and the
analog models are read in place from `dut/ucie`.

## Running it

    # first time on a new machine, point env/site.sh at your tool installs
    source env/setup.sh
    make dut       # fetch both dies at their pinned commits and patch them
    make elab      # elaborate UcieTL. Needed again after any Chisel change.
    make run

`make run` builds if it needs to. A full run takes a few minutes. The log is
`results/run.log` and ends with:

    ---- end of run, cycle 362723 ----
      ucie -> madsim : 18779 flits sent, 18745 checked in order, 0 wrong
      madsim -> ucie : 2000 flits sent, 2000 checked in order, 0 wrong

Other targets:

    make cosim     build without running
    make wave      run with an FSDB dump
    make verdi     open the dump
    make clean     discard build/
    make patches   rewrite patches/ with whatever is in dut/ now

## Knobs

Everything worth changing is in `variables.mk`, and any of it can be overridden
on the command line:

    make run CYCLES=30000            short run
    make run VERBOSE=1               signal-level tracing in the log
    make run UCIE_TX_MSGS=10000      more TileLink traffic
    make run CREDIT_FLOW=1           leave TileLink credit flow on

`UCIE_TX_MSGS` and `MADSIM_TX_MSGS` reach both the SystemC wrapper and the
Verilog from that one file. `CLK_PERIOD_PS` sets their shared clock.

## Notes

Changing the Chisel needs `make elab` before `make run` -- the build does not
re-elaborate on its own, and a stale RTL against fresh models fails in ways
that look like an RTL bug.

Deviations from upstream madsim are listed in `MADSIM_MODIFICATIONS.txt`.
