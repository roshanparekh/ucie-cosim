# UcieTL + madsim co-simulation. See README.md for layout and usage.
#
# Tunable variables are in variables.mk. Nothing below should need editing.

ROOT  := $(CURDIR)
SRC   := $(ROOT)/src
BUILD := $(ROOT)/build

include $(ROOT)/variables.mk

CMOD  := $(MADSIM)/cmod
WAVES := $(RESULTS)/waveforms

CAT_SHARED ?= $(CATAPULT_HOME)/Mgc_home/shared

# Set by env/setup.sh. The g++ compiler ver is not a free!!! syscan rejects any
# g++ it has not qualified against its SystemC build.
CXXBIN    ?= $(COSIM_CXX)
CCBIN     ?= $(COSIM_CC)
SYSC_VER  ?= $(COSIM_SYSC_VER)
VCSFLAGS  ?= $(COSIM_VCS_FLAGS)
LDF       ?= $(COSIM_LDFLAGS)
TIMESCALE ?= $(COSIM_TIMESCALE)
EXTRA_INC ?= $(COSIM_EXTRA_INC)

TOP  := ucie_cosim_top
SIMV := simv_cosim
FSDB := $(WAVES)/ucie_cosim.fsdb

TB_SRCS := $(SRC)/ucie_sb_shim.sv $(SRC)/cosim_monitor.sv $(SRC)/$(TOP).sv \
           $(SRC)/cosim_probe.sv

.PHONY: help elab cosim run wave verdi check-tree defs-stamp clean distclean
.DEFAULT_GOAL := help

help:
	@echo "source env/setup.sh, then:"
	@echo "  make dut     clone both dies at their pinned commits and patch them"
	@echo "  make elab    elaborate UcieTL (needed after a Chisel change)"
	@echo "  make cosim   build the co-simulation"
	@echo "  make run     build and run"
	@echo "  make wave    build and run with an FSDB dump"
	@echo "  make verdi   open the dump"
	@echo "  make clean   discard build/"
	@echo ""
	@echo "knobs are in variables.mk, or override them: make run CYCLES=30000"

SC_DEFS := -DHLS_ALGORITHMICC -DHLS_CATAPULT -DVECTOR_SIZE=9 -DARRAY_SIZE=6 \
           -DCONNECTIONS_ACCURATE_SIM -DSC_INCLUDE_DYNAMIC_PROCESSES \
           -DPARAM_NEG_TIMEOUT_CYCLES=50000

TRAFFIC_MODE_NUM := $(strip $(if $(filter ucie_first,$(TRAFFIC_MODE)),1,\
                            $(if $(filter madsim_first,$(TRAFFIC_MODE)),2,0)))

SC_DEFS += -DCOSIM_CLK_PERIOD_PS=$(CLK_PERIOD_PS) \
           -DCOSIM_MADSIM_TX_MSGS=$(MADSIM_TX_MSGS) \
           -DCOSIM_UCIE_TX_MSGS=$(UCIE_TX_MSGS) \
           -DCOSIM_TRAFFIC_MODE=$(TRAFFIC_MODE_NUM) \
           -DCOSIM_RANDOM_SEED=$(RANDOM_SEED)

COSIM_PARAM_DEFS := +define+COSIM_CLK_PERIOD_PS=$(CLK_PERIOD_PS) \
                    +define+COSIM_MADSIM_TX_MSGS=$(MADSIM_TX_MSGS) \
                    +define+COSIM_UCIE_TX_MSGS=$(UCIE_TX_MSGS) \
                    +define+COSIM_TRAFFIC_MODE=$(TRAFFIC_MODE_NUM) \
                    +define+COSIM_RANDOM_SEED=$(RANDOM_SEED) \
                    +define+COSIM_SHOW_FLITS=$(SHOW_FLITS)

SC_INCS := -I$(SRC) -I$(CMOD)/ucie_top -I$(CMOD)/include -I$(CMOD)/d2d \
           -I$(CMOD)/phy_top -I$(CMOD)/phy_top/include -I$(CMOD)/protocol_layer \
           -I$(CAT_SHARED)/include \
           -I$(CAT_SHARED)/pkgs/matchlib/cmod/include \
           -I$(CAT_SHARED)/pkgs/boostpp/pp/include \
           $(if $(EXTRA_INC),-I$(EXTRA_INC))

SC_CFLAGS := -std=c++11 -Wno-unknown-pragmas -Wno-unused-local-typedefs \
             -Wno-deprecated-declarations

# Both are submodules pinned at an upstream commit, with patches applied from
# patches/
.PHONY: dut patches
dut:
	git submodule update --init --recursive
	@git -C $(UCIE) apply --check $(ROOT)/patches/ucie.patch 2>/dev/null \
	  && git -C $(UCIE) apply $(ROOT)/patches/ucie.patch \
	  && echo "ucie patched" || echo "ucie already patched, skipping"
	@git -C $(MADSIM) apply --check $(ROOT)/patches/madsim.patch 2>/dev/null \
	  && git -C $(MADSIM) apply $(ROOT)/patches/madsim.patch \
	  && echo "madsim patched" || echo "madsim already patched, skipping"

patches:
	@mkdir -p $(ROOT)/patches
	@git -C $(UCIE) add -N . >/dev/null 2>&1 || true
	@git -C $(UCIE) diff HEAD -- . ':!scala/testchipip' > $(ROOT)/patches/ucie.patch
	@git -C $(MADSIM) add -N . >/dev/null 2>&1 || true
	@git -C $(MADSIM) diff HEAD > $(ROOT)/patches/madsim.patch
	@echo "patches/ucie.patch   $$(grep -c '^diff --git' $(ROOT)/patches/ucie.patch) files"
	@echo "patches/madsim.patch $$(grep -c '^diff --git' $(ROOT)/patches/madsim.patch) files"

ELAB_ENV := UCIE_RESET_MIN_WAIT_CYCLES=$(UCIE_RESET_MIN_WAIT_CYCLES)

elab:
	rm -rf $(UCIE_RTL)
	cd $(UCIE)/scala && $(ELAB_ENV) ./mill test.testOnly \
	    edu.berkeley.cs.uciedigital.tilelink.TileLinkSpec -- \
	    -z "should generate valid SystemVerilog"
	@echo "elaborated into $(UCIE_RTL) with $(ELAB_ENV)"

# The top-level name is not stable across upstream pulls; see the clean in elab.
$(UCIE_RTL)/UcieTL.sv:
	@echo "UcieTL RTL not found. Run: make elab" >&2; exit 1

$(BUILD)/rtl.f: $(UCIE_RTL)/UcieTL.sv
	@mkdir -p $(BUILD)
	@ls $(UCIE_RTL)/*.sv > $@.abs
	@ls $(UCIE_VSRC)/*.v $(UCIE_VSRC)/*.sv >> $@.abs 2>/dev/null || true
	@realpath --relative-to=$(BUILD) $$(cat $@.abs) > $@
	@rm -f $@.abs
	@echo "rtl.f: $$(wc -l < $@) files, relative to $(BUILD)"

REG_INIT_DEFS ?= +define+ENABLE_INITIAL_REG_ +define+ENABLE_INITIAL_MEM_ \
                 +define+RANDOMIZE_REG_INIT +define+RANDOMIZE_MEM_INIT \
                 +define+RANDOM=0

ifeq ($(VERBOSE),1)
REG_INIT_DEFS += +define+COSIM_VERBOSE
endif

# Every define that reaches vcs, in one place.
ALL_DEFS := $(REG_INIT_DEFS) $(COSIM_PARAM_DEFS) +define+UCIE_RX_SAMPLE_SHIFT_UI

# The analog models and madsim's headers are compiled in, so edits to either
# must force a rebuild. rtl.f only regenerates when the elaborated RTL changes.
VSRC_FILES  := $(wildcard $(UCIE_VSRC)/*.v) $(wildcard $(UCIE_VSRC)/*.sv)
MADSIM_HDRS := $(shell find $(CMOD) -name '*.h' 2>/dev/null)

# syscan and vcs bake absolute paths into csrc, AN.DB and the .daidir and never
# regenerate them, so discard that output when the tree moves.
# Has to be its own make invocation. As a prereq it would wipe rtl.f
# after make had already stat'd it, and vcs would then die on a file make still
# believes is there. The stamp lives outside build/ so the wipe cannot remove it.
PATH_STAMP := $(RESULTS)/.build_path
check-tree:
	@mkdir -p $(RESULTS)
	@if [ -s $(PATH_STAMP) ] && [ "$$(cat $(PATH_STAMP))" != "$(ROOT)" ]; then \
	    echo "build tree moved: $$(cat $(PATH_STAMP)) -> $(ROOT)"; \
	    echo "  discarding syscan/vcs output that recorded the old path"; \
	    rm -rf $(BUILD); \
	  fi
	@printf '%s\n' '$(ROOT)' > $(PATH_STAMP)

# A define stamp, so a flag changed on the command line forces a relink.
# Makefile and variables.mk as prerequisites catch an edited flag, but not
# `make run VERBOSE=1`, which changes no file make tracks.
DEFS_STAMP := $(RESULTS)/.build_defs
defs-stamp:
	@mkdir -p $(RESULTS)
	@printf '%s\n' '$(ALL_DEFS)' > $(DEFS_STAMP).new
	@# SC_DEFS goes to syscan's -cflags, never to vcs, so it is recorded here
	@# rather than added to ALL_DEFS.
	@printf '%s\n' '$(SC_DEFS)' >> $(DEFS_STAMP).new
	@if ! cmp -s $(DEFS_STAMP).new $(DEFS_STAMP); then \
	    mv $(DEFS_STAMP).new $(DEFS_STAMP); \
	    echo "build defines changed -> relink"; \
	  else rm -f $(DEFS_STAMP).new; fi
$(DEFS_STAMP): defs-stamp ;

$(BUILD)/$(SIMV): $(SRC)/madsim_die.h $(TB_SRCS) $(BUILD)/rtl.f \
                  Makefile variables.mk \
                  $(VSRC_FILES) $(MADSIM_HDRS) $(DEFS_STAMP)
	@mkdir -p $(BUILD) $(RESULTS)
	cd $(BUILD) && syscan $(VCSFLAGS) -cpp $(CXXBIN) -cc $(CCBIN) -sysc=$(SYSC_VER) \
	    -cflags "$(SC_CFLAGS) $(SC_DEFS) $(SC_INCS)" \
	    $(SRC)/madsim_die.h:madsim_die
	cd $(BUILD) && vcs $(VCSFLAGS) -sysc=$(SYSC_VER) -cpp $(CXXBIN) -cc $(CCBIN) \
	    -LDFLAGS "$(LDF)" -sverilog -timescale=$(TIMESCALE) \
	    $(ALL_DEFS) \
	    $(EXTRA_VCS) -f rtl.f $(TB_SRCS) -o $(SIMV)

cosim:
	@$(MAKE) --no-print-directory check-tree
	@$(MAKE) --no-print-directory $(BUILD)/$(SIMV)

RUNARGS := +run_cycles=$(CYCLES) +stall_cycles=$(STALL) \
               +credit_flow=$(CREDIT_FLOW)

# Refuses to run a binary older than its sources. A leftover from an earlier
# build would otherwise report plausible results for code that was never built.
run:
	@$(MAKE) --no-print-directory check-tree
	@$(MAKE) --no-print-directory $(BUILD)/$(SIMV)
	@mkdir -p $(RESULTS)
	@for f in $(SRC)/madsim_die.h $(TB_SRCS) $(BUILD)/rtl.f Makefile variables.mk $(VSRC_FILES); do \
	    if [ "$$f" -nt $(BUILD)/$(SIMV) ]; then \
	        echo "ERROR: $$f is newer than $(SIMV) -- the build did not succeed." >&2; \
	        echo "Refusing to run a stale binary." >&2; exit 1; \
	    fi; \
	done
	cd $(BUILD) && ./$(SIMV) $(RUNARGS) 2>&1 | tee $(RESULTS)/run.log

# Verdi 2024.09 wants -debug_access with VERDI_HOME exported, which env/setup.sh
# does. The old '-P novas.tab pli.a' form makes it refuse to dump while the run
# still exits zero, so the only symptom is a missing file.
wave:
	@mkdir -p $(WAVES) $(RESULTS)
	@$(MAKE) --no-print-directory check-tree
	$(MAKE) $(BUILD)/$(SIMV) EXTRA_VCS='+define+DUMP_FSDB +define+DUMP_FILE=\"$(FSDB)\" -debug_access+all -kdb $(EXTRA_VCS)'
	cd $(BUILD) && ./$(SIMV) $(RUNARGS) 2>&1 | tee $(RESULTS)/run.log
	@echo "waveform: $(FSDB)"

verdi:
	@mkdir -p $(RESULTS)/verdiLog
	cd $(RESULTS) && verdi -ssf $(FSDB) -nologo &

clean:
	rm -rf $(BUILD)
	rm -f $(DEFS_STAMP) $(PATH_STAMP)

distclean: clean
	rm -rf $(RESULTS)
