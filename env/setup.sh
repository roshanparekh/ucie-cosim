# Environment for the madsim + UcieTL co-simulation.
#
#   source env/setup.sh
#   make run
#
# Reads tool locations from env/site.sh and derives everything the top-level
# Makefile, madsim's cmod_Makefile and the ucie mill build expect. Edit
# site.sh, not this file.

if [ -z "${BASH_SOURCE[0]}" ]; then
  echo "setup.sh: source this from bash, not sh or csh" >&2
  return 1 2>/dev/null || exit 1
fi

COSIM_ENV="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export COSIM_ROOT="$(dirname "$COSIM_ENV")"

if [ ! -f "$COSIM_ENV/site.sh" ]; then
  echo "setup.sh: env/site.sh is missing" >&2
  return 1
fi
. "$COSIM_ENV/site.sh"

__warn() { echo "setup.sh: $*" >&2; }
__need() { [ -e "$1" ] || __warn "$2 not found at $1"; }

# --- licenses -------------------------------------------------------------
if [ -n "$LICENSE_SCRIPT" ]; then
  if [ -f "$LICENSE_SCRIPT" ]; then . "$LICENSE_SCRIPT"
  else __warn "license script not found at $LICENSE_SCRIPT"; fi
fi

# --- VCS ------------------------------------------------------------------
__need "$VCS_INSTALL" "VCS"
export VCS_HOME="$VCS_INSTALL"
export PATH="$VCS_HOME/bin:$PATH"

# Only the 64-bit build is installed at BWRC, so every vcs and syscan call
# needs -full64.
export COSIM_VCS_FLAGS="-full64"
export COSIM_SYSC_VER="2.3.4"
# VCS ships non-PIC objects; a compiler defaulting to PIE cannot link them.
export COSIM_LDFLAGS="-no-pie"
# SystemC and Verilog must share a time base or the run aborts with SC-TRES-E.
export COSIM_TIMESCALE="1ps/1ps"

# --- Verdi ----------------------------------------------------------------
if [ -n "$VERDI_INSTALL" ] && [ -d "$VERDI_INSTALL" ]; then
  export VERDI_HOME="$VERDI_INSTALL"
  export PATH="$VERDI_HOME/bin:$PATH"
  # Some flows key off these rather than VERDI_HOME.
  export NOVAS_HOME="$VERDI_HOME" NOVAS_INST_DIR="$VERDI_HOME"
  export LD_LIBRARY_PATH="$VERDI_HOME/share/FsdbWriter/linux64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# --- Java -----------------------------------------------------------------
# Verdi puts a Java 8 on PATH, and mill dies on it with "class file version
# 61.0 ... recognizes up to 52.0". Pin a modern JDK in front. Verdi finds its
# own JRE by absolute path, so it does not care.
if [ -x "$JDK_INSTALL/bin/java" ]; then
  export JAVA_HOME="$JDK_INSTALL"
  export PATH="$JAVA_HOME/bin:$PATH"
else
  __warn "JDK not found at $JDK_INSTALL; mill will fail"
fi

# --- Catapult headers (madsim dependencies) -------------------------------
__need "$CATAPULT_INSTALL" "Catapult"
export CATAPULT_HOME="$CATAPULT_INSTALL"
__shared="$CATAPULT_HOME/Mgc_home/shared"
export BOOST_INCL="$__shared/pkgs/boostpp/pp/include"
export BOOST_HOME="$__shared/pkgs/boostpp/pp"
export CONNECTIONS_HOME="$__shared"
export RAPIDJSON_HOME="$__shared"
export MATCHLIB_HOME="$__shared/pkgs/matchlib/cmod"

# Standalone madsim unit tests (make in dut/madsim/cmod/*) link Catapult's own
# libsystemc. The co-simulation does not -- it uses the VCS kernel.
export SYSTEMC_HOME="$__shared"
export REPO_TOP="$COSIM_ROOT/dut/madsim"
export SYSC_HOME="$REPO_TOP/cmod"
export LD_LIBRARY_PATH="$__shared/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# --- compiler -------------------------------------------------------------
# syscan qualifies its SystemC build against specific g++ versions and refuses
# anything else. If site.sh names one, use it; otherwise build one with conda.
export COSIM_EXTRA_INC="$SITE_EXTRA_INC"

if [ -n "$SITE_CXX" ]; then
  export COSIM_CXX="$SITE_CXX" COSIM_CC="$SITE_CC"
else
  __base="$CONDA_INSTALL"
  if [ -z "$__base" ]; then
    if ! command -v conda >/dev/null 2>&1; then
      __warn "site.sh sets neither SITE_CXX nor CONDA_INSTALL, and conda is not on PATH"
      return 1
    fi
    __base="$(conda info --base 2>/dev/null)"
  fi
  if [ ! -f "$__base/etc/profile.d/conda.sh" ]; then
    __warn "could not locate the conda base"
    return 1
  fi
  . "$__base/etc/profile.d/conda.sh"

  COSIM_CONDA_ENV="${COSIM_CONDA_ENV:-ucie-cosim}"
  if ! conda env list | awk '{print $1}' | grep -qx "$COSIM_CONDA_ENV"; then
    echo "setup.sh: creating conda env '$COSIM_CONDA_ENV' (first run, a few minutes)"
    __solver=conda; command -v mamba >/dev/null 2>&1 && __solver=mamba
    "$__solver" env create -n "$COSIM_CONDA_ENV" -f "$COSIM_ENV/environment.yml" \
      || { __warn "conda env creation failed"; return 1; }
  fi
  conda activate "$COSIM_CONDA_ENV" || { __warn "could not activate $COSIM_CONDA_ENV"; return 1; }

  # conda's compiler activation exports -Wl,--as-needed and -Wl,--gc-sections.
  # VCS builds its own link line and those break resolution of its circular
  # static archives, giving a wall of undefined vhpi_*/mhpi_* symbols.
  unset LDFLAGS CFLAGS CXXFLAGS CPPFLAGS DEBUG_CFLAGS DEBUG_CXXFLAGS FFLAGS DEBUG_FFLAGS

  # The conda toolchain only installs target-prefixed names. A bare `g++` would
  # silently resolve to the system one, which syscan rejects.
  export COSIM_CXX="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++"
  export COSIM_CC="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc"
  export COSIM_EXTRA_INC="$CONDA_PREFIX/include"
fi

export CXX="$COSIM_CXX" CC="$COSIM_CC"

if [ ! -x "$COSIM_CXX" ]; then
  __warn "compiler not executable: $COSIM_CXX"
else
  __ver="$("$COSIM_CXX" -dumpfullversion 2>/dev/null)"
  case " 9.2.0 9.5.0 12.3.0 13.2.0 " in
    *" $__ver "*) ;;
    *) __warn "g++ $__ver is not qualified for SystemC $COSIM_SYSC_VER; syscan will refuse it" ;;
  esac
fi

# --- ucie -----------------------------------------------------------------
# ChiselSim defaults to Verilator, but can use VCS at BWRC. The repo
# reads this switch in scala/test/src/UcieSimBackend.scala.
# If you want to use Verilator instead, add verilator, or comment out the 
# export and make sure it is installed and on PATH.
export UCIE_SIM_BACKEND=vcs

unset __shared __base __solver __ver
unset -f __warn __need

echo "cosim env ready"
echo "  root    : $COSIM_ROOT"
echo "  vcs     : $(basename "$VCS_HOME")  systemc $COSIM_SYSC_VER"
echo "  g++     : $("$COSIM_CXX" -dumpfullversion 2>/dev/null) ($COSIM_CXX)"
echo "  catapult: $(basename "$CATAPULT_HOME")"
echo "  java    : $(java -version 2>&1 | head -1 | sed 's/.*version //')"
