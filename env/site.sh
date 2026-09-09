# Where the tools live on this machine. The only file you should need to edit.
# Values below are the BWRC installs. setup.sh derives everything else.

# Synopsys VCS.
VCS_INSTALL=/tools/synopsys/vcs/W-2024.09-1

# Synopsys Verdi. Only needed for `make wave` / `make verdi`.
VERDI_INSTALL=/tools/synopsys/verdi/W-2024.09-1

# Siemens Catapult. Only its header-only libraries are used, so no license is
# needed, but madsim will not compile without them.
CATAPULT_INSTALL=/tools/mentor/catapult/2023.1_1

# JDK 17 or newer, for the mill build. Verdi puts a Java 8 on PATH that would
# otherwise shadow it.
JDK_INSTALL=/usr/lib/jvm/java-17-openjdk

# Script that sets the FlexLM license variables. Leave empty if licenses are
# already in your environment.
LICENSE_SCRIPT=/tools/flexlm/flexlm.sh

# Conda install, used only when SITE_CXX is empty. Leave empty if `conda` is
# already on your PATH.
CONDA_INSTALL=/bwrcq/C/roshanparekh/miniforge3

# C++ compiler. syscan only accepts g++ 9.2.0, 9.5.0, 12.3.0 or 13.2.0 with
# SystemC 2.3.4. Point these at one of those, or leave them empty and setup.sh
# builds one with conda.
SITE_CXX=
SITE_CC=

# Where boost headers live, if SITE_CXX cannot find them itself. matchlib needs
# boost/static_assert.hpp and Catapult ships only the preprocessor subset.
# Usually already on the include path, so usually left empty.
SITE_EXTRA_INC=
