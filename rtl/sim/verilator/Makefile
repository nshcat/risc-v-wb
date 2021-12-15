######################################################################
#
# DESCRIPTION: Verilator Example: Small Makefile
#
# This calls the object directory makefile.  That allows the objects to
# be placed in the "current directory" which simplifies the Makefile.
#
# Copyright 2003-2019 by Wilson Snyder. This program is free software; you can
# redistribute it and/or modify it under the terms of either the GNU
# Lesser General Public License Version 3 or the Perl Artistic License
# Version 2.0.
#

# IMPORTANT:
# For this to work, the MinGW64 verilator binaries need to be copied to the /bin subdirectory of
# VERILATOR_ROOT.
# The root directory has to be specified like this: 
# VERILATOR_ROOT=Y:/riscv-toolchain/msys2/mingw64/share/verilator
#
# Additionally, verilator tries to use localtime_r on mingw, which is not available.
# Therefore, in verilatedos.h, a macro has to be modified to replace its usage with
# localtime_s:
#  		# define VL_LOCALTIME_R(timep, tmp) localtime_s((tmp), (timep))
#

export VERILATOR_ROOT=Y:/riscv-toolchain/msys2/mingw64/share/verilator
GTKWAVEPATH?=Y:/riscv-toolchain/gtkwave64/bin/gtkwave.exe

# Configuration options
FEATURE_DBG_PORT?=OFF
FEATURE_RV32E?=OFF

# Build defines
DEFINES = 

ifeq ($(FEATURE_DBG_PORT),ON)
    DEFINES += +define+FEATURE_DBG_PORT
endif

ifeq ($(FEATURE_RV32E),ON)
    DEFINES += +define+FEATURE_RV32E
endif


######################################################################
# Check for sanity to avoid later confusion

ifneq ($(words $(CURDIR)),1)
 $(error Unsupported: GNU Make cannot build in directories containing spaces, build elsewhere: '$(CURDIR)')
endif

######################################################################
# Set up variables

# If $VERILATOR_ROOT isn't in the environment, we assume it is part of a
# package install, and verilator is in your path. Otherwise find the
# binary relative to $VERILATOR_ROOT (such as when inside the git sources).
ifeq ($(VERILATOR_ROOT),)
VERILATOR = verilator
VERILATOR_COVERAGE = verilator_coverage
else
export VERILATOR_ROOT
VERILATOR = $(VERILATOR_ROOT)/bin/verilator
VERILATOR_COVERAGE = $(VERILATOR_ROOT)/bin/verilator_coverage
endif

VERILATOR_FLAGS =
# Generate C++ in executable form
VERILATOR_FLAGS += -cc --exe

VERILATOR_FLAGS += $(DEFINES)

# Generate makefile dependencies (not shown as complicates the Makefile)
#VERILATOR_FLAGS += -MMD
# Optimize
VERILATOR_FLAGS += -O2 -x-assign 0
# Warn abount lint issues; may not want this on less solid designs
VERILATOR_FLAGS += -Wall -Wpedantic

VERILATOR_FLAGS += -I./../../soc -I./../../soc/core -I./../../soc/peripherals -I./..
# Make waveforms
VERILATOR_FLAGS += --trace
# Check SystemVerilog assertions
VERILATOR_FLAGS += --assert
# Generate coverage analysis
VERILATOR_FLAGS += --coverage

# Run Verilator in debug mode
#VERILATOR_FLAGS += --debug
# Add this trace to get a backtrace in gdb
#VERILATOR_FLAGS += --gdbbt

######################################################################
default: run

run:
	@echo
	@echo "-- Verilator tracing example"

	@echo
	@echo "-- VERILATE ----------------"
	$(VERILATOR) $(VERILATOR_FLAGS) -f input.vc ./../top_verilator.sv sim_main.cpp

	@echo
	@echo "-- COMPILE -----------------"
# To compile, we can either just do what Verilator asks,
# or call a submakefile where we can override the rules ourselves
#	$(MAKE) -j 4 -C obj_dir -f Vtop.mk
	$(MAKE) -j 24 -C obj_dir -f ../Makefile_obj

	@echo
	@echo "-- RUN ---------------------"
	@mkdir -p logs
	obj_dir/Vtop_verilator +trace

	@echo
	@echo "-- COVERAGE ----------------"
	$(VERILATOR_COVERAGE) --annotate logs/annotated logs/coverage.dat

	@echo
	@echo "-- DONE --------------------"
	@echo "To see waveforms, open vlt_dump.vcd in a waveform viewer"
	@echo

gtkwave:
	@$(GTKWAVEPATH) ./logs/vlt_dump.vcd

######################################################################
# Other targets

show-config:
	$(VERILATOR) -V

maintainer-copy::
clean mostlyclean distclean maintainer-clean::
	-rm -rf obj_dir logs *.log *.dmp *.vpd coverage.dat core
