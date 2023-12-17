# Ensure that there are no spaces in path to avoid any undesired behavior
ifneq ($(word 2,$(realpath $(lastword $(MAKEFILE_LIST)))),)
$(error "ERROR: folder path contains spaces")
endif

MAKEFILEDIR := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))
MAKEFILEDIRNAME := $(basename $(MAKEFILEDIR))

DIVIDER_SV_SOURCES := ./ips/divider.v testbed/testbed_divider.sv
SIMDIR := $(MAKEFILEDIR)_sim/
SYNTHDIR := $(MAKEFILEDIR)_synth/
ASMDIR := $(MAKEFILEDIR)/testcases/
ASMSOURCES := 
RTLSOURCES := $(wildcard $(MAKEFILEDIR)/testbed/*.sv) $(wildcard $(MAKEFILEDIR)/ips/*.sv)
CPUNAME := divider
CPUROOT :=
TEST_DIFFS = $(ASMSOURCES:$(ASMDIR)%.asm=$(SIMDIR)%.isa.diff) $(ASMSOURCES:$(ASMDIR)%.asm=$(SIMDIR)%.rtl.diff)

IVERILOGFLAGS ?= -Wall -g2012 -Wno-sensitivity-entire-array -I$(MAKEFILEDIR)/ips
RISCVSIMFLAGS ?= --trace
GTKWAVESAVE   ?= $(MAKEFILEDIR)$(CPUNAME).gtkw

ifeq (, $(shell which iverilog))
  IVERILOG      := iverilog-11
else
  IVERILOG      := iverilog
endif

GTKWAVE  ?= gtkwave
YOSYS    ?= yosys

#help
#helpThe main targets of this Makefile are:
#help	all	generates the verilog simulator and assembles the test programs
all: $(SIMDIR)$(CPUNAME).vvp

#help	clean	clean up temporary files
.PHONY: clean
clean:
	rm -rf $(SIMDIR)
	rm -rf $(SYNTHDIR)
	rm -rf __pycache__

#help	help	displays this help
.PHONY: help
help:
	@cat ${MAKEFILE_LIST} | sed -n 's/^#help//p'

###############################################################################
# RTL SIMULATION
###############################################################################
LOG_ARGS = > $*.rtl.log
.PRECIOUS: $(SIMDIR)%.vcd $(SIMDIR)%.rtl.stdout $(SIMDIR)%.rtl.log
$(SIMDIR)%.vcd $(SIMDIR)%.rtl.stdout $(SIMDIR)%.rtl.log: $(SIMDIR)%.hex $(SIMDIR)$(CPUNAME).vvp $(wildcard $(ASMDIR)*.stdin)
	cp $(ASMDIR)$*.stdin $(SIMDIR)stdin.txt || touch $(SIMDIR)stdin.txt
	cd $(SIMDIR) && ./$(CPUNAME).vvp +PROGRAM=$*.hex $(LOG_ARGS)
	mv $(SIMDIR)$(CPUNAME).vcd $(SIMDIR)$*.vcd
	mv $(SIMDIR)stdout.txt $(SIMDIR)$*.rtl.stdout

$(SIMDIR)divider.vvp: $(DIVIDER_SV_SOURCES)
	@mkdir -p $(SIMDIR)
	$(IVERILOG) $(IVERILOGFLAGS) -o $@ $(DIVIDER_SV_SOURCES)

#help
#help	divider                         builds the divider module alone
divider: $(SIMDIR)divider.vvp ;

#help	run_divider                     runs the divider in the isolated testbed
#help
run_divider: $(SIMDIR)divider.vvp
	$(SIMDIR)divider.vvp

###############################################################################
# INTERACTIVE TARGETS
###############################################################################
#help

#help	run TARGET=<target_name>	runs the verilog simulation of the target
run:
ifeq (${TARGET},)
	@echo "ERROR: No TARGET has been specified!"
	@echo "Usage: make $@ TARGET=<target_name>"
	@exit 1
endif # TARGET is empty
	rm -f $(SIMDIR)${TARGET}.rtl.log
	@$(MAKE) --no-print-directory $(SIMDIR)${TARGET}.rtl.log TARGET=$(TARGET) LOG_ARGS="| tee \$$*.rtl.log"

#help	view TARGET=<target_name>	views the waveforms from the verilog simulation of the target
view:
ifeq (${TARGET},)
	@echo "ERROR: No TARGET has been specified!"
	@echo "Usage: make $@ TARGET=<target_name>"
	@exit 1
endif # TARGET is empty
	@$(MAKE) --no-print-directory $(SIMDIR)${TARGET}.vcd TARGET=$(TARGET)
	$(GTKWAVE) --save $(CPUROOT)$(CPUNAME).gtkw $(SIMDIR)$(TARGET).vcd

#help 	synth    			synthesizes the design with yosys
synth:
	@mkdir -p $(SYNTHDIR)
	sv2v --incdir ips -E logic ips/*.v -w $(SYNTHDIR)/divider.v
	$(YOSYS) -T synth.ys
