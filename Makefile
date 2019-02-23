TARGETS := \
	test_sensor_read.x \
	main.x \
	# other executables here

SDIR = src
IDIR = include
ODIR = obj
BDIR = bin

# Current working directory
CWD := $(shell pwd)
# Define compilation toolchain
CC = g++
# General gcc options
FLAGS := -Wall -Werror -I$(IDIR)

# For when we want to use matplotlib
PYTHON_IDIR = /usr/include/python2.7
PYTHON_LINK = -lpython2.7
STD = -std=c++11
USE_PYTHON := $(STD) -I$(PYTHON_IDIR) $(PYTHON_LINK)

# Flags: D,V,P
# D = debug, for GDB
# V = verbose, print the compilation instructions
# P = print makefile debugging info
ifneq ($(D),1)
FLAGS += -O2
else
FLAGS += -O0
FLAGS += -g
endif
ifneq ($(V),1)
Q = @
V = 0
endif
ifeq ($(P),1)
$(info $$CWD is [${CWD}])
$(info $$FLAGS is [${FLAGS}])
$(info $$USE_PYTHON is [${USE_PYTHON}])
endif

# Rules:
all: $(TARGETS)

test_sensor_read.x: src/test_sensor_read.cpp
	@echo "Compiling" $@ "..."
	$(Q)$(CC) $(FLAGS) -o $(BDIR)/$@ $^

main.x: src/main.cpp src/ekfslam.cpp src/tools.cpp
	@echo "Compiling" $@ "..."
	$(Q)$(CC) $(FLAGS) -o $(BDIR)/$@ $^ $(USE_PYTHON)

clean:
	@echo "CLEAN ROOT DIR: $(CWD)"
	$(Q)rm -f *.o *.x core
	@echo "CLEAN BIN: $(CWD)/$(BDIR)"
	$(Q)rm -f $(BDIR)/*.o $(BDIR)/*.x $(BDIR)/core
	@echo "CLEAN OBJ: $(CWD)/$(ODIR)"
	$(Q)rm -f $(ODIR)/*.o $(ODIR)/*.x $(ODIR)/core
