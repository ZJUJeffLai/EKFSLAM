IDIR =/usr/include/python2.7
CFLAGS=-I$(IDIR)
LINK=-lpython2.7
STD=-std=c++11

programs := \
	test_sensor_read.x
	#EKFSLAM.x
	# other executables here

# FLAGS
# Debug flag
ifneq ($(D),1)
	CFLAGS	+= -O2
else
	CFLAGS	+= -O0
	CFLAGS	+= -g
endif
# Verbose
ifneq ($(V),1)
	Q = @
	V = 0
endif


# Current directory
CUR_PWD := $(shell pwd)
# Define compilation toolchain
CC = g++
# General gcc options
CFLAGS := -Wall -Werror
# Source files to use
src = $(wildcard src/*.cpp)
# Object files to compile based on source files
obj = $(src:.cpp=.o)

# Generate dependencies
DEPFLAGS = -MMD -MF $(@:.o=.d)

# Include dependencies
deps := $(patsubst %.o,%.d,$(obj))
-include $(deps)


# Debug makefile
#$(info $$CFLAGS is [${CFLAGS}])
$(info $$CUR_PWD is [${CUR_PWD}])
$(info $$src is [${src}])
$(info $$obj is [${obj}])
#$(info $$DEPFLAGS is [${DEPFLAGS}])
$(info $$deps is [${deps}])



#Default rule
all: $(programs)

# Generic rule for generating the static library
$(lib): $(obj)
	@echo "LIB	$@"
	$(Q)ar rcs $(lib) $(obj)

# Generic rule for compiling objects
%.o: %.cpp
	@echo "CC	$@"
	$(Q)$(CC) $(CFLAGS) -c -o $@ $< $(DEPFLAGS)

# Generic rule for linking final applications
%.x: %.o
	@echo "LD	$@"
	$(Q)$(CC) $(CFLAGS) -o $@ $<

## TODO ##
#EKFSLAM:

# Clean rule
clean:
	@echo "CLEAN	$(CUR_PWD)"
	$(Q)rm -f $(lib) $(obj) $(deps) $(programs)
