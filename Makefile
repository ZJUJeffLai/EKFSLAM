# For when we want to use matplotlib
IDIR = /usr/include/python2.7
LINK = -lpython2.7
STD = -std=c++11

TARGETS := \
	test_sensor_read.x \
	ekfslam.x
	# other executables here

SRCDIR = src
INCDIR = include
OBJDIR = obj
BINDIR = bin

SOURCES := $(wildcard $(SRCDIR)/*.cpp)

INCLUDES := $(wildcard $(INCDIR)/*.h)
INCLUDES += $(wildcard $(SRCDIR)/*.h)

# Dangerous, objects gets the cpp files and deletes them
#OBJECTS := $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

# Current directory
CUR_PWD := $(shell pwd)
# Define compilation toolchain
CC = g++
# General gcc options
CFLAGS := -Wall -Werror
CFLAGS += -I$(INCDIR)

# Flags: D,V,P
# Debug flag
ifneq ($(D),1)
CFLAGS += -O2
else
CFLAGS += -O0
CFLAGS += -g
endif
# Verbose
ifneq ($(V),1)
Q = @
V = 0
endif
# Debug makefile
ifeq ($(P),1)
$(info $$CUR_PWD is [${CUR_PWD}])
$(info $$SOURCES is [${SOURCES}])
$(info $$INCLUDES is [${INCLUDES}])
$(info $$OBJECTS is [${OBJECTS}])
$(info $$CFLAGS is [${CFLAGS}])
endif

# Default rule
all: $(TARGETS)

#$(BINDIR)/$(TARGET): $(OBJECTS)
#	@$(CC) $(OBJECTS) $(LFLAGS) -o $@
#	@echo "Linking complete!"

#$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.c
#	@$(CC) $(CFLAGS) -c $< -o $@
#	@echo "Compiled "$<" successfully!"

# Dumb rules
test_sensor_read.x:
	@echo "Compiling test_sensor_read..."
	$(Q)$(CC) -o bin/test_sensor_read.x src/test_sensor_read.cpp

# Main rule

# ekfslam rule shouldn't actually make an executable for this one
ekfslam.x:
	@echo "Compiling ekfslam..."
	$(Q)$(CC) -o bin/ekfslam.x src/ekfslam.cpp -I include/Eigen

# Clean rule
clean:
	@echo "CLEAN BIN: $(CUR_PWD)/$(BINDIR)"
	$(Q)rm -f $(BINDIR)/$(TARGETS)



# TODO CALVIN
# Makefile
# VISUALIZATION PART