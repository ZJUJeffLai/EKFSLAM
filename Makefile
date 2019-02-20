# For when we want to use matplotlib
IDIR = /usr/include/python2.7
LINK = -lpython2.7
STD = -std=c++11

TARGETS := \
	test_sensor_read.x
	#EKFSLAM.x
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
#$(info $$CFLAGS is [${CFLAGS}])
$(info $$CUR_PWD is [${CUR_PWD}])
$(info $$SOURCES is [${SOURCES}])
$(info $$INCLUDES is [${INCLUDES}])
$(info $$OBJECTS is [${OBJECTS}])
$(info $$CFLAGS is [${CFLAGS}])

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
	$(Q)$(CC) -o bin/test_sensor_read.x src/test_sensor_read.cpp

# Clean rule
clean:
	@echo "CLEAN BIN: $(CUR_PWD)/$(BINDIR)"
	$(Q)rm -f $(BINDIR)/$(TARGETS)
	# Don't delete object files because we are lazy
	#@echo "CLEAN OBJ: $(CUR_PWD)/$(OBJDIR)"
	#$(Q)rm -f $(OBJDIR)/$(OBJECTS)

# TODO CALVIN
# Makefile
# VISUALIZATION PART