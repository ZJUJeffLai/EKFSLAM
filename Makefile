# For when we want to use matplotlib
IDIR = /usr/include/python2.7
LINK = -lpython2.7
STD = -std=c++11

TARGETS := \
	test_sensor_read \
	main\
	#ekfslam.x
	# other executables here

SRCDIR = src
INCDIR = include
OBJDIR = obj
BINDIR = bin

SOURCES := $(wildcard $(SRCDIR)/*.cpp)

INCLUDES := $(wildcard $(INCDIR)/*.h)
INCLUDES += $(wildcard $(SRCDIR)/*.h)
# Gets python?
INCLUDES += $(wildcard $(IDIR)/*.h)
# Not useful
INCLUDES += $(IDIR)

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
test_sensor_read:
	@echo "Compiling test_sensor_read..."
	$(Q)$(CC) -o bin/test_sensor_read src/test_sensor_read.cpp

# ekfslam object file
obj/ekfslam.o: src/ekfslam.cpp
	@echo "Compiling ekfslam..."
	$(Q)$(CC) -c -o obj/ekfslam.o src/ekfslam.cpp -I include/Eigen

# Main
main: src/main.cpp
	@echo "Compiling main..."
	$(Q)$(CC) $(CFLAGS) -o bin/main src/main.cpp src/ekfslam.cpp src/tools.cpp -std=c++11 -I/usr/include/python2.7 -lpython2.7

# Clean rule
clean:
	@echo "CLEAN BIN: $(CUR_PWD)/$(BINDIR)"
	$(Q)rm -f $(BINDIR)/$(TARGETS)
	@echo "CLEAN OBJ: $(CUR_PWD)/$(OBJDIR)"
	$(Q)rm -f $(OBJDIR)/*.*



# TODO CALVIN
# Makefile
# VISUALIZATION PART