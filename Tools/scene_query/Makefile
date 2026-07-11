# scene_query — host-side dump of procedural SDF scene primitives.
#
# Standalone make rule (not part of the PX4 cmake tree). Builds against
# the lib/terrain + lib/terrain_sdf sources directly, so any change to
# the scene_wall hash chain or terrain.c integer hash is reflected
# without a separate PX4 rebuild.
#
# Build:    make
# Run:      ./scene_query --seed 622 --wall-n 1
# Clean:    make clean

CXX     ?= clang++
# -fno-exceptions / -fno-threadsafe-statics mirror what the firmware build
# gives these sources, so the host tool exercises the same code generation.
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Wpedantic -Wno-unused-parameter \
            -fno-exceptions -fno-threadsafe-statics
ROOT    := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
SRCS    := \
	scene_query.cpp \
	$(ROOT)/src/lib/terrain/terrain.cpp \
	$(ROOT)/src/lib/terrain_sdf/scene.cpp

scene_query: $(SRCS)
	$(CXX) $(CXXFLAGS) -I$(ROOT) $(SRCS) -lm -o $@

clean:
	rm -f scene_query

.PHONY: clean
