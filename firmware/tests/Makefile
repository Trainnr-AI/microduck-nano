# SPDX-License-Identifier: Apache-2.0
# Host-side unit tests for the firmware's portable core.   make -C firmware/tests
SRC := ../app/microduck/sketch/src
CXX ?= g++
CXXFLAGS ?= -std=c++14 -O1 -g -Wall -Wextra -Werror -fno-exceptions -fno-rtti
CORE := $(wildcard $(SRC)/duck/*.cpp)

VECTORS := ../../runtime/tests/data/leg_rule_vectors.txt

.PHONY: test leg-vectors trainnr-limits clean
test: build/duck_tests trainnr-limits
	./build/duck_tests

# The limits written for trainnr (tools/export_trainnr_limits.py) against this firmware's verdict on 60000 poses.
trainnr-limits: build/duck_tests
	./build/duck_tests --pose-verdicts | python3 ../tools/export_trainnr_limits.py --check-poses -

# The firmware's verdicts on 4000 leg poses, for the Rust implementation of the same rules to be checked against
# (runtime/tests). Run after any change to the leg rules or to config/robot.json, and commit the file.
leg-vectors: build/duck_tests
	./build/duck_tests --leg-vectors > $(VECTORS)

build/duck_tests: test_main.cpp fake_servos.h $(CORE) $(wildcard $(SRC)/duck/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -I$(SRC) test_main.cpp $(CORE) -o $@

clean:
	rm -rf build
