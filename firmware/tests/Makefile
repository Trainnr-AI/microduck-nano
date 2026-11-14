# SPDX-License-Identifier: Apache-2.0
# Host-side unit tests for the firmware's portable core.   make -C firmware/tests
SRC := ../app/microduck/sketch/src
CXX ?= g++
CXXFLAGS ?= -std=c++14 -O1 -g -Wall -Wextra -Werror -fno-exceptions -fno-rtti
CORE := $(wildcard $(SRC)/duck/*.cpp)

.PHONY: test clean
test: build/duck_tests
	./build/duck_tests

build/duck_tests: test_main.cpp fake_servos.h $(CORE) $(wildcard $(SRC)/duck/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -I$(SRC) test_main.cpp $(CORE) -o $@

clean:
	rm -rf build
