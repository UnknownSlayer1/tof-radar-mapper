#!/bin/bash
# Host test runner for the 360 Degree ToF Radar Mapper.
# Compiles the pure C++ core (no Arduino dependency) under g++ and
# runs the test suite. Exits non-zero if any test fails.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -Wall -Wextra -I../src -I. test_main.cpp -o test_main
./test_main
