#!/usr/bin/env bash
# Host-side test build. Uses the same language subset as the VEX target:
# C++11, no exceptions, no RTTI, no STL containers.
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p build/host

g++ -std=c++11 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti -Iinclude \
    tests/harness.cpp \
    tests/main.cpp \
    tests/test_shaping.cpp \
    tests/test_kinematics.cpp \
    tests/test_timebase.cpp \
    tests/test_auton.cpp \
    tests/test_mechanism.cpp \
    src/control/shaping.cpp \
    src/control/kinematics.cpp \
    src/control/timebase.cpp \
    src/control/auton.cpp \
    src/control/mechanism.cpp \
    -o build/host/tests.exe

./build/host/tests.exe
