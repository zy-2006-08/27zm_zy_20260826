#!/bin/bash
# Build script.
# Usage: bash make.sh
# This is also the default VS Code build task (F5 / Ctrl+Shift+B).

# Abort on the first failing command, so a broken configure step
# does not fall through into a confusing compile error.
set -e

# Step 1: generate the build system inside the build/ directory.
# CMAKE_EXPORT_COMPILE_COMMANDS also writes build/compile_commands.json, which is
# what clangd reads to get each file's real include paths and defines. Without it
# the editor falls back to guessed flags and reports spurious errors.
cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Step 2: compile, using one parallel job per CPU core of this machine.
make -C build/ -j$(nproc)
