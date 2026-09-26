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

# Step 3: let clangd find the compilation database.
# clangd only auto-searches two locations (tested on clangd 21):
#     <root>/compile_commands.json          <- this symlink
#     <root>/build/compile_commands.json
# Linux already satisfies the second one, but Mac's output lives in build-mac/,
# which clangd does not recognise. Both platforms now publish the DB at the
# first location instead, so one .clangd works everywhere and neither platform
# has to alias the other's build directory.
#
# Do NOT go back to symlinking build -> build-mac: `build/` in .gitignore is a
# directory-only pattern and does not match a symlink, so that alias once got
# committed (ad75477) and broke `cmake -B build` on this machine.
ln -sfn build/compile_commands.json compile_commands.json
