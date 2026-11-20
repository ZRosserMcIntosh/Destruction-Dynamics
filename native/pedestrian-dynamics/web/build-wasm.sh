#!/bin/sh
# Pedestrian Dynamics — build the WebAssembly module with wasi-sdk (the same toolchain and flags
# as Pole Dynamics' and Ballistic Dynamics' builds).
# SPDX-License-Identifier: MIT
# Usage: WASI_SDK=/path/to/wasi-sdk web/build-wasm.sh [output-dir]   (default: web/dist)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=${1:-"$HERE/dist"}
: "${WASI_SDK:?set WASI_SDK to the wasi-sdk directory}"
mkdir -p "$OUT"
EXPORTS=""
for f in $(grep -o 'PED_EXPORT [^(]*(' "$REPO/include/ped/c_api.h" | sed 's/.* \**\([a-z_0-9]*\)($/\1/' | grep '^ped_'); do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O2 -fno-exceptions -fno-rtti -ffp-contract=off -Wall -Wextra -mexec-model=reactor \
  -I"$REPO/include" "$REPO"/src/*.cpp \
  $EXPORTS -Wl,--initial-memory=16777216 -Wl,--max-memory=134217728 -Wl,--strip-all \
  -o "$OUT/ped.wasm"
cp "$HERE/ped_loader.mjs" "$OUT/ped.mjs"
cp "$REPO/data/pedestrian_body.json" "$REPO/data/vehicle_profiles.json" "$OUT/"
ls -l "$OUT"
