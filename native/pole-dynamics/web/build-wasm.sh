#!/bin/sh
# Pole Dynamics — build the WebAssembly module with wasi-sdk (the same
# toolchain and flags as Ballistic Dynamics' web/build_wasi.sh).
# SPDX-License-Identifier: MIT
# Usage: WASI_SDK=/path/to/wasi-sdk web/build-wasm.sh [output-dir]   (default: web/dist)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=${1:-"$HERE/dist"}
: "${WASI_SDK:?set WASI_SDK to the wasi-sdk directory}"
mkdir -p "$OUT"
EXPORTS=""
for f in $(grep -o 'PD_EXPORT [^(]*(' "$REPO/include/pd/c_api.h" | sed 's/.* \**\([a-z_0-9]*\)($/\1/' | grep '^pd_'); do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O2 -fno-exceptions -fno-rtti -ffp-contract=off -Wall -Wextra -mexec-model=reactor \
  -I"$REPO/include" "$REPO"/src/*.cpp \
  $EXPORTS -Wl,--initial-memory=33554432 -Wl,--max-memory=268435456 -Wl,--strip-all \
  -o "$OUT/poles.wasm"
cp "$HERE/poles_loader.mjs" "$OUT/poles.mjs"
ls -l "$OUT"
