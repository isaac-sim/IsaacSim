#!/usr/bin/env bash
set -euo pipefail

# Usage: build_libsurvive.sh <REPO_ROOT>
# If REPO_ROOT is not provided, infer it from this script's location.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${1:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

TP_DIR="$REPO_ROOT/_build/target-deps/libsurvive"
SRC_DIR="$REPO_ROOT/_build/third_party/libsurvive"
BUILD_DIR="$SRC_DIR/build"

# Skip if already built (lib + include present)
if [[ -f "$TP_DIR/lib/libsurvive.so" || -f "$TP_DIR/lib/libsurvive.dylib" || -f "$TP_DIR/lib/survive.lib" ]] && \
   [[ -f "$TP_DIR/include/survive.h" ]]; then
  echo "[libsurvive] Found existing install under $TP_DIR, skipping build."
  exit 0
fi

mkdir -p "$TP_DIR" "$SRC_DIR"

# Clone source if missing
if [[ ! -d "$SRC_DIR/.git" ]]; then
  echo "[libsurvive] Cloning libsurvive into $SRC_DIR"
  git clone --depth=1 https://github.com/cntools/libsurvive.git "$SRC_DIR"
fi

# Configure and build with RPATH so Python bindings can locate libsurvive without LD_LIBRARY_PATH
mkdir -p "$BUILD_DIR"
echo "[libsurvive] Configuring CMake (install prefix $TP_DIR)"
cmake -S "$SRC_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$TP_DIR" \
  -DCMAKE_INSTALL_RPATH='$$ORIGIN/../../lib' \
  -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON

echo "[libsurvive] Building and installing"
cmake --build "$BUILD_DIR" --target install -j"$(nproc || echo 4)"

# Copy Python bindings for runtime import (pysurvive)
if [[ -d "$SRC_DIR/bindings/python" ]]; then
  mkdir -p "$TP_DIR/bindings/python"
  rsync -a --delete "$SRC_DIR/bindings/python/" "$TP_DIR/bindings/python/"
  echo "[libsurvive] Copied Python bindings to $TP_DIR/bindings/python"
else
  echo "[libsurvive] WARNING: Python bindings directory not found; pysurvive may not import"
fi

# If compiled extension .so files exist under bindings/python, ensure their RUNPATH points to ../../lib
if command -v patchelf >/dev/null 2>&1; then
  shopt -s nullglob
  for so in "$TP_DIR"/bindings/python/**/*.so "$TP_DIR"/bindings/python/*.so; do
    echo "[libsurvive] Setting RUNPATH on $(basename "$so") to \$ORIGIN/../../lib"
    patchelf --set-rpath "\$ORIGIN/../../lib" "$so" || true
  done
  shopt -u nullglob
else
  echo "[libsurvive] patchelf not found; assuming CMake RPATH on extension is sufficient"
fi

# Print hints
echo "[libsurvive] Install complete at: $TP_DIR"
echo "[libsurvive] Python bindings at: $TP_DIR/bindings/python"
