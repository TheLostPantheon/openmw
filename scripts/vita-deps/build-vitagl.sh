#!/bin/bash
# Build vitaGL for OpenMW Vita port.
# Usage: ./build-vitagl.sh [vitagl_dir]    (default: ~/vitaGL)
set -e

VITAGL_DIR="${1:-${HOME}/vitaGL}"
VITASDK="${VITASDK:-/usr/local/vitasdk}"

export VITASDK
export PATH="${VITASDK}/bin:${PATH}"

if [ ! -d "${VITASDK}" ]; then
    echo "ERROR: VitaSDK not found at ${VITASDK}"
    exit 1
fi

if ! command -v arm-vita-eabi-gcc &> /dev/null; then
    echo "ERROR: arm-vita-eabi-gcc not on PATH (VITASDK=${VITASDK})"
    exit 1
fi

echo "=== Building vitaGL ==="
echo "Target: ${VITAGL_DIR}"

# Pinned upstream commit + our patches (patches/vitagl/*.patch). vitaGL
# changes that lived only in a local tree were lost once; every change to
# vitaGL goes in a patch here. Bump VITAGL_COMMIT deliberately.
# 6e7fe40 (2026-07-31): newest commit before the Aug 11 batch (VGL_MEM_SLOW ->
# VGL_MEM_PHYCONT rename, display-queue and allocator changes). The app's
# code was last built against this API generation; f4b23b6 (Aug 22) linked
# but crashed in sceClibMspaceMalloc on boot and save load.
VITAGL_COMMIT="${VITAGL_COMMIT:-6e7fe40}"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../patches/vitagl" && pwd)"

if [ ! -d "${VITAGL_DIR}" ]; then
    git clone https://github.com/Rinnegatamante/vitaGL.git "${VITAGL_DIR}"
fi

cd "${VITAGL_DIR}"

if ! git cat-file -e "${VITAGL_COMMIT}^{commit}" 2>/dev/null; then
    git fetch origin
fi
# Reset to the pin; local edits are discarded (they belong in a patch).
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
    echo "NOTE: discarding local vitaGL edits; patches/vitagl/ is the source of truth"
fi
git checkout -q --force "${VITAGL_COMMIT}"
for p in "${PATCH_DIR}"/*.patch; do
    [ -e "$p" ] || continue
    echo "Applying $(basename "$p")"
    git apply --whitespace=nowarn "$p"
done

# vitaGL's Makefile tracks no header dependencies: objects from another
# commit or patch set would mix versions inside one libvitaGL.a. Rebuild
# clean whenever the pin or patches change.
STAMP="$(git rev-parse HEAD) $(cat "${PATCH_DIR}"/*.patch 2>/dev/null | shasum | cut -d' ' -f1)"
if [ "$(cat .omw-build-stamp 2>/dev/null)" != "${STAMP}" ]; then
    echo "vitaGL pin or patches changed: clean build"
    make clean >/dev/null 2>&1 || true
fi

# Flag set must match Dockerfile.vita.
# NOTE: NO_TILE_CLIPPER and USE_SCRATCH_MEMORY were tried and produced visual artifacts
# NO_SPLASHSCREEN: splash thread races GXM init (launch crash).
make -j"$(nproc)" \
    DEPTH_STENCIL_HACK=1 \
    DRAW_SPEEDHACK=1 MATH_SPEEDHACK=1 \
    TEXTURES_SPEEDHACK=1 BUFFERS_SPEEDHACK=1 \
    SAMPLERS_SPEEDHACK=1 UNIFORMS_SPEEDHACK=1 \
    PRIMITIVES_SPEEDHACK=1 \
    PHYCONT_ON_DEMAND=1 \
    NO_DEBUG=1 \
    NO_SPLASHSCREEN=1 \
    HAVE_SHADER_CACHE=1

if [ ! -f "${VITAGL_DIR}/libvitaGL.a" ]; then
    echo "ERROR: build completed but libvitaGL.a not found in ${VITAGL_DIR}"
    exit 1
fi

echo "${STAMP}" > .omw-build-stamp

# Header that matches this library; CMake puts VITAGL_DIR/include ahead of
# the SDK's stock vitaGL.h (a different version).
mkdir -p "${VITAGL_DIR}/include"
cp -f "${VITAGL_DIR}/source/vitaGL.h" "${VITAGL_DIR}/include/vitaGL.h"

echo "[OK] libvitaGL.a produced at ${VITAGL_DIR}/libvitaGL.a (headers in ${VITAGL_DIR}/include)"
