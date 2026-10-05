#!/bin/bash
# Build vitaGL for the OpenMW Vita port from our fork.
#
# Usage: scripts/vita-deps/build-vitagl.sh [vitagl_dir]   (default: ~/vitaGL)
#
# Pinned build (default): checks out VITAGL_COMMIT from VITAGL_REPO (the
# private fork TheLostPantheon/vitaGL, branch openmw-vita) into vitagl_dir
# and builds it there. vitagl_dir is a script-managed checkout: local edits
# in it are discarded. Bump VITAGL_COMMIT deliberately.
#
# Dev build: VITAGL_SRC=~/Dev/vita/vitaGL VITAGL_COMMIT=dev builds that
# working tree as-is (uncommitted changes included) and installs the
# library and header into vitagl_dir, where CMake looks for them.
#
# Changes to vitaGL are commits in the fork, pushed to its remote: vitaGL
# changes that lived only in a local tree were lost once.
set -e

VITAGL_DIR="${1:-${HOME}/vitaGL}"
VITASDK="${VITASDK:-/usr/local/vitasdk}"
VITAGL_REPO="${VITAGL_REPO:-https://github.com/TheLostPantheon/vitaGL.git}"
# openmw-vita @ 9dbd7e4: upstream 6e7fe40 (2026-07-31, the API generation the
# app code matches) + vglSetStaticVboRam.
VITAGL_COMMIT="${VITAGL_COMMIT:-9dbd7e4b5e1caeb0861b92f50ca4a9e1f57c2946}"

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

if [ "${VITAGL_COMMIT}" = "dev" ]; then
    SRC="${VITAGL_SRC:?VITAGL_COMMIT=dev needs VITAGL_SRC=<working tree>}"
    echo "=== Building vitaGL (dev tree ${SRC}) ==="
    cd "${SRC}"
    STAMP="dev $(git rev-parse HEAD 2>/dev/null) $(git diff HEAD 2>/dev/null | shasum | cut -d' ' -f1)"
else
    echo "=== Building vitaGL ${VITAGL_COMMIT:0:10} from ${VITAGL_REPO} ==="
    if [ ! -d "${VITAGL_DIR}/.git" ]; then
        git clone "${VITAGL_REPO}" "${VITAGL_DIR}"
    fi
    cd "${VITAGL_DIR}"
    git remote set-url origin "${VITAGL_REPO}"
    if ! git cat-file -e "${VITAGL_COMMIT}^{commit}" 2>/dev/null; then
        git fetch -q origin
    fi
    if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
        echo "NOTE: discarding local edits in ${VITAGL_DIR} (script-managed checkout; commit to the fork instead)"
    fi
    git checkout -q --force "${VITAGL_COMMIT}"
    SRC="${VITAGL_DIR}"
    STAMP="$(git rev-parse HEAD)"
fi

# vitaGL's Makefile tracks no header dependencies: objects from another
# commit would mix versions inside one libvitaGL.a. Rebuild clean whenever
# the source changes.
if [ "$(cat .omw-build-stamp 2>/dev/null)" != "${STAMP}" ]; then
    echo "vitaGL source changed: clean build"
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

if [ ! -f "${SRC}/libvitaGL.a" ]; then
    echo "ERROR: build completed but libvitaGL.a not found in ${SRC}"
    exit 1
fi
echo "${STAMP}" > .omw-build-stamp

# Install where CMake looks: the library, and the header that matches it
# (CMake puts VITAGL_DIR/include ahead of the SDK's stock vitaGL.h).
mkdir -p "${VITAGL_DIR}/include"
if [ "${SRC}" != "${VITAGL_DIR}" ]; then
    cp -f "${SRC}/libvitaGL.a" "${VITAGL_DIR}/libvitaGL.a"
fi
cp -f "${SRC}/source/vitaGL.h" "${VITAGL_DIR}/include/vitaGL.h"

echo "[OK] libvitaGL.a at ${VITAGL_DIR}/libvitaGL.a (headers in ${VITAGL_DIR}/include)"
