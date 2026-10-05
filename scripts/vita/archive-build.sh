#!/bin/bash
# Archive the linked ELF (with symbols and debug info) under the build id the
# game logs at boot, so any crash dump can be symbolized later:
#   ${VITA_ELF_ARCHIVE:-~/Dev/vita/device-backups/elfs}/<id>.elf.gz
# Keeps the newest ${VITA_ELF_KEEP:-20} archives. Called by build.sh and
# build-fast.sh after a successful build.
#
# Usage: scripts/vita/archive-build.sh <build-id>
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ID="${1:?usage: archive-build.sh <build-id>}"
ELF="${SRC_DIR}/build-vita/apps/openmw/openmw"
ARCHIVE="${VITA_ELF_ARCHIVE:-${HOME}/Dev/vita/device-backups/elfs}"
KEEP="${VITA_ELF_KEEP:-20}"

if [ ! -f "${ELF}" ]; then
    echo "archive-build: ${ELF} not found; nothing archived"
    exit 0
fi
# On-device symbol table for the crash reporter (deployed next to eboot.bin).
python3 "${SCRIPT_DIR}/gen-syms.py" "${ELF}" "${ID}" "${SRC_DIR}/build-vita/apps/openmw/vita_syms.bin"

mkdir -p "${ARCHIVE}"
gzip -1 -c "${ELF}" > "${ARCHIVE}/${ID}.elf.gz.tmp"
mv "${ARCHIVE}/${ID}.elf.gz.tmp" "${ARCHIVE}/${ID}.elf.gz"
echo "archived ELF: ${ARCHIVE}/${ID}.elf.gz"

# Prune oldest beyond KEEP.
ls -1t "${ARCHIVE}"/*.elf.gz 2>/dev/null | tail -n +"$((KEEP + 1))" | while read -r old; do
    rm -f "${old}"
done
