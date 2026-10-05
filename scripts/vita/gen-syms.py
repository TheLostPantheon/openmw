#!/usr/bin/env python3
"""
Write vita_syms.bin, the on-device symbol table the crash reporter
(apps/openmw/vita/VitaCrashReport.cpp) binary-searches to name addresses:

  "VSYM" u32 version=1, char build_id[48], u32 count, u32 strings_offset,
  count x { u32 addr, u32 name_offset } sorted by addr, then NUL-terminated
  names (demangled, truncated to 100 chars).

The build id must match the one compiled into the eboot (VitaBuildId.h), or
the reporter ignores the file rather than print wrong names.

Usage: scripts/vita/gen-syms.py <elf> <build-id> <out.bin>
"""

import os
import re
import struct
import subprocess
import sys

VITASDK_BIN = os.path.join(os.environ.get("VITASDK", os.path.expanduser("~/vitasdk")), "bin")


def main(elf, build_id, out):
    nm = subprocess.run([os.path.join(VITASDK_BIN, "arm-vita-eabi-nm"), "-C", elf],
                        capture_output=True, text=True, check=True).stdout
    syms = {}
    for line in nm.splitlines():
        m = re.match(r"^([0-9a-f]{8}) [TtWw] (.+)$", line)
        if m:
            addr = int(m.group(1), 16) & ~1
            name = m.group(2)
            # Prefer real names over local labels at the same address.
            if addr not in syms or syms[addr].startswith("$") or syms[addr].startswith(".L"):
                syms[addr] = name
    entries = sorted((a, n) for a, n in syms.items() if not n.startswith(("$", ".L")))
    strings = bytearray()
    index = bytearray()
    for addr, name in entries:
        index += struct.pack("<II", addr, len(strings))
        strings += name[:100].encode("utf-8", "replace") + b"\0"
    bid = build_id.encode()[:47]
    header_size = 4 + 4 + 48 + 4 + 4
    header = b"VSYM" + struct.pack("<I", 1) + bid + b"\0" * (48 - len(bid)) + struct.pack(
        "<II", len(entries), header_size + len(index))
    with open(out, "wb") as f:
        f.write(header + index + strings)
    print(f"wrote {out}: {len(entries)} symbols, {(len(header) + len(index) + len(strings)) // 1024} KB")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
