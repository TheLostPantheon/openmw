#!/usr/bin/env python3
"""
Unpack a fake-signed Vita SELF (eboot.bin) back into an ELF.

Reverses vita-make-fself for homebrew builds (unencrypted segments, optionally
zlib-compressed). The result is the VELF that vita-elf-create produced: no
section headers or symbols, but code, data and strings are intact, which is
enough to disassemble or grep a build whose source was lost.

Usage: scripts/vita/unmake-fself.py eboot.bin out.elf
"""

import struct
import sys
import zlib


def main(src, dst):
    data = open(src, 'rb').read()
    if data[:4] != b'SCE\x00':
        sys.exit(f"{src}: not a SELF (magic {data[:4]!r})")

    # SCE header: magic, version, sdk_type, header_type, metadata_offset,
    # then u64 header_len, elf_filesize, self_filesize, unknown, self_offset,
    # appinfo_offset, elf_offset, phdr_offset, shdr_offset, section_info_offset.
    (_, version, _, header_type, _) = struct.unpack_from('<4sIHHI', data, 0)
    (header_len, elf_filesize, self_filesize, _, _, _, elf_off, phdr_off, _,
     seginfo_off) = struct.unpack_from('<10Q', data, 0x10)
    if header_type != 1:
        sys.exit(f"{src}: header_type {header_type}, expected 1 (SELF)")

    ehdr = bytearray(data[elf_off:elf_off + 0x34])
    if ehdr[:4] != b'\x7fELF' or ehdr[4] != 1:
        sys.exit(f"{src}: embedded header is not ELF32")
    e_phoff, = struct.unpack_from('<I', ehdr, 0x1C)
    e_phentsize, e_phnum = struct.unpack_from('<HH', ehdr, 0x2A)

    out = bytearray(elf_filesize)
    out[0:0x34] = ehdr
    # Drop section headers: the SELF does not carry them.
    struct.pack_into('<IHHH', out, 0x20, 0, 0, 0, 0)
    struct.pack_into('<H', out, 0x30, 0)
    phdrs = data[phdr_off:phdr_off + e_phentsize * e_phnum]
    out[e_phoff:e_phoff + len(phdrs)] = phdrs

    for i in range(e_phnum):
        p_type, p_offset, _, _, p_filesz = struct.unpack_from('<5I', phdrs, i * e_phentsize)
        seg_off, seg_len, compression, encryption = struct.unpack_from('<4Q', data, seginfo_off + i * 32)
        if p_filesz == 0:
            continue
        blob = data[seg_off:seg_off + seg_len]
        if encryption == 1:
            sys.exit(f"segment {i}: encrypted (not a fake-signed homebrew SELF)")
        if compression == 2:
            blob = zlib.decompress(blob)
        if len(blob) != p_filesz:
            print(f"warning: segment {i} size {len(blob)} != p_filesz {p_filesz}", file=sys.stderr)
        out[p_offset:p_offset + len(blob)] = blob
        print(f"segment {i}: type 0x{p_type:x} {p_filesz} bytes "
              f"({'zlib' if compression == 2 else 'raw'})")

    open(dst, 'wb').write(out)
    print(f"wrote {dst} ({len(out)} bytes)")


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
