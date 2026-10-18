#!/usr/bin/env python3
"""
Fail the build when a linked call or branch does not reach its target.

GNU ld can mis-link a Thumb BL/BLX whose target sits right at the edge of
the +-16 MB branch range: the long-branch veneer is generated but the call
is not routed through it, and the instruction gets a wrapped offset (seen:
BLX to luaL_newmetatable 16,777,218 bytes away encoded as a branch to the
next instruction, which switched to ARM state and crashed at boot). Any
code change can shift the layout onto that edge, and a bad call in rarely
run code would only crash when it runs.

The ELF is linked with --emit-relocs (-Wl,-q), so every call keeps its
relocation. For each R_ARM_THM_CALL / R_ARM_THM_JUMP24 / R_ARM_CALL /
R_ARM_JUMP24 against a defined function symbol, the decoded branch target
must be the symbol or one of the linker's veneers. Relocations against
section symbols (static functions) carry their addend in the instruction,
so for those only the telltale zero offset is rejected.

With --fix, a Thumb call or branch that missed its target is pointed at the
veneer ld generated for its symbol (the bug leaves the veneer in place,
unused) when that veneer is in range, and checked again.

Usage: check-branches.py [--fix] [--remove-on-fail] <linked ELF>
"""

import struct
import sys

SHT_SYMTAB = 2
SHT_REL = 9
STT_SECTION = 3
R_ARM_THM_CALL = 10
R_ARM_CALL = 28
R_ARM_JUMP24 = 29
R_ARM_THM_JUMP24 = 30


def sign_extend(value, bits):
    sign = 1 << (bits - 1)
    return (value & (sign - 1)) - (value & sign)


def thumb_target(data, off, addr, rtype):
    hw1, hw2 = struct.unpack_from("<HH", data, off)
    s = (hw1 >> 10) & 1
    j1 = (hw2 >> 13) & 1
    j2 = (hw2 >> 11) & 1
    i1 = ~(j1 ^ s) & 1
    i2 = ~(j2 ^ s) & 1
    imm = sign_extend((s << 24) | (i1 << 23) | (i2 << 22) | ((hw1 & 0x3FF) << 12) | ((hw2 & 0x7FF) << 1), 25)
    blx = rtype == R_ARM_THM_CALL and (hw2 & 0x1000) == 0
    base = (addr + 4) & ~3 if blx else addr + 4
    return base + imm, imm


def arm_target(data, off, addr):
    word = struct.unpack_from("<I", data, off)[0]
    imm = sign_extend((word & 0xFFFFFF) << 2, 26)
    if (word >> 28) == 0xF:  # BLX immediate: H bit adds a halfword
        imm |= ((word >> 24) & 1) << 1
    return addr + 8 + imm, imm


def main(path, remove_on_fail=False, fix=False):
    data = open(path, "rb").read()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        print(f"check-branches: {path}: not a 32-bit little-endian ELF", file=sys.stderr)
        return 1
    e_shoff = struct.unpack_from("<I", data, 32)[0]
    e_shentsize, e_shnum, e_shstrndx = struct.unpack_from("<HHH", data, 46)
    sections = []
    for i in range(e_shnum):
        sh = struct.unpack_from("<10I", data, e_shoff + i * e_shentsize)
        sections.append(dict(name=sh[0], type=sh[1], addr=sh[3], offset=sh[4], size=sh[5], link=sh[6],
                             info=sh[7], entsize=sh[9]))
    strtab_of = lambda sh, o: data[sh["offset"] + o: data.index(b"\0", sh["offset"] + o)].decode("latin1")

    symtab = next((s for s in sections if s["type"] == SHT_SYMTAB), None)
    if symtab is None:
        print("check-branches: no symbol table; cannot check", file=sys.stderr)
        return 1
    strtab = sections[symtab["link"]]
    symbols = []
    veneers = set()
    thumb_veneer_for = {}  # served symbol name -> Thumb-entry veneer address
    for st_name, st_value, _size, st_info, _other, st_shndx in struct.iter_unpack(
            "<IIIBBH", data[symtab["offset"]: symtab["offset"] + symtab["size"]]):
        symbols.append((st_value, st_info & 0xF, st_shndx, st_name))
        if st_name and st_shndx != 0:
            name = strtab_of(strtab, st_name)
            if name.endswith("_veneer") or "_from_thumb" in name or "_from_arm" in name:
                veneers.add(st_value & ~1)
                # ld names them __<symbol>_from_thumb (Thumb caller to ARM
                # code, entered in Thumb state) and __<symbol>_veneer.
                if name.startswith("__") and name.endswith("_from_thumb"):
                    thumb_veneer_for[name[2:-len("_from_thumb")]] = st_value & ~1
                elif name.startswith("__") and name.endswith("_veneer"):
                    thumb_veneer_for.setdefault(name[2:-len("_veneer")], st_value & ~1)

    exec_sections = [sh for sh in sections if sh["addr"] and sh["type"] == 1 and sh["size"]]

    def forwarded(addr):
        """Target of a Thumb B.W/BL/BLX at addr (a stub's single branch), else None."""
        sh = next((x for x in exec_sections if x["addr"] <= addr < x["addr"] + x["size"] - 3), None)
        if sh is None:
            return None
        off = sh["offset"] + (addr - sh["addr"])
        hw1, hw2 = struct.unpack_from("<HH", data, off)
        if (hw1 & 0xF800) != 0xF000:
            return None
        if (hw2 & 0xD000) == 0x9000:  # B.W (T4)
            return thumb_target(data, off, addr, R_ARM_THM_JUMP24)[0]
        if (hw2 & 0xC000) == 0xC000:  # BL / BLX
            return thumb_target(data, off, addr, R_ARM_THM_CALL)[0]
        return None

    def reaches(target, want):
        if (target & ~1) == want or (target & ~1) in veneers:
            return True
        # Linker stubs without symbols (the Cortex-A8 erratum fix moves a
        # branch that straddles a 4 KB page into a stub): follow one hop.
        hop = forwarded(target & ~1)
        return hop is not None and ((hop & ~1) == want or (hop & ~1) in veneers)

    checked = 0
    bad = []
    for rel in sections:
        if rel["type"] != SHT_REL:
            continue
        target_sh = sections[rel["info"]]
        if target_sh["addr"] == 0:
            continue
        for r_offset, r_info in struct.iter_unpack("<II", data[rel["offset"]: rel["offset"] + rel["size"]]):
            rtype = r_info & 0xFF
            if rtype not in (R_ARM_THM_CALL, R_ARM_THM_JUMP24, R_ARM_CALL, R_ARM_JUMP24):
                continue
            off = target_sh["offset"] + (r_offset - target_sh["addr"])
            if rtype in (R_ARM_THM_CALL, R_ARM_THM_JUMP24):
                target, imm = thumb_target(data, off, r_offset, rtype)
            else:
                target, imm = arm_target(data, off, r_offset)
            value, stype, shndx, name_off = symbols[r_info >> 8]
            if shndx == 0:
                continue  # undefined (weak) symbol: the linker's own business
            checked += 1
            if stype == STT_SECTION:
                if imm == 0:
                    bad.append((r_offset, target, None, rtype, off, ""))
                continue
            want = value & ~1
            if not reaches(target, want):
                bad.append((r_offset, target, want, rtype, off, strtab_of(strtab, name_off) if name_off else ""))

    if bad and fix:
        data = bytearray(data)
        unfixed = []
        for entry in bad:
            addr, target, want, rtype, off, name = entry
            veneer = thumb_veneer_for.get(name) if name else None
            if veneer is None or rtype not in (R_ARM_THM_CALL, R_ARM_THM_JUMP24):
                unfixed.append(entry)
                continue
            # The veneer's entry state decides the instruction: ld's
            # "_from_thumb" stubs for ARMv5T+ are ARM code (ldr pc, [pc, #-4]
            # or ldr ip, [pc]; bx ip) entered with BLX; v4T-style stubs start
            # in Thumb (bx pc) and take BL. A B.W cannot change state.
            vsh = next((x for x in exec_sections if x["addr"] <= veneer < x["addr"] + x["size"] - 3), None)
            if vsh is None:
                unfixed.append(entry)
                continue
            voff = vsh["offset"] + (veneer - vsh["addr"])
            first_word = struct.unpack_from("<I", data, voff)[0]
            if first_word in (0xE51FF004, 0xE59FC000):
                arm_entry = True
            elif (first_word & 0xFFFF) == 0x4778:
                arm_entry = False
            else:
                unfixed.append(entry)
                continue
            if arm_entry and (rtype != R_ARM_THM_CALL or veneer % 4):
                unfixed.append(entry)
                continue
            base = (addr + 4) & ~3 if arm_entry else addr + 4
            delta = veneer - base
            if not -(1 << 24) <= delta < (1 << 24):
                unfixed.append(entry)
                continue
            sign = (delta >> 24) & 1
            j1 = (~((delta >> 23) & 1) ^ sign) & 1
            j2 = (~((delta >> 22) & 1) ^ sign) & 1
            hw1 = 0xF000 | (sign << 10) | ((delta >> 12) & 0x3FF)
            if arm_entry:
                op = 0xC000  # BLX (bit 12 clear; offset from Align(PC, 4), H bit 0)
            elif rtype == R_ARM_THM_CALL:
                op = 0xD000  # BL
            else:
                op = 0x9000  # B.W
            hw2 = op | (j1 << 13) | (j2 << 11) | ((delta >> 1) & 0x7FF)
            struct.pack_into("<HH", data, off, hw1, hw2)
            check, _ = thumb_target(data, off, addr, rtype)
            decoded_blx = rtype == R_ARM_THM_CALL and (hw2 & 0x1000) == 0
            if check != veneer or decoded_blx != arm_entry:
                unfixed.append(entry)
                continue
            how = "blx" if arm_entry else ("bl" if rtype == R_ARM_THM_CALL else "b.w")
            print(f"check-branches: repaired 0x{addr:08x}: {how} to veneer 0x{veneer:08x} for {name}")
        if not unfixed:
            with open(path, "r+b") as f:
                f.write(data)
            print(f"check-branches: {checked} branches reach their targets ({len(bad)} repaired)")
            return 0
        bad = unfixed

    if bad:
        print(f"check-branches: {len(bad)} of {checked} branches miss their target in {path}:", file=sys.stderr)
        for addr, target, want, *_ in bad[:20]:
            what = f"symbol at 0x{want:08x}" if want is not None else "(section-relative, zero offset)"
            print(f"  0x{addr:08x} -> 0x{target:08x}, expected {what}", file=sys.stderr)
        print("check-branches: rebuild with a layout change (any code edit) or fix the link", file=sys.stderr)
        if remove_on_fail:
            # Move the bad link aside so make relinks next time instead of
            # packaging this ELF (a failed POST_BUILD leaves it up to date).
            import os
            os.replace(path, path + ".badlink")
            print(f"check-branches: moved the ELF to {path}.badlink", file=sys.stderr)
        return 1
    print(f"check-branches: {checked} branches reach their targets")
    return 0


if __name__ == "__main__":
    flags = {"--remove-on-fail", "--fix"}
    args = [a for a in sys.argv[1:] if a not in flags]
    if len(args) != 1:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    sys.exit(main(args[0], remove_on_fail="--remove-on-fail" in sys.argv[1:], fix="--fix" in sys.argv[1:]))
