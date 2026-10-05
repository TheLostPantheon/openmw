#!/usr/bin/env python3
"""
Turn a Vita core dump into a readable crash report.

Fetches the newest psp2core from the Vita (VitaShell FTP), plus boot.log and
crash.txt, finds the ELF matching the build ("[Build] id=" in boot.log ->
~/Dev/vita/device-backups/elfs/<id>.elf.gz, written by archive-build.sh),
and writes report.md: the crashed thread with a call-site-verified backtrace,
every other thread's PC and top frames, registers, and the log tail.

System-module frames (SceLibKernel, SceGxm, ...) are named from function
tables exported from the vita-decomp Ghidra project
(scripts/vita/symbols/*.tsv, made by scripts/vita/ghidra/DumpFunctionTable.java).

Core parsing uses vita-parse-core (VITA_PARSE_CORE, default
~/Dev/vita/vita-parse-core).

Usage:
  scripts/vita/vita_crash.py                 # fetch newest dump from the Vita
  scripts/vita/vita_crash.py --dump X.psp2dmp --boot-log boot.log [--elf openmw]
Options: --keep-on-device (default: archive locally, then delete the dump on
the Vita), --out DIR.
"""

import argparse
import bisect
import datetime
import gzip
import os
import re
import shutil
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
SYMBOL_DIR = os.path.join(HERE, "symbols")
ELF_ARCHIVE = os.path.expanduser(os.environ.get("VITA_ELF_ARCHIVE", "~/Dev/vita/device-backups/elfs"))
RUNS_DIR = os.path.expanduser(os.environ.get("VITA_RUNS_DIR", "~/Dev/vita/device-backups/runs"))
PARSE_CORE = os.path.expanduser(os.environ.get("VITA_PARSE_CORE", "~/Dev/vita/vita-parse-core"))
DECRYPTED = os.path.expanduser(os.environ.get("VITA_DECRYPTED", "~/Dev/vita-decomp/dump/decrypted"))
VITASDK_BIN = os.path.join(os.environ.get("VITASDK", os.path.expanduser("~/vitasdk")), "bin")

# Core module name -> (function table, decrypted module ELF for call-site checks)
SYSTEM_MODULES = {
    "SceLibKernel": ("libkernel.suprx.tsv", "us/libkernel.suprx.elf"),
    "SceGxm": ("libgxm_es4.suprx.tsv", "us/libgxm_es4.suprx.elf"),
}

STOP_REASONS = {
    0x30002: "Undefined instruction",
    0x30003: "Prefetch abort",
    0x30004: "Data abort",
    0x60080: "Division by zero",
}
STATUS = {1: "Running", 2: "Ready", 8: "Waiting", 16: "Not started"}

# Log lines too frequent to be worth showing in the tail.
LOG_NOISE = re.compile(r"\[(Input|CommonWarm|LoadSplit|ImgLoad|DrawSpike)\]|^\[VitaMem\] \[(OsgApply|Scene|"
                       r"OsgProf|CullProf|SimSplit|GuiWalks|Gpu|GlJob)\]")


class Symbols:
    """Address -> 'name+0xoff' for the app ELF and known system modules."""

    def __init__(self, elf_path, core):
        self.core = core
        self.app = []  # (addr, name)
        self.elf_path = elf_path
        self.elf_text = None  # (vaddr, bytes) for call-site checks
        if elf_path:
            nm = subprocess.run([os.path.join(VITASDK_BIN, "arm-vita-eabi-nm"), "-C", elf_path],
                                capture_output=True, text=True).stdout
            for line in nm.splitlines():
                m = re.match(r"^([0-9a-f]{8}) [TtWw] (.+)$", line)
                if m:
                    self.app.append((int(m.group(1), 16) & ~1, m.group(2)))
            self.app.sort()
            self.app_addrs = [a for a, _ in self.app]
            self.elf_text = self._load_text(elf_path)
        self.sys_tables = {}
        self.sys_code = {}
        for mod, (tsv, dec) in SYSTEM_MODULES.items():
            p = os.path.join(SYMBOL_DIR, tsv)
            if os.path.exists(p):
                rows = []
                for l in open(p):
                    if l.startswith("#"):
                        continue
                    o, s, n = l.rstrip("\n").split("\t")
                    rows.append((int(o, 16), int(s), n))
                rows.sort()
                self.sys_tables[mod] = rows
            d = os.path.join(DECRYPTED, dec)
            if os.path.exists(d):
                self.sys_code[mod] = self._load_text(d)

    @staticmethod
    def _load_text(path):
        """(vaddr, bytes) of the first executable PT_LOAD segment."""
        data = open(path, "rb").read()
        phoff, = struct.unpack_from("<I", data, 0x1C)
        phentsize, phnum = struct.unpack_from("<HH", data, 0x2A)
        for i in range(phnum):
            p_type, p_off, p_vaddr, _, p_filesz, _, p_flags, _ = struct.unpack_from("<8I", data, phoff + i * phentsize)
            if p_type == 1 and p_flags & 1:
                return p_vaddr, data[p_off:p_off + p_filesz]
        return None

    def locate(self, addr):
        """(module name, segment number, offset) via the core's module map."""
        for m in self.core.modules:
            for s in m.segments:
                if s.start <= addr < s.start + s.size:
                    return m.name, s.num, addr - s.start, s
        return None, None, None, None

    def name(self, addr):
        mod, seg, off, _ = self.locate(addr)
        if mod is None:
            return None
        if self.app and (mod.endswith(".elf") or mod == "openmw" or mod.startswith("OMWV")):
            i = bisect.bisect_right(self.app_addrs, addr & ~1) - 1
            if i >= 0:
                a, n = self.app[i]
                return f"{n}+0x{(addr & ~1) - a:x}"
        table = self.sys_tables.get(mod)
        if table and seg == 1:
            offs = [r[0] for r in table]
            i = bisect.bisect_right(offs, off & ~1) - 1
            if i >= 0:
                o, size, n = table[i]
                return f"{n}+0x{(off & ~1) - o:x} ({mod})"
        return f"{mod}@{seg}+0x{off:x}"

    def is_app(self, addr):
        mod, _, _, _ = self.locate(addr)
        return mod is not None and (mod.endswith(".elf") or mod == "openmw")

    def is_code(self, addr):
        _, seg, _, s = self.locate(addr)
        return s is not None and (s.attr & 1)  # executable

    def _code_bytes(self, addr, n):
        """n bytes of code ending at addr, from the app ELF or a decrypted module."""
        mod, seg, off, s = self.locate(addr)
        if mod is None:
            return None
        if self.is_app(addr) and self.elf_text:
            base, code = self.elf_text
            i = addr - base
        elif mod in self.sys_code and seg == 1:
            base, code = self.sys_code[mod]
            i = off
        else:
            return None
        if i - n < 0 or i > len(code):
            return None
        return code[i - n:i]

    def is_call_site(self, ret):
        """Does the instruction before return address `ret` look like a call?"""
        if not (ret & 1):
            # ARM state: BL/BLX imm (cond 1011 / 1111101x) or BLX reg.
            b = self._code_bytes(ret, 4)
            if b is None:
                return None
            w, = struct.unpack("<I", b)
            return (w & 0x0F000000) == 0x0B000000 or (w & 0xFE000000) == 0xFA000000 or (w & 0x0FFFFFF0) == 0x012FFF30
        ret &= ~1
        b4 = self._code_bytes(ret, 4)
        b2 = self._code_bytes(ret, 2)
        if b4 is None and b2 is None:
            return None
        if b2 is not None:
            h, = struct.unpack("<H", b2)
            if (h & 0xFF80) == 0x4780:  # BLX Rm
                return True
        if b4 is not None:
            h1, h2 = struct.unpack("<HH", b4)
            if (h1 & 0xF800) == 0xF000 and (h2 & 0xD000) in (0xD000, 0xC000):  # BL / BLX imm
                return True
        return False

    def lines(self, addrs):
        """addr -> 'file:line' via addr2line (app frames only)."""
        if not self.elf_path or not addrs:
            return {}
        res = subprocess.run([os.path.join(VITASDK_BIN, "arm-vita-eabi-addr2line"), "-C", "-e", self.elf_path]
                             + [hex((a & ~1) - 1) for a in addrs], capture_output=True, text=True).stdout.splitlines()
        out = {}
        for a, l in zip(addrs, res):
            if not l.startswith("??"):
                out[a] = re.sub(r"^.*?/(apps|components|extern|build-vita)/", r"\1/", l.split(" (")[0])
                out[a] = re.sub(r"^/Users/[^/]+/vitaGL/", "vitaGL/", out[a])
        return out


def backtrace(core, sym, thread, depth=1024, limit=14):
    """Return addresses found on the stack, call-site verified where possible."""
    sp = thread.regs.gpr[13]
    frames = []
    seen = set()
    for i in range(depth):
        data = core.read_vaddr(sp + 4 * i, 4)
        if not data or len(data) < 4:
            break
        v, = struct.unpack("<I", data)
        if v in seen or not sym.is_code(v):
            continue
        ok = sym.is_call_site(v)
        if ok is False:
            continue
        seen.add(v)
        frames.append((sp + 4 * i, v, ok))
        if len(frames) >= limit:
            break
    return frames


def thread_names_from_log(log):
    names = {}
    m = re.search(r"\[vglGuard\] armed: owner tid (0x[0-9a-f]+)", log)
    if m:
        names[int(m.group(1), 16)] = "gl worker"
    for m in re.finditer(r"\[(SimWorker|GLWorker)\] alive tid=(0x[0-9a-f]+)", log):
        names[int(m.group(2), 16)] = "sim worker" if m.group(1) == "SimWorker" else "gl worker"
    for m in re.finditer(r" on (\w+) \((0x[0-9a-f]+)\)", log):
        names.setdefault(int(m.group(2), 16), m.group(1))
    return names


def report(dump, boot_log_path, elf_path, crash_txt_path=None):
    sys.path.insert(0, PARSE_CORE)
    from core import CoreParser  # vita-parse-core

    core = CoreParser(dump)
    log = open(boot_log_path, errors="replace").read() if boot_log_path and os.path.exists(boot_log_path) else ""
    build = re.search(r"\[Build\] id=(\S+)", log)
    stamp = re.search(r"BOOT: build (.+)", log)
    sym = Symbols(elf_path, core)
    names = thread_names_from_log(log)
    out = []
    w = out.append
    w(f"# Crash report: {os.path.basename(dump)}")
    w("")
    w(f"- build: `{build.group(1) if build else 'unknown'}`" + (f" ({stamp.group(1).strip()})" if stamp else ""))
    w(f"- symbols: `{elf_path or 'none (app frames unnamed)'}`")
    w("")

    def tname(t):
        extra = names.get(t.uid)
        if t.name == "OMWV00001":
            extra = extra or "main"
        return f'"{t.name}" <0x{t.uid:x}>' + (f" ({extra})" if extra else "")

    crashed = [t for t in core.threads if t.stop_reason]
    app_frames = []
    for t in crashed:
        frames = backtrace(core, sym, t, limit=18)
        app_frames += [v for _, v, _ in frames if sym.is_app(v)]
    pcs = [t.pc for t in core.threads if sym.is_app(t.pc)]
    lines = sym.lines(sorted(set(app_frames + pcs + [t.regs.gpr[14] for t in crashed if sym.is_app(t.regs.gpr[14])])))

    for t in crashed:
        r = t.regs.gpr
        w(f"## CRASHED: {tname(t)}: {STOP_REASONS.get(t.stop_reason, hex(t.stop_reason))}")
        w("")
        w("```")
        w(f"PC  0x{t.pc:08x}  {sym.name(t.pc) or '?'}  {lines.get(t.pc, '')}")
        w(f"LR  0x{r[14]:08x}  {sym.name(r[14]) or '?'}  {lines.get(r[14], '')}")
        w("")
        w("Backtrace (stack scan; '?' = call site not verifiable):")
        for slot, v, ok in backtrace(core, sym, t, limit=18):
            w(f"  0x{v:08x}  {sym.name(v)}{'' if ok else '  ?'}  {lines.get(v, '')}")
        w("")
        regs = [f"R{i:<2} 0x{r[i]:08x}" for i in range(13)] + [f"SP  0x{r[13]:08x}"]
        for i in range(0, len(regs), 4):
            w("  ".join(regs[i:i + 4]))
        w("```")
        w("")

    w("## Threads")
    w("")
    w("| thread | status | PC | top frames |")
    w("|---|---|---|---|")
    for t in core.threads:
        if t.stop_reason:
            continue
        top = [sym.name(v) for _, v, ok in backtrace(core, sym, t, limit=4) if ok]
        w(f"| {tname(t)} | {STATUS.get(t.status, t.status)} | {sym.name(t.pc) or hex(t.pc)} | "
          f"{' < '.join(x for x in top if x) or ''} |")
    w("")

    if crash_txt_path and os.path.exists(crash_txt_path):
        w("## crash.txt (in-game reporter)")
        w("")
        w("```")
        w(open(crash_txt_path, errors="replace").read().rstrip())
        w("```")
        w("")

    if log:
        guard = [l for l in log.splitlines() if "[vglGuard] off-thread calls" in l]
        if guard:
            w(f"Guard: `{guard[-1].split('] ', 1)[-1]}`")
            w("")
        tail = [l for l in log.splitlines() if l.strip() and not LOG_NOISE.search(l)][-30:]
        w("## boot.log tail (noise filtered)")
        w("")
        w("```")
        w("\n".join(l[:220] for l in tail))
        w("```")
    return "\n".join(out) + "\n"


def find_elf(boot_log_path, explicit=None, workdir=None):
    if explicit:
        return explicit
    log = open(boot_log_path, errors="replace").read() if boot_log_path and os.path.exists(boot_log_path) else ""
    m = re.search(r"\[Build\] id=(\S+)", log)
    if m:
        gz = os.path.join(ELF_ARCHIVE, m.group(1) + ".elf.gz")
        if os.path.exists(gz):
            dst = os.path.join(workdir or os.path.dirname(gz), m.group(1) + ".elf")
            if not os.path.exists(dst):
                with gzip.open(gz, "rb") as src, open(dst, "wb") as f:
                    shutil.copyfileobj(src, f)
            return dst
        print(f"warning: no archived ELF for build {m.group(1)}", file=sys.stderr)
        return None
    print("warning: boot.log has no [Build] id; pass --elf", file=sys.stderr)
    return None


def fetch_latest(workdir, delete=True):
    """Download the newest psp2core plus logs from the Vita. Returns the dump path or None."""
    sys.path.insert(0, HERE)
    import vita_device_mcp as dev

    ftp = dev.connect()
    try:
        names = [l.split(None, 8)[-1] for l in dev.listing(ftp, "ux0:/data")]
        dumps = sorted((n for n in names if n.startswith("psp2core-") and n.endswith(".psp2dmp")),
                       key=lambda n: int(n.split("-")[1]) if n.split("-")[1].isdigit() else 0)
        if not dumps:
            return None
        newest = dumps[-1]
        dump = os.path.join(workdir, newest)
        with open(dump, "wb") as f:
            f.write(dev.fetch(ftp, f"ux0:/data/{newest}"))
        for name in ("boot.log", "debug.log", "crash.txt"):
            try:
                data = dev.fetch(ftp, f"ux0:/data/openmw/{name}")
                with open(os.path.join(workdir, name), "wb") as f:
                    f.write(data)
            except dev.DeviceError:
                pass
        if delete:
            for n in dumps:
                if n != newest:  # older dumps: archive alongside, then remove
                    with open(os.path.join(workdir, n), "wb") as f:
                        f.write(dev.fetch(ftp, f"ux0:/data/{n}"))
                try:
                    ftp.sendcmd(f"DELE ux0:/data/{n}")
                except Exception:
                    pass
        return dump
    finally:
        ftp.quit()


def run(dump=None, boot_log=None, elf=None, out=None, delete=True):
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    workdir = out or os.path.join(RUNS_DIR, f"{stamp}-crash")
    os.makedirs(workdir, exist_ok=True)
    if dump is None:
        dump = fetch_latest(workdir, delete=delete)
        if dump is None:
            shutil.rmtree(workdir, ignore_errors=True)
            return "No psp2core dump on the Vita."
        boot_log = os.path.join(workdir, "boot.log")
    elf_path = find_elf(boot_log, elf, workdir)
    crash_txt = os.path.join(os.path.dirname(boot_log), "crash.txt") if boot_log else None
    text = report(dump, boot_log, elf_path, crash_txt)
    with open(os.path.join(workdir, "report.md"), "w") as f:
        f.write(text)
    return text + f"\n(report and inputs saved in {workdir})\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump")
    ap.add_argument("--boot-log")
    ap.add_argument("--elf")
    ap.add_argument("--out")
    ap.add_argument("--keep-on-device", action="store_true")
    a = ap.parse_args()
    print(run(a.dump, a.boot_log, a.elf, a.out, delete=not a.keep_on_device))


if __name__ == "__main__":
    main()
