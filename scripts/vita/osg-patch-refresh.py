#!/usr/bin/env python3
"""
Refresh patches/osg/vita-combined.patch from OSG files edited in place in
build-vita/extern/fetched/osg, then verify the whole patch.

For each path given (relative to the OSG source root, e.g.
src/osgUtil/CullVisitor.cpp), its section is rebuilt as a diff between the
pristine file (from the FetchContent archive) and the edited one: replaced
in place if the patch already has it, appended otherwise. Verification
applies the complete patch to pristine copies of every file it touches and
requires each result to match the fetched tree byte for byte.

Usage: scripts/vita/osg-patch-refresh.py src/osgUtil/CullVisitor.cpp [more paths]
"""

import glob
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PATCH = os.path.join(REPO, "patches", "osg", "vita-combined.patch")
FETCHED = os.path.join(REPO, "build-vita", "extern", "fetched", "osg")
HEADER = "diff -ruN '--exclude=.git' pristine-osg/{0} b/{0}\n"


def archive():
    found = glob.glob(os.path.join(REPO, "build-vita", "_deps", "osg-subbuild", "osg-populate-prefix", "src", "*.zip"))
    if not found:
        sys.exit("OSG source archive not found under build-vita/_deps/osg-subbuild")
    return zipfile.ZipFile(found[0])


def pristine(zf, rel):
    for name in zf.namelist():
        if name.endswith("/" + rel) and name.count("/") == rel.count("/") + 1:
            return zf.read(name)
    sys.exit(f"{rel} not in the OSG archive")


def sections(text):
    """Split the patch into (path, text) sections, in order."""
    parts = re.split(r"(?=^diff -ruN )", text, flags=re.M)
    out = []
    for p in parts:
        if not p:
            continue
        m = re.match(r"diff -ruN '--exclude=\.git' pristine-osg/(\S+) ", p)
        out.append((m.group(1) if m else None, p))
    return out


def make_section(zf, rel, tmp):
    src = os.path.join(tmp, "a")
    with open(src, "wb") as f:
        f.write(pristine(zf, rel))
    r = subprocess.run(["diff", "-u", src, os.path.join(FETCHED, rel)], capture_output=True, text=True)
    if r.returncode == 0:
        return None  # identical to pristine: no section
    if r.returncode != 1:
        sys.exit(r.stderr)
    body = r.stdout.split("\n", 2)[2]
    return HEADER.format(rel) + f"--- a/{rel}\n+++ b/{rel}\n" + body


def verify(zf, text, tmp):
    root = os.path.join(tmp, "verify")
    paths = [p for p, _ in sections(text) if p]
    for rel in paths:
        dst = os.path.join(root, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(pristine(zf, rel))
    r = subprocess.run(["patch", "-p1", "-N", "-s"], input=text, text=True, cwd=root, capture_output=True)
    if r.returncode != 0:
        sys.exit("patch does not apply to pristine sources:\n" + r.stdout + r.stderr)
    bad = [rel for rel in paths
           if open(os.path.join(root, rel), "rb").read() != open(os.path.join(FETCHED, rel), "rb").read()]
    if bad:
        sys.exit("patched pristine differs from the fetched tree: " + ", ".join(bad))
    return len(paths)


def main(rels):
    if not rels:
        sys.exit(__doc__)
    zf = archive()
    text = open(PATCH).read()
    secs = sections(text)
    with tempfile.TemporaryDirectory() as tmp:
        for rel in rels:
            new = make_section(zf, rel, tmp)
            idx = next((i for i, (p, _) in enumerate(secs) if p == rel), None)
            if idx is None and new:
                secs.append((rel, new))
                print(f"added    {rel}")
            elif idx is not None and new:
                secs[idx] = (rel, new)
                print(f"updated  {rel}")
            elif idx is not None:
                del secs[idx]
                print(f"removed  {rel} (matches pristine)")
        out = "".join(t for _, t in secs)
        n = verify(zf, out, tmp)
    with open(PATCH, "w") as f:
        f.write(out)
    print(f"verified: the patch reproduces all {n} patched files")


if __name__ == "__main__":
    main(sys.argv[1:])
