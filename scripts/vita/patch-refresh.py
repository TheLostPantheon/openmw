#!/usr/bin/env python3
"""
Refresh an in-repo dependency patch from files edited in place in the
fetched source tree, then verify the whole patch.

  osg    patches/osg/vita-combined.patch  <- build-vita/extern/fetched/osg
  mygui  patches/mygui/vita-gui-dirty.patch <- build-vita/extern/fetched/mygui

For each path given (relative to the library's source root), its section is
rebuilt as a diff between the pristine file (from the FetchContent archive)
and the edited one: replaced in place if the patch has it, appended
otherwise, dropped if the file matches pristine again. Verification applies
the complete patch to pristine copies of every file it touches and requires
each result to match the fetched tree byte for byte.

Usage: scripts/vita/patch-refresh.py osg src/osgUtil/CullVisitor.cpp [...]
       scripts/vita/patch-refresh.py mygui MyGUIEngine/src/MyGUI_LayerNode.cpp [...]
"""

import glob
import os
import re
import subprocess
import sys
import tempfile
import zipfile

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

LIBS = {
    "osg": {
        "patch": "patches/osg/vita-combined.patch",
        "fetched": "build-vita/extern/fetched/osg",
        "archive": "build-vita/_deps/osg-subbuild/osg-populate-prefix/src/*.zip",
        # Sections start with a diff command line, then ---/+++.
        "header": "diff -ruN '--exclude=.git' pristine-osg/{0} b/{0}\n--- a/{0}\n+++ b/{0}\n",
        "split": r"(?=^diff -ruN )",
        "path": r"diff -ruN '--exclude=\.git' pristine-osg/(\S+) ",
    },
    "mygui": {
        "patch": "patches/mygui/vita-gui-dirty.patch",
        "fetched": "build-vita/extern/fetched/mygui",
        "archive": "build-vita/_deps/mygui-subbuild/mygui-populate-prefix/src/*.zip",
        # Plain multi-file unified diff: sections start at "--- a/".
        "header": "--- a/{0}\n+++ b/{0}\n",
        "split": r"(?=^--- a/)",
        "path": r"--- a/(\S+)",
    },
}


def archive(lib):
    found = glob.glob(os.path.join(REPO, lib["archive"]))
    if not found:
        sys.exit(f"source archive not found: {lib['archive']}")
    return zipfile.ZipFile(found[0])


def pristine(zf, rel):
    for name in zf.namelist():
        if name.endswith("/" + rel) and name.count("/") == rel.count("/") + 1:
            return zf.read(name)
    sys.exit(f"{rel} not in the source archive")


def sections(lib, text):
    """Split the patch into (path, text) sections, in order."""
    out = []
    for p in re.split(lib["split"], text, flags=re.M):
        if not p:
            continue
        m = re.match(lib["path"], p)
        out.append((m.group(1) if m else None, p))
    return out


def make_section(lib, zf, rel, tmp):
    src = os.path.join(tmp, "a")
    with open(src, "wb") as f:
        f.write(pristine(zf, rel))
    r = subprocess.run(["diff", "-u", src, os.path.join(REPO, lib["fetched"], rel)], capture_output=True, text=True)
    if r.returncode == 0:
        return None  # identical to pristine: no section
    if r.returncode != 1:
        sys.exit(r.stderr)
    return lib["header"].format(rel) + r.stdout.split("\n", 2)[2]


def verify(lib, zf, text, tmp):
    root = os.path.join(tmp, "verify")
    paths = [p for p, _ in sections(lib, text) if p]
    for rel in paths:
        dst = os.path.join(root, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(pristine(zf, rel))
    r = subprocess.run(["patch", "-p1", "-N", "-s"], input=text, text=True, cwd=root, capture_output=True)
    if r.returncode != 0:
        sys.exit("patch does not apply to pristine sources:\n" + r.stdout + r.stderr)
    fetched = os.path.join(REPO, lib["fetched"])
    bad = [rel for rel in paths
           if open(os.path.join(root, rel), "rb").read() != open(os.path.join(fetched, rel), "rb").read()]
    if bad:
        sys.exit("patched pristine differs from the fetched tree: " + ", ".join(bad))
    return len(paths)


def main(argv):
    if len(argv) < 2 or argv[0] not in LIBS:
        sys.exit(__doc__)
    lib = LIBS[argv[0]]
    patch_path = os.path.join(REPO, lib["patch"])
    zf = archive(lib)
    secs = sections(lib, open(patch_path).read())
    with tempfile.TemporaryDirectory() as tmp:
        for rel in argv[1:]:
            new = make_section(lib, zf, rel, tmp)
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
        n = verify(lib, zf, out, tmp)
    with open(patch_path, "w") as f:
        f.write(out)
    print(f"verified: the patch reproduces all {n} patched files")


if __name__ == "__main__":
    main(sys.argv[1:])
