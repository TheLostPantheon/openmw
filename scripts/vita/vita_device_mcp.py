#!/usr/bin/env python3
"""
MCP server (stdio) for the Vita dev kit-of-one: VitaShell FTP access.

Tools: list a directory, read/grep a log, download, upload, delete, and deploy
an eboot. Every overwrite or delete first copies the remote file into
~/Dev/vita/device-backups/auto/<timestamp>/ so nothing on the device is lost
(a lost build once survived only as the eboot on the Vita).

No dependencies beyond the standard library. Configure with env vars:
  VITA_FTP_HOST (default 192.168.0.50), VITA_FTP_PORT (default 1337),
  VITA_BACKUP_DIR (default ~/Dev/vita/device-backups/auto).
VitaShell's FTP server must be running (SELECT in VitaShell) and the Vita
awake.
"""

import datetime
import ftplib
import io
import json
import os
import re
import sys

HOST = os.environ.get("VITA_FTP_HOST", "192.168.0.50")
PORT = int(os.environ.get("VITA_FTP_PORT", "1337"))
BACKUP_ROOT = os.path.expanduser(os.environ.get("VITA_BACKUP_DIR", "~/Dev/vita/device-backups/auto"))
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
APP_DIR = "ux0:/app/OMWV00001"
DATA_DIR = "ux0:/data/openmw"
PROTOCOL_VERSION = "2024-11-05"


class DeviceError(Exception):
    pass


def connect():
    ftp = ftplib.FTP()
    try:
        ftp.connect(HOST, PORT, timeout=15)
        ftp.login()
    except OSError as e:
        raise DeviceError(
            f"cannot reach VitaShell FTP at {HOST}:{PORT} ({e}). "
            "Open VitaShell, press SELECT to start FTP, and keep the Vita awake.")
    return ftp


def norm(path):
    """Accept 'ux0:data/x', 'ux0:/data/x' or a path relative to ux0:/data/openmw."""
    if re.match(r"^[a-z]+0:", path):
        dev, rest = path.split(":", 1)
        return f"{dev}:/{rest.lstrip('/')}"
    return f"{DATA_DIR}/{path.lstrip('/')}"


def listing(ftp, path):
    # VitaShell rejects "LIST <path>", and resolves CWD relative to the
    # current directory even for "ux0:/..." paths: reset to "/" first.
    lines = []
    try:
        ftp.cwd("/")
        ftp.cwd(path + "/" if path.endswith(":") else path)
    except ftplib.error_perm as e:
        raise DeviceError(f"cannot open directory {path}: {e}")
    ftp.retrlines("LIST", lines.append)
    return lines


def remote_size(ftp, path):
    parent, name = path.rsplit("/", 1)
    for line in listing(ftp, parent):
        parts = line.split(None, 8)
        if len(parts) == 9 and parts[8] == name and not line.startswith("d"):
            return int(parts[4])
    return None


def fetch(ftp, path):
    buf = io.BytesIO()
    try:
        ftp.retrbinary(f"RETR {path}", buf.write)
    except ftplib.error_perm as e:
        raise DeviceError(f"cannot read {path}: {e}")
    return buf.getvalue()


def backup(ftp, path):
    """Copy a remote file into the backup dir before it is replaced/deleted."""
    if remote_size(ftp, path) is None:
        return None
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    local = os.path.join(BACKUP_ROOT, stamp, path.replace(":/", "/").replace(":", ""))
    os.makedirs(os.path.dirname(local), exist_ok=True)
    with open(local, "wb") as f:
        f.write(fetch(ftp, path))
    return local


def tool_ls(args):
    path = norm(args.get("path", DATA_DIR))
    ftp = connect()
    try:
        return "\n".join(listing(ftp, path)) or "(empty)"
    finally:
        ftp.quit()


def tool_log(args):
    path = norm(args.get("file", "boot.log"))
    tail = int(args.get("tail", 200))
    pattern = args.get("grep")
    ftp = connect()
    try:
        text = fetch(ftp, path).decode("utf-8", "replace")
    finally:
        ftp.quit()
    lines = text.splitlines()
    total = len(lines)
    if pattern:
        rx = re.compile(pattern)
        lines = [l for l in lines if rx.search(l)]
    shown = lines[-tail:] if tail > 0 else lines
    head = f"{path}: {total} lines" + (f", {len(lines)} match /{pattern}/" if pattern else "")
    return head + f", showing {len(shown)}\n" + "\n".join(shown)


def tool_get(args):
    path = norm(args["remote"])
    local = os.path.expanduser(args["local"])
    ftp = connect()
    try:
        data = fetch(ftp, path)
    finally:
        ftp.quit()
    os.makedirs(os.path.dirname(os.path.abspath(local)), exist_ok=True)
    with open(local, "wb") as f:
        f.write(data)
    return f"downloaded {path} -> {local} ({len(data)} bytes)"


def put(local, remote):
    local = os.path.expanduser(local)
    if not os.path.isfile(local):
        raise DeviceError(f"local file not found: {local}")
    size = os.path.getsize(local)
    ftp = connect()
    try:
        saved = backup(ftp, remote)
        with open(local, "rb") as f:
            ftp.storbinary(f"STOR {remote}", f)
        got = remote_size(ftp, remote)
    finally:
        ftp.quit()
    if got != size:
        raise DeviceError(f"size mismatch after upload: local {size}, device {got}")
    msg = f"uploaded {local} -> {remote} ({size} bytes, verified)"
    return msg + (f"\nprevious version backed up to {saved}" if saved else "")


def tool_put(args):
    return put(args["local"], norm(args["remote"]))


def tool_delete(args):
    path = norm(args["remote"])
    ftp = connect()
    try:
        saved = backup(ftp, path)
        if saved is None:
            return f"{path} does not exist; nothing deleted"
        # VitaShell answers DELE with 226, which ftplib.delete rejects.
        try:
            resp = ftp.sendcmd(f"DELE {path}")
        except ftplib.error_reply as e:
            resp = str(e)
        if not resp.startswith("2"):
            raise DeviceError(f"delete failed: {resp}")
    finally:
        ftp.quit()
    return f"deleted {path} (backed up to {saved})"


def tool_crash(args):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import vita_crash
    return vita_crash.run(elf=args.get("elf"), delete=not args.get("keep_on_device", False))


def tool_deploy(args):
    local = args.get("eboot") or os.path.join(REPO, "build-vita", "apps", "openmw", "eboot.bin")
    msg = put(local, f"{APP_DIR}/eboot.bin")
    # The crash reporter's symbol table must match the eboot's build id.
    syms = os.path.join(os.path.dirname(os.path.expanduser(local)), "vita_syms.bin")
    if os.path.isfile(syms):
        msg += "\n" + put(syms, f"{APP_DIR}/vita_syms.bin")
    return msg


TOOLS = {
    "vita_ls": (tool_ls, "List a directory on the Vita. Paths: 'ux0:/app/OMWV00001', or relative to "
                "ux0:/data/openmw.", {"path": {"type": "string", "description": "Default ux0:/data/openmw"}}, []),
    "vita_log": (tool_log, "Read a log from ux0:/data/openmw (boot.log, debug.log, crashlogs/crash1.log...), "
                 "optionally filtered by a regex, returning the last N lines.",
                 {"file": {"type": "string", "description": "Default boot.log"},
                  "grep": {"type": "string", "description": "Python regex filter, e.g. '\\[vglGuard\\]'"},
                  "tail": {"type": "integer", "description": "Lines to return (0 = all). Default 200"}}, []),
    "vita_get": (tool_get, "Download a file from the Vita to a local path.",
                 {"remote": {"type": "string"}, "local": {"type": "string"}}, ["remote", "local"]),
    "vita_put": (tool_put, "Upload a local file to the Vita. Backs up any file it replaces, verifies size.",
                 {"local": {"type": "string"}, "remote": {"type": "string"}}, ["local", "remote"]),
    "vita_delete": (tool_delete, "Delete a file on the Vita after backing it up locally.",
                    {"remote": {"type": "string"}}, ["remote"]),
    "vita_crash": (tool_crash, "Fetch the newest psp2core crash dump (plus boot.log, crash.txt) from the Vita and "
                   "return a symbolized report: crashed thread backtrace, all threads, log tail. Finds the ELF by "
                   "the build id in boot.log. Archives inputs locally and removes dumps from the device.",
                   {"elf": {"type": "string", "description": "Override the ELF used for symbols"},
                    "keep_on_device": {"type": "boolean", "description": "Leave dumps on the Vita"}}, []),
    "vita_deploy": (tool_deploy, "Deploy an eboot.bin (and the vita_syms.bin beside it) to ux0:/app/OMWV00001 "
                    "(default: build-vita/apps/openmw/). Backs up the installed files first, verifies size.",
                    {"eboot": {"type": "string", "description": "Local eboot.bin path"}}, []),
}


def handle(msg):
    method = msg.get("method")
    if method == "initialize":
        return {"protocolVersion": PROTOCOL_VERSION, "capabilities": {"tools": {}},
                "serverInfo": {"name": "vita-device", "version": "1.0"}}
    if method == "ping":
        return {}
    if method == "tools/list":
        return {"tools": [{"name": n, "description": d,
                           "inputSchema": {"type": "object", "properties": p, "required": r}}
                          for n, (_, d, p, r) in TOOLS.items()]}
    if method == "tools/call":
        name = msg["params"]["name"]
        args = msg["params"].get("arguments") or {}
        if name not in TOOLS:
            return {"content": [{"type": "text", "text": f"unknown tool {name}"}], "isError": True}
        try:
            text = TOOLS[name][0](args)
            return {"content": [{"type": "text", "text": text}]}
        except (DeviceError, ftplib.Error, OSError, KeyError) as e:
            return {"content": [{"type": "text", "text": f"{type(e).__name__}: {e}"}], "isError": True}
    raise LookupError(method)


def main():
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        msg = json.loads(line)
        if "id" not in msg:
            continue  # notification (e.g. notifications/initialized)
        try:
            reply = {"jsonrpc": "2.0", "id": msg["id"], "result": handle(msg)}
        except LookupError:
            reply = {"jsonrpc": "2.0", "id": msg["id"],
                     "error": {"code": -32601, "message": f"method not found: {msg.get('method')}"}}
        sys.stdout.write(json.dumps(reply) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
