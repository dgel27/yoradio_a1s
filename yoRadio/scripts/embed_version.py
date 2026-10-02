"""Bake the firmware and filesystem identities into the build.

yoRadio shows YOVERSION in the web UI footer, over telnet (`version`) and in the
boot log, and also uses it as the cache-busting query string on every asset. Up
to now YOVERSION was a hand-edited string, so two different builds could both
claim "0.9.434_A1S" and there was no way to tell a firmware from its filesystem.

This script runs before every build and sets BUILD_VERSION, which options.h
falls back to YOVERSION. The result looks like:

    0.9.435_A1S+fw.8b9a8a9.fs.4c1f8a20

  fw  = the git commit the firmware was compiled from
  fs  = a hash of the contents of data/, which is what buildfs packs into the
        SPIFFS image

The fs hash is a content hash rather than a commit hash on purpose: the SPIFFS
image is built from the working tree, not from a commit, so two builds of the
same commit with an edited data/ directory produce different filesystems and
should not claim the same fs hash.

It also writes data/www/version.json so the filesystem carries its own identity
and can be checked against the firmware. If you upload a filesystem built at a
different time than the firmware, that file reports what is actually in the
flash rather than what the firmware was compiled expecting.
"""
import hashlib
import json
import os
import subprocess

Import("env")  # noqa: F821 - provided by PlatformIO/SCons

PROJECT_DIR = env.subst("$PROJECT_DIR")
DATA_DIR = os.path.join(PROJECT_DIR, "data")
GENERATED = os.path.join(DATA_DIR, "www", "version.json")


def git(*args):
    """Run a git command, returning '' if it fails (tarball, no repo, shallow)."""
    try:
        out = subprocess.run(
            ("git",) + args, cwd=PROJECT_DIR, capture_output=True, timeout=10
        )
        if out.returncode != 0:
            return ""
        return out.stdout.decode("utf-8", "replace").strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def base_version():
    """The hand-written version out of options.h, without the build suffix."""
    path = os.path.join(PROJECT_DIR, "src", "core", "options.h")
    try:
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                if "#define YOVERSION" in line:
                    return line.split('"')[1]
    except OSError:
        pass
    return "unknown"


def fs_hash():
    """Short hash over the contents of data/, excluding our own output file.

    Paths are included and sorted so that renaming a file changes the hash even
    when the bytes do not.
    """
    h = hashlib.sha256()
    if not os.path.isdir(DATA_DIR):
        return ""
    for root, dirs, files in os.walk(DATA_DIR):
        dirs.sort()
        for name in sorted(files):
            full = os.path.join(root, name)
            if os.path.abspath(full) == os.path.abspath(GENERATED):
                continue
            rel = os.path.relpath(full, DATA_DIR).replace(os.sep, "/")
            h.update(rel.encode("utf-8"))
            try:
                with open(full, "rb") as fh:
                    while True:
                        chunk = fh.read(65536)
                        if not chunk:
                            break
                        h.update(chunk)
            except OSError:
                continue
    return h.hexdigest()[:8]


fw = git("rev-parse", "--short=8", "HEAD")
fs = fs_hash()
base = base_version()

parts = [base]
if fw:
    parts.append("fw." + fw)
if fs:
    parts.append("fs." + fs)
version = "+".join(parts)

env.Append(CPPDEFINES=[("BUILD_VERSION", env.StringifyMacro(version))])

# Put the same identity inside the filesystem so it can report itself.
info = {"version": base, "full": version, "fw": fw, "fs": fs, "dirty": bool(git("status", "--porcelain"))}
try:
    os.makedirs(os.path.dirname(GENERATED), exist_ok=True)
    with open(GENERATED, "w", encoding="utf-8") as fh:
        json.dump(info, fh, indent=1)
        fh.write("\n")
except OSError as exc:
    print("embed_version: could not write %s (%s)" % (GENERATED, exc))

print("embed_version: %s" % version)
