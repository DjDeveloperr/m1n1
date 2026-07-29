#!/usr/bin/env python3
"""Build and seal the sole J414s Windows m1n1 image."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


BRANCH = "feature/j414s-windows-unified"
FILES = ("m1n1.macho", "m1n1.elf", "m1n1.bin")
REQUIRED_SOURCE = {
    "native_aic": ("config.h", "#define ENABLE_NATIVE_AIC_PASSTHROUGH"),
    "mtp": ("config.h", "#define ENABLE_J414S_WINDOWS_MTP_HANDOFF"),
    "wireless_contract": ("src/wireless_handoff.c", "wlan_validate_reservation"),
    "gpu": ("src/kboot_gpu.c", "rust_gpu_initdata_fill"),
    "tpm": ("src/hv_tpm.c", "hv_map_tpm"),
    "sparse_identity": ("src/platform_identity.c", "platform_is_j414s"),
}


def run(root: Path, *args: str) -> str:
    return subprocess.check_output(args, cwd=root, text=True).strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate(root: Path) -> tuple[str, dict[str, str]]:
    branch = run(root, "git", "branch", "--show-current")
    if branch != BRANCH:
        raise SystemExit(f"refusing branch {branch!r}; expected {BRANCH!r}")
    if run(root, "git", "status", "--porcelain", "--untracked-files=no"):
        raise SystemExit("refusing a modified m1n1 source tree")
    commit = run(root, "git", "rev-parse", "HEAD")
    proofs: dict[str, str] = {}
    for name, (relative, needle) in REQUIRED_SOURCE.items():
        text = (root / relative).read_text(encoding="utf-8")
        if needle not in text:
            raise SystemExit(f"missing {name} source contract: {relative}: {needle}")
        proofs[name] = hashlib.sha256(text.encode()).hexdigest()
    return commit, proofs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=max(1, os.cpu_count() or 1))
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    commit, proofs = validate(root)
    if args.validate_only:
        print(f"J414s unified m1n1 source: PASS {commit}")
        return 0

    subprocess.run(["make", f"-j{args.jobs}"], cwd=root, check=True)
    artifact_dir = args.output.resolve() / commit / "artifacts"
    artifact_dir.mkdir(parents=True, exist_ok=True)
    files: dict[str, dict[str, int | str]] = {}
    for name in FILES:
        source = root / "build" / name
        if not source.is_file():
            raise SystemExit(f"build omitted {source}")
        target = artifact_dir / name
        shutil.copy2(source, target)
        files[name] = {"size": target.stat().st_size, "sha256": sha256(target)}

    manifest = {
        "schema": "ntasi.j414s.m1n1-unified.v1",
        "source": {"path": str(root), "branch": BRANCH, "commit": commit},
        "profile": {
            "native_aic": True,
            "ten_core_sparse": True,
            "dcp_dart_handoff": True,
            "mtp_input": True,
            "pcie_capability": True,
            "ans_explicit": True,
            "wireless_explicit_reserved_range": True,
            "gpu_explicit_live_manifest": True,
            "tpm_explicit_attach": True,
        },
        "source_proofs": proofs,
        "files": files,
    }
    manifest_path = artifact_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"m1n1 unified artifacts: {artifact_dir}")
    print(f"manifest SHA-256: {sha256(manifest_path)}")
    for name in FILES:
        print(f"{name} SHA-256: {files[name]['sha256']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
