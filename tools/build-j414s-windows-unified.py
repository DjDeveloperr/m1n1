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
WINDOWS_LINEAGE = "eb256adf60f181ab79e643d37fcd1aeee63224de"
BCM4388_DORMANT_ORIGIN = "b661647696191a177ea15ea1a9d5f69ae422c31d"
REQUIRED_SOURCE = {
    "native_aic": ("config.h", "#define ENABLE_NATIVE_AIC_PASSTHROUGH"),
    "mtp": ("config.h", "#define ENABLE_J414S_WINDOWS_MTP_HANDOFF"),
    "wireless_contract": ("src/wireless_handoff.c", "wlan_validate_reservation"),
    "wireless_descriptor_abi": (
        "src/wireless_handoff_abi.h",
        "struct wireless_handoff_descriptor_v2",
    ),
    "bcm4388_dormant_transaction": (
        "src/bcm4388_handoff.c",
        "int bcm4388_legacy_dormant_handoff_install(",
    ),
    "gpu": ("src/kboot_gpu.c", "rust_fill_gpu_initdata"),
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


def validate(root: Path) -> tuple[str, dict[str, str], dict[str, object]]:
    branch = run(root, "git", "branch", "--show-current")
    if branch != BRANCH:
        raise SystemExit(f"refusing branch {branch!r}; expected {BRANCH!r}")
    if run(root, "git", "status", "--porcelain", "--untracked-files=no"):
        raise SystemExit("refusing a modified m1n1 source tree")
    commit = run(root, "git", "rev-parse", "HEAD")
    subprocess.run(
        ["git", "merge-base", "--is-ancestor", WINDOWS_LINEAGE, commit],
        cwd=root,
        check=True,
    )
    proofs: dict[str, str] = {}
    for name, (relative, needle) in REQUIRED_SOURCE.items():
        text = (root / relative).read_text(encoding="utf-8")
        if needle not in text:
            raise SystemExit(f"missing {name} source contract: {relative}: {needle}")
        proofs[name] = hashlib.sha256(text.encode()).hexdigest()

    # The descriptor producer is linked for conformance testing but must stay
    # dormant until a profile owns its four pages and publishes the matching
    # Mu resource contract.  A call from any other runtime C unit is a release
    # blocker.
    for source in (root / "src").glob("*.c"):
        if source.name == "bcm4388_handoff.c":
            continue
        if "bcm4388_legacy_dormant_handoff_install(" in source.read_text(encoding="utf-8"):
            raise SystemExit(f"dormant BCM4388 core acquired a runtime call site: {source}")

    integration = run(
        root,
        "git",
        "log",
        "-1",
        "--format=%H",
        "--fixed-strings",
        "--grep=feat(dart): add dormant BCM4388 SID1 handoff core",
    )
    if len(integration) != 40:
        raise SystemExit("missing integrated dormant BCM4388 transaction commit")

    main_head = run(root, "git", "rev-parse", "main")
    merge_base = run(root, "git", "merge-base", commit, main_head)
    main_cherry = run(root, "git", "cherry", commit, main_head)
    provenance: dict[str, object] = {
        "windows_native_aic_ancestor": WINDOWS_LINEAGE,
        "bcm4388_dormant_origin": BCM4388_DORMANT_ORIGIN,
        "bcm4388_dormant_integration": integration,
        "mainline_snapshot": {
            "head": main_head,
            "merge_base": merge_base,
            "cherry_distinct_commits": sum(
                line.startswith("+") for line in main_cherry.splitlines()
            ),
            "debt_record": "docs/windows-unified-main-update-debt.md",
        },
    }
    return commit, proofs, provenance


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=max(1, os.cpu_count() or 1))
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    commit, proofs, provenance = validate(root)
    if args.validate_only:
        print(f"J414s unified m1n1 source: PASS {commit}")
        return 0

    output_root = args.output.resolve() / commit
    build_dir = output_root / "work"
    subprocess.run(
        ["make", f"-j{args.jobs}", f"BUILD_DIR={build_dir}"],
        cwd=root,
        check=True,
    )
    artifact_dir = output_root / "artifacts"
    artifact_dir.mkdir(parents=True, exist_ok=True)
    files: dict[str, dict[str, int | str]] = {}
    for name in FILES:
        source = build_dir / name
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
            "authoritative_wireless_contract": "dynamic_reserved_wireless_handoff_v2",
            "wireless_descriptor": {
                "signature": "NWH2",
                "version": 2,
                "size": 96,
                "offset": "0xc000",
                "capture_manifest_schema": "ntasi.j414s.wireless-handoff.v2",
            },
            "bcm4388_descriptor_transaction": (
                "legacy_reference_fixed_layout_no_current_abi_no_call_site"
            ),
            "gpu_explicit_live_manifest": True,
            "tpm_explicit_attach": True,
        },
        "source_proofs": proofs,
        "provenance": provenance,
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
