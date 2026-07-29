# Unified Windows branch: mainline update debt

This record prevents a broad upstream merge from being mistaken for routine
maintenance of the hardware-proven J414s Windows stack.

## Audited graph

- unified audit commit: `88997ff921ab582a30f962529b12ac6d8f859c3e`
- local `main`: `7c7716b6a196c7e601f9f22bb8af335c1b8173ce`
- merge base: `faa73cd8c64bc0a6f0f4e0f41b34ed02b458b74d`
- raw divergence: 168 unified-only / 200 main-only commits
- patch-equivalent divergence: 159 unified-only / 191 main-only commits
- proposed merge size: 162 files, 9,164 insertions, 869 deletions

`git merge-tree --write-tree feature/j414s-windows-unified main` reports 13
direct conflicts:

```text
.gitignore
Makefile
config.h
proxyclient/m1n1/hv/__init__.py
proxyclient/m1n1/proxy.py
rust/Cargo.lock
rust/Cargo.toml
rust/src/adt.rs
rust/src/float.rs
rust/src/lib.rs
src/hv.c
src/pcie.c
src/soc.h
```

This is not safe to merge as one change. The conflicts overlap the native-AIC
delivery path, Windows proxy ABI, J414s port-zero PCIe containment, GPU
initdata producer, and the isolated-build machinery. A compile success would
not establish preservation of those runtime contracts.

## Staged update plan

Advance the same authoritative branch; do not create lane-specific m1n1
checkouts or launch artifacts from intermediate stages.

1. Import documentation, CI, toolchain portability, and non-runtime build
   changes. Re-run the full host suite and prove isolated artifact hashes are
   reproducible.
2. Reconcile proxy ABI changes (`P_GET_CPU_FEATURES`, device reconnect, and
   related Python changes) with Windows atomic `MEMWRITE`, TPM, wireless
   reservation, and preload opcodes. Add an opcode-uniqueness test before
   accepting the cohort.
3. Import unrelated new-SoC support without changing T6020 switch behavior.
   Pin exact J414s identity and sparse CPU tests after the cohort.
4. Reconcile upstream PCIe additions with the J414s port-zero-only wireless
   initializer, retained MSICFG geometry, transactional RID rollback, and both
   dormant/runtime SID1 handoff suites.
5. Reconcile upstream Rust ADT/GPU restructuring with the current J414s GPU
   initdata/calibration producer and its byte-level handoff tests.
6. Handle `hv.c`, exception, memory, and four-level-paging changes last. These
   directly intersect native AIC/Fast-IPI and Windows guest execution. Require
   all host contracts plus an independently authorized hardware regression
   matrix before declaring the new base canonical.
7. Only after every cohort is committed and the tree is clean, rebuild into a
   new commit-addressed artifact directory and update downstream manifests.

Until this plan closes, `main` is update input—not a launch source—and the
manifest records the exact outstanding snapshot.
