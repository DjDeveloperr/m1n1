#!/bin/sh
# SPDX-License-Identifier: MIT
#
# Host tests for the bulk host<->guest channel: the EL2 device (C) and the
# host-side protocol engine (Python). Neither needs hardware.

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/m1n1-hv-xfer-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc=${CC:-clang}
python=${PYTHON:-python3}
common_flags="-std=c11 -Wall -Wextra -Werror -pedantic"

"$cc" $common_flags \
    -I"$repo_dir/src" \
    "$repo_dir/tests/hv_xfer/test_hv_xfer_device.c" \
    -o "$build_dir/test_hv_xfer_device"
"$build_dir/test_hv_xfer_device"

"$cc" $common_flags \
    -fsanitize=address,undefined \
    -fno-omit-frame-pointer \
    -I"$repo_dir/src" \
    "$repo_dir/tests/hv_xfer/test_hv_xfer_device.c" \
    -o "$build_dir/test_hv_xfer_device-sanitized"
ASAN_OPTIONS=detect_leaks=0 "$build_dir/test_hv_xfer_device-sanitized"

# Engine + ABI parity: stdlib plus construct only.
"$python" "$repo_dir/proxyclient/tests/test_hv_xfer.py"

# HV glue: needs the full proxyclient (pyserial), so run it with m1n1's venv
# for real coverage. Skips cleanly, exit 0, if the imports are unavailable.
"$python" "$repo_dir/proxyclient/tests/test_hv_xfer_integration.py"
