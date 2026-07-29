#!/bin/sh
# SPDX-License-Identifier: MIT

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/m1n1-pcie-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc=${CC:-clang}
common_flags="-std=c11 -Wall -Wextra -Werror -Wconversion -Wsign-conversion -pedantic"

"$cc" $common_flags \
    -DPCIE_T602X_WIRELESS_HOST_TEST=1 \
    -I"$repo_dir/src" \
    "$repo_dir/src/pcie.c" \
    "$repo_dir/tests/pcie/test_t602x_bcm4388.c" \
    -o "$build_dir/test_t602x_bcm4388"
"$build_dir/test_t602x_bcm4388"

"$cc" $common_flags \
    -DPCIE_T602X_WIRELESS_HOST_TEST=1 \
    -fsanitize=address,undefined \
    -fno-omit-frame-pointer \
    -I"$repo_dir/src" \
    "$repo_dir/src/pcie.c" \
    "$repo_dir/tests/pcie/test_t602x_bcm4388.c" \
    -o "$build_dir/test_t602x_bcm4388-sanitized"
ASAN_OPTIONS=detect_leaks=0 "$build_dir/test_t602x_bcm4388-sanitized"

"$cc" $common_flags \
    -DPCIE_T602X_WIRELESS_HOST_TEST=1 \
    -DBCM4388_HANDOFF_HOST_TEST=1 \
    -I"$repo_dir/src" \
    "$repo_dir/src/pcie.c" \
    "$repo_dir/src/bcm4388_handoff.c" \
    "$repo_dir/tests/pcie/test_bcm4388_handoff.c" \
    -o "$build_dir/test_bcm4388_handoff"
"$build_dir/test_bcm4388_handoff"

"$cc" -std=c11 -Wall -Wextra -Werror \
    -DWIRELESS_HANDOFF_ABI_HOST_TEST=1 \
    -I"$repo_dir/src" \
    "$repo_dir/src/wireless_handoff_abi.c" \
    "$repo_dir/tests/pcie/test_wireless_handoff_abi.c" \
    -o "$build_dir/test_wireless_handoff_abi"
"$build_dir/test_wireless_handoff_abi"

"$cc" $common_flags \
    -DPCIE_T602X_WIRELESS_HOST_TEST=1 \
    -DBCM4388_HANDOFF_HOST_TEST=1 \
    -fsanitize=address,undefined \
    -fno-omit-frame-pointer \
    -I"$repo_dir/src" \
    "$repo_dir/src/pcie.c" \
    "$repo_dir/src/bcm4388_handoff.c" \
    "$repo_dir/tests/pcie/test_bcm4388_handoff.c" \
    -o "$build_dir/test_bcm4388_handoff-sanitized"
ASAN_OPTIONS=detect_leaks=0 "$build_dir/test_bcm4388_handoff-sanitized"

"$cc" -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -DWIRELESS_HANDOFF_ABI_HOST_TEST=1 \
    -I"$repo_dir/src" \
    "$repo_dir/src/wireless_handoff_abi.c" \
    "$repo_dir/tests/pcie/test_wireless_handoff_abi.c" \
    -o "$build_dir/test_wireless_handoff_abi-sanitized"
ASAN_OPTIONS=detect_leaks=0 "$build_dir/test_wireless_handoff_abi-sanitized"
