#!/bin/sh
# SPDX-License-Identifier: MIT

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/m1n1-usb-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc=${CC:-clang}
common_flags="-std=c11 -Wall -Wextra -Werror -Wconversion -Wsign-conversion -pedantic"

"$cc" $common_flags \
    -DTPS6598X_HOST_POLICY_HOST_TEST=1 \
    -I"$repo_dir/src" \
    "$repo_dir/src/tps6598x_host_policy.c" \
    "$repo_dir/tests/usb/test_tps6598x_host_policy.c" \
    -o "$build_dir/test_tps6598x_host_policy"
"$build_dir/test_tps6598x_host_policy"

"$cc" $common_flags \
    -DTPS6598X_HOST_POLICY_HOST_TEST=1 \
    -fsanitize=address,undefined \
    -fno-omit-frame-pointer \
    -I"$repo_dir/src" \
    "$repo_dir/src/tps6598x_host_policy.c" \
    "$repo_dir/tests/usb/test_tps6598x_host_policy.c" \
    -o "$build_dir/test_tps6598x_host_policy-sanitized"
ASAN_OPTIONS=detect_leaks=0 "$build_dir/test_tps6598x_host_policy-sanitized"
