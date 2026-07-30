#!/bin/sh
# SPDX-License-Identifier: MIT

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/m1n1-atcphy-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc=${CC:-clang}
common_flags="-std=c11 -Wall -Wextra -Werror -Wconversion -Wsign-conversion -pedantic"

"$cc" $common_flags \
    -DATCPHY_CORE_HOST_TEST=1 \
    -I"$repo_dir/src" \
    "$repo_dir/src/atcphy_core.c" \
    "$repo_dir/tests/atcphy/test_atcphy_core.c" \
    -o "$build_dir/test_atcphy_core"
"$build_dir/test_atcphy_core"

"$cc" $common_flags \
    -DATCPHY_CORE_HOST_TEST=1 \
    -fsanitize=address,undefined \
    -fno-omit-frame-pointer \
    -I"$repo_dir/src" \
    "$repo_dir/src/atcphy_core.c" \
    "$repo_dir/tests/atcphy/test_atcphy_core.c" \
    -o "$build_dir/test_atcphy_core-sanitized"
ASAN_OPTIONS=detect_leaks=0 "$build_dir/test_atcphy_core-sanitized"
