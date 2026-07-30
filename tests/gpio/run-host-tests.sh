#!/bin/sh
# SPDX-License-Identifier: MIT

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/m1n1-gpio-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc=${CC:-clang}
common_flags="-std=c11 -Wall -Wextra -Werror -Wconversion -Wsign-conversion -pedantic"

"$cc" $common_flags \
    -DAPPLE_GPIO_HOST_TEST=1 \
    -I"$repo_dir/src" \
    "$repo_dir/src/gpio.c" \
    "$repo_dir/tests/gpio/test_apple_gpio.c" \
    -o "$build_dir/test_apple_gpio"
"$build_dir/test_apple_gpio"

"$cc" $common_flags \
    -DAPPLE_GPIO_HOST_TEST=1 \
    -fsanitize=address,undefined \
    -fno-omit-frame-pointer \
    -I"$repo_dir/src" \
    "$repo_dir/src/gpio.c" \
    "$repo_dir/tests/gpio/test_apple_gpio.c" \
    -o "$build_dir/test_apple_gpio-sanitized"
ASAN_OPTIONS=detect_leaks=0 "$build_dir/test_apple_gpio-sanitized"
