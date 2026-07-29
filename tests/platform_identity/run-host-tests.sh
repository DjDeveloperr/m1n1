#!/bin/sh
# SPDX-License-Identifier: MIT

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/m1n1-platform-identity-tests.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc=${CC:-clang}
common_flags="-std=c11 -Wall -Wextra -Werror -Wconversion -Wsign-conversion -pedantic"

build_and_run()
{
    output=$1
    shift
    "$cc" $common_flags "$@" \
        -DPLATFORM_IDENTITY_HOST_TEST=1 \
        -I"$repo_dir/src" \
        "$repo_dir/src/platform_identity.c" \
        "$repo_dir/tests/platform_identity/test_j414s_identity.c" \
        -o "$output"
    "$output"
}

build_and_run "$build_dir/test_j414s_identity"
build_and_run "$build_dir/test_j414s_identity-sanitized" \
    -fsanitize=address,undefined -fno-omit-frame-pointer
