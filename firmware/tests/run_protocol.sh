#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/xl330-protocol.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
${CC:-clang} -std=c11 -ffreestanding -fno-builtin -Wall -Wextra -Werror \
    -pedantic -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I "$root/firmware" "$root/tests/test_protocol.c" \
    "$root/firmware/protocol.c" -o "$build_dir/test_protocol"
"$build_dir/test_protocol"
