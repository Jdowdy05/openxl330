#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build/native
clang -std=c11 -O2 -g -Wall -Wextra -Werror -Wno-misleading-indentation -fsanitize=address,undefined -fno-omit-frame-pointer -DPLATFORM_UART_TEST -Ifirmware tests/test_rx_dma.c firmware/platform.c firmware/observability.c -o build/native/rx-dma
if [ "$#" -gt 0 ];then ASAN_OPTIONS=detect_leaks=0 build/native/rx-dma "$1";else
 for t in config batch reads laps starved fuzz race errors te tx quiesce quiet;do ASAN_OPTIONS=detect_leaks=0 build/native/rx-dma "$t";done
fi
