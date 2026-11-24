#!/bin/sh
# Compile the production main loop, ISR, protocol and controller with synthetic
# peripherals. Run from any directory. Optional arguments select named cases.
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/fw231-main-test.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT
trap 'exit 1' HUP INT TERM

"${CC:-clang}" -std=c11 -O1 -g -Wall -Wextra -Werror \
    -Wno-misleading-indentation -fsanitize=address,undefined \
    -fno-sanitize-recover=undefined -fno-omit-frame-pointer \
    -DAPP_ALLOW_MOTOR=1 -DAPP_FAST_START=1 -DAPP_LAB=0 -DPLATFORM_DEBUG_MARKS=0 \
    -DAPP_DEFAULT_COMP=1 -DAPP_BAM=0 -DAPP_DAQ=0 -DAPP_TUNE2=0 -DAPP_DIAG860=0 \
    -Ifirmware -Iresearch/calibration \
    tests/test_fw231_main.c firmware/app.c firmware/bam_native.c firmware/bus.c \
    firmware/deadline.c firmware/est32.c firmware/general_reference.c \
    firmware/joint_control.c firmware/motion.c firmware/observability.c \
    firmware/protocol.c firmware/torque.c firmware/idlog.c research/calibration/nv_read.c \
    research/calibration/xl330_calibration.c -o "$build_dir/test_fw231_main"

if [ "$#" -eq 0 ]; then
    set -- deadline-51 deadline-threshold deadline-overrun parser-fragmented parser-loss \
        parser-skip-loss goal-unicast-pending goal-sync-pending \
        goal-retry-ceiling goal-invalid goal-reference-race watchdog-main-loop-only loader-crc-cached lease-renew-torque lease-renew-other torque-writes-masked trimmed-registers-0 trimmed-registers-1 trimmed-registers-2 trimmed-registers-3 ping-model set-id set-id-guards
fi
for test_case do
    "$build_dir/test_fw231_main" "$test_case"
done
