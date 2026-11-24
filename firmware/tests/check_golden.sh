#!/bin/sh
# Golden traces of the control law (tests/golden_control.c), native with ASan/UBSan, for the four released variants
# (owner 2026-11-24: {M288, M077} x {performance, logging}), each against its own expected trace:
#   comp           -DAPP_DAQ=0 -DAPP_DIAG860=0             tests/golden_expected.txt
#   comp-log       -DAPP_DAQ=1 -DAPP_DIAG860=1             tests/golden_expected_log.txt
#   comp-m077      -DAPP_GEAR=77 (M077 plant, K, gains)   tests/golden_expected_m077.txt
#   comp-m077-log  -DAPP_GEAR=77 -DAPP_DAQ=1 ...           tests/golden_expected_m077_log.txt
# fast16 parity (tests/golden_fast16.txt, from the fast16 sources): the comp trace equals fast16 in every scenario that
# fast17 does not change by design ('sensors': sensor reacquire). Each scenario starts with fast16's 848 values (vmax 7.0 V),
# so fast17's 8.0 V default does not show; the 7 V warning and 1152 are covered by tests/test_voltage.c.
# tests/golden_qemu/run.sh repeats a trace on the image's own ARM objects (GOLDEN_FLAGS / EXPECTED for the others).
set -eu
cd "$(dirname "$0")/.."
out=$(mktemp -d "${TMPDIR:-/tmp}/golden.XXXXXX"); trap 'rm -rf "$out"' EXIT
src="tests/hw_stub.c firmware/app.c firmware/joint_control.c firmware/est32.c firmware/bam_native.c firmware/deadline.c firmware/general_reference.c firmware/torque.c firmware/observability.c firmware/motion.c research/calibration/xl330_calibration.c research/calibration/nv_read.c"
run() {   # name expected-file flags...
  name=$1; exp=$2; shift 2
  # shellcheck disable=SC2086
  clang -std=c11 -O1 -g -Wall -Wextra -Werror -Wno-misleading-indentation -fsanitize=address,undefined -fno-sanitize-recover=undefined \
    -DAPP_ALLOW_MOTOR=1 -DAPP_DEFAULT_COMP=1 -DAPP_BAM=0 -DAPP_TUNE2=0 -DAPP_LAB=0 "$@" -Ifirmware -Iresearch/calibration \
    tests/golden_control.c $src -o "$out/$name"
  "$out/$name" > "$out/$name.txt"
  if [ "${GOLDEN_WRITE:-0}" = 1 ]; then cp "$out/$name.txt" "$exp"; fi
  if ! cmp -s "$out/$name.txt" "$exp"; then echo "golden traces of $name differ from $exp:" >&2; diff "$exp" "$out/$name.txt" >&2; exit 1; fi
  echo "golden traces $name: $(tail -1 "$out/$name.txt" | cut -d' ' -f2) PASS"
}
run comp tests/golden_expected.txt -DAPP_DAQ=0 -DAPP_DIAG860=0
run comp-log tests/golden_expected_log.txt -DAPP_DAQ=1 -DAPP_DIAG860=1
run comp-m077 tests/golden_expected_m077.txt -DAPP_GEAR=77 -DAPP_DAQ=0 -DAPP_DIAG860=0
run comp-m077-log tests/golden_expected_m077_log.txt -DAPP_GEAR=77 -DAPP_DAQ=1 -DAPP_DIAG860=1
keep='^sensors \|^golden '
grep -v "$keep" tests/golden_fast16.txt > "$out/f16"; grep -v "$keep" "$out/comp.txt" > "$out/c6"
cmp -s "$out/f16" "$out/c6" || { echo "comp: a scenario fast17 should not change differs from fast16" >&2; diff "$out/f16" "$out/c6" >&2; exit 1; }
echo "golden traces: comp == fast16 in the 7 scenarios fast17 does not change by design (sensors: reacquire) PASS"
