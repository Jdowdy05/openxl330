#!/bin/sh
# tests/golden_qemu/run.sh OBJDIR [SOURCE_TREE]: link tests/golden_control.c (built like the firmware, -Os) with the ARM
# objects of a firmware build (e.g. build/role-1-comp), run it on qemu -M microbit (Cortex-M0, same ARMv6-M code), compare
# the trace with tests/golden_expected.txt, then run it again under the M0+ cycle model (m0cycles.c qemu plugin) and print
# the cycles of each tick's control computation (app_tick + app_commit, as TIM16 calls them). SOURCE_TREE gives the
# headers the objects were built from (default: this tree). Needs arm-none-eabi-gcc, qemu-system-arm with plugins, glib.
set -eu
here=$(cd "$(dirname "$0")" && pwd); repo=$(cd "$here/../.." && pwd); obj=$(cd "$1" && pwd); inc=${2:-$repo}
tmp=$(mktemp -d "${TMPDIR:-/tmp}/golden-qemu.XXXXXX"); trap 'rm -rf "$tmp"' EXIT
flags="-std=c11 -Os -g -mcpu=cortex-m0plus -mthumb -ffreestanding -fno-builtin -fno-stack-protector -fdata-sections -ffunction-sections -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-tree-loop-distribute-patterns -DAPP_ALLOW_MOTOR=1 -DAPP_DEFAULT_COMP=1 -DAPP_BAM=0 -DAPP_DAQ=0 -DAPP_TUNE2=0 -DAPP_LAB=0 -DGOLDEN_EMBEDDED=1 -DGOLDEN_MARK_WRITE=${MARK_WRITE:-0} ${GOLDEN_FLAGS:-}"   # MARK_WRITE=320: count one write address instead
# GOLDEN_FLAGS / EXPECTED: another build's driver flags and reference trace, e.g. the M077 beta objects:
#   GOLDEN_FLAGS="-DAPP_GEAR=77 -DGOLDEN_MODEL=1190u" EXPECTED=<native trace of the same flags> run.sh build/role-1-comp-m077
expected=${EXPECTED:-$repo/tests/golden_expected.txt}
# shellcheck disable=SC2086
arm-none-eabi-gcc $flags -I"$inc/firmware" -I"$inc/research/calibration" -c "$repo/tests/golden_control.c" -o "$tmp/golden.o"
# shellcheck disable=SC2086
arm-none-eabi-gcc $flags -c "$here/runtime.c" -o "$tmp/runtime.o"
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -nostdlib -Wl,--gc-sections,--no-undefined,-e,Reset_Handler -T "$here/microbit.ld" \
  "$tmp/runtime.o" "$tmp/golden.o" "$obj/app.o" "$obj/joint_control.o" "$obj/est32.o" "$obj/general_reference.o" "$obj/torque.o" \
  "$obj/motion.o" "$obj/deadline.o" "$obj/xl330_calibration.o" "$obj/nv_read.o" "$obj/bam_native.o" "$obj/observability.o" \
  "$obj/platform.o" -lgcc -o "$tmp/golden.elf"   # platform.o: the image's own memcpy/memset only (--gc-sections)
glib=$(pkg-config --cflags --libs glib-2.0 2>/dev/null || echo "-I/opt/homebrew/include/glib-2.0 -I/opt/homebrew/lib/glib-2.0/include -L/opt/homebrew/lib -lglib-2.0")
qinc=$(dirname "$(command -v qemu-system-arm)")/../include
# shellcheck disable=SC2086
cc -O2 -shared -fPIC -undefined dynamic_lookup -I"$qinc" $glib "$here/m0cycles.c" -o "$tmp/m0cycles.so"
run() { out=$1; shift; perl -e 'alarm 1800; exec @ARGV' qemu-system-arm -M microbit -nographic -chardev "file,id=sh0,path=$out"   -semihosting-config enable=on,target=native,chardev=sh0 "$@" -kernel "$tmp/golden.elf"; }   # semihosting output -> $out
run "$tmp/trace.txt"
if ! cmp -s "$tmp/trace.txt" "$expected"; then echo "ARM golden traces differ from $expected:" >&2; diff "$expected" "$tmp/trace.txt" >&2; exit 1; fi
echo "ARM golden traces (qemu, $obj): $(tail -1 "$tmp/trace.txt" | cut -d' ' -f2) == $(basename "$expected") PASS"
b=$(arm-none-eabi-nm "$tmp/golden.elf" | awk '$3=="golden_mark_begin"{print "0x"$1}'); e=$(arm-none-eabi-nm "$tmp/golden.elf" | awk '$3=="golden_mark_end"{print "0x"$1}')
run "$tmp/trace2.txt" -plugin "$tmp/m0cycles.so,begin=$b,end=$e,ws=${WS:-2}" -d plugin -D "$tmp/cycles.txt"
cmp -s "$tmp/trace2.txt" "$tmp/trace.txt"; echo "$([ "${MARK_WRITE:-0}" = 0 ] && echo "tick computation (app_tick + app_commit)" || echo "app_write $MARK_WRITE"), M0+ model, flash ${WS:-2} wait states:"; cat "$tmp/cycles.txt"
