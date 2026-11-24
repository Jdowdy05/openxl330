#!/bin/sh
# PD baseline (2026-11-16): motor-enabled role1 image only. Learner, friction/
# inertia bank, energy ledger and role0 synthetic stress are retired (see retired/).
# DEMO=1 builds the separate boot-into-hand-feel demo image into build/role-1-demo (never the default).
# FW231 (2026-11-20, six-servo bus): + firmware/bus.c (chained group replies, baud, device block)
# and firmware/devcfg.c (the 32-byte device block linked at 0x080040C0).
# COMP=1 builds the PD-comp image into build/role-1-comp: boots OFF, gear-train cancellation loaded at power-up
# (ported 2026-11-20 from the earlier PD-comp image, unchanged).
# LAB=1: FW230 lab reads (976 command trace, 896 observation snapshot + its per-tick GOBS publish and idle stack scan,
# 864 DMA counters; the bench host tools) and the boot/fault stage marks in retained RAM. Off in every FW231 build (flash). LAB=1 needs FAST=0 MOTION=0 to fit.
# MOTION=0: leaves out the read-1088 motion block (on by default).
# FAST=0/1: fast start (USART-IRQ-started chained / Fast Sync Read replies). Default on, off for DEMO=1 (flash).
# BAM=1: native BAM profiles (write 272). Off in every FW231 build since the opt-in lease (flash); no FW231 tool uses them.
# fast17-size (v1.1 Stage 1): every file -Os (joint_control.c and est32.c were -O2: about -1.6 KB, the tick computation
# +3.7 us modelled); DAQ=1 / TUNE2=1 / DIAG860=1 put back the DAQ ring (1536/840/600), the live tunings 336/340 and read
# 860, all off in the performance images (the openxl330 host library does not use them; bench and qualification tools use the logging builds).
# DEMO=1 keeps TUNE2=1 (its power-up writes 336/340).
# GEAR=77 (with COMP=1): the XL330-M077 BETA PD-comp image into build/role-1-comp-m077 (v1.1 Stage 3; constants scaled
# from the M288, not measured; accepts only stock model 1190). Default GEAR=288 (XL330-M288).
# LOG=1 (owner 2026-11-24, openxl330 0.2.0 bundles performance + logging images): the logging build = DAQ=1 DIAG860=1
# (2 kHz DAQ ring and read 860 back; LAB stays 0), into build/role-1-comp-log or build/role-1-comp-m077-log.
set -eu
cd "$(dirname "$0")"
role=1
[ "${DEMO:-0}" = 1 ] && { echo "DEMO=1: the demo images are retired (build them from an earlier release)" >&2; exit 2; };demo=0;comp=${COMP:-0};dir="build/role-$role";[ "$demo" = 1 ] && dir="build/role-$role-demo";[ "$comp" = 1 ] && dir="build/role-$role-comp";[ "$demo$comp" = 11 ] && { echo "DEMO and COMP are exclusive" >&2; exit 2; };gear=${GEAR:-288};case "$gear" in 288) ;; 77) [ "$comp" = 1 ] || { echo "GEAR=77 needs COMP=1" >&2; exit 2; };dir="build/role-$role-comp-m077";; *) echo "GEAR must be 288 or 77" >&2; exit 2;; esac;log=${LOG:-0};daq=${DAQ:-$log};diag860=${DIAG860:-$log};[ "$log" = 1 ] && { [ "$comp" = 1 ] || { echo "LOG=1 needs COMP=1" >&2; exit 2; };[ "$daq$diag860" = 11 ] || { echo "LOG=1 means DAQ=1 DIAG860=1" >&2; exit 2; };dir="$dir-log"; };bam=${BAM:-0};lab=${LAB:-0};fast=${FAST:-1};motion=${MOTION:-1};[ "$demo" = 1 ] && fast=${FAST:-0};rm -rf "$dir";mkdir -p "$dir"
for source in firmware/*.c research/calibration/xl330_calibration.c research/calibration/nv_read.c; do
 name=$(basename "$source" .c)
 opt=-Os
 arm-none-eabi-gcc -std=c11 "$opt" -g -mcpu=cortex-m0plus -mthumb -ffreestanding -fno-builtin -fno-stack-protector -fdata-sections -ffunction-sections -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-tree-loop-distribute-patterns -fstack-usage -Wall -Wextra -Werror -Wno-misleading-indentation -DAPP_ALLOW_MOTOR="$role" -DAPP_DEMO_TRANSPARENT="$demo" -DAPP_DEFAULT_COMP="$comp" -DAPP_BAM="$bam" -DAPP_LAB="$lab" -DPLATFORM_DEBUG_MARKS="$lab" -DAPP_FAST_START="$fast" -DAPP_MOTION="$motion" -DAPP_DAQ="$daq" -DAPP_TUNE2="${TUNE2:-$demo}" -DAPP_DIAG860="$diag860" -DAPP_GEAR="$gear" -DAPP_IDLOG="${IDLOG:-1}" -DFW_SOURCE_ID="${FW_SOURCE_ID:-0}u" -DAPP_ASYNC_TX=0 -DAPP_COOPERATIVE_LEARNING=0 -DSENSING_FAST_I2C=1 -I firmware -I research/calibration -c "$source" -o "$dir/$name.o"
done
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -nostdlib -Wl,--gc-sections,--no-undefined,-Map="$dir/candidate.map" -T firmware/controller.ld "$dir"/*.o -lgcc -o "$dir/candidate.elf"
arm-none-eabi-objcopy -O binary "$dir/candidate.elf" "$dir/candidate.bin"
arm-none-eabi-objdump -d "$dir/candidate.elf" > "$dir/candidate.disassembly.txt"
arm-none-eabi-nm -n "$dir/candidate.elf" > "$dir/candidate.symbols.txt"
arm-none-eabi-size "$dir/candidate.elf"
python3 -B tests/check_image.py "$dir/candidate.bin" $([ "${IDLOG:-1}" = 1 ] && echo --idlog)
