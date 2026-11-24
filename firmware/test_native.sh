#!/bin/sh
# PD baseline native suite: new PD/app tests plus the unchanged platform,
# RX DMA, protocol, bridge, sensing, reference and velocity suites inherited
# from the qualified RX DMA image. Learner/energy/role0 suites are retired.
set -eu
cd "$(dirname "$0")"
# PYTHON: interpreter for the Python checks (default python3).
# XL330_HOST_TESTS=1 also runs the host tool unit tests (need external host dependencies).
PYTHON="${PYTHON:-python3}"
mkdir -p build/native
flags='-std=c11 -O1 -g -Wall -Wextra -Werror -Wno-misleading-indentation -fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer -DAPP_ASYNC_TX=0 -DSENSING_FAST_I2C=1'
clang $flags -DAPP_ALLOW_MOTOR=1 -Ifirmware -Iresearch/calibration tests/test_pd.c tests/hw_stub.c firmware/app.c firmware/joint_control.c firmware/est32.c firmware/bam_native.c firmware/deadline.c firmware/general_reference.c firmware/torque.c firmware/observability.c research/calibration/xl330_calibration.c research/calibration/nv_read.c -o build/native/pd;build/native/pd
clang $flags -Wno-unused-function -DAPP_ALLOW_MOTOR=1 -DAPP_DEFAULT_COMP=1 -Ifirmware -Iresearch/calibration tests/test_pd.c tests/hw_stub.c firmware/app.c firmware/joint_control.c firmware/est32.c firmware/bam_native.c firmware/deadline.c firmware/general_reference.c firmware/torque.c firmware/observability.c research/calibration/xl330_calibration.c research/calibration/nv_read.c -o build/native/pd-comp;build/native/pd-comp
"$PYTHON" -B tests/check_comp_defaults.py
sh tests/check_golden.sh   # fast17-size: control-law golden traces with the deployment build flags == fast16, bit for bit
clang $flags -Ifirmware tests/test_deadline.c firmware/deadline.c -o build/native/deadline;build/native/deadline
clang $flags -Ifirmware tests/test_reference.c firmware/general_reference.c -o build/native/reference;build/native/reference
clang $flags -Ifirmware tests/test_observability.c firmware/observability.c -o build/native/observability;build/native/observability
for module in bridge sensing; do clang $flags -Ifirmware tests/test_$module.c firmware/$module.c -o build/native/$module;build/native/$module;done
for variant in time uart; do case $variant in time) macro=PLATFORM_TIME_TEST;;*) macro=PLATFORM_UART_TEST;;esac;clang $flags -D$macro -Ifirmware tests/test_platform_$variant.c firmware/platform.c firmware/observability.c -o build/native/platform-$variant;build/native/platform-$variant;done
clang $flags -DPLATFORM_UART_TEST -DPLATFORM_DEBUG_MARKS=1 -Ifirmware tests/test_dma_contract.c firmware/platform.c firmware/observability.c -o build/native/dma;build/native/dma
for t in protocol_scratch protocol_id2 unstuff; do clang $flags -Ifirmware tests/test_$t.c firmware/protocol.c -o build/native/$t;build/native/$t;done
sh tests/run_protocol.sh
clang $flags -Ifirmware tests/test_fractional_velocity.c firmware/torque.c -o build/native/fractional;build/native/fractional
sh tests/run_rx_dma.sh
# Fractional calibration map vs the stock integer map and an exact rational oracle (synthetic fixture).
clang -std=c11 -O2 -shared -fPIC -Iresearch/calibration research/calibration/xl330_calibration.c -o build/native/fractional-map.so
clang -std=c11 -O2 -shared -fPIC -Iresearch/calibration tests/stock_calibration.c -o build/native/stock-map.so
"$PYTHON" -B tests/test_fractional_oracle.py build/native/fractional-map.so build/native/stock-map.so tests/fixtures/synthetic-calibration.bin
"$PYTHON" -B tests/check_bam_oracle.py
"$PYTHON" -B tests/make_temperature_lut.py --check firmware/sensing.c
# FW231 motion block (read 1088): fixed-point torque estimate vs the double model, layout, edges.
clang $flags -Ifirmware tests/test_motion.c firmware/motion.c -lm -o build/native/motion;build/native/motion
# fast17: ID log (set ID over the bus) against a simulated page; sensor reacquire (both gears); voltage decision (848/1152).
clang $flags -DAPP_ALLOW_MOTOR=1 -DAPP_DEFAULT_COMP=1 -DAPP_BAM=0 -Ifirmware -Iresearch/calibration tests/test_voltage.c tests/hw_stub.c firmware/app.c firmware/joint_control.c firmware/est32.c firmware/bam_native.c firmware/deadline.c firmware/general_reference.c firmware/torque.c firmware/observability.c firmware/motion.c research/calibration/xl330_calibration.c research/calibration/nv_read.c -o build/native/voltage;build/native/voltage
clang $flags -Ifirmware tests/test_idlog.c firmware/idlog.c firmware/protocol.c -o build/native/idlog;build/native/idlog
for gear in 288 77; do clang $flags -DAPP_ALLOW_MOTOR=1 -DAPP_DEFAULT_COMP=1 -DAPP_GEAR=$gear -DAPP_BAM=0 -Ifirmware -Iresearch/calibration tests/test_reacquire.c tests/hw_stub.c firmware/app.c firmware/joint_control.c firmware/est32.c firmware/bam_native.c firmware/deadline.c firmware/general_reference.c firmware/torque.c firmware/observability.c firmware/motion.c research/calibration/xl330_calibration.c research/calibration/nv_read.c -o build/native/reacquire-$gear;build/native/reacquire-$gear;done
# v1.1 Stage 3: XL330-M077 BETA build (GEAR=77): scaled motion constants vs the double model; app identity/tables/gate.
clang $flags -Wno-unused-function -DAPP_GEAR=77 -Ifirmware tests/test_motion.c firmware/motion.c -lm -o build/native/motion-m077;build/native/motion-m077
clang $flags -DAPP_ALLOW_MOTOR=1 -DAPP_DEFAULT_COMP=1 -DAPP_GEAR=77 -DAPP_BAM=0 -DAPP_DAQ=0 -DAPP_TUNE2=0 -Ifirmware -Iresearch/calibration tests/test_m077.c tests/hw_stub.c firmware/app.c firmware/joint_control.c firmware/est32.c firmware/bam_native.c firmware/deadline.c firmware/general_reference.c firmware/torque.c firmware/observability.c firmware/motion.c research/calibration/xl330_calibration.c research/calibration/nv_read.c -o build/native/m077;build/native/m077
# FW231 six-servo bus: bus.c units + 6-servo wire simulation (real protocol.c/bus.c), scheduler mutations.
clang $flags -Ifirmware tests/test_bus.c firmware/bus.c firmware/protocol.c -o build/native/bus;build/native/bus
"$PYTHON" -B tests/check_bus_mutations.py
"$PYTHON" -B tests/check_torque_mutations.py   # fast16: every torque-path mutant must fail the tests
# FW231 fast9: byte-level model of main.c's receive loop + PendSV watcher + tick vs back-to-back Fast Sync Reads
# (hardware bus3-05: 4-byte rows lost, middle servo). Old loop must reproduce the loss, the fixed loop lose nothing.
# Optimised, UBSan only: ~200k simulated reads. main.c must carry the fix and counters the model mirrors.
clang -std=c11 -O2 -g -Wall -Wextra -Werror -Wno-misleading-indentation -fsanitize=undefined -fno-sanitize-recover=undefined -Ifirmware tests/test_fsr_loop.c firmware/bus.c firmware/protocol.c -o build/native/fsr_loop;build/native/fsr_loop
for pattern in 'if(APP_FAST_START&&watch_sent)continue;' 'hdr=(hdr<<8)|byte;if(hdr==0xfffffd00u&&!skip_left&&have!=3u)hdr_missed++;' \
  'if(!platform_uart_tx_idle())last_byte_us=now;' 'bus_reply_sent(&reply,t0);pending_size=0;have=total=0;last_byte_us=hw_micros();' \
  'uint8_t gate=bus_lease_gate(&lease,a,arm,app.enabled);' 'request_loader();app_fault(&app,APP_FAULT_TIMEOUT);platform_hang_reset(PLATFORM_RESET_STALL);}' 'platform_boot_capture();' 'platform_irq_enable();platform_watchdog_start();'; do
  grep -qF "$pattern" firmware/main.c || { echo "main.c lacks a fast9/fast10 line the models and units verify: $pattern" >&2; exit 1; }
done   # fast9 guard + counters, fast10 review F-4 (both send paths), F-2 (lease gate), F-6 (stall -> loader from the tick)
sh tests/run_fw231_main_native.sh
# Dynamixel SDK (official ROBOTIS Python SDK) against the firmware's protocol.c over a pty.
if "$PYTHON" -c 'import dynamixel_sdk' 2>/dev/null; then "$PYTHON" -B tests/test_sdk_compat.py; else echo "Dynamixel SDK compatibility SKIPPED (pip install dynamixel-sdk)"; fi
# Host tool tests against the simulated bus (only where the host tools are present).
if [ "${XL330_HOST_TESTS:-0}" = 1 ] && [ -d host ]; then (cd host && "$PYTHON" -B -m unittest tests.test_xlbus tests.test_wizard); else echo "host tool tests SKIPPED (set XL330_HOST_TESTS=1)"; fi
echo "FW231 native suite: ALL PASS"
