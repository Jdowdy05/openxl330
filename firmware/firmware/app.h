#ifndef GENERAL_APP_H
#define GENERAL_APP_H
#include "sensing.h"
#include "torque.h"
#include "joint_control.h"
#include "general_reference.h"
#include "deadline.h"

#include "../research/calibration/xl330_calibration.h"
#include "../research/calibration/nv_read.h"
#include <stdbool.h>
#include <stdint.h>
#ifndef APP_ALLOW_MOTOR
#define APP_ALLOW_MOTOR 1
#endif
#if APP_ALLOW_MOTOR != 1
#error PD baseline is a motor-enabled (role1) image only
#endif
#define APP_PERIOD_US 500u
#define APP_FIRMWARE_VERSION 231u /* FW231 six-servo bus (2026-11-20); FW230 = demo6 lineage */
#define APP_SPEED_TRIP_V3 UINT32_C(0x33505352)
#define APP_SPEED_WINDOW_TICKS 4u
/* fast17-size (v1.1 Stage 1, owner 2026-11-24: "heavy optimizations on the firmware so we can shrink it"): registers
 * the openxl330 host library does not use are build options, off in the performance images (build.sh) and
 * on by default here so the native tests keep covering them (like APP_BAM):
 *  APP_DAQ   the DAQ ring: read 1536 drain, write/read 840 decimation, read 600 layout; status words 108 (drops) and 112
 *            (records queued) then read 0 and APP_PERIOD_US<<16;
 *  APP_TUNE2 the demo-era live tunings 336 (comp2: b drop, inertia gate) and 340 (high-pass damping), read and write. The
 *            comp image still loads its power-up comp2 values through joint_set_comp2; its high-pass damping default is
 *            off, which is the joint_init state, so the power-up state is bit-identical (tests/golden_control.c). */
/* v1.1 Stage 3 (owner 2026-11-24: XL330-M077 BETA, "identical hardware except the lower gear ratio"): APP_GEAR 288 =
 * XL330-M288-T (288.4:1, stock model 1200, the qualified build); 77 = XL330-M077-T (77.5:1, stock model 1190) BETA:
 * power-up compensation, jump gate and motion-block constants scaled from the M288 by r = 77.5/288.4 (ASSUMPTIONS,
 * docs/M077_BETA.md), to be re-identified on hardware. app_init accepts only
 * the stock model number of its own gear, so an image never arms on the other model. build.sh GEAR=77. */
#ifndef APP_GEAR
#define APP_GEAR 288
#endif
#if APP_GEAR == 77
#define APP_MODEL_NUMBER 1190u
#elif APP_GEAR == 288
#define APP_MODEL_NUMBER 1200u
#else
#error APP_GEAR must be 288 (XL330-M288) or 77 (XL330-M077 beta)
#endif
#ifndef APP_DAQ
#define APP_DAQ 1
#endif
#ifndef APP_DIAG860
#define APP_DIAG860 1   /* fast17: read 860, its tick counters and the first-trip words 224..255 (logging builds) */
#endif
#ifndef APP_TUNE2
#define APP_TUNE2 1
#endif
#define APP_RING_RECORDS 32u
#define APP_RECORD_BYTES 32u
enum { APP_FAULT_SENSOR=1u,APP_FAULT_TIMEOUT=2u,APP_FAULT_CURRENT=4u,APP_FAULT_VOLTAGE=8u,APP_FAULT_TEMPERATURE=16u,APP_FAULT_TIMING=32u,APP_FAULT_CONFIG=64u,APP_FAULT_TRAVEL=128u,APP_FAULT_SPEED=256u,APP_FAULT_DAQ=512u,APP_FAULT_ENERGY=1024u };
typedef struct {uint32_t t;int32_t counts,qd,vd,v_hat,torque;int16_t command,actual,a_hat_q4;uint16_t flags;} app_record;
typedef struct {int32_t center_counts,center_q16;uint32_t tick,stride;uint8_t kind;bool selected,running,released,learning,first;} app_bam;
typedef struct {int32_t counts,fraction;uint32_t time;uint16_t raw;uint8_t ticks;bool ready;} app_speed_guard;
typedef struct {
 app_speed_guard speed_guard;
 xl330_calibration calibration;xl330_position_tracker tracker;xl330_nv_snapshot nv;xl330_nv_config nv_config;sensing_adc adc;torque_velocity_t velocity;
 joint_control control;general_reference reference;app_bam bam;
#if APP_DAQ
 app_record ring[APP_RING_RECORDS];
#else
 app_record ring_reserved[APP_RING_RECORDS];   /* unused RAM: keeps the app_state layout of fast16 (app_tick and TIM16 code unchanged) */
#endif

 uint32_t epoch,now_us,last_tick_us,last_heartbeat_us,samples,faults,hardware_faults,worst_tick_us,drops;
 uint32_t high_current_since_us,freeze_count,reject_count,reason_bits,last_flags,first_trip[8];
 int32_t position_counts,velocity_mrad_s,arm_counts,target_counts,requested_q24;uint32_t duration_ticks,travel_limit,speed_limit,current_limit_ma;
 uint64_t i2_q12;uint32_t current_rms_limit_ma;uint8_t temp_limit_c,vmax_dv;
 bucket_budget sample_budget;uint32_t jump_rejects,magnet_lost,magnet_warnings;uint16_t encoder_status;uint8_t bad_streak;bool have_adc,coast;
 int16_t commanded_pwm,actual_pwm;uint16_t raw_angle,sensor_conf_before,sensor_conf_after,ring_head,ring_count;
 int16_t openloop_pwm;
 uint8_t good_ticks,daq_phase,target_profile,daq_decimation;bool tick_missed,openloop,ready,enabled,motor_allowed,startup_valid,sensor_healthy,have_tick,high_current,target_configured;
 uint32_t reacquisitions,reacq_rejects;uint16_t reacq_cand;bool have_cand;   /* fast17 sensor reacquire (read 856 words 6-7); appended */
 bool upper_volt_on;uint8_t volt_warn;   /* fast17 voltage decision (1152, status word 3); appended */
 uint16_t nv_model;   /* Stage 3: the servo's stock model number from its NV journal (0: unreadable); last, so no earlier field moves */
} app_state;
typedef struct {
 uint32_t epoch,faults,duration,requested_profile,profile;int32_t position,target,ref_q,requested_target;uint32_t requested_duration;
 bool enabled,ready,motor_allowed,startup_valid,sensor_healthy,configured,ref_active,changed;
 uint16_t kind;general_reference reference;app_bam bam,requested_bam;
} app_write_plan;
void app_write_capture(const app_state *,app_write_plan *);
uint8_t app_write_prepare(app_write_plan *,uint16_t,uint16_t,const uint8_t *);
uint8_t app_write_commit(app_state *,const app_write_plan *);
bool app_init(app_state *,const uint8_t *,const uint8_t *,bool);
#ifndef APP_BAM
#define APP_BAM 1   /* native tests keep BAM covered; the ARM build passes -DAPP_BAM=$BAM (default 0) */
#endif
#ifndef APP_DEFAULT_COMP
#define APP_DEFAULT_COMP 0
#endif
bool app_load_default_comp(joint_control *); /* PD-comp image power-up gear-train cancellation (324/336/340 values) */
bool app_tick(app_state *,bool,uint16_t,const sensing_adc *,bool,uint32_t);

bool app_commit(app_state *,int16_t,bool);
void app_fault(app_state *,uint32_t);
void app_record_timing(app_state *,uint32_t);
void app_note_timing_trip(app_state *,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);
/* FW231 motion block: the planned acceleration (Q16 rad/s^2) of the running profile; 0 for hold, pure steps,
 * linear profiles, open-loop duty, BAM and OFF (firmware/motion.h est_tau_out). */
int32_t app_planned_accel(const app_state *s);
void app_record_applied(app_state *);
#define APP_LOW_SNAPSHOT_WORDS 25u
void app_low_snapshot(app_state *,uint32_t *);
void app_low_format(const app_state *,const uint32_t *,uint16_t,uint16_t,uint8_t *);
uint8_t app_read(void *,uint16_t,uint16_t,uint8_t *);
uint8_t app_write(void *,uint16_t,uint16_t,const uint8_t *);
/* Stage 3: the model number PING and addresses 0/1 report: the NV journal's when readable, else APP_MODEL_NUMBER. */
uint16_t app_model(const app_state *);
/* A0 flash read: our program region and the servo's own NVM only (fast12: never the retained loader). */
enum { APP_LOADER_BASE = 0x08000000u, APP_PROGRAM_BASE = 0x08004000u, APP_LOADER_BYTES = 0x4000u };
uint8_t app_flash_read(void *,uint32_t,uint16_t,uint8_t *);
uint32_t app_loader_crc32(void);   /* fast12: CRC-32 (zlib) of the whole loader region (read 1136) */
#endif
