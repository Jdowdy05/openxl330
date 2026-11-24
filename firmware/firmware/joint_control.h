#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H
/* PD baseline (2026-11-16): plain position-based torque PD, no learner, no
 * friction/inertia compensation, no payload/gravity model, no energy ledger.
 *   tau = Kp*(q_ref - q) + Kd*(v_ref - v_hat), clamped to +-torque_limit
 *   pwm = tau / (A * V_supply) * 480   (A = actuator gain Nm/V, user supplied)
 * q is the calibrated fractional encoder angle; v_hat/a_hat come from the
 * 32-bit constant-acceleration tracker (est32). Gains are user tuning values
 * for whatever is mounted on the horn; nothing here assumes a payload. */
#include "est32.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
 int32_t kp_q16;            /* Nm/rad Q16, >0 */
 int32_t kd_q16;            /* Nm*s/rad Q16, >=0 */
 uint32_t torque_limit_q24; /* Nm Q24, >0 */
 bool blank_reversal;        /* config word 40 bit 8; default 0 = fast reversal */
 est32_config est;
} pd_config;
/* Gearbox compensation (2026-11-16), all zero = off. Output torque tau (PD + inertia FF):
 *   tau_motor = tau + sat(v_hat/v_eps)*(c0 + c1*|tau|) + b*v_hat  [+ backlash boost]
 * c0 Nm Q24, c1 dimensionless Q16, b Nm*s/rad Q16, ja kg*m^2 Q24 (x reference accel),
 * v_eps rad/s Q16 (>0 when c0/c1 used), boost Nm Q24 for boost_ticks after a sign flip. */
typedef struct {int32_t c0_q24,c1_q16,b_q16,ja_q24,veps_q16,boost_q24;uint32_t boost_ticks;int32_t jfb_q24;uint32_t afilt_shift;bool fast_accel;} jc_comp;
/* jfb: rotor-inertia feedback (kg*m^2 Q24) x low-passed estimated acceleration (shift k:
 * a_f += (a_hat - a_f) >> k, k 0..7). The motor accelerates its own rotor so the gears
 * carry less of it; also adds to the load used by the c1 term. 40 B, magic at 36.
 * v12: word 32 = afilt_shift (bits 0..7) | fast_accel (bit 8). fast_accel=1 takes the
 * acceleration for jfb from a second, fixed fast tracker (KF30 gains, lead 1) that runs on
 * every sample, so a slow D-term estimator (e.g. T15) no longer lags the inertia feedback
 * (lag x jfb injects energy: hardware run motor-02). */
#define JC_COMP_BYTES 40u
/* demo4 candidate (owner, 2026-11-20), all zero = off (identical to demo3):
 *   b_drop: back-EMF compensation falls with the last applied duty, b_eff = b - b_drop*|pwm|/480 (>= 0).
 *           Measured slope 0.0225 Nm*s/rad at low duty -> 0.016 at full duty; one constant b over-compensates
 *           at speed (negative net damping: hardware runs vh-probe-01/02).
 *   gate:   flags bit0 drops the rotor-inertia torque near standstill (|v_hat| < vgate) unless it pushes in the
 *           direction of motion. The lagged deceleration estimate otherwise keeps braking after the stop and
 *           starts the backswing (hardware run demo-probe-06). 16 B {b_drop_q16, vgate_q16, flags, magic}. */
typedef struct {int32_t b_drop_q16,vgate_q16;uint32_t flags;} jc_comp2;
/* demo5 candidate (owner, 2026-11-20): high-pass damping. tau += -kd_hp*(v_hat - v_lp), v_lp += (v_hat - v_lp) >> shift
 * (corner ~ 2000/2^shift/(2*pi) Hz: shift 8 -> 1.2 Hz, 9 -> 0.6 Hz). Damps a contact oscillation (thumb: 3-8 Hz) while
 * slow or steady backdriving sees almost no damping (a constant Kd made it hard to backdrive). 12 B {kd_q16, shift, magic};
 * kd 0 = off (identical to demo4). */
typedef struct {int32_t kd_q16;uint32_t shift;} jc_hpd;
#define JC_HPD_BYTES 12u
#define JC_HPD_MAGIC UINT32_C(0x31445048) /* "HPD1" */
#define JC_COMP2_BYTES 16u
#define JC_COMP2_MAGIC UINT32_C(0x32504d43) /* "CMP2" */
#define JC_COMP_MAGIC UINT32_C(0x31504d43) /* "CMP1" */
/* fast16 (owner 2026-11-23: "yes please add a real torque command"). tau_mode JC_TAU_COMP: the output-torque request is
 * tau_q24 (Nm Q24) instead of the PD law and goes through the gear-train compensation exactly like a PD output (the
 * horn should see tau_q24); JC_TAU_RAW: tau_q24 is the motor torque referred to the output, no compensation. Both are
 * limited only by the configured torque limit (config 320, as the PD output) and the duty saturation. */
enum { JC_TAU_PD = 0, JC_TAU_COMP = 1, JC_TAU_RAW = 2 };

typedef struct {
 pd_config config; est32_state est,est_fast; jc_comp comp; int32_t a_fast_q16; jc_comp2 comp2; jc_hpd hpd; int32_t v_lp_q16;
 int64_t comp_q24; int32_t prev_tau_sign; uint32_t boost_left; int32_t a_filt_q16;
 int64_t angle_q32;               /* last raw angle */
 int64_t requested_q24;
 int32_t tau_q24; uint8_t tau_mode;   /* fast16 torque mode (JC_TAU_*): app.c writes 264/268 */
 int32_t v_hat_q16,a_hat_q16,v_raw_q16;
 uint32_t token,epoch,gain_q24,clock,observations,rejections;
 int32_t nominal_q24;
 int16_t planned_pwm,actual_pwm;
 uint16_t flags;
 uint8_t history;
 bool configured,terminal,busy,synthetic,reanchor;
} joint_control;
enum {JC_QUANTIZED=1,JC_SATURATED=2,JC_BLANKED=4,JC_LIMITED=8,JC_DRIVER_REJECT=16,JC_OBSERVATION_REJECT=32,JC_RESYNC=64};
typedef struct {int16_t pwm;int32_t nominal_q24;uint32_t flags;} joint_plan;
#define JC_CONFIG_BYTES 48u
#define JC_CONFIG_MAGIC UINT32_C(0x31424450) /* "PDB1" */
#define JC_KP_MAX_Q16 (INT32_C(50)<<16)       /* 50 Nm/rad */
#define JC_KD_MAX_Q16 (INT32_C(2)<<16)        /* 2 Nm*s/rad */
#define JC_TORQUE_LIMIT_MAX_Q24 (UINT32_C(2)<<24) /* 2 Nm */
void joint_init(joint_control *);
bool joint_off(const joint_control *);
void joint_reseed(joint_control *,int64_t y_q16,int64_t y_old_q16,uint32_t gap);   /* fast17: sensor reacquire */
void joint_stop(joint_control *);
bool joint_plan_pwm(int64_t,uint32_t,uint16_t,int16_t,bool,joint_plan *);
/* OFF/terminal only. Layout (little endian, 48 B):
 * 0 token(u32,>previous) 4 kp_q16 8 kd_q16 12 gain_q24(Nm/V) 16 torque_limit_q24
 * 20 k0 24 k1 28 k2 (Q32) 32 deadzone_q20 36 resync_q20 40 lead(bit0) | blank_reversal(bit8) 44 magic */
bool joint_configure(joint_control *,const uint8_t *,uint16_t);
bool jc_angle_from_fractional_counts(int32_t whole,int32_t residue_q16,int64_t *q32);
bool joint_observe(joint_control *,int32_t counts,int32_t fraction_q16,bool valid);
bool joint_prepare(joint_control *,int64_t target_q32,int32_t velocity_q16,int32_t accel_q16,uint16_t mv,bool permit);
/* Live (allowed while enabled): gains 8 B {kp_q16>=0, kd_q16>=0}; compensation 32 B (+magic). */
bool joint_set_gains(joint_control *,const uint8_t *,uint16_t);
bool joint_set_comp(joint_control *,const uint8_t *,uint16_t);
bool joint_set_comp2(joint_control *,const uint8_t *,uint16_t);
bool joint_set_hpd(joint_control *,const uint8_t *,uint16_t);
bool joint_applied(joint_control *,int16_t,bool);
/* Bad sample: advance on the estimator's own prediction (no reject, no stop). */
/* v11: coast at constant velocity (acceleration dropped); freeze=true also zeroes velocity. */
bool joint_coast(joint_control *,bool freeze);
/* Open-loop duty (PWM mode): applies pwm in [-480,480] directly; same gating as PD. */
bool joint_prepare_pwm(joint_control *,int16_t pwm,uint16_t mv,bool permit);
#endif
