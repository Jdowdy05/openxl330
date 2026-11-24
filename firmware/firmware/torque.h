#ifndef PENDULUM_TORQUE_H
#define PENDULUM_TORQUE_H

#include <stdbool.h>
#include <stdint.h>

/* RAM-only, model-estimated output-referred torque. No measured-current loop. */
enum {
    TORQUE_PWM_FULL_TICKS = 480,
    TORQUE_MAX_GAIN = 100000,
    TORQUE_MAX_MODEL = 1000000,
    TORQUE_MAX_VELOCITY_MRAD_S = 1000000,
    TORQUE_MAX_DT_US = 100000,
    TORQUE_MAX_TAU_US = 1000000
};

typedef struct {
    int32_t kp_mNm_rad;             /* 0..100000 mNm/rad;0 disables P term */
    int32_t kd_mNm_rad_s;           /* 0..100000 mNm s/rad */
    int32_t a_mNm_per_V;            /* 1..1000000; drive torque/volt */
    int32_t b_mNm_per_rad_s;        /* 0..1000000; combined speed loss */
    int32_t c_positive_mNm;         /* 0..1000000; positive-motion loss */
    int32_t c_negative_mNm;         /* 0..1000000; negative-motion loss */
    int32_t gravity_mNm;            /* -1000000..1000000; optional feedforward */
    int32_t gravity_zero_counts;    /* any unwrapped count; q=0 down */
    uint32_t velocity_tau_us;       /* 0 bypass; otherwise <=1000000 us */
    uint16_t pwm_limit_ticks;       /* 0..480; 0 is an output-disabled cap */
} torque_config_t;

typedef struct {
    int32_t target_counts;          /* unwrapped target, never nearest-turn */
    int32_t target_velocity_mrad_s; /* +/-1000000 */
    int32_t feedforward_mNm;        /* +/-1000000 */
} torque_command_t;

typedef struct {
    int32_t position_counts;
    int32_t velocity_mrad_s;        /* filtered output velocity, +/-1000000 */
    uint16_t voltage_mV;            /* 1000..20000; app owns physical limits */
    uint32_t dt_us;                 /* 1..100000; no integral/slew */
} torque_input_t;

typedef struct {
    int16_t high_ticks;             /* signed duty; bridge owns reversal blank */
    int32_t requested_mNm;          /* PD + explicit FF + gravity, pre-duty cap */
    uint8_t saturated;              /* duty cap or requested telemetry clamp */
} torque_output_t;

typedef struct {
    uint32_t last_us;
    int32_t last_position_counts;
    int32_t velocity_mrad_s;
    int32_t last_fraction_q16;      /* signed Q16 count residue; reuses padding */
    int64_t velocity_q16;
    uint8_t initialized;
} torque_velocity_t;

/* Conservative software defaults are uncalibrated, not hardware qualification. */
torque_config_t torque_config_default(void);
bool torque_config_valid(const torque_config_t *config);
/* Invalid arguments return false and clear output, when output is non-null. */
bool torque_step(const torque_config_t *config, const torque_command_t *command,
                 const torque_input_t *input, torque_output_t *output);
bool torque_step_with_loss(const torque_config_t *config, const torque_command_t *command,
                           const torque_input_t *input, int32_t loss_mNm, torque_output_t *output);
void torque_velocity_reset(torque_velocity_t *state);
/* First sample initializes to zero without a derivative kick. Unsigned time
 * subtraction handles uint32 wrap. Invalid dt/raw speed clears velocity and
 * reanchors position/time, then returns false; caller must disable output. */
bool torque_velocity_update(torque_velocity_t *state, int32_t position_counts,
                            uint32_t now_us, uint32_t tau_us);
/* Raw derivative of unwrapped integer+signed Q16 residue, no IIR/window.
 * Same first-sample/actual-dt/raw-speed/reanchor contract as the old helper.
 * Rejects invalid residue or >=one-turn displacement before multiplication.
 * Reset when changing coordinate/configuration or switching helper APIs. */
/* Pure same two-stage raw fractional difference. Rejection preserves output. */
bool torque_fractional_speed(int32_t,int32_t,int32_t,int32_t,uint32_t,int32_t *);
bool torque_velocity_update_fractional(torque_velocity_t *,int32_t,int32_t,uint32_t);
/* Periodic sine, 4096 counts/turn, Q15 in [-32767,32767]. */
int32_t torque_sin_q15(int32_t phase_counts);

#endif
