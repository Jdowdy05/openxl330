#ifndef XL330_SENSING_H
#define XL330_SENSING_H
#include <stdbool.h>
#include <stdint.h>
#ifndef SENSING_FAST_I2C
#define SENSING_FAST_I2C 0
#endif
#if SENSING_FAST_I2C != 0 && SENSING_FAST_I2C != 1
#error SENSING_FAST_I2C must be0 or1
#endif

typedef enum {
    SENSING_FAULT_NONE = 0,
    SENSING_FAULT_NOT_INITIALIZED = 1u << 0,
    SENSING_FAULT_ARGUMENT = 1u << 1,
    SENSING_FAULT_I2C_TIMEOUT = 1u << 2,
    SENSING_FAULT_I2C_NACK = 1u << 3,
    SENSING_FAULT_I2C_BUS = 1u << 4,
    SENSING_FAULT_I2C_STUCK = 1u << 5,
    SENSING_FAULT_ENCODER_INVALID = 1u << 6,
    SENSING_FAULT_ADC_TIMEOUT = 1u << 7,
    SENSING_FAULT_ADC_REFERENCE = 1u << 8,
    SENSING_FAULT_ADC_INIT = 1u << 9,
    SENSING_FAULT_ADC_OVERRUN = 1u << 10,
    SENSING_FAULT_ENCODER_CONFIG = 1u << 11,
    SENSING_FAULT_ENCODER_RESTORE = 1u << 12
} sensing_fault;

typedef struct {
    uint16_t raw_current;
    uint16_t raw_voltage;
    uint16_t raw_temperature;
    uint16_t raw_reference;
    uint16_t current_magnitude_ma; /* Unsigned magnitude; polarity is not measured. */
    uint16_t voltage_decivolts;
    uint8_t temperature_c;
    uint32_t timestamp_us; /* Newest conversion; tick API retains three older channels. */
    uint32_t faults;
} sensing_adc;

/* Caller supplies HCLK=PCLK=PLL64 and enabled HSI16, a running hw_micros(), and bridge OFF.
 * Sampling functions never drive the bridge or write flash/sensor configuration.
 * Default profile0 keeps the archived HSI16/filter-on ~400kHz configuration.
 * Experimental profile1 selects PCLK64/Fm+ drive/analog-off/DNF1, with
 * declared tr/tf<=120ns bounds; functional bench qualification is still needed.
 * Encoder budget:200us(default) or120us(fast), each plus10us cleanup.
 * ADC remains PCLK64/4 with150us full-acquisition budget plus10us cleanup.
 * MMIO/interrupt overhead must be included in measured complete-tick qualification.
 * Sampling failure clears caller output, except faults in sensing_adc.
 * Initialization must run before sampling. API is single-caller/non-reentrant. */
bool sensing_init(void);
bool sensing_read_encoder(uint16_t *raw);
uint16_t sensing_last_encoder_word(void);
/* raw is the unmasked 16-bit word (bits 15..12 must be 0); status = AS5600 0x0B. */
bool sensing_read_encoder_status(uint16_t *raw, uint8_t *status);
bool sensing_read_encoder_status_byte(uint8_t *status);
enum { AS5600_MH = 1u << 3, AS5600_ML = 1u << 4, AS5600_MD = 1u << 5 };
/* Startup-only, bridge OFF. Read/RMW/readback of volatile AS5600 CONF:
 * SF11,NOM,FTH000,WD0; preserve all other fields, never write OTP.
 * before/after are zero until complete corresponding reads; diagnostics may
 * retain before on later failure. Three transactions on success; up to five
 * with rollback. Each transaction has its own bounded deadline.
 * Caller must fault output OFF if this optional operation returns false.
 * Failed write/verification attempts rollback, retaining original error flags.
 * Saved original is kept from the first write until verified restoration. */
bool sensing_encoder_config(uint16_t *before, uint16_t *after);
/* Bridge OFF, before orderly F9/probe-expiry/main-stall reset. Restore and
 * verify the captured original; no-op success if no original was captured.
 * Failure retains the original for retry and reports ENCODER_RESTORE.
 * Two bounded I2C transactions, startup/shutdown only, not a control-tick API.
 * MCU reset loses this saved RAM state without resetting the powered AS5600;
 * arbitrary fault/reset can therefore leave its volatile CONF changed.
 * sensing_init is startup-only and resets the saved-original state. */
bool sensing_encoder_restore(void);
/* Full four-channel acquisition,150us total budget plus10us cleanup. Success
 * seeds/reseeds the tick cache and restarts its channel order at13. */
bool sensing_read_adc(sensing_adc *out);
/* Requires a successful full read after initialization or ANY ADC failure.
 * Converts exactly one channel per call, order13(reference),0(temp),1(voltage),
 * 9(unsigned diagnostic current), with60us total budget plus10us cleanup.
 * Recomputes derived values from the newest four raw values. On failure all
 * output/cache values clear; another tick cannot silently reuse the old seed.
 * With successful calls each native tick, channel age is at most four tick
 * periods (2ms at2kHz). Caller must enforce regular tick/deadline progress;
 * timestamp_us is the newest conversion, not simultaneous channel freshness. */
bool sensing_read_adc_tick(sensing_adc *out);
uint32_t sensing_last_faults(void);
#endif
