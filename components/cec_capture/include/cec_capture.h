/*
 * Burst capture engine - EPS variant.
 *
 * Maintains a pre-trigger ring buffer of low-rate samples (50 Hz) that
 * the main sample task pushes to on every iteration. On a trigger,
 * dumps the ring to TelePlot and then captures a high-speed (10 kHz)
 * window into a separate PSRAM buffer using the ESP-IDF adc_continuous
 * DMA driver, then dumps that too.
 *
 * Parity with the 24-pin module's cec_capture.h: same trigger enum,
 * same TelePlot >BURST_BEGIN / >BURST_END envelope, same busy +
 * cooldown semantics. Diverges where it matters:
 *
 *   - HS source is adc_continuous (DMA) rather than a 1 kHz polling
 *     callback. adc_oneshot tops out around 1-2 kHz of effective
 *     per-channel sampling once curve-fit cali is applied; continuous
 *     mode hardware-clocks the conversions and lets the EPS hit its
 *     10 kHz target.
 *
 *   - HS sample shape carries the two ACS758 cable currents only (the
 *     24-pin sample carries all six main-rail V+I channels).
 *
 * adc_continuous and adc_oneshot cannot coexist on the same ADC unit
 * per IDF, so the engine "borrows" ADC1 from cec_adc via cec_adc_pause
 * for the duration of an HS capture and returns it with cec_adc_resume.
 * cec_adc_read_mv calls during that window return
 * ESP_ERR_INVALID_STATE; the main sample task is expected to skip
 * ADC-touching work while cec_capture_is_busy() is true.
 *
 * TelePlot dump format:
 *
 *   >burst_begin:<ts_ms>:<reason_int>    (numeric, CSV-friendly)
 *   >BURST_BEGIN:<reason>:<n_pre>_normal+<n_hs>_hs:<state>
 *   >BURST_ANNOTATION:<text>            (optional, only if trigger_with_text)
 *   >b_eps1_a:<ts_ms>:<value>           (pre-trigger samples, 50 Hz)
 *   >b_eps2_a:<ts_ms>:<value>
 *   >b_eps1_raw_a:<ts_ms>:<value>
 *   >b_eps2_raw_a:<ts_ms>:<value>
 *   >b_bus_v:<ts_ms>:<value>
 *   >b_temp_c:<ts_ms>:<value>
 *   >b_load:<ts_ms>:<value>
 *   >b_flags:<ts_ms>:<value>
 *   >hs_eps1_a:<ts_us_offset>:<value>   (HS samples, 10 kHz)
 *   >hs_eps2_a:<ts_us_offset>:<value>
 *   >burst_end:<ts_ms>:0                 (numeric, CSV-friendly)
 *   >BURST_END
 *
 * The lowercase >burst_begin / >burst_end lines carry numeric values
 * (reason_int from cec_trigger_t for begin, 0 for end) so they survive
 * TelePlot's CSV export and produce step pulses that bracket each burst
 * in CSV analysis. The uppercase >BURST_BEGIN / >BURST_ANNOTATION /
 * >BURST_END lines are kept for raw-stream consumption (idf.py monitor
 * and the 24-pin's capture-analysis tooling that already parses that
 * format).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "cec_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pre-trigger sample - 50 Hz, full state snapshot. */
typedef struct {
    uint32_t ts_ms;
    float    eps1_a, eps2_a;
    float    eps1_raw_a, eps2_raw_a;
    float    bus_voltage_v;  /* 12V rail measurement (V) */
    float    temp_c;
    uint8_t  load_state;     /* cec_load_state_t cast to byte */
    uint8_t  flags;          /* CEC_FLAG_* bits */
} cec_capture_sample_t;

/* HS sample - 10 kHz, currents only. Slim by design so the PSRAM
 * footprint stays modest and the DMA consumer's per-sample work is
 * cheap. */
typedef struct {
    uint32_t ts_us_offset;   /* microseconds since HS capture start */
    float    eps1_a;
    float    eps2_a;
} cec_capture_hs_sample_t;

/* Per-channel conversion params used by the HS consumer to translate
 * a calibrated millivolt reading into amps. Mirrors the math in
 * acs758_read_current and is passed in by value at init time so
 * cec_capture doesn't depend on the acs758 driver's runtime context. */
typedef struct {
    adc_channel_t channel;
    float         divider_gain;      /* recover chip voltage from ADC pin (1.5 for ACS758) */
    float         quiescent_v;       /* chip output at 0 A */
    float         sensitivity_v_a;   /* V/A (ratiometric with Vcc) */
    float         zero_offset_v;     /* per-unit zero trim */
} cec_capture_channel_t;

/* Init-time configuration. Buffers are allocated in PSRAM. */
typedef struct {
    int    pre_trigger_capacity;    /* pre-trigger ring depth (samples) */
    int    hs_sample_rate_hz;       /* per-channel HS rate (e.g. 10000) */
    int    hs_duration_ms;          /* HS capture window length */
    int    cooldown_ms;              /* min gap between bursts; 0 = no cooldown */

    /* Emit every Nth HS sample to TelePlot (1 = no decimation, 10 =
     * every 10th row). Capture itself stays at full hs_sample_rate_hz
     * so Layer 2/3 still see the resolution; this only thins the dump
     * to keep USB Serial-JTAG out of the critical path. */
    int    hs_dump_decimation;

    int                    n_channels;
    cec_capture_channel_t  channels[CEC_NUM_CABLES];
} cec_capture_config_t;

/* Update the dump decimation factor at runtime. clamped to >= 1. */
esp_err_t cec_capture_set_hs_dump_decimation(int decim);
int       cec_capture_get_hs_dump_decimation(void);

/* Human-readable trigger name for log / dump output. */
const char *cec_trigger_name(cec_trigger_t t);

/*
 * Allocate the pre-trigger + HS buffers in PSRAM, start the burst
 * dispatcher task on Core 1, and remember the channel conversion
 * params. cec_adc_init() must have run first.
 */
esp_err_t cec_capture_init(const cec_capture_config_t *cfg);

/*
 * Push one 50 Hz sample into the pre-trigger ring. Called by the main
 * sample task on every iteration. Lock-free for the writer; the ring
 * is read only at dump time on the dispatcher task.
 */
void cec_capture_push(const cec_capture_sample_t *s);

/*
 * Request a burst capture.
 *   ESP_OK                  - capture queued
 *   ESP_ERR_NOT_FINISHED    - a capture is already running
 *   ESP_ERR_INVALID_STATE   - within cooldown window
 *   ESP_ERR_INVALID_STATE   - cec_capture_init was not called
 */
esp_err_t cec_capture_trigger(cec_trigger_t reason);

/*
 * Same as cec_capture_trigger but attaches a short annotation string
 * (e.g. "manual via cli") to the dump as `>BURST_ANNOTATION:<text>`.
 */
esp_err_t cec_capture_trigger_with_text(cec_trigger_t reason, const char *text);

/*
 * True from the moment a trigger is accepted until the dump finishes.
 * The sample task should check this and skip ADC-touching work while
 * it returns true (cec_adc is paused for the duration).
 */
bool cec_capture_is_busy(void);

/*
 * Update the cached conversion params for one channel. The capture
 * engine took a snapshot at cec_capture_init; calling this after a
 * runtime calibration change (cal zero / cal span / set supply) keeps
 * subsequent HS captures emitting accurate amps without a reboot.
 *
 * idx must be in [0, n_channels). The .channel field of `params` is
 * ignored - only divider_gain / quiescent_v / sensitivity_v_a /
 * zero_offset_v are copied through.
 */
esp_err_t cec_capture_update_channel(int idx, const cec_capture_channel_t *params);

#ifdef __cplusplus
}
#endif
