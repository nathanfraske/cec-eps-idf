#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CEC_NUM_CABLES 2

// Module identity
#define CEC_MODULE_TYPE_EPS 0x02

// Status flag bits
#define CEC_FLAG_OVERCURRENT  (1 << 0)   /* Layer 1 CRITICAL on current ceiling */
#define CEC_FLAG_SWING        (1 << 1)   /* Layer 2 fast-transient (dI/dt) */
#define CEC_FLAG_FAULT        (1 << 2)   /* generic fault placeholder */
#define CEC_FLAG_DROPOUT      (1 << 3)   /* Layer 1 dropout floor */
#define CEC_FLAG_ANOMALY      (1 << 4)   /* Layer 3 z-score exceeded */

// Per-cable load classifier output. The 24-pin module's cec_state_t names
// the whole-PSU operating mode (OFF/STANDBY/...); EPS is per-cable, so
// the enum is deliberately named differently to avoid collision when
// future code shares headers.
typedef enum {
    CEC_LOAD_IDLE = 0,
    CEC_LOAD_LIGHT,
    CEC_LOAD_MODERATE,
    CEC_LOAD_HEAVY,
    CEC_LOAD_TRANSIENT,
    CEC_LOAD_COUNT,
} cec_load_state_t;

// Anomaly severity grade used by the detection layers. Shared with the
// 24-pin module; see cec_layer1.h for the warn/crit band semantics.
typedef enum {
    CEC_SEV_NONE = 0,
    CEC_SEV_WARNING,
    CEC_SEV_CRITICAL,
} cec_severity_t;

// Burst-capture trigger reasons. Shared vocabulary across the
// detection layers (which produce them) and cec_capture (which
// consumes them in BURST_BEGIN headers). Parity with the 24-pin's
// enum; some entries (e.g. STATE_CHANGE, POWER_SWING) are reserved
// for use as the corresponding detection paths come online.
typedef enum {
    CEC_TRIG_NONE = 0,
    CEC_TRIG_MANUAL,
    CEC_TRIG_STATIC_WARN,
    CEC_TRIG_STATIC_CRIT,
    CEC_TRIG_TRANSIENT,
    CEC_TRIG_ANOMALY,
    CEC_TRIG_STATE_CHANGE,
    CEC_TRIG_SHUTDOWN,
    CEC_TRIG_POWER_SWING,
    CEC_TRIG_CURRENT_SWING,
    CEC_TRIG_COUNT,
} cec_trigger_t;

// Shared measurement state. sample_task is the only writer.
// Readers (output, comms) take the mutex briefly to snapshot.
typedef struct {
    float current_a[CEC_NUM_CABLES];      // filtered current per cable (amps)
    float current_raw_a[CEC_NUM_CABLES];  // unfiltered current (amps)
    float bus_voltage_v;                  // 12V rail measured via divider on GPIO 1
    float board_temp_c;                   // NTC board temperature
    cec_load_state_t load_state;          // per-cable load classifier output
    uint8_t status_flags;                 // CEC_FLAG_* bits
    int64_t timestamp_us;                 // esp_timer time of last update
    SemaphoreHandle_t mutex;
} cec_state_t;

// Runtime configuration (persisted in NVS)
typedef struct {
    uint8_t module_id;          // instance ID for multi-module setups
    float supply_voltage;       // measured ACS758 Vcc (ratiometric reference)
    float oc_threshold_a;       // overcurrent threshold per cable
    float ema_alpha;            // filter responsiveness
    bool output_raw;            // telemetry mode: raw vs filtered
    bool layer1_enabled;        // detection layer 1 (current threshold)
    bool layer2_enabled;        // detection layer 2 (dI/dt swing)
    bool layer3_enabled;        // detection layer 3 (rail profile / z-score)
} cec_config_t;

// Defaults (used when NVS is empty)
#define CEC_DEFAULT_SUPPLY_V    4.4f    // measured: USB Vbus through dev board diode
#define CEC_DEFAULT_OC_A        35.0f   // above normal EPS load, below sensor limit
#define CEC_DEFAULT_EMA_ALPHA   0.2f
#define CEC_DEFAULT_MODULE_ID   1

// Hardware presence flag. Set to 1 once the daughterboard with the
// CAN transceiver is attached; the migrated esp_twai node-handle code
// in cec_can.c becomes active.
#define CEC_CAN_ENABLED         1
