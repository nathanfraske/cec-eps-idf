# CEC EPS Module — Firmware Specification

Firmware reference for the dual-EPS current monitoring module in the Critical Error Computing (CEC) PC power monitoring platform. This document contains everything needed to start firmware development for the EPS module prototype.

> **Status:** Hardware prototype in bring-up. Firmware not yet started. This spec is the starting point.

---

## Table of Contents

1. [Overview](#overview)
2. [Hardware Reference](#hardware-reference)
3. [Sensor Theory: ACS758LCB-050B](#sensor-theory-acs758lcb-050b)
4. [Signal Chain](#signal-chain)
5. [Measurement Math](#measurement-math)
6. [Calibration](#calibration)
7. [Firmware Architecture](#firmware-architecture)
8. [ADC Driver](#adc-driver)
9. [Signal Filtering](#signal-filtering)
10. [Detection Layers](#detection-layers)
11. [Output Format](#output-format)
12. [CAN Protocol](#can-protocol)
13. [NVS Configuration](#nvs-configuration)
14. [Serial Command Interface](#serial-command-interface)
15. [Build and Flash](#build-and-flash)
16. [Bring-Up Checklist](#bring-up-checklist)
17. [Development Roadmap](#development-roadmap)
18. [Codebase Parity with the 24-pin Module](#codebase-parity-with-the-24-pin-module)
19. [Appendix: Differences from 24-pin Module](#appendix-differences-from-24-pin-module)

---

## Overview

The EPS module monitors current on the EPS (CPU power) cables feeding a motherboard. Each module supports up to two EPS cables, with one Hall-effect current sensor per cable. It reports per-cable current telemetry to the central Hub over CAN, and participates in the platform's anomaly detection system.

### Prototype vs. production

| Aspect | Prototype (this build) | Production |
|---|---|---|
| Current sensor | ACS758LCB-050B (Hall) | INA226 + 1 mΩ shunt |
| Sensor interface | Analog ADC | I2C |
| Supply | USB-C (USB Vbus 5V) | 5VSB from Hub via RJ-45 |
| Hub link | CAN (loopback-testable) | CAN over RJ-45 |
| Noise floor | ~10-30 mA (filtered) | ~0.5-2 mA |

The prototype validates the sensing architecture, detection algorithms, and protocol behavior. The production sensing topology (INA226 + shunt) will be a different driver but the same detection and protocol layers carry over.

### Firmware goals for the prototype

1. Read both ACS758 current channels via ADC
2. Convert raw ADC to calibrated current in amps
3. Filter the inherently noisy Hall signal
4. Classify operating state and detect anomalies (transients, sags, surges)
5. Stream telemetry over serial (Teleplot) for development
6. Report to Hub over CAN (when Hub interface is connected)
7. Persist calibration and config in NVS

---

## Hardware Reference

### MCU

| Property | Value |
|---|---|
| Module | ESP32-S3 N16R8 (Lonely Binary dev board) |
| Flash | 16 MB |
| PSRAM | 8 MB |
| ADC | 12-bit SAR, ADC1 used for analog sensing |
| Console | USB native (USB-Serial-JTAG), UART0 pins left free |

### Current Sensors

| Property | Value |
|---|---|
| Part | ACS758LCB-050B |
| Type | Hall-effect, bidirectional |
| Range | ±50 A |
| Sensitivity | 40 mV/A |
| Supply | 5 V |
| Quiescent output | Vcc/2 = 2.5 V (at 0 A) |
| Output range | 0.5 V (at -50 A) to 4.5 V (at +50 A) |
| Bandwidth | ~120 kHz |
| Internal resistance (current path) | ~100 µΩ |

### Pin Map

ESP32-S3 GPIO assignments for the EPS module:

| GPIO | ADC channel | Function | Refboard location |
|---|---|---|---|
| GPIO 6 | ADC1_CH5 | ACS758 #1 current (EPS cable 1) | Col I row 17 |
| GPIO 10 | ADC1_CH9 | ACS758 #2 current (EPS cable 2) | Col I row 6 |
| GPIO 1 | ADC1_CH0 | 12V rail tap (47k/10k divider, V_rail = V_pin × 5.7) | — |
| GPIO 7 | ADC1_CH6 | NTC thermistor (board temp) | Col I row 16 (via daughterboard) |
| GPIO 4 | — | CAN TX (TWAI) | Col I row 19 (via daughterboard) |
| GPIO 15 | — | CAN RX (TWAI) | via daughterboard |
| GPIO 8 | — | I2C SDA (reserved, unused on EPS) | Col I row 11 |
| GPIO 9 | — | I2C SCL (reserved, unused on EPS) | Col I row 8 |

Reserved/avoid:
- GPIO 3 (ADC1_CH2): strapping pin (JTAG enable), avoid for clean analog use
- GPIO 43/44 (UART0): left free for optional serial console

### Power Architecture

| Rail | Source (prototype) | Used by |
|---|---|---|
| 5 V | USB Vbus (col I row 2) | ACS758 VCC pins |
| 3.3 V | ESP32 onboard LDO (col I row 21) | MCU, CAN transceiver, NTC divider |
| GND | Common | All |

The 5 V supply routes from the USB Vbus pin down col J to row 23, with a cross-gap jumper bringing it to both ACS758 VCC pins. In production this becomes 5VSB delivered from the Hub.

---

## Sensor Theory: ACS758LCB-050B

The ACS758 measures current by sensing the magnetic field produced by current flowing through an internal conductor. The output is a voltage proportional to that current, centered at half the supply voltage.

### Transfer function

```
VIOUT = (Vcc / 2) + (Sensitivity × I)
      = 2.5 V + (0.040 V/A × I)
```

Where `I` is the current in amps (positive in the Ip+ to Ip- direction).

Examples at 5 V supply:

| Current | VIOUT |
|---|---|
| -50 A | 0.5 V |
| -10 A | 2.1 V |
| 0 A | 2.5 V |
| +10 A | 2.9 V |
| +30 A | 3.7 V |
| +50 A | 4.5 V |

### EPS current range in practice

EPS cables feed the CPU. Typical current per cable:

| Load condition | Current per cable |
|---|---|
| Idle | 2-5 A |
| Gaming/moderate | 8-15 A |
| Full load (stress test) | 20-30 A |
| Extreme OC (rare) | up to ~40 A |

The ±50 A range of the ACS758 gives comfortable headroom. The sensor never approaches its limit in normal EPS use.

### Polarity

Current flowing from Ip+ to Ip- produces a positive deflection (above 2.5 V). Wire the PSU side to Ip+ and the motherboard side to Ip- so that normal operation produces positive current readings. If reversed, the firmware will read negative currents (fixable by swapping wires or flipping the sign in software).

### FAULT pin (4-pin modules)

If your module exposes a FAULT pin (OUT2), it's an open-drain output that pulls low when current exceeds the chip's internal threshold. Optional to wire. If used, connect to a GPIO with a 10 kΩ pull-up to 3.3 V and configure an interrupt on the falling edge.

---

## Signal Chain

```
EPS cable current
    │
    ▼
ACS758 (Hall sensor)  ──→  VIOUT: 0.5-4.5 V, centered 2.5 V
    │
    ▼
Voltage divider (2:3)  ──→  scaled: 0.33-3.0 V, centered 1.67 V
    │
    ▼
ESP32 ADC1 (12-bit, 11dB atten)  ──→  raw count 0-4095
    │
    ▼
Firmware conversion + calibration  ──→  current in amps
    │
    ▼
Filter (median + EMA)  ──→  clean current value
    │
    ▼
Detection + telemetry
```

### Voltage Divider

The ACS758 output swings 0.5-4.5 V, which exceeds the ESP32 ADC's usable range (~0-3.1 V at 11 dB attenuation). A 2:3 divider scales it down.

| Component | Value |
|---|---|
| R1 (top, from VIOUT) | 10 kΩ, 1% |
| R2 (bottom, to GND) | 20 kΩ, 1% |

```
V_adc = VIOUT × R2 / (R1 + R2)
      = VIOUT × 20K / 30K
      = VIOUT × (2/3)
```

| VIOUT | V_adc |
|---|---|
| 0.5 V | 0.33 V |
| 2.5 V (quiescent) | 1.67 V |
| 4.5 V | 3.0 V |

All within the ADC's clean range.

### ADC Configuration

| Setting | Value | Reason |
|---|---|---|
| Unit | ADC1 | ADC2 conflicts with WiFi (unused here, but ADC1 is the safe choice) |
| Bit width | 12-bit (default) | Full resolution |
| Attenuation | ADC_ATTEN_DB_12 | ~0-3.1 V input range, covers the divider output |
| Channels | CH5 (GPIO 6), CH9 (GPIO 10), CH6 (GPIO 7 NTC) | |

---

## Measurement Math

### Raw to Current Conversion

```c
// Calibration constants
#define ADC_VOLTS_PER_LSB   (3.1f / 4095.0f)   // 12-bit, 11dB atten
#define DIVIDER_GAIN        (3.0f / 2.0f)        // inverse of 2:3 divider = 1.5
#define ACS758_SENSITIVITY  0.040f               // V/A for LCB-050B
#define ACS758_QUIESCENT    2.5f                 // V at 0 A (nominal)

// Per-sensor zero offset, measured during calibration (typically ±30 mV)
float zero_offset[2] = {0.0f, 0.0f};

float adc_to_current(int raw, int sensor_idx) {
    float v_adc   = raw * ADC_VOLTS_PER_LSB;        // voltage at ADC pin
    float v_acs   = v_adc * DIVIDER_GAIN;            // recover ACS758 output
    float v_signal = v_acs - ACS758_QUIESCENT - zero_offset[sensor_idx];
    return v_signal / ACS758_SENSITIVITY;            // current in amps
}
```

### Resolution and noise

| Metric | Value | Notes |
|---|---|---|
| ADC LSB at sensor | ~1.13 mV per LSB at ACS758 output (after divider gain) | 3.1/4095 × 1.5 |
| Current per LSB | ~28 mA | 1.13 mV ÷ 40 mV/A |
| Raw noise floor | ~50-100 mA RMS | Hall sensor inherent noise |
| Filtered noise floor | ~10-30 mA RMS | After 5-sample median + EMA |

The ~28 mA quantization step and ~10-30 mA filtered noise are fine for EPS currents of 5-30 A (relative accuracy ~0.1-0.3% at typical load).

### Power calculation note

The ACS758 measures current only. The EPS rail is 12 V nominal. The firmware now reads the actual rail voltage via a 47k/10k divider on GPIO 1 (ADC1_CH0, `V_rail = V_pin × 5.7`); the result lands in `cec_state_t::bus_voltage_v` and is emitted to TelePlot as `bus_voltage`. Power is `bus_voltage × (eps1_current + eps2_current)`.

The 24-pin module separately measures and reports its own 12 V rail voltage over CAN; once both modules are on the bus together, cross-checking the EPS's local tap against the 24-pin's reading is a useful sanity check (and detects voltage drop along the cable, which is a sign of connector resistance or cable degradation).

A single divider on the shared cable bundle is the current setup — both EPS cables come off the same PSU rail. Per-cable taps (two dividers) would resolve drop per cable but are deferred until a use case actually needs it.

The CAN telemetry payload doesn't yet carry the bus voltage byte (the 8-byte frame is full); extending the protocol is a Hub-side coordination on the running 24-pin TODO list.

---

## Calibration

Hall sensors have a quiescent offset that varies between units (typically ±30 mV, equivalent to ±0.75 A error if uncorrected). Calibrate the zero offset per sensor at startup or on command.

### Zero-offset calibration procedure

1. Ensure no current flows through the sensors (PSU off, or PSU on with motherboard off so the EPS cables carry no load).
2. Read each ADC channel, average many samples (e.g., 256) to reduce noise.
3. Compute the offset from the expected quiescent.
4. Store in NVS.

```c
void calibrate_zero_offset(adc_oneshot_unit_handle_t adc, int sensor_idx,
                           adc_channel_t channel) {
    const int N = 256;
    int64_t sum = 0;
    for (int i = 0; i < N; i++) {
        int raw;
        adc_oneshot_read(adc, channel, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    float avg_raw   = (float)sum / N;
    float v_adc     = avg_raw * ADC_VOLTS_PER_LSB;
    float v_acs     = v_adc * DIVIDER_GAIN;
    zero_offset[sensor_idx] = v_acs - ACS758_QUIESCENT;   // store this
    // persist zero_offset[sensor_idx] to NVS
}
```

### Span calibration (optional, higher accuracy)

If you have a known reference current source (a bench supply with a calibrated load, or a clamp meter for comparison), you can refine the sensitivity constant per sensor:

```
measured_sensitivity = (V_acs_at_known_I - V_quiescent) / known_I
```

Store per-sensor sensitivity and use it in place of the nominal 0.040 V/A. For the prototype, the nominal value is good enough; span calibration is a production refinement.

---

## Firmware Architecture

The EPS firmware reuses the component-based ESP-IDF structure from the 24-pin module, swapping the sensor driver.

### Component Structure

```
cec-eps-idf/
├── CMakeLists.txt
├── sdkconfig
├── main/
│   ├── CMakeLists.txt
│   └── eps_main.c
└── components/
    ├── cec_sensors/        # ACS758 ADC driver (replaces INA226 for EPS)
    │   ├── acs758.c
    │   ├── acs758.h
    │   ├── ntc.c
    │   └── ntc.h
    ├── cec_detection/      # shared with 24-pin: state classifier, anomaly layers
    │   ├── detection.c
    │   └── detection.h
    ├── cec_filter/         # shared: median + EMA filters
    │   ├── filter.c
    │   └── filter.h
    ├── cec_capture/        # shared: PSRAM ring buffer for burst capture
    │   ├── capture.c
    │   └── capture.h
    ├── cec_comms/          # shared: CAN/TWAI driver + frame definitions
    │   ├── can.c
    │   └── can.h
    └── cec_output/         # shared: Teleplot serial output
        ├── teleplot.c
        └── teleplot.h
```

The `cec_detection`, `cec_filter`, `cec_capture`, `cec_comms`, and `cec_output` components are shared with the 24-pin module. Only `cec_sensors` differs (ACS758 ADC driver vs INA226 I2C driver).

### FreeRTOS Tasks

| Task | Priority | Rate | Core | Purpose |
|---|---|---|---|---|
| sample_task | 5 | 50 Hz (steady) | 0 | Read both ADC channels, convert, filter, push to detection |
| burst_task | 7 | event-driven | 0 | High-rate capture (1 kHz+) into ring buffer on trigger |
| output_task | 3 | 10 Hz | 1 | Teleplot serial telemetry |
| comms_task | 4 | 20 Hz | 1 | CAN frame TX/RX to Hub |
| command_task | 2 | on-demand | 1 | Serial command parsing |

Sample task pinned to core 0 (isolated from WiFi/comms jitter). Output and comms on core 1.

### Data Flow

```
sample_task (50 Hz)
   reads ADC → converts → filters → updates shared state
        │
        ├──→ detection (runs inside or after sample)
        │       └──→ on anomaly: trigger burst_task
        │
        ├──→ output_task reads shared state → Teleplot (10 Hz)
        │
        └──→ comms_task reads shared state → CAN frames (20 Hz)
```

Shared state protected by a mutex or use a lock-free single-writer/multi-reader pattern (sample_task is the only writer).

---

## ADC Driver

### Initialization

```c
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

typedef struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    bool cali_enabled;
} acs758_ctx_t;

#define ACS758_1_CHANNEL  ADC_CHANNEL_5   // GPIO 6
#define ACS758_2_CHANNEL  ADC_CHANNEL_9   // GPIO 10
#define NTC_CHANNEL       ADC_CHANNEL_6   // GPIO 7

esp_err_t acs758_init(acs758_ctx_t *ctx) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &ctx->adc_handle),
                        "acs758", "unit init failed");

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(ctx->adc_handle, ACS758_1_CHANNEL, &ch_cfg);
    adc_oneshot_config_channel(ctx->adc_handle, ACS758_2_CHANNEL, &ch_cfg);
    adc_oneshot_config_channel(ctx->adc_handle, NTC_CHANNEL, &ch_cfg);

    // Optional: ADC calibration scheme for better voltage accuracy
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &ctx->cali_handle) == ESP_OK) {
        ctx->cali_enabled = true;
    }
    return ESP_OK;
}
```

### Reading

```c
// Read raw and calibrated millivolts
int acs758_read_raw(acs758_ctx_t *ctx, adc_channel_t channel) {
    int raw = 0;
    adc_oneshot_read(ctx->adc_handle, channel, &raw);
    return raw;
}

int acs758_read_mv(acs758_ctx_t *ctx, adc_channel_t channel) {
    int raw = acs758_read_raw(ctx, channel);
    int mv = 0;
    if (ctx->cali_enabled) {
        adc_cali_raw_to_voltage(ctx->cali_handle, raw, &mv);
    } else {
        mv = (int)(raw * ADC_VOLTS_PER_LSB * 1000.0f);
    }
    return mv;  // millivolts at the ADC pin
}
```

Using the ESP-IDF ADC calibration scheme (curve fitting) gives better absolute voltage accuracy than the simple `raw × LSB` math, because it accounts for the ADC's nonlinearity and the chip-specific reference voltage stored in eFuse. Prefer it when available.

### Oversampling for noise reduction

Hall sensors are noisy. Oversample in the read path to knock down the noise before it reaches the filter:

```c
// Average M samples per logical reading
int acs758_read_raw_avg(acs758_ctx_t *ctx, adc_channel_t channel, int M) {
    int64_t sum = 0;
    for (int i = 0; i < M; i++) {
        int raw;
        adc_oneshot_read(ctx->adc_handle, channel, &raw);
        sum += raw;
    }
    return (int)(sum / M);
}
```

M = 8 to 16 is a good starting point. At 50 Hz sample rate with M = 16, you read 800 ADC conversions/sec/channel, well within the SAR ADC's capability.

---

## Signal Filtering

Two-stage filter matching the v0.5.9 baseline: a median filter to reject impulse noise, followed by an EMA to smooth.

### Median filter (5-sample)

Rejects single-sample spikes (common in Hall sensors near switching supplies).

```c
#define MEDIAN_WINDOW 5

typedef struct {
    float buffer[MEDIAN_WINDOW];
    int idx;
    int count;
} median_filter_t;

float median_update(median_filter_t *f, float sample) {
    f->buffer[f->idx] = sample;
    f->idx = (f->idx + 1) % MEDIAN_WINDOW;
    if (f->count < MEDIAN_WINDOW) f->count++;

    // Copy and sort
    float tmp[MEDIAN_WINDOW];
    for (int i = 0; i < f->count; i++) tmp[i] = f->buffer[i];
    for (int i = 0; i < f->count - 1; i++) {
        for (int j = i + 1; j < f->count; j++) {
            if (tmp[j] < tmp[i]) {
                float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t;
            }
        }
    }
    return tmp[f->count / 2];
}
```

### EMA (exponential moving average)

Smooths the median output. Alpha controls responsiveness vs. smoothness.

```c
typedef struct {
    float value;
    float alpha;     // 0.1-0.3 typical; higher = more responsive
    bool initialized;
} ema_filter_t;

float ema_update(ema_filter_t *f, float sample) {
    if (!f->initialized) {
        f->value = sample;
        f->initialized = true;
    } else {
        f->value = f->alpha * sample + (1.0f - f->alpha) * f->value;
    }
    return f->value;
}
```

### Filter tuning for EPS

| Parameter | Starting value | Effect |
|---|---|---|
| Oversample M | 8-16 | Reduces per-read noise |
| Median window | 5 | Rejects impulse spikes |
| EMA alpha | 0.2 | Balances smoothness and response |

The filtered output should settle to ~10-30 mA RMS noise. Tune alpha down (more smoothing) if you need cleaner steady-state readings, or up (more responsive) if you're missing fast transients. Keep the raw/unfiltered path available for the burst capture so transient detail isn't lost to filtering.

---

## Detection Layers

The detection logic is shared with the 24-pin module. It operates on the filtered current stream (and the raw stream for transient detection).

### Layer 1: Threshold detection

Static and dynamic thresholds on absolute current.

- **Overcurrent:** current exceeds a configured ceiling (e.g., 35 A per EPS cable, well above normal but below the sensor limit)
- **Undercurrent/dropout:** current falls below an expected floor while the system should be active (possible connector failure)

### Layer 2: Rate-of-change (swing) detection

Detects fast transients that threshold detection misses.

- Compute dI/dt over a short window
- Flag swings exceeding a threshold (e.g., >10 A in <10 ms)
- Useful for catching load steps, brownout precursors, and connector arcing signatures

```c
typedef struct {
    float last_value;
    int64_t last_time_us;
    float threshold_a_per_ms;
} swing_detector_t;

bool swing_check(swing_detector_t *d, float current, int64_t now_us) {
    if (d->last_time_us == 0) {
        d->last_value = current;
        d->last_time_us = now_us;
        return false;
    }
    float dt_ms = (now_us - d->last_time_us) / 1000.0f;
    float di = current - d->last_value;
    float rate = (dt_ms > 0) ? (di / dt_ms) : 0.0f;
    d->last_value = current;
    d->last_time_us = now_us;
    return (rate > d->threshold_a_per_ms) || (rate < -d->threshold_a_per_ms);
}
```

### Layer 3: Statistical/behavioral detection

Longer-horizon anomaly detection.

- Per-cable `cec_rail_profile_t` tracking running mean + std-deviation of the filtered current. Two adaptation rates: fast (0.01) for the first 100 samples after init, then a slow steady-state rate (0.0005 → ~40 s effective window at 50 Hz) once the profile is warm.
- A profile is "warm" after `CEC_PROFILE_WARM_SAMPLES` (1000 = 20 s of dwell time at 50 Hz). Z-score returns 0 before that so anomaly detection doesn't fire during the warm-up.
- Per-cable z-score check on every sample: `|z| > 4.0` → `CEC_FLAG_ANOMALY` → burst capture with `CEC_TRIG_ANOMALY`.
- The classifier (idle / light / moderate / heavy / transient) reads the same profiles' `std_dev` as its noise/variability gauge.
- Profiles are snapshotted to NVS every 5 minutes (`l3_profiles` key, magic `0xCEC50201`) and reloaded on boot, so the warm-up window isn't paid every reboot.

The state classifier uses current magnitude and the running `std_dev` to bucket the current operating regime.

### Auto-trigger sources

`sample_task` drives `cec_capture_trigger` directly from the detection outputs. The trigger reason is picked from the actual flags by `cec_trigger_for_flags`, so the `>BURST_BEGIN` envelope carries the cause rather than always reporting `anomaly`:

| Source | Flag / detector | Trigger reason | Notes |
|---|---|---|---|
| Layer 1 critical (overcurrent or dropout) | `CEC_FLAG_OVERCURRENT` / `CEC_FLAG_DROPOUT` | `CEC_TRIG_STATIC_CRIT` | Debounced (3 consecutive frames) |
| Layer 2 fast transient | `CEC_FLAG_SWING` | `CEC_TRIG_CURRENT_SWING` | `\|dI/dt\| > 1 A/ms` on the raw stream |
| Layer 3 z-score anomaly | `CEC_FLAG_ANOMALY` | `CEC_TRIG_ANOMALY` | `\|z\| > 4` once the rail profile is warm |
| Load-state classifier edge | (transition between cec_load_state_t buckets) | `CEC_TRIG_STATE_CHANGE` | Cooldown gate throttles rapid load chatter |
| Bus-voltage rate-of-change | 1 s window slope < −0.5 V/s | `CEC_TRIG_SHUTDOWN` | **Bypasses cooldown.** Followed by a 30 s mute window that suppresses other auto-triggers so cascading-rail noise doesn't generate spurious bursts. |
| CLI `burst <text>` | manual | `CEC_TRIG_MANUAL` | For bench testing or operator-initiated capture |

Individual layers can be silenced at runtime with `set layer1 off` (etc.) without rebuilding; the disabled layer still updates its internal state (debounce counter, EMA, rail profile) so re-enabling doesn't see stale values.

---

## Output Format

### Teleplot (development)

Teleplot is a simple serial telemetry format for live plotting during development. One line per metric.

```c
void teleplot_emit(const char *name, float value, int64_t time_ms) {
    printf(">%s:%lld:%.3f\n", name, time_ms, value);
}

// In output_task:
teleplot_emit("eps1_current", filtered_current[0], now_ms);
teleplot_emit("eps2_current", filtered_current[1], now_ms);
teleplot_emit("eps1_raw", raw_current[0], now_ms);
teleplot_emit("eps2_raw", raw_current[1], now_ms);
teleplot_emit("board_temp", ntc_temp_c, now_ms);
```

View with the Teleplot VS Code extension or the standalone Teleplot app over the USB serial port.

### Production telemetry

In production, telemetry flows over CAN to the Hub (see next section). Teleplot stays available as a debug output.

---

## CAN Protocol

The EPS module reports to the Hub over CAN (TWAI on ESP32). Frame format shared across all CEC modules.

### TWAI configuration

```c
#include "driver/twai.h"

#define CAN_TX_GPIO  GPIO_NUM_4
#define CAN_RX_GPIO  GPIO_NUM_5

void can_init(void) {
    twai_general_config_t g_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
    twai_start();
}
```

500 kbps for the control/telemetry bus (CAN1). Loopback mode (`TWAI_MODE_NO_ACK` or self-test) lets you validate the driver without a Hub connected.

### Frame layout (proposed)

| Field | Bytes | Description |
|---|---|---|
| Module type | 1 | 0x02 = EPS module |
| Module instance | 1 | Which EPS module (multiple supported) |
| Cable 1 current | 2 | int16, milliamps |
| Cable 2 current | 2 | int16, milliamps |
| Status flags | 1 | Bit field: fault, overcurrent, swing detected |
| Board temp | 1 | int8, degrees C |

8-byte payload fits a standard CAN frame. Use a CAN ID that encodes module type and priority (anomaly frames get a lower ID for higher priority).

### Frame IDs (proposed scheme)

| ID range | Purpose |
|---|---|
| 0x100-0x1FF | Anomaly/alert frames (high priority) |
| 0x200-0x2FF | Periodic telemetry |
| 0x300-0x3FF | Hub commands to modules |
| 0x400-0x4FF | Module responses to commands |

Define the full ID map in `cec_comms/can.h` shared across modules.

---

## NVS Configuration

Persist calibration and config in non-volatile storage.

### Stored values

All blobs go through the `cec_nvs` wrapper which prefixes each payload with a 4-byte magic. A firmware revision that changes the payload layout bumps the magic; the load path then rejects the stale blob cleanly with `ESP_ERR_INVALID_VERSION` instead of feeding garbage into the new struct.

| Key | Magic | Payload | Description |
|---|---|---|---|
| `config` | `0xCEC50002` | `cec_config_t` | module_id, supply_voltage, oc_threshold, ema_alpha, output_raw, layer1/2/3_enabled |
| `zero_off0` / `zero_off1` | `0xCEC50101` | float | Per-sensor ACS758 zero offset (volts at chip output) |
| `l3_profiles` | `0xCEC50201` | `cec_rail_profile_t[CEC_NUM_CABLES]` | Layer 3 learned baselines (mean + std + sample_count per cable). Snapshotted every 5 minutes by `sample_task`; loaded once at boot. |

### Access pattern

```c
#include "nvs_flash.h"
#include "nvs.h"

void config_load(void) {
    nvs_handle_t h;
    if (nvs_open("cec_eps", NVS_READWRITE, &h) != ESP_OK) return;

    size_t sz = sizeof(float);
    nvs_get_blob(h, "eps_zero_off0", &zero_offset[0], &sz);
    sz = sizeof(float);
    nvs_get_blob(h, "eps_zero_off1", &zero_offset[1], &sz);
    // ... load remaining config

    nvs_close(h);
}

void config_save_offset(int idx, float offset) {
    nvs_handle_t h;
    if (nvs_open("cec_eps", NVS_READWRITE, &h) != ESP_OK) return;
    const char *key = (idx == 0) ? "eps_zero_off0" : "eps_zero_off1";
    nvs_set_blob(h, key, &offset, sizeof(float));
    nvs_commit(h);
    nvs_close(h);
}
```

Initialize NVS in app_main before loading config:

```c
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    ret = nvs_flash_init();
}
```

---

## Serial Command Interface

Line-based command parser over the USB Serial-JTAG console for development control. CR / CRLF / LF line endings all work.

| Command | Action |
|---|---|
| `cal` | Run zero-offset calibration (ensure no load first) |
| `cal span <I>` | Span calibration with known current `I` amps |
| `show` | Print currents, calibration, config, layer enables, burst params |
| `set alpha <v>` | Set EMA alpha (in-memory; `save` to persist) |
| `set oc <A>` | Set overcurrent threshold |
| `set supply <V>` | Set measured ACS758 Vcc (re-applies ratiometric scaling + capture cal snapshot) |
| `set decim <N>` | HS dump decimation: emit every Nth row of the 10 kHz capture (runtime-only, not persisted) |
| `set layer1 on/off` | Toggle Layer 1 (current threshold) contribution to flags |
| `set layer2 on/off` | Toggle Layer 2 (dI/dt swing) contribution to flags |
| `set layer3 on/off` | Toggle Layer 3 (rail profile z-score) contribution to flags |
| `mode raw` / `mode filt` | Telemetry emit mode |
| `burst [<text>]` | Trigger a manual burst capture with optional annotation |
| `can` | Print TWAI controller state + error counters + RX/bus-off counts |
| `save` | Persist current config (including layer enables) to NVS |

Implement in `command_task` reading from stdin (USB-Serial-JTAG console).

---

## Build and Flash

### Toolchain

| Tool | Version |
|---|---|
| ESP-IDF | 6.0.1 |
| Target | esp32s3 |

### Commands

```bash
# Set target (once)
idf.py set-target esp32s3

# Configure (PSRAM, flash size)
idf.py menuconfig
#   Component config → ESP PSRAM → Support for external SPI RAM (enable)
#   Serial flasher config → Flash size → 16 MB
#   Partition Table → custom (if using NVS + large app)

# Build
idf.py build

# Flash and monitor
idf.py -p <PORT> flash monitor
```

### sdkconfig essentials

```
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y          # N16R8 uses octal PSRAM
CONFIG_FREERTOS_HZ=1000           # 1ms tick for timing precision
CONFIG_ESP_TASK_WDT_TIMEOUT_S=10
```

The N16R8 module uses octal-mode PSRAM. Set `CONFIG_SPIRAM_MODE_OCT=y` or PSRAM won't initialize.

---

## Bring-Up Checklist

Firmware bring-up after the hardware passes its power-on tests:

- [ ] Flash a minimal ADC read sketch, confirm both channels report ~1937 raw (~1.467 V at the ADC pin, = 2.2 V at the chip output) at zero current with the dev board's 4.4 V Vbus supply. At a nominal 5.0 V supply the expected values are ~2210 raw / ~1.67 V instead.
- [ ] Verify NTC channel reads a sensible board temperature
- [ ] Run zero-offset calibration, confirm offsets are small (±30 mV → ±0.05 V at ADC)
- [ ] Apply a known current (bench supply through one sensor) and confirm the reading matches within a few percent
- [ ] Confirm filter reduces noise to ~10-30 mA RMS
- [ ] Verify swing detector fires on a fast load step
- [ ] Teleplot output renders both current channels live (on the UART USB-C, not the JTAG port)
- [ ] CAN driver installs cleanly (`I (...) can: TWAI node up @ 125000 bps (self-test)`); flip the production TODO in `cec_can.c` to 500 kbps once moved off the Waveshare bench breakout
- [ ] NVS save/load round-trips calibration correctly (`cal`, then `save`, then power-cycle, then `show`)
- [ ] Layer 3 baseline persists across reboots (let it warm for 20+ s, power-cycle, confirm `show` reports load state stable rather than starting in TRANSIENT)
- [ ] Manual `burst` from the CLI emits `>BURST_BEGIN ... >BURST_END` on the UART transport
- [ ] Confirm auto-trigger on a real anomaly: apply a fast load step → check that a burst fires with `reason=current_swing` in the dispatcher log
- [ ] Bus voltage tap reads close to 12 V under load (within ±5 % before trim, ±1 % after a `set supply` calibration pass)
- [ ] Shutdown detection: kill the PSU → confirm `bus shutdown detected ...` log line and a `>BURST_BEGIN:shutdown:` envelope in the dump
- [ ] Serial commands respond on the JTAG USB-C (CLI doesn't move to the UART transport)
- [ ] Full EPS cable splice in place, readings track real CPU load

---

## Development Roadmap

Suggested order of implementation:

1. **ADC driver** (`acs758.c`) — init, read, oversample. Validate against multimeter.
2. **Conversion + calibration** — raw to amps, zero-offset cal, NVS persistence.
3. **Filtering** (`filter.c`) — median + EMA. Tune for EPS noise.
4. **Teleplot output** (`teleplot.c`) — live telemetry for development.
5. **Sample task** — tie ADC + conversion + filter together at 50 Hz.
6. **NTC** (`ntc.c`) — board temperature.
7. **Detection layers** (`detection.c`) — port from 24-pin, tune thresholds for EPS.
8. **Swing detectors** — rate-of-change on raw stream.
9. **State classifier** — idle/light/moderate/heavy buckets.
10. **Ring buffer + burst capture** (`capture.c`) — PSRAM pre-trigger buffer.
11. **CAN/TWAI** (`can.c`) — loopback first, then Hub when available.
12. **Serial commands** (`command_task`) — runtime control.
13. **NVS profiles** — full config persistence.

Steps 1-6 get you a working current monitor with live telemetry. Steps 7-13 add the detection and integration features.

---

## Codebase Parity with the 24-pin Module

The EPS firmware deliberately tracks the [24-pin module's](https://github.com/nathanfraske/cec-24pin-idf) architecture, naming, and component split so a developer fluent in one repo recognizes the other. Shared primitives — filters, NVS schemas, ADC plumbing, TelePlot envelope, CLI dispatcher, capture trigger system — are byte-for-byte the same code path with names and APIs that match.

### Components

| Component | Role | Status |
|---|---|---|
| `cec_common`  | shared types (`cec_state_t`, `cec_load_state_t`, `cec_severity_t`, flag bits) | EPS-only, 24-pin TODO to extract |
| `cec_filters` | `ema_t` + `median_t` primitives (caller-owned buffer, full `init/update/value/reset/count` set) | parity with 24-pin's `cec_detection/cec_filters.*` |
| `cec_sensors` | `cec_adc` ADC1 wrapper, `ntc` thermistor driver, `acs758` Hall-current driver | shared `cec_adc` interface; chip drivers are inherently module-specific |
| `cec_nvs`     | NVS wrapper with magic-prefixed schema versioning | parity with 24-pin's `cec_detection/cec_nvs.*` (24-pin TODO: extract into own component) |
| `cec_detection` | layered detection: `cec_layer1` (threshold), `cec_layer2` (transient), `cec_layer3` (rail profile), `cec_swing` (windowed swing), `cec_classifier` | layer naming + rail-profile + swing primitive are shared; per-layer algorithms diverge where the underlying signal differs (current vs voltage) |
| `cec_telemetry` | TelePlot output (`teleplot_emit` / `teleplot_emit_t`, `%.6f` precision, `PRId64` timestamps); also owns the UART-transport hand-off via `cec_telemetry_init_uart` | parity with 24-pin's `cec_telemetry/cec_teleplot.*`. The UART transport is EPS-specific; 24-pin's printf-only path stays as is. |
| `cec_capture` | trigger system + pre-trigger ring + HS burst capture + `>BURST_BEGIN/END` dump | API + dump envelope parity; HS source diverges (EPS uses `adc_continuous` DMA for 10 kHz; 24-pin uses `adc_oneshot` callback at 1 kHz) |
| `cec_comms`   | CAN/TWAI driver (new `esp_twai` node-handle API; gated by `CEC_CAN_ENABLED` until the daughterboard lands) | EPS-only, 24-pin TODO when CAN ships there |
| `cec_cli`     | line-based serial command interface over USB Serial-JTAG | parity with 24-pin's `cec_cli/cec_cli.*` |

### Naming conventions

- Shared component sources carry the `cec_` prefix (`cec_capture.c`, `cec_filters.c`, `cec_teleplot.c`, `cec_can.c`).
- Chip-specific sensor drivers keep their part names (`acs758.c`, `ntc.c`, in the 24-pin: `ina226.c`, `acs712.c`, `thermistor.c`).
- Types follow the 24-pin's `_t` suffix convention (`ema_t`, `median_t`, `cec_rail_profile_t`, `cec_swing_detector_t`).

### Intentional divergences

The architecture is shared; the algorithms are not always identical. EPS deviates where the underlying signal demands it:

| Topic | 24-pin | EPS |
|---|---|---|
| Layer 1 detector | static voltage band (% deviation) | static current ceiling (absolute amps) |
| Layer 2 detector | adaptive transient (`\|instant - ema\| > k_sigma * std`) | rate-of-change (`\|dI/dt\| > A/ms`), since cable current legitimately swings as the CPU load steps |
| Capture HS source | `adc_oneshot` callback at 1 kHz | `adc_continuous` DMA at 10 kHz |
| State enum | `cec_state_t` (OFF/STANDBY/IDLE/ACTIVE/PEAK — whole PSU) | `cec_load_state_t` (IDLE/LIGHT/MODERATE/HEAVY/TRANSIENT — per cable) |
| Power calc | sums per-rail V×I | assumes 12 V nominal × I (or 12 V rail voltage from CAN once production lands) |

The `cec_state_t` and `cec_load_state_t` names are distinct on purpose — both modules can include each other's headers without a name clash.

### What's "shared" actually means

`cec_filters`, `cec_nvs`, `cec_cli`, `cec_teleplot`, `cec_swing`, `cec_rail_profile_t`, the capture trigger enum + dump envelope: implementation matches the 24-pin's source byte-for-byte (modulo licensing headers). `cec_capture`'s HS path and `cec_detection`'s per-layer algorithms intentionally diverge per the table above, but the file layout, type names, and call shapes are common across both modules.

The flip side: if the 24-pin module changes one of the shared primitives, the EPS module needs the same change, and vice versa. The two trees should be kept in lock-step on those.

### Transports (hybrid)

The Lonely Binary N16R8 board exposes two USB-C ports. EPS firmware uses both:

- **JTAG USB-C** (ESP32-S3 native USB Serial-JTAG): CLI input, ESP_LOG output, command responses, boot banners. Default IDF console.
- **UART USB-C** (CH340K bridge to UART0 on GPIO 43/44, 921600 baud): every TelePlot line — steady-state 10 Hz telemetry and burst dumps both go here. `cec_telemetry_init_uart()` installs the UART driver and `teleplot_*` helpers + `dump_burst` route through `uart_write_bytes` from then on.

This keeps the heavy traffic (~600 KB per burst at full fidelity) off the same wire that's carrying CLI input — typing commands during a burst dump is responsive, and 921600 is the highest standard rate that CH340K + Windows hosts handle reliably without baud-divisor jitter (non-standard rates like 1500000 / 2000000 sometimes produce silent byte corruption depending on the host driver). Real-world wire speed here is ~92 KB/s, ~3× faster than USB Serial-JTAG on the same workload. If the UART init fails (cable not plugged in, etc.), the helpers fall back to stdio so TelePlot keeps working over the JTAG port at the slower rate.

Practical setup: connect both USB-C ports to the host. Run `idf.py monitor` or your terminal of choice on the JTAG port for CLI + logs, and point TelePlot at the UART port for data visualization. Add this to the 24-pin TODO if/when its USB Serial-JTAG throughput becomes the bottleneck — the `cec_telemetry_init_uart` plumbing in EPS is the reference.

---

## Appendix: Differences from 24-pin Module

For developers familiar with the 24-pin firmware, here's what changes for EPS:

| Aspect | 24-pin | EPS |
|---|---|---|
| Sensor count | 4 | 2 |
| Sensor type | INA226 (I2C) | ACS758 (analog ADC) |
| Driver component | `ina226.c` | `acs758.c` |
| Measures | Voltage + current per rail | Current per cable only |
| Bus interface | I2C at 400-800 kHz | ADC1 oneshot |
| Calibration | Trim factors (voltage/current) | Zero offset + optional span |
| Noise floor | ~0.5-2 mA | ~10-30 mA (filtered) |
| Voltage available | Yes (direct) | No (assume 12 V or get from CAN) |
| Power supply | 3.3 V | 5 V (for ACS758) + 3.3 V (MCU) |

**Shared (no change):** filter primitives, NVS schema wrapper, TelePlot output, CLI dispatcher, ADC abstraction, capture trigger system, layer naming + rail profile + swing primitive, FreeRTOS task structure, daughterboard (CAN + NTC + Hub interface). See the [Codebase Parity](#codebase-parity-with-the-24-pin-module) section above for the file-by-file map.

The architectural payoff: swapping `ina226.c` for `acs758.c` and adjusting the sample task's read calls is most of the module-specific work. Everything downstream of "filtered current value" is common code.

### Trim values reference (24-pin, for context)

The 24-pin module's v0.5.9 trim values (not directly applicable to EPS, but illustrate the calibration philosophy):

```
TRIM_12V  = 1.0000
TRIM_5V   = 0.9962
TRIM_3V3  = 0.9915
TRIM_5VSB = 0.9901
```

EPS uses zero-offset calibration instead of multiplicative trim, because the ACS758's dominant error is offset (quiescent point variation), not gain.

---

## References

- ACS758 datasheet (Allegro MicroSystems) for transfer function, FAULT behavior, thermal specs
- ESP-IDF ADC oneshot driver docs for `adc_oneshot_*` API
- ESP-IDF TWAI driver docs for CAN
- 24-pin module firmware (`cec-24pin-idf`) for shared component implementations
- EPS refboard build guide (`eps-refboard-build-guide.md`) for hardware wiring
