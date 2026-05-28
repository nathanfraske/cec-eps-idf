#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "cec_state.h"
#include "cec_config.h"
#include "cec_adc.h"
#include "acs758.h"
#include "ntc.h"
#include "cec_filters.h"
#include "cec_detection.h"
#include "cec_capture.h"
#include "cec_can.h"
#include "cec_teleplot.h"
#include "cec_cli.h"
#include "cec_classifier.h"

static const char *TAG = "eps_main";

// ---- Global state ----
static cec_state_t   g_state;
static cec_config_t  g_config;
static acs758_ctx_t  g_acs;
// NTC on GPIO 7 -> ADC1_CH6. 10K @ 25C, B=3950, 10K series pull-up to
// 3V3 - matches the EPS daughterboard schematic.
static ntc_t g_ntc = {
    .channel              = ADC_CHANNEL_6,
    .samples              = 8,
    .beta                 = 3950.0f,
    .nominal_resistance   = 10000.0f,
    .nominal_temperature_k = 298.15f,
    .pull_up_resistance   = 10000.0f,
    .vcc                  = 3.3f,
};
// 12V rail tap on GPIO 1 -> ADC1_CH0 via a 47k/10k divider, so
// V_rail = V_pin * (47k+10k)/10k = V_pin * 5.7. Trim is unity; a per-
// unit trim ('set vtrim <f>' later, or a one-shot calibration helper)
// can refine it against a known reference.
static const cec_adc_rail_t s_rail_12v = {
    .channel = ADC_CHANNEL_0,                          // GPIO 1
    .samples = 4,
    .scale   = (47000.0f + 10000.0f) / 10000.0f,       // 5.7
    .trim    = 1.0f,
};
// Two-stage filter per cable: median to reject impulse spikes, then EMA
// to smooth. Median window size is per-module tunable; 5 is the EPS
// starting point for the Hall sensor's ~50-100 mA RMS raw noise.
#define EPS_MEDIAN_WINDOW   5
static median_t g_median[CEC_NUM_CABLES];
static ema_t    g_ema[CEC_NUM_CABLES];
static float    g_median_buf[CEC_NUM_CABLES][EPS_MEDIAN_WINDOW];
static cec_detection_ctx_t g_detect;

// Burst capture engine config. 10 kHz HS rate (per channel) for 1 s, with
// 20 s of 50 Hz pre-trigger context. cooldown_ms=10000 matches the 24-pin.
#define EPS_BURST_HS_RATE_HZ        10000
#define EPS_BURST_HS_DURATION       1000
#define EPS_PRE_TRIGGER_SECONDS     20
#define EPS_BURST_COOLDOWN_MS       10000
// Default dump decimation: emit every 5th HS row to TelePlot (2 kHz
// visible from 10 kHz captured). Override via `set decim <N>`.
#define EPS_BURST_HS_DUMP_DECIM     5

// Bus voltage shutdown detector. 1 s rolling window of bus_voltage
// samples; if (newest - oldest) drops faster than the threshold the
// rail is collapsing - fire CEC_TRIG_SHUTDOWN (cooldown-bypassed) so
// the rail's full collapse is captured. After firing, mute the normal
// anomaly triggers for SHUTDOWN_MUTE_MS so cascading layer fires from
// the collapsing rail don't spam the dump path. Parity with the
// 24-pin's v0.5.9 v_12v rate detector.
#define BUS_V_HIST_SIZE              SAMPLE_RATE_HZ      // 1 sec window
#define BUS_V_SHUTDOWN_RATE_V_PER_S  (-0.5f)             // V/s, negative
#define BUS_V_ARMED_THRESHOLD_V       5.0f               // arm once seen above
#define SHUTDOWN_MUTE_MS              30000              // 30 s mute window

// Telemetry transport - hybrid setup on the Lonely Binary N16R8 board.
// UART USB-C (CH340K bridge) carries TelePlot output (steady telemetry
// + burst dumps). JTAG USB-C carries CLI input / ESP_LOG / banners.
//
// 921600 is the safe-everywhere standard rate that CH340K + Windows
// handles without baud-divisor jitter. Non-standard rates (1500000,
// 2000000) sometimes work and sometimes silently produce garbage
// depending on the host driver; bump only after verifying TelePlot
// output is clean at this rate first. Real-world throughput here is
// ~92 KB/s, which is still ~3x faster than USB Serial-JTAG's
// effective ceiling on the same workload.
#define EPS_TELEMETRY_UART_PORT     0          // UART0
// MCU-side TX = GPIO 43, RX = GPIO 44 (matches the ESP32-S3 UART0
// default pinout and the silkscreen labels). The swapped variant
// produced silence on COM5 - confirming the original pin direction
// was right and the garble had a different root cause (see
// source_clk = UART_SCLK_APB in cec_telemetry_init_uart).
#define EPS_TELEMETRY_UART_TX       43
#define EPS_TELEMETRY_UART_RX       44
#define EPS_TELEMETRY_UART_BAUD     921600
#define EPS_TELEMETRY_UART_TX_BUF   16384

// ---- Timing ----
#define SAMPLE_RATE_HZ   50
#define SAMPLE_PERIOD_MS  (1000 / SAMPLE_RATE_HZ)
#define OUTPUT_RATE_HZ   10
#define OUTPUT_PERIOD_MS  (1000 / OUTPUT_RATE_HZ)
#define COMMS_RATE_HZ    20
#define COMMS_PERIOD_MS   (1000 / COMMS_RATE_HZ)

// ---- Bus voltage shutdown detector helpers ----
//
// Rolling 1 s window of bus voltage. The detector arms once the bus
// has been observed above BUS_V_ARMED_THRESHOLD_V (so cold-boot
// pre-PSU-up doesn't fire) and disarms after a shutdown fires (so a
// fully-collapsed rail doesn't re-fire on every iteration).
// bus_shutdown_muted() gates the normal anomaly trigger path during
// the post-shutdown window.

static float    s_bus_v_hist[BUS_V_HIST_SIZE];
static size_t   s_bus_v_hist_idx = 0;
static size_t   s_bus_v_hist_count = 0;
static bool     s_bus_shutdown_armed = false;
static int64_t  s_bus_shutdown_mute_until_us = 0;

static bool bus_shutdown_check(float bus_v, int64_t now_us)
{
    s_bus_v_hist[s_bus_v_hist_idx] = bus_v;
    s_bus_v_hist_idx = (s_bus_v_hist_idx + 1) % BUS_V_HIST_SIZE;
    if (s_bus_v_hist_count < BUS_V_HIST_SIZE) {
        s_bus_v_hist_count++;
        return false;   // need a full window before slope is meaningful
    }

    if (bus_v >= BUS_V_ARMED_THRESHOLD_V) {
        s_bus_shutdown_armed = true;
    }
    if (!s_bus_shutdown_armed) return false;

    /* idx now points at the about-to-be-overwritten oldest sample. */
    float oldest = s_bus_v_hist[s_bus_v_hist_idx];
    float slope_v_per_s = bus_v - oldest;  // window is exactly 1 s wide

    if (slope_v_per_s < BUS_V_SHUTDOWN_RATE_V_PER_S) {
        s_bus_shutdown_armed = false;  // disarm; re-arms when bus recovers above threshold
        s_bus_shutdown_mute_until_us = now_us + (int64_t)SHUTDOWN_MUTE_MS * 1000;
        return true;
    }
    return false;
}

static bool bus_shutdown_muted(int64_t now_us)
{
    return now_us < s_bus_shutdown_mute_until_us;
}

// ---- Load-state edge tracking ----
//
// Fires CEC_TRIG_STATE_CHANGE on any transition between load buckets
// (IDLE/LIGHT/MODERATE/HEAVY/TRANSIENT). cec_capture's cooldown gate
// throttles back-to-back transitions, so a rapidly fluctuating load
// produces one burst per cooldown window, not one per transition.

static bool             s_load_state_initialized = false;
static cec_load_state_t s_prev_load_state;

// ---- Sample task: read, convert, filter, detect, store ----
static void sample_task(void *arg)
{
    ESP_LOGI(TAG, "sample task started on core %d", xPortGetCoreID());
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        int64_t now_us = esp_timer_get_time();

        // While the burst engine has ADC1, skip every read this iteration
        // (cec_adc_read_mv would return ESP_ERR_INVALID_STATE anyway).
        // The shared-state snapshot from the last good iteration stays
        // valid for downstream consumers during the burst window.
        if (cec_capture_is_busy()) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        float raw[CEC_NUM_CABLES];
        float filt[CEC_NUM_CABLES];
        for (int i = 0; i < CEC_NUM_CABLES; i++) {
            raw[i] = acs758_read_current(&g_acs, i);
            filt[i] = ema_update(&g_ema[i], median_update(&g_median[i], raw[i]));
        }

        // Run detection layers
        uint8_t flags = 0;
        cec_load_state_t load_state = CEC_LOAD_IDLE;
        bool anomaly = cec_detection_run(&g_detect, raw, filt, now_us, &flags, &load_state);

        float temp = 0.0f;
        (void)ntc_read_celsius(&g_ntc, &temp);   // leaves temp at 0 on open/short

        float bus_v = 0.0f;
        (void)cec_adc_read(&s_rail_12v, &bus_v); // leaves at 0 on failure

        // Bus voltage shutdown detection. Bypasses cooldown so the
        // collapse is always captured even if a recent burst is in
        // cooldown. Also arms the post-shutdown mute window that
        // suppresses normal anomaly triggers from the cascading rail.
        if (bus_shutdown_check(bus_v, now_us)) {
            esp_err_t tr = cec_capture_trigger(CEC_TRIG_SHUTDOWN);
            ESP_LOGW(TAG, "bus shutdown detected (bus=%.2fV) - burst (%s) mute=%dms",
                     bus_v, esp_err_to_name(tr), SHUTDOWN_MUTE_MS);
        }

        // Update shared state
        if (xSemaphoreTake(g_state.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int i = 0; i < CEC_NUM_CABLES; i++) {
                g_state.current_a[i] = filt[i];
                g_state.current_raw_a[i] = raw[i];
            }
            g_state.bus_voltage_v = bus_v;
            g_state.board_temp_c = temp;
            g_state.load_state = load_state;
            g_state.status_flags = flags;
            g_state.timestamp_us = now_us;
            xSemaphoreGive(g_state.mutex);
        }

        // Push the full per-iteration snapshot into the pre-trigger ring.
        cec_capture_sample_t cap_sample = {
            .ts_ms         = (uint32_t)(now_us / 1000),
            .eps1_a        = filt[0],
            .eps2_a        = filt[1],
            .eps1_raw_a    = raw[0],
            .eps2_raw_a    = raw[1],
            .bus_voltage_v = bus_v,
            .temp_c        = temp,
            .load_state    = (uint8_t)load_state,
            .flags         = flags,
        };
        cec_capture_push(&cap_sample);

        if (anomaly && !bus_shutdown_muted(now_us)) {
            // Fire a burst capture on anomaly with the trigger reason
            // picked from the actual flags (so the burst envelope reads
            // STATIC_CRIT / ANOMALY / CURRENT_SWING rather than always
            // ANOMALY). cec_capture's busy/cooldown gates absorb
            // back-to-back triggers; we don't gate again here. The
            // shutdown-mute test silences cascading detector fires
            // from a collapsing rail in the 30 s after SHUTDOWN.
            cec_trigger_t reason = cec_trigger_for_flags(flags);
            esp_err_t tr = cec_capture_trigger(reason);
            if (tr == ESP_OK) {
                ESP_LOGW(TAG, "flags=0x%02x reason=%s - burst triggered",
                         flags, cec_trigger_name(reason));
            } else if (tr == ESP_ERR_NOT_FINISHED || tr == ESP_ERR_INVALID_STATE) {
                ESP_LOGD(TAG, "flags=0x%02x reason=%s - burst skipped (%s)",
                         flags, cec_trigger_name(reason), esp_err_to_name(tr));
            } else {
                ESP_LOGW(TAG, "flags=0x%02x reason=%s - trigger failed: %s",
                         flags, cec_trigger_name(reason), esp_err_to_name(tr));
            }
        }

        // Load state transition trigger. Cooldown gate on cec_capture
        // throttles rapid back-to-back transitions to one burst per
        // cooldown window, which is what we want for steady-state CPU
        // load chatter. Suppressed during the shutdown mute window.
        if (s_load_state_initialized
            && load_state != s_prev_load_state
            && !bus_shutdown_muted(now_us)) {
            char ann[40];
            snprintf(ann, sizeof(ann), "load %s -> %s",
                     cec_load_state_name(s_prev_load_state),
                     cec_load_state_name(load_state));
            esp_err_t tr = cec_capture_trigger_with_text(CEC_TRIG_STATE_CHANGE, ann);
            if (tr == ESP_OK) {
                ESP_LOGI(TAG, "state change: %s", ann);
            }
        }
        s_prev_load_state = load_state;
        s_load_state_initialized = true;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// ---- Output task: Teleplot telemetry ----
static void output_task(void *arg)
{
    ESP_LOGI(TAG, "output task started on core %d", xPortGetCoreID());
    (void)arg;

    while (1) {
        cec_state_t snap;
        if (xSemaphoreTake(g_state.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            snap = g_state;
            xSemaphoreGive(g_state.mutex);
            teleplot_emit_state(&snap, g_config.output_raw);
        }
        // vTaskDelay (not DelayUntil) so a long burst dump can't leave
        // this task in "catch up" mode where DelayUntil returns instantly
        // and the task spins iteration-on-iteration, starving IDLE1.
        vTaskDelay(pdMS_TO_TICKS(OUTPUT_PERIOD_MS));
    }
}

// ---- CLI command handlers ----

// Snapshot the live ACS758 cal for one channel into a
// cec_capture_channel_t for cec_capture_update_channel.
static void capture_channel_snapshot(int idx, cec_capture_channel_t *out)
{
    *out = (cec_capture_channel_t){
        .channel         = g_acs.channels[idx],
        .divider_gain    = ACS758_DIVIDER_GAIN,
        .quiescent_v     = g_acs.cal[idx].quiescent_v,
        .sensitivity_v_a = g_acs.cal[idx].sensitivity_v_a,
        .zero_offset_v   = g_acs.cal[idx].zero_offset_v,
    };
}

static int cmd_show(int argc, char **argv)
{
    (void)argc; (void)argv;
    cec_state_t snap;
    if (xSemaphoreTake(g_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        printf("error: state mutex timeout\n");
        return 1;
    }
    snap = g_state;
    xSemaphoreGive(g_state.mutex);

    printf("eps1   raw=%7.3f A   filt=%7.3f A   zero_off=%.4f V\n",
           snap.current_raw_a[0], snap.current_a[0], g_acs.cal[0].zero_offset_v);
    printf("eps2   raw=%7.3f A   filt=%7.3f A   zero_off=%.4f V\n",
           snap.current_raw_a[1], snap.current_a[1], g_acs.cal[1].zero_offset_v);
    float bus_v = snap.bus_voltage_v;
    float total_a = snap.current_a[0] + snap.current_a[1];
    printf("bus    %.3f V   total=%.3f A   power=%.1f W\n",
           bus_v, total_a, bus_v * total_a);
    printf("temp   %.2f C   load=%s   flags=0x%02x\n",
           snap.board_temp_c, cec_load_state_name(snap.load_state), snap.status_flags);
    printf("config id=%u supply=%.2f V oc=%.1f A alpha=%.2f raw_telem=%d\n",
           g_config.module_id, g_config.supply_voltage, g_config.oc_threshold_a,
           g_config.ema_alpha, g_config.output_raw);
    int decim = cec_capture_get_hs_dump_decimation();
    if (decim > 0) {
        printf("burst  hs_rate=%d Hz/ch dump_decim=%d (~%d Hz visible) cooldown=%d ms\n",
               EPS_BURST_HS_RATE_HZ, decim,
               EPS_BURST_HS_RATE_HZ / decim, EPS_BURST_COOLDOWN_MS);
    }
    return 0;
}

static int cmd_cal(int argc, char **argv)
{
    // 'cal'              -> zero-offset cal on both sensors
    // 'cal span <amps>'  -> span calibration with known current
    if (argc >= 2 && strcmp(argv[1], "span") == 0) {
        if (argc < 3) {
            printf("usage: cal span <known_amps>\n");
            return 1;
        }
        float known = strtof(argv[2], NULL);
        if (known == 0.0f) {
            printf("error: known current must be nonzero\n");
            return 1;
        }
        for (int i = 0; i < CEC_NUM_CABLES; i++) {
            acs758_calibrate_span(&g_acs, i, known);
            cec_capture_channel_t ch;
            capture_channel_snapshot(i, &ch);
            cec_capture_update_channel(i, &ch);
        }
        printf("span cal done at %.2f A\n", known);
        return 0;
    }

    printf("zero-cal: ensure NO current flows in either cable...\n");
    for (int i = 0; i < CEC_NUM_CABLES; i++) {
        float off = acs758_calibrate_zero(&g_acs, i);
        cec_config_save_zero_offset(i, off);
        cec_capture_channel_t ch;
        capture_channel_snapshot(i, &ch);
        cec_capture_update_channel(i, &ch);
        printf("sensor %d zero offset = %.4f V (saved)\n", i, off);
    }
    return 0;
}

static int cmd_save(int argc, char **argv)
{
    (void)argc; (void)argv;
    cec_config_save(&g_config);
    return 0;
}

static int cmd_set(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: set <alpha|oc|supply> <value>\n");
        return 1;
    }
    float v = strtof(argv[2], NULL);
    if (strcmp(argv[1], "alpha") == 0) {
        g_config.ema_alpha = v;
        for (int i = 0; i < CEC_NUM_CABLES; i++) g_ema[i].alpha = v;
        printf("alpha=%.3f\n", v);
    } else if (strcmp(argv[1], "oc") == 0) {
        g_config.oc_threshold_a = v;
        cec_detection_init(&g_detect, v);   // re-init layer 1 with new threshold
        printf("oc=%.2f A\n", v);
    } else if (strcmp(argv[1], "supply") == 0) {
        g_config.supply_voltage = v;
        acs758_set_supply(&g_acs, v);
        for (int i = 0; i < CEC_NUM_CABLES; i++) {
            cec_capture_channel_t ch;
            capture_channel_snapshot(i, &ch);
            cec_capture_update_channel(i, &ch);
        }
        printf("supply=%.3f V\n", v);
    } else if (strcmp(argv[1], "decim") == 0) {
        int d = (int)strtol(argv[2], NULL, 10);
        if (cec_capture_set_hs_dump_decimation(d) != ESP_OK) {
            printf("error: capture not initialized\n");
            return 1;
        }
        printf("hs_dump_decim=%d (capture stays %d Hz; ~%d Hz visible)\n",
               cec_capture_get_hs_dump_decimation(),
               EPS_BURST_HS_RATE_HZ,
               EPS_BURST_HS_RATE_HZ / cec_capture_get_hs_dump_decimation());
        return 0;   // runtime-only, not persisted
    } else {
        printf("error: unknown key '%s'\n", argv[1]);
        return 1;
    }
    printf("(use 'save' to persist)\n");
    return 0;
}

static int cmd_can(int argc, char **argv)
{
    (void)argc; (void)argv;
    int state = -1;
    uint16_t tx_err = 0, rx_err = 0;
    uint32_t tx_q = 0, bus_err = 0;
    esp_err_t r = can_get_info(&state, &tx_err, &rx_err, &tx_q, &bus_err);
    if (r != ESP_OK) {
        printf("error: %s\n", esp_err_to_name(r));
        return 1;
    }
    const char *state_name;
    switch (state) {
    case 0:  state_name = "ACTIVE  (normal)"; break;
    case 1:  state_name = "WARNING (>96 errs)"; break;
    case 2:  state_name = "PASSIVE (>127 errs)"; break;
    case 3:  state_name = "BUS_OFF (>255 errs)"; break;
    default: state_name = "?"; break;
    }
    printf("state       %s\n", state_name);
    printf("tx_err      %u  (counter, decays on success)\n", (unsigned)tx_err);
    printf("rx_err      %u  (counter, decays on success)\n", (unsigned)rx_err);
    printf("tx_q_free   %u  (slots free in TX queue)\n", (unsigned)tx_q);
    printf("bus_err_num %u  (cumulative since init)\n", (unsigned)bus_err);
    printf("rx_count    %u  (cumulative frames received)\n", (unsigned)can_get_rx_count());
    printf("bus_off_cnt %u  (cumulative bus-off events recovered)\n",
           (unsigned)can_get_bus_off_count());
    return 0;
}

static int cmd_burst(int argc, char **argv)
{
    const char *text = (argc >= 2) ? argv[1] : "manual";
    esp_err_t r = cec_capture_trigger_with_text(CEC_TRIG_MANUAL, text);
    if (r == ESP_OK) {
        printf("burst triggered (annotation=%s)\n", text);
        return 0;
    }
    if (r == ESP_ERR_NOT_FINISHED) {
        printf("error: a burst is already running\n");
        return 1;
    }
    if (r == ESP_ERR_INVALID_STATE) {
        printf("error: cooldown active, try again shortly\n");
        return 1;
    }
    printf("error: trigger failed: %s\n", esp_err_to_name(r));
    return 1;
}

static int cmd_mode(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: mode <raw|filt>\n");
        return 1;
    }
    if (strcmp(argv[1], "raw")  == 0) g_config.output_raw = true;
    else if (strcmp(argv[1], "filt") == 0) g_config.output_raw = false;
    else { printf("error: unknown mode '%s'\n", argv[1]); return 1; }
    printf("output_raw=%d (use 'save' to persist)\n", g_config.output_raw);
    return 0;
}

static const cec_cli_command_t CLI_COMMANDS[] = {
    { "show",  "print current readings, calibration, and config",       cmd_show  },
    { "cal",   "zero-offset cal on both sensors, or 'cal span <amps>'", cmd_cal   },
    { "set",   "set <alpha|oc|supply|decim> <value> (decim is runtime-only)", cmd_set },
    { "save",  "persist current config to NVS",                         cmd_save  },
    { "mode",  "set telemetry mode: 'mode raw' or 'mode filt'",         cmd_mode  },
    { "burst", "trigger a manual burst capture ('burst <annotation>')", cmd_burst },
    { "can",   "print TWAI controller state + error counters",           cmd_can   },
};
#define CLI_COMMAND_COUNT (sizeof(CLI_COMMANDS) / sizeof(CLI_COMMANDS[0]))

// ---- Comms task: CAN telemetry to Hub ----
static void comms_task(void *arg)
{
    ESP_LOGI(TAG, "comms task started on core %d", xPortGetCoreID());
    (void)arg;

    while (1) {
        cec_state_t snap;
        if (xSemaphoreTake(g_state.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            snap = g_state;
            xSemaphoreGive(g_state.mutex);

            can_send_telemetry(CEC_MODULE_TYPE_EPS, g_config.module_id,
                               snap.current_a, snap.status_flags, snap.board_temp_c);
            if (snap.status_flags != 0) {
                can_send_anomaly(CEC_MODULE_TYPE_EPS, g_config.module_id,
                                 snap.status_flags);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(COMMS_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "CEC EPS module firmware starting");

    // NVS + config
    ESP_ERROR_CHECK(cec_config_init_nvs());
    cec_config_load(&g_config);

    // Route TelePlot output to the CH340K UART USB-C. Steady-state
    // telemetry + burst dump go via this transport from here on; CLI
    // input + ESP_LOG continue over USB Serial-JTAG. If the UART init
    // fails for any reason, teleplot_* helpers fall back to stdio so
    // TelePlot still works over the JTAG port (just slower).
    esp_err_t telem_ret = cec_telemetry_init_uart(EPS_TELEMETRY_UART_PORT,
                                                  EPS_TELEMETRY_UART_TX,
                                                  EPS_TELEMETRY_UART_RX,
                                                  EPS_TELEMETRY_UART_BAUD,
                                                  EPS_TELEMETRY_UART_TX_BUF);
    if (telem_ret != ESP_OK) {
        ESP_LOGW(TAG, "telemetry UART init failed (%s); falling back to stdio",
                 esp_err_to_name(telem_ret));
    }

    // Shared state
    memset(&g_state, 0, sizeof(g_state));
    g_state.mutex = xSemaphoreCreateMutex();
    if (g_state.mutex == NULL) {
        ESP_LOGE(TAG, "mutex create failed");
        abort();
    }

    // ADC1 + curve-fit calibration, shared by every sensor driver.
    ESP_ERROR_CHECK(cec_adc_init());

    // Sensors
    ESP_ERROR_CHECK(acs758_init(&g_acs));
    // Apply the measured supply voltage for ratiometric correction.
    acs758_set_supply(&g_acs, g_config.supply_voltage);
    // Load any stored zero offsets
    for (int i = 0; i < CEC_NUM_CABLES; i++) {
        float off;
        if (cec_config_load_zero_offset(i, &off)) {
            acs758_set_zero_offset(&g_acs, i, off);
            ESP_LOGI(TAG, "loaded zero offset[%d] = %.4fV", i, off);
        }
    }

    ESP_ERROR_CHECK(ntc_setup(&g_ntc));

    // Register the 12V rail tap channel on cec_adc (same ADC1 unit).
    ESP_ERROR_CHECK(cec_adc_setup_channel(s_rail_12v.channel));

    // Filters: median (impulse rejection) then EMA (smoothing).
    for (int i = 0; i < CEC_NUM_CABLES; i++) {
        median_init(&g_median[i], g_median_buf[i], EPS_MEDIAN_WINDOW);
        ema_init(&g_ema[i], g_config.ema_alpha);
    }

    // Detection
    cec_detection_init(&g_detect, g_config.oc_threshold_a);

    // Burst capture engine: pre-trigger ring at SAMPLE_RATE_HZ, HS path
    // at 10 kHz/channel via adc_continuous. Channel conversion params
    // come from the live ACS758 calibration so HS samples land in amps.
    cec_capture_config_t cap_cfg = {
        .pre_trigger_capacity = EPS_PRE_TRIGGER_SECONDS * SAMPLE_RATE_HZ,
        .hs_sample_rate_hz    = EPS_BURST_HS_RATE_HZ,
        .hs_duration_ms       = EPS_BURST_HS_DURATION,
        .cooldown_ms          = EPS_BURST_COOLDOWN_MS,
        .hs_dump_decimation   = EPS_BURST_HS_DUMP_DECIM,
        .n_channels           = CEC_NUM_CABLES,
    };
    for (int i = 0; i < CEC_NUM_CABLES; i++) {
        cap_cfg.channels[i] = (cec_capture_channel_t){
            .channel         = g_acs.channels[i],
            .divider_gain    = ACS758_DIVIDER_GAIN,
            .quiescent_v     = g_acs.cal[i].quiescent_v,
            .sensitivity_v_a = g_acs.cal[i].sensitivity_v_a,
            .zero_offset_v   = g_acs.cal[i].zero_offset_v,
        };
    }
    ESP_ERROR_CHECK(cec_capture_init(&cap_cfg));

#if CEC_CAN_ENABLED
    // ====== TODO: FLIP TO can_init(false) WHEN HUB IS WIRED ======
    // TRUE = self-test (NO_ACK) mode. TX goes on the wire through
    // the transceiver but doesn't need another node to ACK, so the
    // controller stays happy until the Hub joins the bus. ESP32-S3
    // has no hardware loopback, so RX of our own TX won't appear in
    // on_rx_done - verify TX with a scope or USB-CAN dongle until
    // the Hub is up. Once the Hub is on the bus and ACKing, flip to
    // can_init(false) for normal mode (TX with ACK required).
    // =============================================================
    can_init(true);
#else
    ESP_LOGW(TAG, "CAN disabled (CEC_CAN_ENABLED=0); skipping TWAI init and comms task");
#endif

    // Tasks: sample on core 0 (isolated), output/comms on core 1
    xTaskCreatePinnedToCore(sample_task, "sample", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(output_task, "output", 4096, NULL, 3, NULL, 1);
#if CEC_CAN_ENABLED
    xTaskCreatePinnedToCore(comms_task,  "comms",  4096, NULL, 4, NULL, 1);
#else
    (void)comms_task;
#endif

    // Serial command interface (USB Serial-JTAG console).
    ESP_ERROR_CHECK(cec_cli_init(CLI_COMMANDS, CLI_COMMAND_COUNT));

    ESP_LOGI(TAG, "init complete, tasks running");
}
