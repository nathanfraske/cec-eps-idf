/*
 * Burst capture engine - EPS variant.
 *
 * Pre-trigger ring is populated synchronously by the main sample task
 * via cec_capture_push (50 Hz). On a trigger, a dispatcher task
 * (Core 1) borrows ADC1 from cec_adc, runs a 10 kHz adc_continuous
 * capture into a separate PSRAM buffer, returns ADC1, and dumps both
 * buffers as TelePlot lines wrapped in >BURST_BEGIN / >BURST_END.
 *
 * adc_continuous and adc_oneshot cannot coexist on the same ADC unit,
 * so the sample task should check cec_capture_is_busy() and skip
 * ADC-touching work while a burst is in flight (~1 s by default).
 */

#include "cec_capture.h"
#include "cec_adc.h"
#include "cec_teleplot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "hal/adc_types.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static const char *TAG = "cec_capture";

/* DMA frame size. Each raw sample is SOC_ADC_DIGI_RESULT_BYTES (4 on
 * ESP32-S3). 2 KB/frame = 512 raw samples = 256 HS rows at 2 channels,
 * which is ~26 ms of data at 10 kHz per channel. The dispatcher task
 * comfortably drains that before the next frame is ready. */
#define HS_DMA_FRAME_BYTES        2048
#define HS_DMA_STORE_BYTES        (HS_DMA_FRAME_BYTES * 4)
#define DISPATCH_TASK_STACK       6144
#define DISPATCH_TASK_PRIORITY    (configMAX_PRIORITIES - 2)
#define DISPATCH_TASK_CORE_ID     1
#define ANNOTATION_MAX            96

/* Trigger names (parity with 24-pin's cec_capture.c). */
static const char *TRIGGER_NAMES[CEC_TRIG_COUNT] = {
    [CEC_TRIG_NONE]          = "none",
    [CEC_TRIG_MANUAL]        = "manual",
    [CEC_TRIG_STATIC_WARN]   = "static_warn",
    [CEC_TRIG_STATIC_CRIT]   = "static_crit",
    [CEC_TRIG_TRANSIENT]     = "transient",
    [CEC_TRIG_ANOMALY]       = "anomaly",
    [CEC_TRIG_STATE_CHANGE]  = "state_change",
    [CEC_TRIG_SHUTDOWN]      = "shutdown",
    [CEC_TRIG_POWER_SWING]   = "power_swing",
    [CEC_TRIG_CURRENT_SWING] = "current_swing",
};

/* Static state. */
static cec_capture_config_t s_cfg;
static cec_capture_sample_t *s_pre_buf = NULL;
static size_t s_pre_capacity = 0;
static volatile size_t s_pre_write_idx = 0;
static volatile size_t s_pre_count = 0;

static cec_capture_hs_sample_t *s_hs_buf = NULL;
static size_t s_hs_capacity = 0;

static SemaphoreHandle_t s_trigger_sem = NULL;
static TaskHandle_t      s_dispatch_task = NULL;
static volatile bool     s_inited = false;
static volatile bool     s_busy = false;
static volatile cec_trigger_t s_pending_reason = CEC_TRIG_NONE;
static volatile int64_t  s_last_complete_us = 0;
static char              s_pending_annotation[ANNOTATION_MAX];

/* Scratch DMA buffer. Static (not stack) because it's bigger than a
 * task stack comfortably holds. */
static uint8_t s_dma_scratch[HS_DMA_FRAME_BYTES];

const char *cec_trigger_name(cec_trigger_t t)
{
    if ((int)t < 0 || (int)t >= CEC_TRIG_COUNT) return "?";
    return TRIGGER_NAMES[t];
}

bool cec_capture_is_busy(void)
{
    return s_busy;
}

void cec_capture_push(const cec_capture_sample_t *s)
{
    if (!s_inited || s == NULL || s_pre_buf == NULL) return;
    size_t idx = s_pre_write_idx;
    s_pre_buf[idx] = *s;
    s_pre_write_idx = (idx + 1) % s_pre_capacity;
    if (s_pre_count < s_pre_capacity) s_pre_count++;
}

/* ---- HS sample conversion ----
 * raw ADC count -> calibrated mV -> chip voltage (mV * divider_gain)
 * -> signal voltage (chip - quiescent - zero_offset) -> amps (/ sensitivity).
 */
static float convert_to_amps(const cec_capture_channel_t *ch, int mv)
{
    float v_chip = (mv / 1000.0f) * ch->divider_gain;
    float v_sig  = v_chip - ch->quiescent_v - ch->zero_offset_v;
    if (ch->sensitivity_v_a == 0.0f) return 0.0f;
    return v_sig / ch->sensitivity_v_a;
}

static int channel_index(adc_channel_t channel)
{
    for (int c = 0; c < s_cfg.n_channels; c++) {
        if (s_cfg.channels[c].channel == channel) return c;
    }
    return -1;
}

/* Configure + start adc_continuous, drain frames until hs_duration_ms
 * elapses or the HS buffer fills, then tear it down. Returns the
 * actual number of HS rows captured via *out_count. */
static esp_err_t run_hs_capture(size_t *out_count)
{
    *out_count = 0;
    adc_continuous_handle_t handle = NULL;
    adc_cali_handle_t cali = cec_adc_get_cali_handle();

    /* Pause cec_adc so adc_continuous can take ADC1. */
    esp_err_t err = cec_adc_pause();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cec_adc_pause failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Build pattern + handle config. */
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = HS_DMA_STORE_BYTES,
        .conv_frame_size    = HS_DMA_FRAME_BYTES,
    };
    err = adc_continuous_new_handle(&handle_cfg, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_new_handle failed: %s", esp_err_to_name(err));
        goto resume_and_exit;
    }

    adc_digi_pattern_config_t pattern[CEC_NUM_CABLES];
    for (int i = 0; i < s_cfg.n_channels; i++) {
        pattern[i].atten     = ADC_ATTEN_DB_12;
        pattern[i].channel   = (uint8_t)s_cfg.channels[i].channel;
        pattern[i].unit      = ADC_UNIT_1;
        /* adc_continuous wants the chip's actual digital-domain bit
         * width here (12 on ESP32-S3). ADC_BITWIDTH_DEFAULT (= 0) is
         * only accepted by the oneshot driver. */
        pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    adc_continuous_config_t dig_cfg = {
        .pattern_num     = s_cfg.n_channels,
        .adc_pattern     = pattern,
        .sample_freq_hz  = s_cfg.hs_sample_rate_hz * s_cfg.n_channels,
        .conv_mode       = ADC_CONV_SINGLE_UNIT_1,
        .format          = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    err = adc_continuous_config(handle, &dig_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_config failed: %s", esp_err_to_name(err));
        adc_continuous_deinit(handle);
        goto resume_and_exit;
    }

    err = adc_continuous_start(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_start failed: %s", esp_err_to_name(err));
        adc_continuous_deinit(handle);
        goto resume_and_exit;
    }

    /* Drain frames into the HS buffer. */
    size_t   hs_idx     = 0;
    uint8_t  row_filled = 0;
    int64_t  start_us   = esp_timer_get_time();
    cec_capture_hs_sample_t *row = &s_hs_buf[0];

    while (hs_idx < s_hs_capacity) {
        int64_t now = esp_timer_get_time();
        if ((now - start_us) / 1000 >= s_cfg.hs_duration_ms) break;

        uint32_t out_len = 0;
        err = adc_continuous_read(handle, s_dma_scratch, sizeof(s_dma_scratch),
                                  &out_len, 200);
        if (err == ESP_ERR_TIMEOUT) continue;
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "adc_continuous_read: %s", esp_err_to_name(err));
            continue;
        }

        for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= out_len
                              && hs_idx < s_hs_capacity;
             i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)(s_dma_scratch + i);
            uint32_t chan = p->type2.channel;
            uint32_t raw  = p->type2.data;

            int mv = 0;
            if (cali) {
                if (adc_cali_raw_to_voltage(cali, raw, &mv) != ESP_OK) {
                    mv = (int)((raw * 3100u) / 4095u);
                }
            } else {
                mv = (int)((raw * 3100u) / 4095u);
            }

            int cidx = channel_index((adc_channel_t)chan);
            if (cidx < 0) continue;

            float amps = convert_to_amps(&s_cfg.channels[cidx], mv);
            if (cidx == 0) {
                row->eps1_a = amps;
                row_filled |= 0x1;
            } else if (cidx == 1) {
                row->eps2_a = amps;
                row_filled |= 0x2;
            }

            if (row_filled == ((1u << s_cfg.n_channels) - 1u)) {
                /* Synthesize a clean evenly-spaced timestamp from the
                 * row index + nominal sample period. The actual DMA
                 * delivery time can jitter on frame boundaries; the
                 * sample clock itself is hardware-precise. */
                row->ts_us_offset = (uint32_t)((1000000ull * hs_idx) /
                                               (uint64_t)s_cfg.hs_sample_rate_hz);
                hs_idx++;
                row_filled = 0;
                row = &s_hs_buf[hs_idx];
            }
        }
    }

    adc_continuous_stop(handle);
    adc_continuous_deinit(handle);
    *out_count = hs_idx;

resume_and_exit:
    /* Always try to put ADC1 back even if the capture above fell over,
     * otherwise the steady-state sample task is wedged for good. */
    {
        esp_err_t rret = cec_adc_resume();
        if (rret != ESP_OK) {
            ESP_LOGE(TAG, "cec_adc_resume failed: %s", esp_err_to_name(rret));
            if (err == ESP_OK) err = rret;
        }
    }
    return err;
}

static void dump_burst(cec_trigger_t reason, const char *annotation,
                       size_t pre_start, size_t pre_count, size_t hs_count)
{
    /* Burst dump goes through teleplot_write_raw, which routes to the
     * UART transport when cec_telemetry_init_uart() has succeeded and
     * falls back to stdio (printf) otherwise. snprintf -> bulk write is
     * meaningfully faster than line-at-a-time printf for either path:
     * one stdio/VFS round trip (or one uart_write_bytes) per sample
     * instead of seven for pre-trigger / two for HS. */

#define DUMP_YIELD_EVERY 64
    /* A pre-trigger sample formats into ~280 bytes (8 lines * ~35).
     * An HS row into ~70. 384 is a safe stack budget for both. */
    char chunk[384];
    int  n;

    /* TelePlot CSV exporter drops rows whose value field isn't numeric,
     * so the human-readable >BURST_BEGIN line below (3 colons + a name
     * like "anomaly" in the value slot) doesn't survive a CSV save.
     * Emit a parallel numeric series first - >burst_begin:<ts>:<reason>
     * with reason as the cec_trigger_t int (1..9) so each burst plots
     * as a step pulse the analysis tooling can latch onto. >burst_end
     * uses value 0 to differentiate the closing marker. */
    uint32_t begin_ts_ms = (uint32_t)(esp_timer_get_time() / 1000);
    n = snprintf(chunk, sizeof(chunk),
                 ">burst_begin:%" PRIu32 ":%u\n",
                 begin_ts_ms, (unsigned)reason);
    if (n > 0) teleplot_write_raw(chunk, (size_t)n);

    /* Human-readable envelope for raw-stream consumers (idf.py monitor,
     * the 24-pin's capture-analysis tooling). Kept as-is for parity. */
    n = snprintf(chunk, sizeof(chunk),
                 ">BURST_BEGIN:%s:%u_normal+%u_hs:cap\n",
                 cec_trigger_name(reason),
                 (unsigned)pre_count, (unsigned)hs_count);
    if (n > 0) teleplot_write_raw(chunk, (size_t)n);
    if (annotation && annotation[0]) {
        n = snprintf(chunk, sizeof(chunk), ">BURST_ANNOTATION:%s\n", annotation);
        if (n > 0) teleplot_write_raw(chunk, (size_t)n);
    }

    int decim = s_cfg.hs_dump_decimation > 0 ? s_cfg.hs_dump_decimation : 1;

    /* Pre-trigger: walk oldest -> newest. */
    for (size_t k = 0; k < pre_count; k++) {
        size_t idx = (pre_start + k) % s_pre_capacity;
        const cec_capture_sample_t *s = &s_pre_buf[idx];
        n = snprintf(chunk, sizeof(chunk),
                     ">b_eps1_a:%" PRIu32 ":%.6f\n"
                     ">b_eps2_a:%" PRIu32 ":%.6f\n"
                     ">b_eps1_raw_a:%" PRIu32 ":%.6f\n"
                     ">b_eps2_raw_a:%" PRIu32 ":%.6f\n"
                     ">b_bus_v:%" PRIu32 ":%.6f\n"
                     ">b_temp_c:%" PRIu32 ":%.6f\n"
                     ">b_load:%" PRIu32 ":%u\n"
                     ">b_flags:%" PRIu32 ":%u\n",
                     s->ts_ms, s->eps1_a,
                     s->ts_ms, s->eps2_a,
                     s->ts_ms, s->eps1_raw_a,
                     s->ts_ms, s->eps2_raw_a,
                     s->ts_ms, s->bus_voltage_v,
                     s->ts_ms, s->temp_c,
                     s->ts_ms, (unsigned)s->load_state,
                     s->ts_ms, (unsigned)s->flags);
        if (n > 0) teleplot_write_raw(chunk, (size_t)n);
        if ((k & (DUMP_YIELD_EVERY - 1)) == (DUMP_YIELD_EVERY - 1)) {
            vTaskDelay(1);
        }
    }

    /* HS samples: timestamps are microseconds-since-capture-start.
     * Decimation thins the dump without thinning the capture. */
    for (size_t i = 0; i < hs_count; i += (size_t)decim) {
        const cec_capture_hs_sample_t *h = &s_hs_buf[i];
        n = snprintf(chunk, sizeof(chunk),
                     ">hs_eps1_a:%" PRIu32 ":%.6f\n"
                     ">hs_eps2_a:%" PRIu32 ":%.6f\n",
                     h->ts_us_offset, h->eps1_a,
                     h->ts_us_offset, h->eps2_a);
        if (n > 0) teleplot_write_raw(chunk, (size_t)n);
        if (((i / (size_t)decim) & (DUMP_YIELD_EVERY - 1)) == (DUMP_YIELD_EVERY - 1)) {
            vTaskDelay(1);
        }
    }

    /* CSV-friendly numeric end marker, paired with >burst_begin above.
     * Value 0 distinguishes "end" from the reason int that >burst_begin
     * emits (1..9). Plotted together the two series produce a step
     * pulse that brackets each burst in TelePlot's CSV. */
    uint32_t end_ts_ms = (uint32_t)(esp_timer_get_time() / 1000);
    n = snprintf(chunk, sizeof(chunk),
                 ">burst_end:%" PRIu32 ":0\n", end_ts_ms);
    if (n > 0) teleplot_write_raw(chunk, (size_t)n);

    n = snprintf(chunk, sizeof(chunk), ">BURST_END\n");
    if (n > 0) teleplot_write_raw(chunk, (size_t)n);
}

static void dispatch_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "dispatch task started on core %d", xPortGetCoreID());

    while (1) {
        if (xSemaphoreTake(s_trigger_sem, portMAX_DELAY) != pdTRUE) continue;

        cec_trigger_t reason = s_pending_reason;
        char annotation[ANNOTATION_MAX];
        memcpy(annotation, s_pending_annotation, ANNOTATION_MAX);
        annotation[ANNOTATION_MAX - 1] = '\0';

        /* Snapshot the pre-trigger ring positions BEFORE the HS run
         * so the dump reflects the state at trigger time, not the
         * end of capture. Pre-trigger writes from sample_task during
         * the burst window will fall on top of the oldest entries -
         * the snapshot keeps them out of this dump. */
        size_t pre_count_snap = s_pre_count;
        size_t pre_start_snap = (pre_count_snap == s_pre_capacity) ?
                                 s_pre_write_idx : 0;

        size_t hs_count = 0;
        esp_err_t r = run_hs_capture(&hs_count);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "HS capture failed: %s", esp_err_to_name(r));
        }

        /* HS capture is done; the dump phase is just blocking printfs
         * and doesn't need the high DMA-consumer priority. Drop to
         * priority 1 so output (3) preempts freely and IDLE1 (0) gets
         * proper slices via the in-loop vTaskDelay yields. Restore
         * before sleeping so the next HS run has its priority back. */
        UBaseType_t old_prio = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 1);
        dump_burst(reason, annotation, pre_start_snap, pre_count_snap, hs_count);
        vTaskPrioritySet(NULL, old_prio);

        s_last_complete_us = esp_timer_get_time();
        s_pending_reason = CEC_TRIG_NONE;
        s_pending_annotation[0] = '\0';
        s_busy = false;
    }
}

esp_err_t cec_capture_init(const cec_capture_config_t *cfg)
{
    if (s_inited) return ESP_OK;
    if (cfg == NULL || cfg->pre_trigger_capacity <= 0 ||
        cfg->hs_sample_rate_hz <= 0 || cfg->hs_duration_ms <= 0 ||
        cfg->n_channels <= 0 || cfg->n_channels > CEC_NUM_CABLES) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_cfg, cfg, sizeof(s_cfg));
    s_pre_capacity = (size_t)cfg->pre_trigger_capacity;

    /* HS capacity = rate * duration, with one extra row of slack. */
    s_hs_capacity = (size_t)((cfg->hs_sample_rate_hz *
                              (long long)cfg->hs_duration_ms) / 1000);

    /* Allocate buffers in PSRAM, fall back to internal heap if missing. */
    size_t pre_bytes = s_pre_capacity * sizeof(cec_capture_sample_t);
    size_t hs_bytes  = s_hs_capacity  * sizeof(cec_capture_hs_sample_t);

    s_pre_buf = heap_caps_calloc(s_pre_capacity, sizeof(cec_capture_sample_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_pre_buf == NULL) {
        s_pre_buf = heap_caps_calloc(s_pre_capacity, sizeof(cec_capture_sample_t),
                                      MALLOC_CAP_8BIT);
    }
    s_hs_buf = heap_caps_calloc(s_hs_capacity, sizeof(cec_capture_hs_sample_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_hs_buf == NULL) {
        s_hs_buf = heap_caps_calloc(s_hs_capacity, sizeof(cec_capture_hs_sample_t),
                                     MALLOC_CAP_8BIT);
    }
    if (s_pre_buf == NULL || s_hs_buf == NULL) {
        ESP_LOGE(TAG, "buffer alloc failed (pre=%u bytes, hs=%u bytes)",
                 (unsigned)pre_bytes, (unsigned)hs_bytes);
        if (s_pre_buf) { heap_caps_free(s_pre_buf); s_pre_buf = NULL; }
        if (s_hs_buf)  { heap_caps_free(s_hs_buf);  s_hs_buf  = NULL; }
        return ESP_ERR_NO_MEM;
    }

    s_trigger_sem = xSemaphoreCreateBinary();
    if (s_trigger_sem == NULL) {
        heap_caps_free(s_pre_buf); s_pre_buf = NULL;
        heap_caps_free(s_hs_buf);  s_hs_buf  = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_pre_write_idx = 0;
    s_pre_count     = 0;
    s_busy          = false;
    s_inited        = true;

    BaseType_t tr = xTaskCreatePinnedToCore(dispatch_task, "cec_burst",
                                            DISPATCH_TASK_STACK, NULL,
                                            DISPATCH_TASK_PRIORITY,
                                            &s_dispatch_task,
                                            DISPATCH_TASK_CORE_ID);
    if (tr != pdPASS) {
        s_inited = false;
        vSemaphoreDelete(s_trigger_sem); s_trigger_sem = NULL;
        heap_caps_free(s_pre_buf); s_pre_buf = NULL;
        heap_caps_free(s_hs_buf);  s_hs_buf  = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ready: pre=%u samples (%u bytes), hs=%u samples (%u bytes), "
                  "hs_rate=%d Hz/ch, hs_window=%d ms, cooldown=%d ms",
             (unsigned)s_pre_capacity, (unsigned)pre_bytes,
             (unsigned)s_hs_capacity,  (unsigned)hs_bytes,
             cfg->hs_sample_rate_hz, cfg->hs_duration_ms, cfg->cooldown_ms);
    return ESP_OK;
}

static esp_err_t enqueue_trigger(cec_trigger_t reason, const char *text)
{
    if (!s_inited)              return ESP_ERR_INVALID_STATE;
    if (s_busy)                 return ESP_ERR_NOT_FINISHED;

    /* Cooldown gate. SHUTDOWN bypasses cooldown so a real shutdown is
     * never missed; parity with the 24-pin. */
    if (reason != CEC_TRIG_SHUTDOWN && s_cfg.cooldown_ms > 0) {
        int64_t since = esp_timer_get_time() - s_last_complete_us;
        if (since < (int64_t)s_cfg.cooldown_ms * 1000) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    s_pending_reason = reason;
    if (text) {
        strncpy(s_pending_annotation, text, ANNOTATION_MAX - 1);
        s_pending_annotation[ANNOTATION_MAX - 1] = '\0';
    } else {
        s_pending_annotation[0] = '\0';
    }
    s_busy = true;
    xSemaphoreGive(s_trigger_sem);
    return ESP_OK;
}

esp_err_t cec_capture_set_hs_dump_decimation(int decim)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (decim < 1) decim = 1;
    s_cfg.hs_dump_decimation = decim;
    return ESP_OK;
}

int cec_capture_get_hs_dump_decimation(void)
{
    return s_inited ? s_cfg.hs_dump_decimation : 0;
}

esp_err_t cec_capture_update_channel(int idx, const cec_capture_channel_t *p)
{
    if (!s_inited)                return ESP_ERR_INVALID_STATE;
    if (p == NULL)                return ESP_ERR_INVALID_ARG;
    if (idx < 0 || idx >= s_cfg.n_channels) return ESP_ERR_INVALID_ARG;

    /* The .channel itself is fixed at init (we configured ADC pattern
     * against it); refresh only the conversion params. */
    s_cfg.channels[idx].divider_gain    = p->divider_gain;
    s_cfg.channels[idx].quiescent_v     = p->quiescent_v;
    s_cfg.channels[idx].sensitivity_v_a = p->sensitivity_v_a;
    s_cfg.channels[idx].zero_offset_v   = p->zero_offset_v;
    return ESP_OK;
}

esp_err_t cec_capture_trigger(cec_trigger_t reason)
{
    return enqueue_trigger(reason, NULL);
}

esp_err_t cec_capture_trigger_with_text(cec_trigger_t reason, const char *text)
{
    return enqueue_trigger(reason, text);
}
