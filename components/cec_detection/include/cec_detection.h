/*
 * Top-level detection orchestrator.
 *
 * Owns one cec_layer1/2/3 detector per cable and runs them on every
 * sample. Layers fire independently; the result is folded into the
 * CEC_FLAG_* status_flags bitfield and a cec_load_state_t load
 * classification.
 *
 * Individual layers can be silenced at runtime via the layer_enabled
 * flags in the ctx (default all on). cec_capture trigger reason for a
 * given flag set is picked by cec_trigger_for_flags so the burst dump
 * envelope carries the actual cause, not always CEC_TRIG_ANOMALY.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cec_state.h"
#include "cec_layer1.h"
#include "cec_layer2.h"
#include "cec_layer3.h"
#include "cec_classifier.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    cec_layer1_detector_t l1[CEC_NUM_CABLES];   /* current threshold */
    cec_layer2_detector_t l2[CEC_NUM_CABLES];   /* fast transient (dI/dt) */
    cec_rail_profile_t    l3[CEC_NUM_CABLES];   /* mean+std rail profile */

    /* Runtime layer enables. Default true on init; flip via
     * cec_detection_set_layer_enabled. A disabled layer is updated but
     * never contributes flags. */
    bool layer1_enabled;
    bool layer2_enabled;
    bool layer3_enabled;
} cec_detection_ctx_t;

/*
 * Initialize all detectors. The OC ceiling comes from cec_config; the
 * warn band is set to the same value initially (single-tier) and can
 * be lowered later for a sub-critical warning. All three layers start
 * enabled.
 */
void cec_detection_init(cec_detection_ctx_t *ctx, float oc_threshold_a);

/*
 * Enable / disable an individual layer at runtime. layer is 1, 2, or 3.
 */
void cec_detection_set_layer_enabled(cec_detection_ctx_t *ctx, int layer, bool enabled);

/*
 * Run all layers on one sample set. Folds Layer 1 severities, Layer 2
 * transient hits, and Layer 3 z-score anomalies into out_flags;
 * classifies the load level using max-current + max-std into out_state.
 *
 * Returns true if any layer fired (callers may trigger burst capture).
 */
bool cec_detection_run(cec_detection_ctx_t *ctx,
                       const float current_raw[CEC_NUM_CABLES],
                       const float current_filt[CEC_NUM_CABLES],
                       int64_t now_us,
                       uint8_t *out_flags,
                       cec_load_state_t *out_state);

/*
 * Pick the most-severe cec_trigger_t for a given out_flags bitmask.
 * Used by sample_task to thread the actual cause into the burst dump's
 * BURST_BEGIN envelope rather than always reporting "anomaly".
 */
cec_trigger_t cec_trigger_for_flags(uint8_t flags);

#ifdef __cplusplus
}
#endif
