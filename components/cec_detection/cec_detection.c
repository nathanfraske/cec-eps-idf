/*
 * Top-level detection orchestrator.
 *
 * Layer split mirrors the 24-pin module:
 *   cec_layer1     - static threshold (overcurrent + dropout)
 *   cec_layer2     - fast transient (dI/dt for EPS)
 *   cec_layer3     - rail profile (mean + std, z-score for anomaly)
 *   cec_classifier - load-state classifier
 *
 * Layer 3 z-score (|z| > LAYER3_Z_THRESHOLD) folds into
 * CEC_FLAG_ANOMALY so a sustained drift from the learned baseline
 * fires a burst with reason CEC_TRIG_ANOMALY. The classifier still
 * pulls std_dev as a noise gauge.
 */

#include <math.h>
#include "cec_detection.h"

/* Detector-tuning defaults. */
#define LAYER1_CRIT_REQUIRED        3
#define LAYER1_DROPOUT_FLOOR_A      0.5f
#define LAYER2_THRESHOLD_A_PER_MS   1.0f
/* Layer 3 adapt rate. 0.0005 at 50 Hz gives a ~2000-sample (40 s)
 * effective averaging window once warm, matching the 24-pin's value. */
#define LAYER3_ADAPT_RATE           0.0005f
/* z-score threshold. The 24-pin's v0.5.9 uses 4.0 - a >4 sigma
 * deviation from learned-normal is the anomaly trigger. */
#define LAYER3_Z_THRESHOLD          4.0f

void cec_detection_init(cec_detection_ctx_t *ctx, float oc_threshold_a)
{
    for (int i = 0; i < CEC_NUM_CABLES; i++) {
        cec_layer1_init(&ctx->l1[i],
                        oc_threshold_a, oc_threshold_a,
                        LAYER1_DROPOUT_FLOOR_A,
                        LAYER1_CRIT_REQUIRED);
        cec_layer2_init(&ctx->l2[i], LAYER2_THRESHOLD_A_PER_MS);
        cec_rail_profile_init(&ctx->l3[i]);
    }
    ctx->layer1_enabled = true;
    ctx->layer2_enabled = true;
    ctx->layer3_enabled = true;
}

void cec_detection_set_layer_enabled(cec_detection_ctx_t *ctx, int layer, bool enabled)
{
    if (ctx == NULL) return;
    switch (layer) {
    case 1: ctx->layer1_enabled = enabled; break;
    case 2: ctx->layer2_enabled = enabled; break;
    case 3: ctx->layer3_enabled = enabled; break;
    default: break;
    }
}

bool cec_detection_run(cec_detection_ctx_t *ctx,
                       const float current_raw[CEC_NUM_CABLES],
                       const float current_filt[CEC_NUM_CABLES],
                       int64_t now_us,
                       uint8_t *out_flags,
                       cec_load_state_t *out_state)
{
    uint8_t flags = 0;
    float   max_std     = 0.0f;
    float   max_current = 0.0f;

    for (int i = 0; i < CEC_NUM_CABLES; i++) {
        /* Layer 1: severity-graded threshold on the filtered value.
         * Even when the layer is disabled we still update its internal
         * debounce counter (so the next enable doesn't see a stale
         * crit_consecutive). The flag contribution is gated. */
        cec_severity_t sev = cec_layer1_update(&ctx->l1[i], current_filt[i]);
        bool dropout = cec_layer1_check_dropout(&ctx->l1[i], current_filt[i]);
        if (ctx->layer1_enabled) {
            if (sev == CEC_SEV_CRITICAL) flags |= CEC_FLAG_OVERCURRENT;
            if (dropout)                 flags |= CEC_FLAG_DROPOUT;
        }

        /* Layer 2: rate-of-change on the raw stream (fast transients). */
        bool swing = cec_layer2_update(&ctx->l2[i], current_raw[i], now_us);
        if (ctx->layer2_enabled && swing) {
            flags |= CEC_FLAG_SWING;
        }

        /* Layer 3: rail profile. Update unconditionally so the baseline
         * keeps tracking even with the layer's flag contribution gated;
         * a runtime disable just stops the z-score from triggering a
         * burst, it doesn't freeze the learned mean/std. */
        cec_rail_profile_update(&ctx->l3[i], current_filt[i], LAYER3_ADAPT_RATE);
        float z = cec_rail_profile_z_score(&ctx->l3[i], current_filt[i]);
        if (ctx->layer3_enabled && fabsf(z) > LAYER3_Z_THRESHOLD) {
            flags |= CEC_FLAG_ANOMALY;
        }

        if (ctx->l3[i].std_dev > max_std)  max_std     = ctx->l3[i].std_dev;
        if (current_filt[i]    > max_current) max_current = current_filt[i];
    }

    *out_state = cec_classify_load(max_current, max_std);
    *out_flags = flags;
    return (flags != 0);
}

cec_trigger_t cec_trigger_for_flags(uint8_t flags)
{
    /* Priority: most severe / most specific first.
     * SHUTDOWN isn't represented as a flag (it's a separate, bypass-
     * cooldown detection on the bus voltage rail-of-change) so it
     * doesn't appear in this map. */
    if (flags & CEC_FLAG_OVERCURRENT) return CEC_TRIG_STATIC_CRIT;
    if (flags & CEC_FLAG_DROPOUT)     return CEC_TRIG_STATIC_CRIT;
    if (flags & CEC_FLAG_ANOMALY)     return CEC_TRIG_ANOMALY;
    if (flags & CEC_FLAG_SWING)       return CEC_TRIG_CURRENT_SWING;
    if (flags & CEC_FLAG_FAULT)       return CEC_TRIG_ANOMALY;
    return CEC_TRIG_ANOMALY;  /* fallback for non-zero flags with no map */
}
