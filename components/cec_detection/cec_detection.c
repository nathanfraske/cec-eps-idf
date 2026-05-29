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
 * CEC_FLAG_ANOMALY after LAYER3_REQUIRED consecutive over-threshold
 * samples (debounce, mirroring cec_layer1's crit_required). The
 * classifier still pulls std_dev as a noise gauge.
 *
 * Adapt coupling: the rail profile is FROZEN (not updated toward the
 * sample) while a warm sample is over threshold. Without this, a
 * sustained transient slowly drags the learned mean toward the bad
 * value, which pulls |z| back under threshold and produces the
 * on/off/on/off flag chatter seen on the bench - and would also let
 * the mean catch up fast enough to defeat the debounce. Freezing
 * during the anomaly keeps |z| solidly above threshold so the
 * debounce latches once and the flag stays cleanly asserted for the
 * duration. This mirrors the 24-pin cec_layer2's documented "variance
 * estimator only updates on calm samples" behavior.
 *
 * Tradeoff: EPS has one profile per cable (no per-(state,rail) profile
 * bank like the 24-pin), so a genuinely sustained shift to a new
 * operating level keeps ANOMALY asserted rather than relearning it as
 * the new normal. The burst cooldown throttles captures to one per
 * window regardless, and the load classifier separately reports the
 * magnitude bucket. Per-load-state profiles (relearn after a
 * sustained shift) are a deferred enhancement.
 */

#include <math.h>
#include "cec_detection.h"

/* Detector-tuning defaults. */
#define LAYER1_CRIT_REQUIRED        3
#define LAYER1_DROPOUT_FLOOR_A      0.5f
#define LAYER2_THRESHOLD_A_PER_MS   1.0f
/* Layer 3 adapt rate. 0.0005 at 50 Hz gives a ~2000-sample (40 s)
 * effective averaging window once warm, matching the 24-pin's value.
 * Kept at this value: with the adapt-coupling freeze + debounce below,
 * the slow rate no longer causes false-positive chatter, so there's no
 * need to widen the z threshold or speed up adaptation. */
#define LAYER3_ADAPT_RATE           0.0005f
/* z-score threshold. The 24-pin's v0.5.9 uses 4.0 - a >4 sigma
 * deviation from learned-normal is the anomaly trigger. */
#define LAYER3_Z_THRESHOLD          4.0f
/* Consecutive over-threshold samples before CEC_FLAG_ANOMALY asserts.
 * 3 matches cec_layer1's crit_required; at 50 Hz that's a 60 ms
 * sustain, enough to reject single-sample z spikes. */
#define LAYER3_REQUIRED             3

void cec_detection_init(cec_detection_ctx_t *ctx, float oc_threshold_a)
{
    for (int i = 0; i < CEC_NUM_CABLES; i++) {
        cec_layer1_init(&ctx->l1[i],
                        oc_threshold_a, oc_threshold_a,
                        LAYER1_DROPOUT_FLOOR_A,
                        LAYER1_CRIT_REQUIRED);
        cec_layer2_init(&ctx->l2[i], LAYER2_THRESHOLD_A_PER_MS);
        cec_rail_profile_init(&ctx->l3[i]);
        ctx->l3_consecutive[i] = 0;
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

        /* Layer 3: rail profile + debounced z-score.
         *
         * z is measured against the CURRENT (pre-update) baseline.
         * Before warm, z_score returns 0 so z_over is false and the
         * profile adapts normally through the warm-up window. */
        float z = cec_rail_profile_z_score(&ctx->l3[i], current_filt[i]);
        bool z_over = fabsf(z) > LAYER3_Z_THRESHOLD;

        /* Debounce counter (saturating). Resets the moment the sample
         * is calm so a brief excursion never reaches LAYER3_REQUIRED. */
        if (z_over) {
            if (ctx->l3_consecutive[i] < LAYER3_REQUIRED) ctx->l3_consecutive[i]++;
        } else {
            ctx->l3_consecutive[i] = 0;
        }

        /* Adapt coupling: freeze the profile while a (warm) sample is
         * over threshold so the anomaly can't drag the baseline toward
         * itself. When the layer is disabled, adapt unconditionally so
         * the baseline stays current for a later re-enable. */
        if (!ctx->layer3_enabled || !z_over) {
            cec_rail_profile_update(&ctx->l3[i], current_filt[i], LAYER3_ADAPT_RATE);
        }

        if (ctx->layer3_enabled && ctx->l3_consecutive[i] >= LAYER3_REQUIRED) {
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
