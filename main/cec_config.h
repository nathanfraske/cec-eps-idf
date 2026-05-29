#pragma once

#include "cec_state.h"
#include "cec_layer3.h"
#include "esp_err.h"
#include <stdio.h>

void      cec_config_defaults(cec_config_t *cfg);
esp_err_t cec_config_init_nvs(void);
void      cec_config_load(cec_config_t *cfg);
void      cec_config_save(const cec_config_t *cfg);
void      cec_config_save_zero_offset(int sensor, float offset_v);
bool      cec_config_load_zero_offset(int sensor, float *offset_v);

/*
 * Layer 3 rail-profile persistence. The detection ctx holds the live
 * profiles in cec_rail_profile_t form; cec_config saves a snapshot
 * blob (one entry per cable) so the learned baseline survives reboots
 * and the warm-up window doesn't have to be paid every power cycle.
 */
void cec_config_save_l3_profiles(const cec_rail_profile_t profiles[CEC_NUM_CABLES]);
bool cec_config_load_l3_profiles(cec_rail_profile_t profiles[CEC_NUM_CABLES]);
