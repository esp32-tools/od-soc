#pragma once

// Thin C99 API for pure-C projects using esp32-tools/od-soc.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t flags;
    uint16_t ocv_mv[17];
    uint16_t q_max_dmah;
    uint16_t r0_mohm;
    uint16_t r1_mohm;
    uint16_t tau_s;
    uint16_t v_full_mv;
    uint8_t i_term_ma;
    uint8_t chg_eff_sub900;
    uint16_t crc16;
} od_soc_params_t;

typedef struct {
    uint16_t v_mv;
    int16_t i_ma;
    int32_t dq_uah;
    uint16_t dt_s;
    uint8_t flags;
    uint8_t reserved;
} od_soc_sample_t;

typedef struct {
    uint32_t z_q16;
    int32_t v_rc_q4;
    uint32_t p_q18;
    int16_t i_prev_ma;
    uint16_t sample_count;
} od_soc_state_t;

void od_soc_seed_from_vi(od_soc_state_t* st, const od_soc_params_t* params, uint16_t v_mv, int16_t i_ma);
uint16_t od_soc_step(od_soc_state_t* st, const od_soc_params_t* params, const od_soc_sample_t* sample);
bool od_soc_validate_params(const od_soc_params_t* params);

#ifdef __cplusplus
}
#endif
