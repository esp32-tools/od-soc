#include "od_soc/od_soc_c.h"
#include "od_soc/od_soc.hpp"

static_assert(sizeof(od_soc_params_t) == sizeof(od_soc::CellParams), "C/C++ params ABI mismatch");
static_assert(sizeof(od_soc_sample_t) == sizeof(od_soc::Sample), "C/C++ sample ABI mismatch");
static_assert(sizeof(od_soc_state_t) == sizeof(od_soc::OdSocState), "C/C++ state ABI mismatch");

extern "C" {

void od_soc_seed_from_vi(od_soc_state_t* st, const od_soc_params_t* params, uint16_t v_mv, int16_t i_ma) {
    od_soc::seed_from_vi(*reinterpret_cast<od_soc::OdSocState*>(st),
                         *reinterpret_cast<const od_soc::CellParams*>(params), v_mv, i_ma);
}

uint16_t od_soc_step(od_soc_state_t* st, const od_soc_params_t* params, const od_soc_sample_t* sample) {
    return od_soc::step(*reinterpret_cast<od_soc::OdSocState*>(st),
                        *reinterpret_cast<const od_soc::CellParams*>(params),
                        *reinterpret_cast<const od_soc::Sample*>(sample));
}

bool od_soc_validate_params(const od_soc_params_t* params) {
    return od_soc::validate_params(*reinterpret_cast<const od_soc::CellParams*>(params));
}

}
