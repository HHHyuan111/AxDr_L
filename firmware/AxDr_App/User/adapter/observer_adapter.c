/**
 * @file observer_adapter.c
 * @brief 在线磁链观测器与现有 FOC 对象之间的数据适配。
 */

#include "observer_adapter.h"

#include <math.h>

#include "algorithm_config.h"
#include "common.h"
#include "compiler.h"

obs_t g_obs;

void obs_init(const foc_t *foc)
{
    g_obs.cfg = (mc_flux_config_t){
        .rs_ohm = foc->motor.Rs,
        .pole_pairs = foc->motor.pn,
        .sample_time_s = foc->rate.foc_ts,
        .filter_cutoff_hz = DIAG_FLUX_FILTER_CUTOFF_HZ,
        .minimum_electrical_speed_rad_s = DIAG_FLUX_MIN_ELEC_SPEED_RAD_S,
        .reject_voltage_saturation = DIAG_FLUX_REJECT_VOLTAGE_SATURATION,
    };
    mc_flux_observer_init(&g_obs.flux);
    g_obs.status = MC_REJECTED;
}

PLATFORM_FAST_CODE void obs_step(const foc_t *foc)
{
    mc_sample_t sample;

    if (!foc->pwm_active)
    {
        return;
    }

    sample = (mc_sample_t){
        .id_a = foc->sig.i_d,
        .iq_a = foc->sig.i_q,
        .vd_v = foc->sig.v_d,
        .vq_v = foc->sig.v_q,
        .omega_mech_rad_s = foc->sig.wr_f,
        .voltage_saturated = (foc->sig.vs > 0.0f)
            && (hypotf(foc->sig.v_d, foc->sig.v_q) >= (0.999f * foc->sig.vs)),
    };

    g_obs.status = mc_flux_observer_step(&g_obs.flux, &g_obs.cfg, &sample);
}
