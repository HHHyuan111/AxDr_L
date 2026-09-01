#include "mc_bias_bandwidth.h"

#include <math.h>
#include <string.h>

#define MC_MINUS_3DB (3.01029995664f)

mc_status_t mc_closed_loop_bandwidth(const mc_current_sweep_result_t *sweep,
                                     bool reject_saturated_points,
                                     float *bandwidth_hz)
{
    uint32_t base_indices[3];
    uint32_t base_count = 0u;
    uint32_t i;
    float base_db = 0.0f;
    float target;

    if (!sweep || !bandwidth_hz || sweep->completed_points < 2u ||
        sweep->completed_points > MC_SWEEP_MAX_POINTS)
        return MC_INVALID_ARGUMENT;
    for (i = 0u; i < sweep->completed_points && base_count < 3u; ++i) {
        if (!sweep->valid[i]) continue;
        if (reject_saturated_points && sweep->saturated[i]) continue;
        if (!mc_float_is_finite(sweep->frequency_hz[i]) ||
            !mc_float_is_finite(sweep->closed_magnitude_db[i]) ||
            sweep->frequency_hz[i] <= 0.0f) continue;
        base_indices[base_count++] = i;
        base_db += sweep->closed_magnitude_db[i];
    }
    if (base_count < 2u) return MC_REJECTED;
    base_db /= (float)base_count;
    if (!mc_float_is_finite(base_db)) return MC_NUMERIC_ERROR;
    target = base_db - MC_MINUS_3DB;

    for (i = base_indices[0] + 1u; i < sweep->completed_points; ++i) {
        uint32_t p = i - 1u;
        float previous;
        float current;
        float fraction;
        if (!sweep->valid[p] || !sweep->valid[i]) continue;
        if (reject_saturated_points && (sweep->saturated[p] || sweep->saturated[i])) continue;
        if (!mc_float_is_finite(sweep->frequency_hz[p]) ||
            !mc_float_is_finite(sweep->frequency_hz[i]) ||
            !mc_float_is_finite(sweep->closed_magnitude_db[p]) ||
            !mc_float_is_finite(sweep->closed_magnitude_db[i]) ||
            sweep->frequency_hz[p] <= 0.0f || sweep->frequency_hz[i] <= 0.0f)
            continue;
        previous = sweep->closed_magnitude_db[p];
        current = sweep->closed_magnitude_db[i];
        if (!(previous > target && current <= target)) continue;
        if (fabsf(current - previous) < 1e-6f) {
            *bandwidth_hz = sweep->frequency_hz[i];
        } else {
            fraction = mc_clampf((target - previous) / (current - previous), 0.0f, 1.0f);
            *bandwidth_hz = expf(logf(sweep->frequency_hz[p]) + fraction *
                (logf(sweep->frequency_hz[i]) - logf(sweep->frequency_hz[p])));
        }
        return mc_float_is_finite(*bandwidth_hz) && *bandwidth_hz > 0.0f ?
               MC_OK : MC_NUMERIC_ERROR;
    }
    return MC_REJECTED;
}

mc_status_t mc_bias_bandwidth_build(const float *bias_current_a,
                                    const mc_current_sweep_result_t *sweeps,
                                    uint32_t count,
                                    float initial_kp,
                                    bool reject_saturated_points,
                                    mc_bias_bandwidth_result_t *result)
{
    uint32_t i;
    float base_bandwidth = 0.0f;
    if (!bias_current_a || !sweeps || !result || count == 0u ||
        count > MC_BIAS_MAX_POINTS || !mc_float_is_finite(initial_kp) ||
        initial_kp <= 0.0f) return MC_INVALID_ARGUMENT;
    memset(result, 0, sizeof(*result));
    result->point_count = count;
    for (i = 0u; i < count; ++i) {
        float bw = 0.0f;
        if (!mc_float_is_finite(bias_current_a[i])) continue;
        result->bias_current_a[i] = bias_current_a[i];
        if (mc_closed_loop_bandwidth(&sweeps[i], reject_saturated_points, &bw) != MC_OK)
            continue;
        if (base_bandwidth <= 0.0f) base_bandwidth = bw;
        result->bandwidth_hz[i] = bw;
        result->bandwidth_ratio[i] = bw / base_bandwidth;
        result->suggested_kp[i] = initial_kp / result->bandwidth_ratio[i];
        if (!mc_float_is_finite(result->bandwidth_ratio[i]) ||
            !mc_float_is_finite(result->suggested_kp[i]) ||
            result->bandwidth_ratio[i] <= 0.0f) continue;
        result->valid[i] = 1u;
    }
    return base_bandwidth > 0.0f ? MC_OK : MC_REJECTED;
}




