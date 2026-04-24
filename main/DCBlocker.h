#pragma once

#include <cmath>

class DCBlocker
{
private:
    float dc_estimate = NAN;
    float alpha;

public:
    DCBlocker(float alpha_val = 0.001f) {
        alpha = alpha_val;
    }

    // Feed it a raw millivolt sample, get back a perfectly centered AC sample
    float process(float raw_mv_sample) {
        // 1. Update the running average using Exponentially Weighted Moving Average (EWMA) filter (track the drift)
        dc_estimate = dc_estimate + alpha * (raw_mv_sample - dc_estimate);
        
        // 2. Subtract the DC estimate to center the wave at 0V
        return raw_mv_sample - dc_estimate;
    }

    // Use for ad-hoc estimation
    void set_current_dc_bias(float value) {
        dc_estimate = value;
    }
    
    // Optional: useful for debugging if your 3.3V rail is sagging
    float get_current_dc_bias() const {
        return dc_estimate;
    }
};
