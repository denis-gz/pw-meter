#pragma once

#include <array>
#include <cmath>


class EMAFilter
{
private:
    float current_value;
    float alpha;

public:
    EMAFilter(float alpha_val = 0.2f) {
        alpha = alpha_val;
        current_value = NAN;
    }

    float process(float new_reading) {
        // If this is the very first reading, snap instantly to it 
        // instead of slowly ramping up from 0
        if (std::isnan(current_value)) {
            current_value = new_reading;
            return current_value;
        }

        // Apply the EMA smoothing
        current_value = current_value + alpha * (new_reading - current_value);
        return current_value;
    }
    
    // Quick way to grab the value without updating it
    float get() const {
        return current_value;
    }
};


class AdaptiveDisplayFilter
{
private:
    float current_value;
    float variance;
    float alpha;
    float sigma_threshold;
    float absolute_jump_threshold;

public:
    // Initialize with NAN. Default to a 20-sigma jump threshold.
    AdaptiveDisplayFilter(float absolute_limit, float sigma_limit = 20.0f, float alpha_val = 0.45f) {
        absolute_jump_threshold = absolute_limit;
        sigma_threshold = sigma_limit;
        alpha = alpha_val;
        reset();
    }

    float process(float new_reading) {
        if (std::isnan(current_value)) {
            current_value = new_reading;
            return current_value;
        }

        float diff = new_reading - current_value;
        variance = variance + alpha * ((diff * diff) - variance);
        float sigma = std::sqrt(variance);

        // Trigger the snap if it exceeds the statistical noise (20 sigma)
        // OR if it exceeds a hardcoded absolute difference.

        bool is_statistical_jump = std::abs(diff) > (sigma_threshold * sigma);
        bool is_absolute_jump = std::abs(diff) > absolute_jump_threshold;

        if (is_statistical_jump || is_absolute_jump) {
            current_value = new_reading; // Snap!
            variance = diff * diff;      // Reset variance to new baseline
        }
        else {
            // With alpha = 0.45, this will settle smoothly in exactly 1 second
            current_value = current_value + alpha * diff;
        }

        return current_value;
    }

    void reset() {
        current_value = NAN;
        variance = 0.0f;
    }

    float get() const {
        return current_value;
    }
};

/*
template<size_t SIZE>
class AdaptiveSMAFilter
{
private:
    float current_value;
    float sigma_threshold;

    float sum;
    std::array<float, SIZE> samples;
    size_t last_index;

public:
    // Initialize with NAN. Default to a 20-sigma jump threshold.
    AdaptiveSMAFilter(float sigma_limit = 20.0f) {
        sigma_threshold = sigma_limit;
        current_value = NAN;
        last_index = 0;
        sum = 0;
    }

    float process(float new_reading) {

        samples[last_index % SIZE] = new_reading;
        int old_sample = samples[++last_index % SIZE];
        float count = last_index < SIZE ? last_index : SIZE;

        // Calculate mean (average)
        sum += new_reading;
        float avg = sum / count;
        sum -= old_sample;

        // Calculate standard deviation
        float sigma = 0;
        if (count > 1) {
            for (int j = 0; j < count; ++j) {
                float s = samples[j] - avg;
                sigma += s * s;
            }
            sigma = std::sqrt(sigma / (count - 1));
        }

        // Detect signal change, reset filter
        if (sigma > sigma_threshold && last_index > 3) {
            avg = new_reading;
            samples.fill(0);
            sum = 0;
            last_index = 0;
        }

        current_value = avg;

        return current_value;
    }

    float get() const {
        return current_value;
    }
};
*/
