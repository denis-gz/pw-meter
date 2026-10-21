#pragma once

#include <array>
#include <cmath>
#include <algorithm>


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
    AdaptiveDisplayFilter(float absolute_limit, float sigma_limit = 20.0f, float alpha_val = 0.4f) {
        absolute_jump_threshold = absolute_limit;
        sigma_threshold = sigma_limit;
        alpha = alpha_val;
        reset();
    }

    float process(float new_reading, int* rapid_change = nullptr) {
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
            if (rapid_change)
                *rapid_change = 5;       // Will be used as a counter for inversion timer
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


class MedianEMAFilter
{
private:
    std::array<float, 5> window {};
    size_t count = 0;
    size_t index = 0;
    float smoothed_value = NAN;
    float alpha;

public:
    MedianEMAFilter(float alpha_val = 0.25f) : alpha(alpha_val) {}

    float process(float new_reading, int* rapid_change = nullptr) {
        if (std::isnan(new_reading)) {
            return smoothed_value;
        }

        // 1. Push into 5-point ring buffer (1 second of 200ms chunks)
        window[index] = new_reading;
        index = (index + 1) % window.size();
        if (count < window.size()) {
            count++;
        }

        // 2. Find median to completely strip 1- or 2-chunk spikes
        std::array<float, 5> sorted = window;
        std::sort(sorted.begin(), sorted.begin() + count);
        float median = sorted[count / 2];

        // 3. Apply EMA smoothing on the spike-free median
        if (std::isnan(smoothed_value)) {
            smoothed_value = median;
        } else {
            smoothed_value = smoothed_value + alpha * (median - smoothed_value);
        }

        if (rapid_change) {
            *rapid_change = 0;
        }
        return smoothed_value;
    }

    void reset() {
        count = 0;
        index = 0;
        smoothed_value = NAN;
    }

    float get() const {
        return smoothed_value;
    }
};


// Rounds to a step, but moves to a new step only when the input crosses
// the rounding boundary by a margin: no last-digit flicker
class HysteresisQuantizer
{
private:
    float step;
    float threshold;
    float output = NAN;

public:
    HysteresisQuantizer(float step_val, float margin = 0.3f)
        : step(step_val), threshold(step_val * (0.5f + margin)) {}

    float process(float value) {
        if (std::isnan(output) || std::abs(value - output) > threshold)
            output = std::round(value / step) * step;
        return output;
    }

    void reset() {
        output = NAN;
    }
};

