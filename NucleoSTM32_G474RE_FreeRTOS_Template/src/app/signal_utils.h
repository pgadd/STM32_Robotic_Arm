#ifndef SIGNAL_UTILS_H
#define SIGNAL_UTILS_H

#include <stdint.h>
#include <stdbool.h>

/* Exponential moving-average filter -- cheap low-pass for noisy ADC counts,
 * updated every control tick instead of averaging a fixed window. */
typedef struct {
    float value;
    bool initialized;
} EmaFilter;

void EmaFilter_Reset(EmaFilter *f);
float EmaFilter_Update(EmaFilter *f, float raw, float alpha);

/* Snaps `raw` to `center` when within `deadband` counts of it, so a
 * joystick that doesn't rest exactly at midpoint doesn't cause drift.
 * Returns `raw` unchanged outside the deadband. */
float ApplyDeadband(float raw, float center, float deadband);

float ClampF(float x, float min, float max);

/* Linear map, clamping x to [in_min, in_max] first. */
float MapRange(float x, float in_min, float in_max, float out_min, float out_max);

/* Moves `current` toward `target` by at most `max_step`, for slewing a
 * servo a few degrees per control tick instead of snapping to it. */
float RateLimitStep(float current, float target, float max_step);

/* Degrees -> servo pulse width in microseconds. */
uint32_t AngleDegToPulseUs(float angle_deg, float angle_min_deg, float angle_max_deg,
                            uint32_t pulse_min_us, uint32_t pulse_max_us);

#endif /* SIGNAL_UTILS_H */
