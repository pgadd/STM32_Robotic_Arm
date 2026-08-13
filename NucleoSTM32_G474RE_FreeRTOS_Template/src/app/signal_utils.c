#include "signal_utils.h"
#include <math.h>

void EmaFilter_Reset(EmaFilter *f)
{
    f->value = 0.0f;
    f->initialized = false;
}

float EmaFilter_Update(EmaFilter *f, float raw, float alpha)
{
    if (!f->initialized) {
        f->value = raw;
        f->initialized = true;
    } else {
        f->value += alpha * (raw - f->value);
    }
    return f->value;
}

float ApplyDeadband(float raw, float center, float deadband)
{
    return (fabsf(raw - center) < deadband) ? center : raw;
}

float ClampF(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

float MapRange(float x, float in_min, float in_max, float out_min, float out_max)
{
    x = ClampF(x, in_min, in_max);
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

float RateLimitStep(float current, float target, float max_step)
{
    float diff = target - current;
    if (diff > max_step) diff = max_step;
    if (diff < -max_step) diff = -max_step;
    return current + diff;
}

uint32_t AngleDegToPulseUs(float angle_deg, float angle_min_deg, float angle_max_deg,
                            uint32_t pulse_min_us, uint32_t pulse_max_us)
{
    angle_deg = ClampF(angle_deg, angle_min_deg, angle_max_deg);
    float t = (angle_deg - angle_min_deg) / (angle_max_deg - angle_min_deg);
    return pulse_min_us + (uint32_t)(t * (float)(pulse_max_us - pulse_min_us) + 0.5f);
}
