#include "kinematics.h"
#include <math.h>

#define IK_RAD_TO_DEG (180.0f / 3.14159265358979323846f)

bool ArmIK_Solve(float x_mm, float y_mm, float link1_mm, float link2_mm, ArmIK_Result *out)
{
    if (link1_mm <= 0.0f || link2_mm <= 0.0f || out == NULL) {
        return false;
    }

    float r = sqrtf(x_mm * x_mm + y_mm * y_mm);

    /* Clamp the target onto the reachable annulus instead of rejecting it --
     * a joystick can easily ask for a point outside the arm's reach, and we
     * always want a valid pose to servo toward. */
    const float r_max = link1_mm + link2_mm;
    const float r_min = fabsf(link1_mm - link2_mm);
    if (r > r_max) {
        r = r_max;
    } else if (r < r_min) {
        r = r_min;
    }
    /* Guard the atan2(y, x) below: r==0 only if x==y==0, which can only
     * happen here if r_min==0 (link1==link2) and the raw target was the
     * origin. Nudge off the singularity. */
    if (r < 1e-3f) {
        r = 1e-3f;
    }

    const float cos_elbow = (r * r - link1_mm * link1_mm - link2_mm * link2_mm)
                             / (2.0f * link1_mm * link2_mm);
    const float cos_elbow_clamped = (cos_elbow > 1.0f) ? 1.0f : (cos_elbow < -1.0f ? -1.0f : cos_elbow);
    const float sin_elbow = sqrtf(1.0f - cos_elbow_clamped * cos_elbow_clamped); /* elbow-up: sin >= 0 */

    const float elbow_rad = atan2f(sin_elbow, cos_elbow_clamped);
    const float phi = atan2f(link2_mm * sin_elbow, link1_mm + link2_mm * cos_elbow_clamped);

    /* Recompute the direction to the (possibly-clamped) target from its
     * scaled coordinates so theta1 stays consistent with the clamped r. */
    const float theta_to_target = atan2f(y_mm, x_mm);
    const float shoulder_rad = theta_to_target - phi;

    out->shoulder_deg = shoulder_rad * IK_RAD_TO_DEG;
    out->elbow_deg = elbow_rad * IK_RAD_TO_DEG;
    return true;
}
