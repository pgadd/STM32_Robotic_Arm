#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>

typedef struct {
    float shoulder_deg;   /* theta1: angle of link1 from the base's x-axis */
    float elbow_deg;      /* theta2: angle of link2 relative to link1 */
} ArmIK_Result;

/* Solves the 2-link planar arm for target point (x_mm, y_mm) in a plane
 * parallel to the base. Picks the elbow-up solution. Out-of-reach targets
 * are clamped onto the nearest edge of the reachable annulus (radius
 * between |link1-link2| and link1+link2) rather than failing, so the
 * caller always gets a usable angle pair. Returns false only if the link
 * lengths themselves are degenerate (<= 0). */
bool ArmIK_Solve(float x_mm, float y_mm, float link1_mm, float link2_mm, ArmIK_Result *out);

#endif /* KINEMATICS_H */
