#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ===========================================================================
 * Link geometry (mm)
 * Physical arm not built yet -- these are placeholders. Update once the
 * shoulder->elbow and elbow->wrist link lengths are actually measured.
 * ===========================================================================*/
#define LINK1_LENGTH_MM             120.0f   /* shoulder -> elbow */
#define LINK2_LENGTH_MM             100.0f   /* elbow -> wrist */

/* ===========================================================================
 * Joint angle limits (degrees, servo-horn frame, 0-180 hobby servo range)
 * Placeholders -- narrow these once the physical linkage's mechanical
 * limits (what angle actually collides with the frame) are known.
 * ===========================================================================*/
#define BASE_ANGLE_MIN_DEG             0.0f
#define BASE_ANGLE_MAX_DEG           180.0f

#define SHOULDER_ANGLE_MIN_DEG         0.0f
#define SHOULDER_ANGLE_MAX_DEG       180.0f

#define ELBOW_ANGLE_MIN_DEG            0.0f
#define ELBOW_ANGLE_MAX_DEG          180.0f

#define WRIST_ANGLE_MIN_DEG            0.0f
#define WRIST_ANGLE_MAX_DEG          180.0f

/* ===========================================================================
 * Workspace target for the shoulder/elbow IK pair, driven by joystick 1.
 * (x, y) live in a plane parallel to the base -- like a pen moving over a
 * sheet of paper lying on the table. Base rotation (theta0) then sweeps
 * that whole plane around the vertical axis.
 *
 * Y is kept off zero so atan2(y, x) never hits the (0,0) singularity, and
 * the range is kept inside the [ |L1-L2|, L1+L2 ] reachable annulus for the
 * placeholder link lengths above so the IK solver rarely has to clamp.
 * ===========================================================================*/
#define WORKSPACE_X_MIN_MM          -150.0f
#define WORKSPACE_X_MAX_MM           150.0f
#define WORKSPACE_Y_MIN_MM            40.0f
#define WORKSPACE_Y_MAX_MM           200.0f

/* ===========================================================================
 * Servo PWM (50 Hz hobby servo, 1000-2000us pulse for 0-180 degrees)
 * ===========================================================================*/
#define SERVO_PULSE_MIN_US          1000u
#define SERVO_PULSE_MAX_US          2000u
#define SERVO_PWM_PERIOD_US        20000u   /* 20ms -> 50Hz */

/* ===========================================================================
 * ADC / joystick conditioning
 * ===========================================================================*/
#define ADC_COUNTS_MAX               4095.0f
#define ADC_CENTER                   2048.0f
#define ADC_DEADBAND                    80.0f  /* raw counts around center to ignore */
#define ADC_FILTER_ALPHA                 0.2f  /* EMA smoothing factor, 0 < a <= 1 */

/* ===========================================================================
 * Control loop
 * ===========================================================================*/
#define CONTROL_LOOP_PERIOD_MS         15u
#define MAX_STEP_DEG_PER_TICK           3.0f  /* servo slew limit per control tick */

#endif /* ROBOT_CONFIG_H */
