/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/time.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "programming/logic_condition.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "common/utils.h"

// Base frequencies for smoothing pitch and roll
#define NAV_FW_BASE_PITCH_CUTOFF_FREQUENCY_HZ     2.0f
#define NAV_FW_BASE_ROLL_CUTOFF_FREQUENCY_HZ     10.0f

// If this is enabled navigation won't be applied if velocity is below 3 m/s
//#define NAV_FW_LIMIT_MIN_FLY_VELOCITY

static bool isPitchAdjustmentValid = false;
static bool isRollAdjustmentValid = false;
static bool isYawAdjustmentValid = false;
static int32_t navHeadingError;

// Calculates the cutoff frequency for smoothing out roll/pitch commands
// control_smoothness valid range from 0 to 9
// resulting cutoff_freq ranging from baseFreq downwards to ~0.11Hz
static float getSmoothnessCutoffFreq(float baseFreq)
{
    uint16_t smoothness = 10 - navConfig()->fw.control_smoothness;
    return 0.001f * baseFreq * (float)(smoothness*smoothness*smoothness) + 0.1f;
}

void resetAntennaTrackerAltitudeController(void)
{
    navPidReset(&posControl.pids.fw_alt);
    posControl.rcAdjustment[PITCH] = 0;
    isPitchAdjustmentValid = false;
}

// Position to velocity controller for Z axis
static void updateAltitudeVelocityAndPitchController_AT(timeDelta_t deltaMicros)
{
    UNUSED(deltaMicros);

    const fpVector3_t desiredPos = posControl.desiredState.pos;
    const fpVector3_t currentPos = navGetCurrentActualPositionAndVelocity()->pos;
    const float distance = calc_length_pythagorean_2D(ABS(desiredPos.x - currentPos.x), ABS(desiredPos.y - currentPos.y));

    // Here we use negative values for dive for better clarity
    const float maxClimbDeciDeg = DEGREES_TO_DECIDEGREES(navConfig()->fw.max_climb_angle);
    const float minDiveDeciDeg = -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle);

    float targetPitchAngle = RADIANS_TO_DECIDEGREES(atan2_approx(distance, desiredPos.z - currentPos.z));

    // Reconstrain pitch angle ( >0 - climb, <0 - dive)
    targetPitchAngle = constrainf(targetPitchAngle, minDiveDeciDeg, maxClimbDeciDeg);
    posControl.rcAdjustment[PITCH] = targetPitchAngle;
}

void applyAntennaTrackerAltitudeController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate = 0;         // Occurs @ altitude sensor update rate (max MAX_ALTITUDE_UPDATE_RATE_HZ)

    if ((posControl.flags.estAltStatus >= EST_USABLE)) {
        if (posControl.flags.verticalPositionDataNew) {
            const timeDeltaLarge_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            // Check if last correction was not too long ago
            if (deltaMicrosPositionUpdate < MAX_POSITION_UPDATE_INTERVAL_US) {
                updateAltitudeVelocityAndPitchController_AT(deltaMicrosPositionUpdate);
            }
            else {
                // Position update has not occurred in time (first iteration or glitch), reset altitude controller
                resetAntennaTrackerAltitudeController();
            }

            // Indicate that information is no longer usable
            posControl.flags.verticalPositionDataConsumed = true;
        }

        isPitchAdjustmentValid = true;
    }
    else {
        // No valid altitude sensor data, don't adjust pitch automatically, rcCommand[PITCH] is passed through to PID controller
        isPitchAdjustmentValid = false;
    }
}

/*-----------------------------------------------------------
 * XY-position controller for multicopter aircraft
 *-----------------------------------------------------------*/
static fpVector3_t virtualDesiredPosition;
static pt1Filter_t fwPosControllerCorrectionFilterState;

/*
 * TODO Currently this function resets both FixedWing and Rover & Boat position controller
 */
void resetAntennaTrackerPositionController(void)
{
    virtualDesiredPosition.x = 0;
    virtualDesiredPosition.y = 0;
    virtualDesiredPosition.z = 0;

    navPidReset(&posControl.pids.fw_nav);
    navPidReset(&posControl.pids.fw_heading);
    posControl.rcAdjustment[ROLL] = 0;
    posControl.rcAdjustment[YAW] = 0;
    isRollAdjustmentValid = false;
    isYawAdjustmentValid = false;

    pt1FilterReset(&fwPosControllerCorrectionFilterState, 0.0f);
}

static void updatePositionHeadingController_AT(timeUs_t currentTimeUs, timeDelta_t deltaMicros)
{
    UNUSED(currentTimeUs);

    static timeUs_t previousTimeMonitoringUpdate;
    static int32_t previousHeadingError;
    static bool errorIsDecreasing;

    // We have virtual position target, calculate heading error
    int32_t virtualTargetBearing = calculateBearingToDestination(&posControl.desiredState.pos);

    /*
     * Calculate NAV heading error
     * Units are centidegrees
     */
    navHeadingError = wrap_18000(virtualTargetBearing - posControl.actualState.yaw);

    // Slow error monitoring (2Hz rate)
    if ((currentTimeUs - previousTimeMonitoringUpdate) >= HZ2US(NAV_FW_CONTROL_MONITORING_RATE)) {
        // Check if error is decreasing over time
        errorIsDecreasing = (ABS(previousHeadingError) > ABS(navHeadingError));

        // Save values for next iteration
        previousHeadingError = navHeadingError;
        previousTimeMonitoringUpdate = currentTimeUs;
    }

    // Only allow PID integrator to shrink if error is decreasing over time
    const pidControllerFlags_e pidFlags = PID_DTERM_FROM_ERROR | (errorIsDecreasing ? PID_SHRINK_INTEGRATOR : 0);

    // Input error in (deg*100), output roll angle (deg*100)
    float rollAdjustment = navPidApply2(&posControl.pids.fw_nav, posControl.actualState.yaw + navHeadingError, posControl.actualState.yaw, US2S(deltaMicros),
                                       -DEGREES_TO_CENTIDEGREES(navConfig()->fw.max_bank_angle),
                                        DEGREES_TO_CENTIDEGREES(navConfig()->fw.max_bank_angle),
                                        pidFlags);

    // Apply low-pass filter to prevent rapid correction
    rollAdjustment = pt1FilterApply4(&fwPosControllerCorrectionFilterState, rollAdjustment, getSmoothnessCutoffFreq(NAV_FW_BASE_ROLL_CUTOFF_FREQUENCY_HZ), US2S(deltaMicros));

    // Convert rollAdjustment to decidegrees (rcAdjustment holds decidegrees)
    posControl.rcAdjustment[ROLL] = CENTIDEGREES_TO_DECIDEGREES(rollAdjustment);
}

void applyAntennaTrackerPositionController(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimePositionUpdate = 0;         // Occurs @ GPS update rate

    // Apply controller only if position source is valid. In absence of valid pos sensor (GPS loss), we'd stick in forced ANGLE mode
    if ((posControl.flags.estPosStatus >= EST_USABLE)) {
        // If we have new position - update velocity and acceleration controllers
        if (posControl.flags.horizontalPositionDataNew) {
            const timeDeltaLarge_t deltaMicrosPositionUpdate = currentTimeUs - previousTimePositionUpdate;
            previousTimePositionUpdate = currentTimeUs;

            if (deltaMicrosPositionUpdate < MAX_POSITION_UPDATE_INTERVAL_US) {
                updatePositionHeadingController_AT(currentTimeUs, deltaMicrosPositionUpdate);
            }
            else {
                // Position update has not occurred in time (first iteration or glitch), reset altitude controller
                resetAntennaTrackerPositionController();
            }

            // Indicate that information is no longer usable
            posControl.flags.horizontalPositionDataConsumed = true;
        }

        isRollAdjustmentValid = true;
        isYawAdjustmentValid = true;
    }
    else {
        // No valid pos sensor data, don't adjust pitch automatically, rcCommand[ROLL] is passed through to PID controller
        isRollAdjustmentValid = false;
        isYawAdjustmentValid = false;
    }
}

void applyAntennaTrackerPitchRollController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (isRollAdjustmentValid && (navStateFlags & NAV_CTL_POS)) {
        // ROLL >0 right, <0 left
        int16_t rollCorrection = constrain(posControl.rcAdjustment[ROLL], -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_bank_angle), DEGREES_TO_DECIDEGREES(navConfig()->fw.max_bank_angle));
        rcCommand[ROLL] = pidAngleToRcCommand(rollCorrection, pidProfile()->max_angle_inclination[FD_ROLL]);
    }

    if (isPitchAdjustmentValid && (navStateFlags & NAV_CTL_ALT)) {
        // PITCH >0 dive, <0 climb
        int16_t pitchCorrection = constrain(posControl.rcAdjustment[PITCH], -DEGREES_TO_DECIDEGREES(navConfig()->fw.max_dive_angle), DEGREES_TO_DECIDEGREES(navConfig()->fw.max_climb_angle));
        rcCommand[PITCH] = -pidAngleToRcCommand(pitchCorrection, pidProfile()->max_angle_inclination[FD_PITCH]);
    }
}

void resetAntennaTrackerHeadingController(void)
{
    updateHeadingHoldTarget(CENTIDEGREES_TO_DEGREES(posControl.actualState.yaw));
}

void applyAntennaTrackerNavigationController(navigationFSMStateFlags_t navStateFlags, timeUs_t currentTimeUs)
{
    if (navStateFlags & NAV_CTL_ALT) {
        if (getMotorStatus() == MOTOR_STOPPED_USER || FLIGHT_MODE(SOARING_MODE)) {
            // Motor has been stopped by user or soaring mode enabled to override altitude control
            resetAntennaTrackerAltitudeController();
            setDesiredPosition(&navGetCurrentActualPositionAndVelocity()->pos, posControl.actualState.yaw, NAV_POS_UPDATE_Z);
        } else {
            applyAntennaTrackerAltitudeController(currentTimeUs);
        }
    }

    if (navStateFlags & NAV_CTL_POS) {
        applyAntennaTrackerPositionController(currentTimeUs);
    }

    if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE) && posControl.flags.isAdjustingPosition) {
        rcCommand[ROLL] = applyDeadbandRescaled(rcCommand[ROLL], rcControlsConfig()->pos_hold_deadband, -500, 500);
    }

    //if (navStateFlags & NAV_CTL_YAW)
    if ((navStateFlags & NAV_CTL_ALT) || (navStateFlags & NAV_CTL_POS)) {
        applyAntennaTrackerPitchRollController(navStateFlags, currentTimeUs);
    }

    if (FLIGHT_MODE(SOARING_MODE) && navConfig()->general.flags.soaring_motor_stop) {
        ENABLE_STATE(NAV_MOTOR_STOP_OR_IDLE);
    }
}
