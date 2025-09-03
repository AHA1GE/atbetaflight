/* * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "blackbox/blackbox.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"
#include "fc/rc_adjustments.h"
#include "common/axis.h"
#include "common/filter.h"
#include "config/feature.h"
#include "drivers/pwm_esc_detect.h"
#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "io/beeper.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "config/simplified_tuning.h"
#include "pg/rx.h"
#include "pg/motor.h"
#include "sensors/battery.h"
#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "blackbox/blackbox.h"
#include "rx/rx.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 10000

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TMR1, CH1, PA8, TIM_USE_ANY | TIM_USE_LED, 0, 7, 1), // PWM1 - OUT1  LEDSTRIP /MCO1

    DEF_TIM(TMR4, CH1, PB6, TIM_USE_MOTOR, 0, 0, 0), // motor1 DMA1 CH1
    DEF_TIM(TMR4, CH2, PB7, TIM_USE_MOTOR, 0, 1, 1), // motor2 DMA1 CH2
    DEF_TIM(TMR3, CH3, PB0, TIM_USE_MOTOR, 0, 2, 2), // motor3 DMA1 CH3
    DEF_TIM(TMR3, CH4, PB1, TIM_USE_MOTOR, 0, 3, 3), // motor4 DMA1 CH4
};

void targetConfiguration(void)
{
    if (getDetectedMotorType() == MOTOR_BRUSHED)
    {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1040; // for 6mm and 7mm brushed
    }

    // Set AUX1 low = ARM as default
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1300);
    // Set AUX1 full range = Angle as default
    modeActivationConditionsMutable(1)->modeId = BOXANGLE;
    modeActivationConditionsMutable(1)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    analyzeModeActivationConditions();

    // Master Configuration from preset
//     // Set accelerometer calibration values: acc_calibration = -3,52,-8,1
// #ifdef USE_ACC
//     accelerometerConfigMutable()->accZero.raw[X] = -3;
//     accelerometerConfigMutable()->accZero.raw[Y] = 52;
//     accelerometerConfigMutable()->accZero.raw[Z] = -8;
//     accelerometerConfigMutable()->accZero.values.calibrationCompleted = 1;
// #endif

    // Set mag_hardware = NONE (compass disabled)
#ifdef USE_MAG
    compassConfigMutable()->mag_hardware = MAG_NONE;
#endif

    // Set blackbox_sample_rate = 1/16
#ifdef USE_BLACKBOX
    blackboxConfigMutable()->sample_rate = BLACKBOX_RATE_16TH;
#endif

    // Set bat_capacity = 500
    batteryConfigMutable()->batteryCapacity = 500;

    // PID Profile Configuration from preset - Profile 0
    pidProfile_t *pidProfile = pidProfilesMutable(0);
    pidProfile->simplified_pids_mode = PID_SIMPLIFIED_TUNING_RP;
    pidProfile->simplified_master_multiplier = 175;
    pidProfile->simplified_i_gain = 35;
    pidProfile->simplified_pi_gain = 200;
    pidProfile->simplified_dmin_ratio = 140;
    pidProfile->simplified_feedforward_gain = 55;

    // Iterm relax settings
    pidProfile->iterm_relax_type = ITERM_RELAX_GYRO;
    pidProfile->iterm_relax_cutoff = 10;

    // PID values from preset
    pidProfile->pid[PID_PITCH].P = 164;
    pidProfile->pid[PID_PITCH].I = 102;
    pidProfile->pid[PID_PITCH].D = 88;
    pidProfile->pid[PID_PITCH].F = 120;

    pidProfile->pid[PID_ROLL].P = 157;
    pidProfile->pid[PID_ROLL].I = 98;
    pidProfile->pid[PID_ROLL].D = 77;
    pidProfile->pid[PID_ROLL].F = 115;

    pidProfile->pid[PID_YAW].P = 157;
    pidProfile->pid[PID_YAW].I = 98;
    pidProfile->pid[PID_YAW].D = 0; // YAW D is typically 0
    pidProfile->pid[PID_YAW].F = 115;

    // Level mode settings
    pidProfile->pid[PID_LEVEL].P = 80; // angle_level_strength
    pidProfile->levelAngleLimit = 40;  // level_limit

#ifdef USE_D_MIN
    // D_min values
    pidProfile->d_min[FD_ROLL] = 52;
    pidProfile->d_min[FD_PITCH] = 59;
    pidProfile->d_min[FD_YAW] = 0;  // typically 0 for yaw
    pidProfile->d_min_advance = 25; // d_max_advance
#endif

    // Battery voltage sag compensation
#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
    pidProfile->vbat_sag_compensation = 100;
#endif

#ifdef USE_THRUST_LINEARIZATION
    // Thrust linearization
    pidProfile->thrustLinearization = 25;
#endif

#ifdef USE_FEEDFORWARD
    // Feedforward averaging
    pidProfile->feedforward_averaging = FEEDFORWARD_AVERAGING_2_POINT;
#endif

    // Set throttle curve for 50% stick = 38% output (hover throttle)
    controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(0);
    controlRateConfig->thrMid8 = 38; // 38% output at 50% stick for hover

    // Disable airmode feature by default
    featureConfigClear(FEATURE_AIRMODE);

    // LED Strip configuration
    // Set LED profile default to STATUS mode instead of race/beacon
    ledStripConfigMutable()->ledstrip_profile = LED_PROFILE_STATUS;

    // Keep race and beacon colors but default to status
    ledStripConfigMutable()->ledstrip_race_color = COLOR_RED;
    ledStripConfigMutable()->ledstrip_beacon_color = COLOR_BLUE;
    ledStripConfigMutable()->ledstrip_beacon_period_ms = 500;
    ledStripConfigMutable()->ledstrip_beacon_percent = 100;
    ledStripConfigMutable()->ledstrip_beacon_armed_only = 0;

#ifdef USE_LED_STRIP_STATUS_MODE
    // LED Status mode configuration - 8 LEDs setup
    // led 0 0,15::CI:10 - Corner Indicator at position (0,15) color 10
    ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(0, 15, 10, 0, LF(COLOR), LO(INDICATOR), 0);

    // led 1 15,15::CI:10 - Corner Indicator at position (15,15) color 10
    ledStripStatusModeConfigMutable()->ledConfigs[1] = DEFINE_LED(15, 15, 10, 0, LF(COLOR), LO(INDICATOR), 0);

    // led 2 15,0::CI:6 - Corner Indicator at position (15,0) color 6
    ledStripStatusModeConfigMutable()->ledConfigs[2] = DEFINE_LED(15, 0, 6, 0, LF(COLOR), LO(INDICATOR), 0);

    // led 3 0,0::CI:6 - Corner Indicator at position (0,0) color 6
    ledStripStatusModeConfigMutable()->ledConfigs[3] = DEFINE_LED(0, 0, 6, 0, LF(COLOR), LO(INDICATOR), 0);

    // led 4 8,15::CYO:9 - Color Yellow Orange overlay at position (8,15) color 9
    ledStripStatusModeConfigMutable()->ledConfigs[4] = DEFINE_LED(8, 15, COLOR_YELLOW, 0, LF(COLOR), LO(WARNING), 0);

    // led 5 8,14::CYO:6 - Color Yellow Orange overlay at position (8,14) color 6
    ledStripStatusModeConfigMutable()->ledConfigs[5] = DEFINE_LED(8, 14, COLOR_YELLOW, 0, LF(COLOR), LO(WARNING), 0);

    // led 6 8,13::CYO:4 - Color Yellow Orange overlay at position (8,13) color 4
    ledStripStatusModeConfigMutable()->ledConfigs[6] = DEFINE_LED(8, 13, COLOR_YELLOW, 0, LF(COLOR), LO(WARNING), 0);

    // led 7 8,12::CYOW:2 - Color Yellow Orange Warning at position (8,12) color 2
    ledStripStatusModeConfigMutable()->ledConfigs[7] = DEFINE_LED(8, 12, COLOR_YELLOW, 0, LF(COLOR), LO(WARNING), 0);

    // Re-evaluate LED configuration after changes
    reevaluateLedConfig();
#endif

    // AUX4 (index 3) adjustment range from preset: adjrange 0 0 0 900 1300 30 3 0 0
    adjustmentRange_t *adjRange = adjustmentRangesMutable(0);
    adjRange->auxChannelIndex = 0; // range channel (not used here)
    adjRange->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    adjRange->range.endStep = CHANNEL_VALUE_TO_STEP(2100);          // Updated to match preset
    adjRange->adjustmentConfig = 30;                                // LED profile adjustment function index (ADJUSTMENT_LED_PROFILE + offset)
    adjRange->auxSwitchChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT; // AUX4 = 3 in array (0-based)
    adjRange->adjustmentCenter = 0;
    adjRange->adjustmentScale = 0;
}
#endif