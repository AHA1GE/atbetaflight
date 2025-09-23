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
#include "pg/pinio.h"
#include "pg/piniobox.h"

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

    // Master Configuration from preset
    // Gyro filter settings from preset
    gyroConfigMutable()->gyro_lpf1_static_hz = 212;              // gyro_lpf1_static_hz
    gyroConfigMutable()->gyro_lpf2_static_hz = 425;              // gyro_lpf2_static_hz
    gyroConfigMutable()->gyro_lpf1_dyn_min_hz = 212;             // gyro_lpf1_dyn_min_hz
    gyroConfigMutable()->gyro_lpf1_dyn_max_hz = 425;             // gyro_lpf1_dyn_max_hz
    gyroConfigMutable()->simplified_gyro_filter_multiplier = 85; // simplified_gyro_filter_multiplier

    // Set accelerometer calibration values: acc_calibration = 0,0,0,1
    // #ifdef USE_ACC
    //     accelerometerConfigMutable()->accZero.raw[X] = 0;
    //     accelerometerConfigMutable()->accZero.raw[Y] = 0;
    //     accelerometerConfigMutable()->accZero.raw[Z] = 0;
    //     accelerometerConfigMutable()->accZero.values.calibrationCompleted = 1;
    // #endif

    // Set mag_hardware = NONE (compass disabled)
#ifdef USE_MAG
    compassConfigMutable()->mag_hardware = MAG_NONE;
#endif

    // Set bat_capacity = 500
    batteryConfigMutable()->batteryCapacity = 500;

    // PID Profile Configuration from preset - Profile 0
    pidProfile_t *pidProfile = pidProfilesMutable(0);

    // Dterm filter settings from preset
    pidProfile->dterm_lpf1_dyn_min_hz = 71;  // dterm_lpf1_dyn_min_hz
    pidProfile->dterm_lpf1_dyn_max_hz = 142; // dterm_lpf1_dyn_max_hz
    pidProfile->dterm_lpf1_static_hz = 71;   // dterm_lpf1_static_hz
    pidProfile->dterm_lpf2_static_hz = 142;  // dterm_lpf2_static_hz

    // Anti-gravity settings from preset
    pidProfile->itermAcceleratorGain = 8000; // anti_gravity_gain

    pidProfile->simplified_pids_mode = PID_SIMPLIFIED_TUNING_OFF;
    // pidProfile->simplified_master_multiplier = 150; // simplified_pi_gain from preset
    // pidProfile->simplified_i_gain = 35;
    // pidProfile->simplified_pi_gain = 150; // from preset
    // pidProfile->simplified_dmin_ratio = 95; // simplified_dterm_filter_multiplier from preset
    // pidProfile->simplified_feedforward_gain = 45; // from preset

    // Iterm relax settings
    pidProfile->iterm_relax_type = ITERM_RELAX_GYRO;
    pidProfile->iterm_relax_cutoff = 7;

    // PID values from preset
    pidProfile->pid[PID_PITCH].P = 115; // p_pitch
    pidProfile->pid[PID_PITCH].I = 90;  // i_pitch
    pidProfile->pid[PID_PITCH].D = 88;  // calculated based on dterm settings
    pidProfile->pid[PID_PITCH].F = 100; // f_pitch

    pidProfile->pid[PID_ROLL].P = 110; // p_roll
    pidProfile->pid[PID_ROLL].I = 80;  // i_roll
    pidProfile->pid[PID_ROLL].D = 77;  // calculated based on dterm settings
    pidProfile->pid[PID_ROLL].F = 90;  // f_roll

    pidProfile->pid[PID_YAW].P = 110; // p_yaw
    pidProfile->pid[PID_YAW].I = 80;  // i_yaw
    pidProfile->pid[PID_YAW].D = 50;  // YAW D is typically 0
    pidProfile->pid[PID_YAW].F = 90;  // f_yaw

    // Level mode settings from preset
    pidProfile->pid[PID_LEVEL].P = 60; // angle_level_strength from preset
    pidProfile->levelAngleLimit = 35;  // level_limit from preset

#ifdef USE_D_MIN
    // D_min values
    pidProfile->d_min[FD_ROLL] = 55;
    pidProfile->d_min[FD_PITCH] = 60;
    pidProfile->d_min[FD_YAW] = 30; // typically 0 for yaw
    pidProfile->d_min_advance = 25; // d_max_advance
#endif

    // Battery voltage sag compensation
#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
    pidProfile->vbat_sag_compensation = 100;
#endif

#ifdef USE_THRUST_LINEARIZATION
    // Thrust linearization from preset
    pidProfile->thrustLinearization = 30; // thrust_linear from preset
#endif

#ifdef USE_FEEDFORWARD
    // Feedforward averaging
    pidProfile->feedforward_averaging = FEEDFORWARD_AVERAGING_2_POINT;
#endif

    // Set throttle curve for 50% stick = 49% output (hover throttle is 38)
    controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(0);

    // Rate settings from preset (rateprofile 0)
    controlRateConfig->rcRates[FD_ROLL] = 4;  // roll_rc_rate
    controlRateConfig->rcRates[FD_PITCH] = 4; // pitch_rc_rate
    controlRateConfig->rcRates[FD_YAW] = 4;   // yaw_rc_rate
    controlRateConfig->rates[FD_ROLL] = 65;   // roll_srate
    controlRateConfig->rates[FD_PITCH] = 66;  // pitch_srate

    // Throttle settings from preset
    controlRateConfig->thrMid8 = 38;                                    // 38% output at 50% stick for hover
    controlRateConfig->throttle_limit_type = THROTTLE_LIMIT_TYPE_SCALE; // throttle_limit_type = SCALE
    controlRateConfig->throttle_limit_percent = 95;                     // throttle_limit_percent = 95

    // Set runaway takeoff prevention deactivate throttle to 12%
    pidConfigMutable()->runaway_takeoff_deactivate_throttle = 12;

    // Set runaway takeoff prevention deactivate delay to 100ms instead of 500ms
    pidConfigMutable()->runaway_takeoff_deactivate_delay = 100;

    // // Set PID rate to 4kHz (assuming 8kHz gyro rate, pid_process_denom = 2)
    // pidConfigMutable()->pid_process_denom = 2;

    // Disable airmode feature by default
    featureConfigClear(FEATURE_AIRMODE);
    // Disable inflight_acc_cal by default
    featureConfigClear(FEATURE_INFLIGHT_ACC_CAL);
    // Disable OSD by default
    featureConfigClear(FEATURE_OSD);
    // Disable blackbox by default by set blackbox device to NONE
#ifdef USE_BLACKBOX
    // Set blackbox_sample_rate = 1/16
    blackboxConfigMutable()->sample_rate = BLACKBOX_RATE_16TH;
    blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
#endif

    // LED Strip configuration
    // Keep race and beacon colors but default to status
    ledStripConfigMutable()->ledstrip_race_color = COLOR_RED;
    ledStripConfigMutable()->ledstrip_beacon_color = COLOR_BLUE;
    ledStripConfigMutable()->ledstrip_beacon_period_ms = 500;
    ledStripConfigMutable()->ledstrip_beacon_percent = 100;
    ledStripConfigMutable()->ledstrip_beacon_armed_only = 0;

    // Set LED profile default to STATUS mode instead of race/beacon
    ledStripConfigMutable()->ledstrip_profile = LED_PROFILE_STATUS;

#ifdef USE_LED_STRIP_STATUS_MODE
    // LED Status mode configuration - 8 LEDs setup
    // Front LEDs (0,0 and 15,0) - Battery indicators
    ledStripStatusModeConfigMutable()->ledConfigs[3] = DEFINE_LED(0, 0, 0, 0, LED_FUNCTION_BATTERY, 0, 0);
    ledStripStatusModeConfigMutable()->ledConfigs[2] = DEFINE_LED(15, 0, 0, 0, LED_FUNCTION_BATTERY, 0, 0);

    // Back LEDs (0,15 and 15,15) - DEEP_PINK color indicators
    // ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(0, 15, COLOR_DEEP_PINK, 0, LED_FUNCTION_COLOR, 0, 0);
    // ledStripStatusModeConfigMutable()->ledConfigs[1] = DEFINE_LED(15, 15, COLOR_DEEP_PINK, 0, LED_FUNCTION_COLOR, 0, 0);
    
    // Back LEDs (0,15 and 15,15) - Battery indicators
    ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(0, 15, 0,0,LED_FUNCTION_BATTERY, 0, 0);
    ledStripStatusModeConfigMutable()->ledConfigs[1] = DEFINE_LED(15, 15, 0,0,LED_FUNCTION_BATTERY, 0, 0);

    // LED strip - DEEP_PINK color (center positions)
    ledStripStatusModeConfigMutable()->ledConfigs[4] = DEFINE_LED(8, 15, COLOR_DEEP_PINK, 0, LED_FUNCTION_COLOR, 0, 0);
    ledStripStatusModeConfigMutable()->ledConfigs[5] = DEFINE_LED(8, 14, COLOR_DEEP_PINK, 0, LED_FUNCTION_COLOR, 0, 0);
    ledStripStatusModeConfigMutable()->ledConfigs[6] = DEFINE_LED(8, 13, COLOR_DEEP_PINK, 0, LED_FUNCTION_COLOR, 0, 0);
    ledStripStatusModeConfigMutable()->ledConfigs[7] = DEFINE_LED(8, 12, COLOR_DEEP_PINK, 0, LED_FUNCTION_COLOR, 0, 0);

    // Re-evaluate LED configuration after changes
    reevaluateLedConfig();
#endif

    // // AUX4 adjustment range from preset: adjrange 0 0 0 900 1300 30 3 0 0
    adjustmentRangesMutable(0)->auxChannelIndex = 0; // range channel (not used here)
    adjustmentRangesMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    adjustmentRangesMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(910);
    adjustmentRangesMutable(0)->adjustmentConfig = 30;                                // LED profile adjustment function index (ADJUSTMENT_LED_PROFILE + offset)
    adjustmentRangesMutable(0)->auxSwitchChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT; // AUX4 = 3 in array (0-based)
    adjustmentRangesMutable(0)->adjustmentCenter = 0;
    adjustmentRangesMutable(0)->adjustmentScale = 0;


    // PinIO configuration
    pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioConfigMutable()->config[1] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = 40;
    pinioBoxConfigMutable()->permanentId[1] = 41;

    // Default mode: AUX1 = ARM, AUX4 = Angle(full Range), AUX5 = USER1, AUX6 = USER2
    // Set AUX1 low = ARM as default
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1300);
    // Set AUX4 full range = Angle as default
    modeActivationConditionsMutable(1)->modeId = BOXANGLE;
    modeActivationConditionsMutable(1)->auxChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    // Set AUX5 high = USER1 as default
    modeActivationConditionsMutable(2)->modeId = BOXUSER1;
    modeActivationConditionsMutable(2)->auxChannelIndex = AUX5 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    // Set AUX6 high = USER2 as default
    modeActivationConditionsMutable(3)->modeId = BOXUSER2;
    modeActivationConditionsMutable(3)->auxChannelIndex = AUX6 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(3)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(3)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    analyzeModeActivationConditions();
}
#endif