/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#include <px4_platform_common/param.h>

/**
 * Sensor attack detector enable
 *
 * @boolean
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_INT32(SAD_EN, 1);

/**
 * Maximum detector evaluation rate
 *
 * @unit Hz
 * @min 1
 * @max 20
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_RATE, 10.f);

/**
 * Detector warmup duration
 *
 * @unit s
 * @min 8
 * @max 60
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_WARMUP, 10.f);

/**
 * Acceleration residual weight
 *
 * @min 0.000001
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_WA, 1.f);

/**
 * Velocity residual weight
 *
 * @min 0.000001
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_WV, 1.f);

/**
 * Position residual weight
 *
 * @min 0.000001
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_WP, 1.f);

/**
 * Actuator source
 *
 * @value 0 actuator_motors normalized commands
 * @value 1 actuator_outputs legacy PWM outputs
 * @min 0
 * @max 1
 * @reboot_required true
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_INT32(SAD_ACT_SRC, 1);

/**
 * Actuator thrust-indicator mapping
 *
 * @value 0 linear sum of normalized motor commands
 * @value 1 sum of squared normalized motor commands
 * @min 0
 * @max 1
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_INT32(SAD_THR_MAP, 0);

/**
 * Acceleration per summed normalized motor command
 *
 * @unit m/s^2
 * @min 0.01
 * @max 20
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_THR_GAIN, 3.47118998f);

/**
 * Maximum accepted GPS horizontal position error
 *
 * @unit m
 * @min 0.1
 * @max 100
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_GPS_EPH, 5.f);

/**
 * Attack coefficient second-difference regularization
 *
 * @min 0
 * @max 100000
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_REG, 0.1f);


/**
 * Fixed nominal actuator-scale correction coefficient
 *
 * Calibrate from dedicated nominal maneuver flights before detector
 * normalization or threshold calibration.
 *
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_AS_MU, 0.f);

/**
 * Legacy V2 actuator-scale prior width
 *
 * Retained for experiment compatibility. The fixed 7x7 detector does not
 * read this parameter.
 *
 * @min 0.001
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_AS_SD, 1.f);

/**
 * Fixed nominal actuator-dynamic correction coefficient
 *
 * Calibrate from dedicated nominal maneuver flights before detector
 * normalization or threshold calibration.
 *
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_AD_MU, 0.f);

/**
 * Legacy V2 actuator-dynamic prior width
 *
 * Retained for experiment compatibility. The fixed 7x7 detector does not
 * read this parameter.
 *
 * @min 0.001
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_AD_SD, 1.f);

/**
 * Nominal mean of the eight-second GLRT
 *
 * This value must be calibrated from independent nominal flights.
 *
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_GLRT_MU, 3.99645138f);

/**
 * Nominal standard deviation of the eight-second GLRT
 *
 * This value must be calibrated from independent nominal flights.
 *
 * @min 0.000001
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_GLRT_SD, 5.98480034f);

/**
 * CUSUM drift
 *
 * @min 0
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_CUS_DR, 0.5f);

/**
 * CUSUM alert threshold
 *
 * This value must be calibrated from independent nominal flights.
 *
 * @min 0
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_FLOAT(SAD_THRESH, 60.4715042f);

/**
 * Consecutive threshold crossings required for alert
 *
 * @min 1
 * @max 20
 * @group Sensor Attack Detector
 */
PARAM_DEFINE_INT32(SAD_CONSEC, 3);
