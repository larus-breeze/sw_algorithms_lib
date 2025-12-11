/***********************************************************************//**
 * @file		NAV_tuning_parameters.h
 * @brief		Tuning parameters for navigation, wind an variometer
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

#ifndef NAV_ALGORITHMS_NAV_TUNING_PARAMETERS_H_
#define NAV_ALGORITHMS_NAV_TUNING_PARAMETERS_H_

#define OMEGA_EARTH 7.2921150e-5f

#define MINIMUM_MAG_CALIBRATION_SAMPLES 6000
#define MAGNETIC_DISTURBANCE_LIMIT 0.1f
#define USE_SOFT_IRON_COMPENSATION	1
#define D_GNSS_GNSS_DELAY	13
#define D_GNSS_HEADING_DELAY	6
#define SINGLE_GNSS_DELAY	30
#define MAX_GNSS_DELAY		30

#define MINIMUM_MAG_CALIBRATION_SAMPLES 6000
#define MAGNETIC_DISTURBANCE_LIMIT 	0.1f
#define UNCOMPENSATED_MAG_DISTURBANCE_LIMIT	0.05f
#define SOFT_IRON_LETHARGY		0.8f
#define MAX_EXPECTED_INDUCTION_SLOPE	1.1f

#define CIRCLE_LIMIT (10 * 100) //!< 10 * 1/100 s delay into / out of circling state

// filters for CAN information (turn-coordinator, G-load ...)
#define ANGLE_F_BY_FS  ( 1.0f / 0.5f / 100.0f) 			// 0.5s
#define G_LOAD_F_BY_FS ( 1.0f / 0.25f / 100.0f) 		// 0.25s

// variometer tuning parameters, will be written into EEPROM as defaults if no config-file is given
#define DEFAULT_VARIO_TC       	2.0f
#define DEFAULT_AVG_VARIO_TC 	30.0f
#define DEFAULT_WIND_TC 	5.0f
#define DEFAULT_WIND_AVG_TC 	30.0f
#define NEGLECTABLE_WIND	0.01f

// AHRS tuning parameters:
// These parameters have been tuned for the flight-dynamics of gliders
// and the use of the MTI high-precision IMU
#define P_GAIN 0.03f			//!< Attitude controller: proportional gain
#define I_GAIN 0.00006f 		//!< Attitude controller: integral gain
#define H_GAIN 38.0f			//!< Attitude controller: horizontal gain
#define M_H_GAIN 6.0f			//!< Attitude controller: horizontal gain magnetic
#define CROSS_GAIN 0.05f		//!< Attitude controller: cross-product gain
#define INDUCTION_ERROR	0.015		//!< Maximum std deviation to update earth induction parameters
#define NAV_CORRECTION_LIMIT 5.0f	//!< limit for "low AHRS correcting variable"
#define HIGH_TURN_RATE 8.0*M_PI/180.0f	//!< turn rate high limit
#define LOW_TURN_RATE  1.0*M_PI/180.0f	//!< turn rate low limit
#define SPEED_COMPENSATION_FUSIONER_FEEDBACK 0.99995f // empirically tuned alpha
#define USE_OLD_FASHIONED_PRESSURE_VARIO 1 // for vario comparison tests (offline)

#define USE_ACCELERATION_CROSS_GAIN_ALONE_WHEN_CIRCLING 1 //!< if 1: do not use induction to control attitude while circling
#define DISABLE_CIRCLING_STATE		0	//!< for tests only: never use circling AHRS algorithm

#define INDUCTION_STD_DEVIATION_LIMIT	0.03 	//!< results outperforming this number will be used further on

#define AIRBORNE_TRIGGER_SPEED_COMP	0.5f //!< speed-compensator vario value m/s
#define AIRBORNE_TRIGGER_SPEED		15.0f //!< ground speed / m/s

#define MAG_SCALE			10000.0f //!< scale factor for high-precision integer statistics
#define FAST_SAMPLING_REQUENCY 		100.0f
#define FAST_SAMPLING_TIME 		0.01f
#define SLOW_SAMPLING_REQUENCY 		10.0f
#define SLOW_SAMPLING_TIME 		0.1f

#define GRAVITY				9.81f
#define VECTOR_AVERAGE_COUNT		100 //!< average count for sensor orientation setup

#endif /* NAV_ALGORITHMS_NAV_TUNING_PARAMETERS_H_ */
