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

#define MINIMUM_MAG_CALIBRATION_SAMPLES 6000
#define MAG_CALIBRATION_CHANGE_LIMIT 6.0e-4f //!< variance average of changes: 3 * { offset, scale }
//this means an average change of all 6 parameters of 1 % STD-deviation (= 1e-4 variance)

#define CIRCLE_LIMIT (10 * 100) //!< 10 * 1/100 s delay into / out of circling state

#define VARIO_USE_SQUARED_VELOCITY 1 // use squared absolute air velocity for speed-compensation

// filters for CAN information (turn-coordinator, G-load ...)
#define ANGLE_F_BY_FS  ( 1.0f / 0.5f / 100.0f) 			// 0.5s
#define G_LOAD_F_BY_FS ( 1.0f / 0.25f / 100.0f) 		// 0.25s

// variometer tuning parameters, will be written into EEPROM as defaults if no config-file is given
#define DEFAULT_VARIO_TC       	2.0f
#define DEFAULT_AVG_VARIO_TC 	30.0f
#define DEFAULT_WIND_TC 	5.0f
#define DEFAULT_WIND_AVG_TC 	30.0f

// AHRS tuning parameters:
// These parameters have been tuned for the flight-dynamics of gliders
// and the use of the MTI high-precision IMU
#define P_GAIN 0.03f			//!< Attitude controller: proportional gain
#define I_GAIN 0.00006f 		//!< Attitude controller: integral gain
#define H_GAIN 38.0f			//!< Attitude controller: horizontal gain
#define M_H_GAIN 6.0f			//!< Attitude controller: horizontal gain magnetic
#define CROSS_GAIN 0.05f		//!< Attitude controller: cross-product gain
#define INDUCTION_ERROR	0.03		//!< Maximum std deviation to update earth induction parameters
#define NAV_CORRECTION_LIMIT 5.0f	//!< limit for "low AHRS correcting variable"
#define HIGH_TURN_RATE 8.0*M_PI/180.0f	//!< turn rate high limit
#define LOW_TURN_RATE  4.0*M_PI/180.0f	//!< turn rate low limit
#define SPEED_COMPENSATION_FUSIONER_FEEDBACK 0.99f // empirically tuned alpha

#define UPDATE_MAGNETIC_CALIB		1	//!< if 1: update magnetic calibration when new data are available
#define MODIFY_EXPECTED_INDUCTION	1 	//!< set to 1 to update inclination and declination automatically
#define CROSS_GAIN_ONLY			0 	//!< if 1: do not use induction to control attitude while circling
#define MAGNETIC_CALIB_FROM_EEPROM	1 	//!< initially read data from EEPROM, may be set to 0 for tests
#define DISABLE_CIRCLING_STATE		0	//!< for tests only: never use circling AHRS algorithm

#define FAST_SAMPLING_REQUENCY 		100.0f
#define FAST_SAMPLING_TIME 		0.01f
#define SLOW_SAMPLING_REQUENCY 		10.0f
#define SLOW_SAMPLING_TIME 		0.1f

#define GRAVITY				9.81f

#endif /* NAV_ALGORITHMS_NAV_TUNING_PARAMETERS_H_ */
