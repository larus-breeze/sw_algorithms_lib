/***********************************************************************//**
 * @file		AHRS.cpp
 * @brief		attitude and heading reference system (implementation)
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

#include <AHRS.h>
#include "system_configuration.h"
#include "GNSS.h"
#include "magnetic_induction_report.h"
#include "embedded_memory.h"
#include "NAV_tuning_parameters.h"
#include "soft_iron_compensator.h"
#include "compass_calibrator_3D.h"

#if USE_HARDWARE_EEPROM	== 0
#include "EEPROM_emulation.h"
#endif

/**
 * @brief initial attitude setup from observables
 */
void
AHRS_type::attitude_setup (const float3vector &acceleration,
			   const float3vector &mag)
{
  float3vector north, east, down;

  float3vector induction;
  if( compass_calibration.available())
    induction = compass_calibration.calibrate( mag);
  else
    induction = mag;

  for( unsigned i=0; i<3; ++i)
    mag_filter[i].set( induction[i]);

  down = acceleration;

  down.negate ();
  down.normalize ();

  north = induction; // deviation neglected here
  north.normalize ();

  // setup world coordinate system
  east = down.vector_multiply (north);
  east.normalize ();
  north = east.vector_multiply (down);
  north.normalize ();

  // create rotation matrix from unity direction vectors
  float fcoordinates[] =
    { 	north[0], north[1], north[2],
	east[0], east[1], east[2],
	down[0], down[1], down[2] };

  float3matrix coordinates (fcoordinates);
  attitude.from_rotation_matrix (coordinates);
  attitude.get_rotation_matrix (body2nav);
  euler = attitude;
}

/**
 * @brief  decide about circling state
 */
circle_state_t
AHRS_type::update_circling_state ()
{
#if DISABLE_CIRCLING_STATE
  return STRAIGHT_FLIGHT;
#else
  float turn_rate_abs = abs (turn_rate_averager.get_output());

  if (circling_counter < CIRCLE_LIMIT)
    if (turn_rate_abs > HIGH_TURN_RATE)
      ++circling_counter;

  if (circling_counter > 0)
    if (turn_rate_abs < LOW_TURN_RATE)
      --circling_counter;

  if (circling_counter == 0)
    circling_state = STRAIGHT_FLIGHT;
  else if (circling_counter == CIRCLE_LIMIT)
    circling_state = CIRCLING;
  else
    circling_state = TRANSITION;

  return circling_state;
#endif
}

void AHRS_type::feed_magnetic_induction_observer( const float3vector &mag_sensor, const float3vector &mag_delta)
{
  float error_margin = nav_correction.abs();
  if(  error_margin > NAV_CORRECTION_LIMIT)
    return;

  bool turning_right = turn_rate_averager.get_output() > 0.0f;

#if USE_SOFT_IRON_COMPENSATION
  if( mag_delta.abs() < 0.05) // only if the precision is already reasonably good ...
    {
      bool calibration_data_complete = soft_iron_compensator.learn( mag_delta, attitude, turning_right, error_margin);
      if( calibration_data_complete)
	trigger_soft_iron_compensator_calculation();
    }
#endif

  for (unsigned i = 0; i < 3; ++i)
    if( turning_right)
      mag_calibration_data_collector_right_turn[i].add_value ( MAG_SCALE * expected_body_induction[i], MAG_SCALE * mag_sensor[i]);
    else
      mag_calibration_data_collector_left_turn[i].add_value ( MAG_SCALE * expected_body_induction[i], MAG_SCALE * mag_sensor[i]);
}

AHRS_type::AHRS_type (float sampling_time)
:
  Ts(sampling_time),
  Ts_div_2 (sampling_time / 2.0f),
  attitude(),
  gyro_integrator({0}),
  circling_counter(0),
  circling_state( STRAIGHT_FLIGHT),
  nav_correction(),
  gyro_correction(),
  acceleration_nav_frame(),
  induction_nav_frame(),
  expected_nav_induction(),
  body2nav(),
  euler(),
  slip_angle_averager( ANGLE_F_BY_FS),
  pitch_angle_averager( ANGLE_F_BY_FS),
  turn_rate_averager( ANGLE_F_BY_FS),
  G_load_averager(     G_LOAD_F_BY_FS),
  compass_calibration(),
  antenna_DOWN_correction(  configuration( ANT_SLAVE_DOWN)  / configuration( ANT_BASELENGTH)),
  antenna_RIGHT_correction( configuration( ANT_SLAVE_RIGHT) / configuration( ANT_BASELENGTH)),
  heading_difference_AHRS_DGNSS(0.0f),
  cross_acc_correction(0.0f),
  magnetic_disturbance_averager( 0.001f),
  magnetic_control_gain(1.0f),
  automatic_magnetic_calibration( (magnetic_calibration_type)(round)(configuration(MAG_AUTO_CALIB))),
  magnetic_calibration_updated( false)
{
  update_magnetic_loop_gain(); // adapt to magnetic inclination

  bool fail = compass_calibration.read_from_EEPROM();
  if( fail)
    compass_calibration.set_default();
}

void AHRS_type::tune( void)
{
  antenna_DOWN_correction = configuration( ANT_SLAVE_DOWN)  / configuration( ANT_BASELENGTH);
  antenna_RIGHT_correction = configuration( ANT_SLAVE_RIGHT) / configuration( ANT_BASELENGTH);
  automatic_magnetic_calibration = (magnetic_calibration_type)(round)(configuration(MAG_AUTO_CALIB));
}

void
AHRS_type::update (const float3vector &gyro,
		   const float3vector &acc,
		   const float3vector &mag,
		   const float3vector &GNSS_acceleration,
		   float GNSS_heading,
		   bool GNSS_heading_valid)
{
#if DISABLE_SAT_COMPASS
  update_compass(gyro, acc, mag, GNSS_acceleration);
#else
  if( GNSS_heading_valid)
    update_diff_GNSS (gyro, acc, mag, GNSS_acceleration, GNSS_heading);
  else
    update_compass(gyro, acc, mag, GNSS_acceleration);
#endif
}

/**
 * @brief  generic update of AHRS
 *
 * Side-effect: create rotation matrices, NAV-acceleration, NAV-induction
 */
void
AHRS_type::update_attitude ( const float3vector &acc,
			     const float3vector &gyro,
			     const float3vector &mag)
{
  attitude.rotate (gyro[ROLL] * Ts_div_2,
		   gyro[PITCH] * Ts_div_2,
		   gyro[YAW]  * Ts_div_2);

  attitude.normalize ();

  attitude.get_rotation_matrix (body2nav);

  acceleration_nav_frame = body2nav * acc;
  induction_nav_frame 	 = body2nav * mag;
  euler = attitude;

  float3vector nav_rotation;
  nav_rotation = body2nav * gyro;
  turn_rate_averager.respond( nav_rotation[DOWN]);

  slip_angle_averager.respond( ATAN2( -acc[RIGHT], -acc[DOWN]));
  pitch_angle_averager.respond( ATAN2( +acc[FRONT], -acc[DOWN]));
  G_load_averager.respond( acc.abs());
  magnetic_disturbance_averager.feed( (induction_nav_frame - expected_nav_induction).abs() );
}

/**
 * @brief  update attitude from IMU data D-GNSS compass
 */
void
AHRS_type::update_diff_GNSS (const float3vector &gyro,
			     const float3vector &acc,
			     const float3vector &mag_sensor,
			     const float3vector &GNSS_acceleration,
			     float GNSS_heading)
{
  float3vector mag = mag_sensor; // make it writable
  filter_magnetic_induction( gyro, mag);
  body_induction = compass_calibration.calibrate(mag);

  expected_body_induction = body2nav.reverse_map( expected_nav_induction);

#if USE_SOFT_IRON_COMPENSATION
  if( automatic_magnetic_calibration == AUTO_SOFT_IRON_COMPENSATE)
    body_induction = body_induction - soft_iron_compensator.compensate( expected_body_induction, attitude);
#endif

  body_induction_error = body_induction - expected_body_induction;

  float3vector nav_acceleration = body2nav * acc;

  float heading_gnss_work = GNSS_heading	// correct for antenna alignment
      + antenna_DOWN_correction  * SIN (euler.roll)
      - antenna_RIGHT_correction * COS (euler.roll);

  heading_gnss_work = heading_gnss_work - euler.yaw; // = heading difference D-GNSS - AHRS

  if (heading_gnss_work > M_PI_F) // map into { -PI PI}
    heading_gnss_work -= 2.0f * M_PI_F;
  if (heading_gnss_work < -M_PI_F)
    heading_gnss_work += 2.0f * M_PI_F;

  heading_difference_AHRS_DGNSS = heading_gnss_work;

  nav_correction[NORTH] = - nav_acceleration[EAST]  + GNSS_acceleration[EAST];
  nav_correction[EAST]  = + nav_acceleration[NORTH] - GNSS_acceleration[NORTH];

  cross_acc_correction = // vector cross product GNSS-acc und INS-acc -> heading error
	   + nav_acceleration[NORTH] * GNSS_acceleration[EAST]
	   - nav_acceleration[EAST]  * GNSS_acceleration[NORTH];

  circle_state_t old_circle_state = circling_state;
  update_circling_state ();

  if( circling_state == CIRCLING) // heading correction using acceleration cross product GNSS * INS
    {

#if USE_ACCELERATION_CROSS_GAIN_ALONE_WHEN_CIRCLING
      nav_correction[DOWN] = cross_acc_correction * CROSS_GAIN; // no MAG or D-GNSS use here !
#else
      float3vector nav_induction    = body2nav * body_induction;
      float mag_correction =
    	+ nav_induction[NORTH] * expected_nav_induction[EAST]
    	- nav_induction[EAST]  * expected_nav_induction[NORTH];
      nav_correction[DOWN] = cross_acc_correction * CROSS_GAIN + mag_correction * magnetic_control_gain ; // use X-ACC and MAG: better !
#endif
    }
  else
      nav_correction[DOWN]  =   heading_gnss_work * H_GAIN;

  gyro_correction = body2nav.reverse_map(nav_correction);
  gyro_correction *= P_GAIN;

  if (circling_state == STRAIGHT_FLIGHT)
      gyro_integrator += gyro_correction; // update integrator

  gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
  gyro_correction_power = SQR( gyro_correction[0]) + SQR( gyro_correction[1]) +SQR( gyro_correction[2]);

  update_attitude (acc, gyro + gyro_correction, body_induction);

  // only here we get fresh magnetic entropy
  // and: wait for low control loop error
  if ( circling_state == CIRCLING)
    feed_magnetic_induction_observer (mag_sensor, body_induction_error);

  // when circling is finished eventually update the magnetic calibration
  if (automatic_magnetic_calibration && (old_circle_state == CIRCLING) && (circling_state == TRANSITION))
	  handle_magnetic_calibration();
}

/**
 * @brief  update attitude from IMU data and magnetometer
 */
void
AHRS_type::update_compass (const float3vector &gyro, const float3vector &acc,
			   const float3vector &mag_sensor,
			   const float3vector &GNSS_acceleration)
{
  float3vector mag = mag_sensor;
  filter_magnetic_induction( gyro, mag);
  body_induction = compass_calibration.calibrate(mag);

  expected_body_induction = body2nav.reverse_map( expected_nav_induction);

#if USE_SOFT_IRON_COMPENSATION
  if( automatic_magnetic_calibration == AUTO_SOFT_IRON_COMPENSATE)
    body_induction = body_induction - soft_iron_compensator.compensate( expected_body_induction, attitude);
#endif

  body_induction_error = body_induction - expected_body_induction;

  float3vector nav_acceleration = body2nav * acc;
  float3vector nav_induction = body2nav * body_induction;

  // calculate horizontal leveling error
  nav_correction[NORTH] = -nav_acceleration[EAST] + GNSS_acceleration[EAST];
  nav_correction[EAST] = +nav_acceleration[NORTH] - GNSS_acceleration[NORTH];

  // *******************************************************************************************************
  // calculate heading error depending on the present circling state
  // on state changes handle MAG auto calibration

  float mag_correction =
      + nav_induction[NORTH] * expected_nav_induction[EAST]
      - nav_induction[EAST]  * expected_nav_induction[NORTH];

  cross_acc_correction = // vector cross product GNSS-acc und INS-acc -> heading error
	   + nav_acceleration[NORTH] * GNSS_acceleration[EAST]
	   - nav_acceleration[EAST]  * GNSS_acceleration[NORTH];

  circle_state_t old_circle_state = circling_state;
  update_circling_state ();

  switch (circling_state)
    {
    case STRAIGHT_FLIGHT:
    case TRANSITION:
    default:
      {
	nav_correction[DOWN] = magnetic_control_gain * mag_correction;
	gyro_correction = body2nav.reverse_map (nav_correction);
	gyro_correction *= P_GAIN;
	gyro_integrator += gyro_correction; // update integrator
      }
      break;
      // *******************************************************************************************************
    case CIRCLING:
      {
#if USE_ACCELERATION_CROSS_GAIN_ALONE_WHEN_CIRCLING
	    nav_correction[DOWN] = cross_acc_correction * CROSS_GAIN; // no MAG or D-GNSS use here ! (old version)
#else
	nav_correction[DOWN] = cross_acc_correction * CROSS_GAIN
	    + mag_correction * M_H_GAIN; // use cross-acceleration and induction: better !
#endif
	gyro_correction = body2nav.reverse_map (nav_correction);
	gyro_correction *= P_GAIN;
      }
      break;
    }

  gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
  gyro_correction_power = SQR( gyro_correction[0]) + SQR( gyro_correction[1]) +SQR( gyro_correction[2]);

  // feed quaternion update with corrected sensor readings
  update_attitude (acc, gyro + gyro_correction, body_induction);

  // only here we get fresh magnetic entropy
  // and: wait for low control loop error
  if ( (circling_state == CIRCLING) && ( nav_correction.abs() < NAV_CORRECTION_LIMIT))
    feed_magnetic_induction_observer (mag_sensor, body_induction_error);

 // when circling is finished eventually update the magnetic calibration
 if (automatic_magnetic_calibration && (old_circle_state == CIRCLING) && (circling_state == TRANSITION))
  handle_magnetic_calibration();
}

void AHRS_type::write_calibration_into_EEPROM( void)
{
  if( ! magnetic_calibration_updated)
    return;

  compass_calibration.write_into_EEPROM();
  magnetic_calibration_updated = false; // done ...
}

void AHRS_type::handle_magnetic_calibration ( void)
{
  bool poor_mag_calibration = magnetic_disturbance_averager.get_output() > MAGNETIC_DISTURBANCE_LIMIT;
  bool calibration_changed = compass_calibration.set_calibration_if_changed ( mag_calibration_data_collector_right_turn, mag_calibration_data_collector_left_turn, MAG_SCALE, poor_mag_calibration);

  if( calibration_changed)
    {
#if REPORT_MAGNETIC_CALIBRATION == 1
      magnetic_induction_report_t magnetic_induction_report;
      for( unsigned i=0; i<3; ++i)
	magnetic_induction_report.calibration[i] = (compass_calibration.get_calibration())[i];
#endif
      magnetic_calibration_updated = true;
    }
}

void AHRS_type::filter_magnetic_induction( const float3vector &gyro, float3vector &mag)
{
  float absolute_rotation = gyro.abs();
  for( unsigned i=0; i<3; ++i)
	mag[i] = mag_filter[i].respond(mag[i], absolute_rotation * Ts * MAX_EXPECTED_INDUCTION_SLOPE);
}
