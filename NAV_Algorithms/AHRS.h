/***********************************************************************//**
 * @file		AHRS.h
 * @brief		attitude and heading reference system (interface)
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

#ifndef AHRS_H_
#define AHRS_H_

#include "embedded_math.h"
#include "quaternion.h"
#include "float3vector.h"
#include "float3matrix.h"
#include "integrator.h"
#include "compass_calibration.h"
#include "HP_LP_fusion.h"
#include "induction_observer.h"
#include "pt2.h"

extern float3vector nav_induction;

enum { ROLL, PITCH, YAW};
enum { FRONT, RIGHT, BOTTOM};
enum { NORTH, EAST, DOWN};

typedef enum  { STRAIGHT_FLIGHT, TRANSITION, CIRCLING} circle_state_t;

typedef integrator<float, float3vector> vector3integrator;

//! Attitude and heading reference system class
class AHRS_type
{
public:
	AHRS_type(float sampling_time);
	void attitude_setup( const float3vector & acceleration, const float3vector & induction);

	void update_magnetic_induction_data( float declination, float inclination)
	{
	  declination *= (M_PI_F / 180.0f); // degrees to radiant
	  inclination *= (M_PI_F / 180.0f);

	  expected_nav_induction[NORTH] = COS( inclination);
	  expected_nav_induction[EAST]  = COS( inclination) * SIN( declination);
	  expected_nav_induction[DOWN]  = SIN( inclination);
	  update_magnetic_loop_gain(); // adapt to magnetic inclination
	}

	void update( const float3vector &gyro, const float3vector &acc, const float3vector &mag,
		const float3vector &GNSS_acceleration,
		float GNSS_heading,
		bool GNSS_heading_valid
		);

	inline void set_from_euler( float r, float n, float y)
	{
		attitude.from_euler( r, n, y);
		attitude.get_rotation_matrix( body2nav);
		euler = attitude;
	}
	inline eulerangle<ftype> get_euler(void) const
	{
		return euler;
	}
	inline quaternion<ftype> get_attitude(void) const
	{
		return attitude;
	}
	inline const float3vector &get_nav_acceleration(void) const
	{
		return acceleration_nav_frame;
	}
	inline const float3vector &get_nav_induction(void) const
	{
		return induction_nav_frame;
	}
	inline float get_lin_e0(void) const
	{
		return attitude.lin_e0();
	}
	inline float get_lin_e1(void) const
	{
		return attitude.lin_e1();
	}
	inline float get_e2(void) const
	{
		return attitude.get_e2();
	}
	inline float get_north(void) const
	{
		return attitude.get_north();
	}
	inline float get_east(void) const
	{
		return attitude.get_east();
	}
	inline float get_down(void) const
	{
		return attitude.get_down();
	}
	inline float get_cross_acc_correction(void) const
	{
		return cross_acc_correction;
	}
	inline const float3vector get_orientation(void) const
	{
	  float3vector retv;
	  retv[NORTH] = get_north();
	  retv[EAST]  = get_east();
	  retv[DOWN]  = get_down();
	  return retv;
	}
	inline const float3vector &get_gyro_correction(void) const
	{
		return gyro_correction;
	}
	inline const float3matrix &get_body2nav( void) const
	{
	  return body2nav;
	}
	inline const float3vector &get_nav_correction(void)
	{
	  return nav_correction;
	}
	inline const float3vector &get_gyro_correction(void)
	{
	  return gyro_correction;
	}
	circle_state_t get_circling_state(void) const
	{
	  return circling_state;
	}
	void write_calibration_into_EEPROM( void);
  float
  getSlipAngle () const
  {
    return slip_angle_averager.get_output();
  }

  float
  getPitchAngle () const
  {
    return pitch_angle_averager.get_output();
  }

  float get_turn_rate( void ) const
  {
    return turn_rate_averager.get_output();
  }
  float get_G_load( void ) const
  {
    return G_load_averager.get_output();
  }

  void update_compass(
		  const float3vector &gyro, const float3vector &acc, const float3vector &mag,
		  const float3vector &GNSS_acceleration); //!< rotate quaternion taking angular rate readings
#if 1 // SOFT_IRON_TEST
  void update_ACC_only(
		  const float3vector &gyro, const float3vector &acc, const float3vector &mag,
		  const float3vector &GNSS_acceleration); //!< rotate quaternion taking angular rate readings
#endif
  float
  getHeadingDifferenceAhrsDgnss () const
  {
    return heading_difference_AHRS_DGNSS;
  }

  float getMagneticDisturbance () const
  {
    return magnetic_disturbance;
  }

  float3vector getBodyInductionError () const
  {
    return body_induction_error;
  }

  float3vector getBodyInduction () const
  {
    return body_induction;
  }

private:
  void handle_magnetic_calibration( char type);

  void update_magnetic_loop_gain( void)
  {
    float expected_horizontal_induction = SQRT( SQR(expected_nav_induction[EAST])+SQR(expected_nav_induction[NORTH]));
    if( expected_horizontal_induction < 0.001f) // fail-safe default
      magnetic_control_gain = M_H_GAIN;
    else
      magnetic_control_gain = M_H_GAIN / expected_horizontal_induction;
  }

  void feed_magnetic_induction_observer(const float3vector &mag_sensor);
  circle_state_t update_circling_state( void);

  void update_diff_GNSS( const float3vector &gyro, const float3vector &acc, const float3vector &mag,
	  const float3vector &GNSS_acceleration,
	  float GNSS_heading);

  void update_attitude( const float3vector &acc, const float3vector &gyro, const float3vector &mag);

  ftype Ts;
  ftype Ts_div_2;
  quaternion<ftype>attitude;
  float3vector gyro_integrator;
  unsigned circling_counter;
  circle_state_t circling_state;
  float3vector nav_correction;
  float3vector gyro_correction;
  float3vector acceleration_nav_frame;
  float3vector induction_nav_frame; 	//!< observed NAV induction
  float3vector expected_nav_induction;	//!< expected NAV induction
  float3matrix body2nav;
  eulerangle<ftype> euler;
  pt2<float,float> slip_angle_averager;
  pt2<float,float> pitch_angle_averager;
  pt2<float,float> turn_rate_averager;
  pt2<float,float> G_load_averager;
  linear_least_square_fit<int64_t, float> mag_calibration_data_collector_right_turn[3];
  linear_least_square_fit<int64_t, float> mag_calibration_data_collector_left_turn[3];
  compass_calibration_t <int64_t, float> compass_calibration;
#if USE_EARTH_INDUCTION_DATA_COLLECTOR
  induction_observer_t <int64_t> earth_induction_data_collector;
#endif
  float antenna_DOWN_correction;  //!< slave antenna lower / DGNSS base length
  float antenna_RIGHT_correction; //!< slave antenna more right / DGNSS base length
  float heading_difference_AHRS_DGNSS;
  float cross_acc_correction;
  float magnetic_disturbance; //!< abs( observed_induction - expected_induction)
  float magnetic_control_gain; //!< declination-dependent magnetic control loop gain
  bool automatic_magnetic_calibration;
  bool automatic_earth_field_parameters; // todo unused, remove me some day
  bool magnetic_calibration_updated;
  float3vector body_induction;
  float3vector body_induction_error;
};

#endif /* AHRS_H_ */
