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
#include "soft_iron_compensator.h"
#include "HP_LP_fusion.h"
#include "pt2.h"
#include "slope_limiter.h"
#include "RMS_rectifier.h"
#include "delay_line.h"
#include "gyro_gain_adjust.h"

enum { ROLL, PITCH, HEADING};	//!< euler angles
enum { FRONT, RIGHT, BOTTOM};	//!< BODY frame
enum { NORTH, EAST, DOWN};	//!< NAV frame

typedef enum  { STRAIGHT_FLIGHT, TRANSITION, CIRCLING, ON_GROUND} flight_state_t;

typedef integrator<float, float3vector> vector3integrator;

//! Attitude and heading reference system
class AHRS_type
{
public:
  enum magnetic_calibration_type
  {
    NONE, AUTO_1D, AUTO_SOFT_IRON_COMPENSATE
  };

  AHRS_type (float sampling_time);

  void attitude_setup (const float3vector &acceleration,
		  const float3vector &induction);

  void tune (void);

  //! compute and remember any changes in the earth's magnetic induction
  void update_magnetic_induction_data (float declination, float inclination)
  {
    declination *= (M_PI_F / 180.0f); // degrees to radiant
    inclination *= (M_PI_F / 180.0f);

    expected_nav_induction[NORTH] = COS(inclination);
    expected_nav_induction[EAST] = COS( inclination) * SIN(declination);
    expected_nav_induction[DOWN] = SIN(inclination);
    update_magnetic_loop_gain (); // adapt to magnetic inclination
  }

  void set_earth_rotation (float latitude)
  {
    earth_rotation[NORTH] = COS( latitude) * OMEGA_EARTH;
    earth_rotation[EAST] = 0.0f;
    earth_rotation[DOWN] = SIN( latitude) * OMEGA_EARTH;
  }

  //! update the AHRS taking the current measurement data
  void update (const float3vector &gyro, const float3vector &acc,
	       const float3vector &mag, const float3vector &GNSS_acceleration,
	  float GNSS_heading, bool GNSS_heading_valid,
	  const float3vector &x_mag, bool x_mag_valid);

  //! set the AHRS attitude using known euler angles
  inline void set_from_euler (float roll, float pitch, float heading)
  {
    attitude.from_euler (roll, pitch, heading);
    attitude.get_rotation_matrix (body2nav);
    euler = attitude;
  }

  inline eulerangle<ftype> get_euler (void) const
  {
    return euler;
  }

  inline quaternion<ftype> get_attitude (void) const
  {
    return attitude;
  }

  inline const float3vector& get_nav_acceleration (void) const
  {
    return acceleration_nav_frame;
  }

  inline const float3vector& get_nav_induction (void) const
  {
    return induction_nav_frame;
  }

  inline float get_north (void) const
  {
    return attitude.get_north ();
  }

  inline float get_east (void) const
  {
    return attitude.get_east ();
  }

  inline float get_down (void) const
  {
    return attitude.get_down ();
  }

  inline float get_cross_acc_correction (void) const
  {
    return cross_acc_correction;
  }

  inline const float3vector get_orientation (void) const
  {
    float3vector retv;
    retv[NORTH] = get_north ();
    retv[EAST] = get_east ();
    retv[DOWN] = get_down ();
    return retv;
  }

  inline const float3vector& get_gyro_correction (void) const
  {
    return gyro_correction;
  }

  inline const float3matrix& get_body2nav (void) const
  {
    return body2nav;
  }

  inline const float3vector& get_nav_correction (void)
  {
    return nav_correction;
  }

  inline const float3vector& get_gyro_correction (void)
  {
    return gyro_correction;
  }

  flight_state_t get_circling_state (void) const
  {
    return circling_state;
  }
  void write_calibration_into_EEPROM (void);

  float getSlipAngle () const
  {
    return slip_angle_averager.get_output ();
  }

  float getPitchAngle () const
  {
    return pitch_angle_averager.get_output ();
  }

  float get_turn_rate (void) const
  {
    return turn_rate_averager.get_output ();
  }

  float get_G_load (void) const
  {
    return G_load_averager.get_output ();
  }

  //! update the AHRS taking magnetic compass data as a reference
  void update_compass (
    const float3vector &gyro, const float3vector &acc,
    const float3vector &mag,
    const float3vector &GNSS_acceleration,
    const float3vector &x_mag, bool x_mag_valid);

  //! update the AHRS taking D-GNSS compass data as a reference
  void update_diff_GNSS (
    const float3vector &gyro, const float3vector &acc,
    const float3vector &mag,
    const float3vector &GNSS_acceleration, float GNSS_heading,
    const float3vector &x_mag, bool x_mag_valid);

  //! update the AHRS taking only the acceleration as a reference
  void update_ACC_only (const float3vector &gyro, const float3vector &acc,
    const float3vector &mag,
    const float3vector &GNSS_acceleration);

  float getHeadingDifferenceAhrsDgnss () const
  {
    return heading_difference_AHRS_DGNSS;
  }

  float getMagneticDisturbance () const
  {
    return magnetic_disturbance_averager.get_output ();
  }

  float3vector getBodyInductionError () const
  {
    return body_induction_error;
//    return attitude_error;
  }

  float3vector getBodyInduction () const
  {
    return body_induction;
  }

  float getGyro_correction_Power () const
  {
//    return gyro_correction_power; todo patch
    return uncompensated_magnetic_disturbance_averager.get_output ();
  }

private:
  enum heading_type
  {
    MAGNETIC, D_GNSS
  };

  void handle_magnetic_induction( float3vector measured_induction, const float3vector &x_mag_, bool x_mag_valid, const float3vector &gyro);
  void handle_magnetic_calibration (void);

  //! generic helper function to update the AHRS attitude
  void update_attitude (const float3vector &acc, const float3vector &gyro,
		   const float3vector &mag);

  void update_magnetic_loop_gain (void)
  {
    float expected_horizontal_induction = SQRT(
	SQR(expected_nav_induction[EAST])+SQR(expected_nav_induction[NORTH]));
    if (expected_horizontal_induction < 0.001f) // fail-safe default
      magnetic_control_gain = M_H_GAIN;
    else
      magnetic_control_gain = M_H_GAIN / expected_horizontal_induction;
  }

  void feed_magnetic_induction_observer (
      const float3vector &mag_sensor,
      const float3vector &mag_delta,
      bool do_soft_iron_correction);

  flight_state_t
  update_circling_state (void);

  void filter_magnetic_induction (const float3vector &gyro, float3vector &mag);

  ftype Ts;
  ftype Ts_div_2;
  quaternion<ftype> attitude;
  float3vector gyro_integrator;
  unsigned circling_counter;
  flight_state_t circling_state;
  heading_type heading_source;
  bool heading_source_changed;
  float3vector nav_correction;
  float3vector gyro_correction;
  float3vector earth_rotation;
  float3vector acceleration_nav_frame;
  float3vector induction_nav_frame; 	//!< observed NAV induction
  float3vector expected_nav_induction;	//!< expected NAV induction
  float3vector expected_body_induction;	//!< expected body frame induction
  float3matrix body2nav;
  eulerangle<ftype> euler;
  pt2<float, float> slip_angle_averager;
  pt2<float, float> pitch_angle_averager;
  pt2<float, float> turn_rate_averager;
  pt2<float, float> G_load_averager;
  linear_least_square_fit<int64_t, float> mag_calibration_data_collector_right_turn[3];
  linear_least_square_fit<int64_t, float> mag_calibration_data_collector_left_turn[3];
  compass_calibration_t<int64_t, float> compass_calibration;
  float antenna_DOWN_correction;  //!< slave antenna lower / DGNSS base length
  float antenna_RIGHT_correction; //!< slave antenna more right / DGNSS base length
  float heading_difference_AHRS_DGNSS;
  float cross_acc_correction;
  RMS_rectifier<float> magnetic_disturbance_averager; //!< abs( observed_induction - expected_induction)
  RMS_rectifier<float> uncompensated_magnetic_disturbance_averager; //!< abs( observed_induction - expected_induction)
  float magnetic_control_gain; //!< declination-dependent magnetic control loop gain
  magnetic_calibration_type automatic_magnetic_calibration;
  bool magnetic_calibration_updated;
  float3vector body_induction;
  float3vector corrected_body_induction;
  float3vector body_induction_error;
  float gyro_correction_power;
  slope_limiter<float> mag_filter[3];
  delay_line<float3vector, MAX_GNSS_DELAY> GNSS_delay_compensation;
  delay_line<float, MAX_GNSS_DELAY> GNSS_heading_delay_compensation;
  float3vector attitude_error;
};

#endif /* AHRS_H_ */
