/***********************************************************************//**
 * @file		compass_calibration.h
 * @brief		Automatic compass calibration using a linear least square fit algorithm
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

#ifndef COMPASS_CALIBRATION_H_
#define COMPASS_CALIBRATION_H_

#include "system_configuration.h"
#include "float3vector.h"
#include "Linear_Least_Square_Fit.h"
#include "persistent_data.h"
#include "NAV_tuning_parameters.h"

//! maintain offset and slope data for one sensor axis
class single_axis_calibration_t
{
public:
  single_axis_calibration_t( float _offset=0.0f, float slope=1.0f)
  : offset( _offset),
    scale( 1.0f / slope),
    variance ( 1.0e10f)
  {}

  //! feed in new calibration from linear least square fit
  void refresh ( linear_least_square_result<float> &result)
    {
      offset = result.y_offset;
      scale = 1.0f / result.slope;
      variance = (result.variance_offset + result.variance_slope) * 0.5f; // use variance average
    }

  //! feed in new calibration from linear least square fit
  void refresh ( float _offset, float _slope, float _variance_offset, float _variance_slope)
    {
      offset = _offset;
      scale = 1.0f / _slope;
      variance = ( _variance_offset + _variance_slope) * 0.5f; // use variance average
    }

  //! calibrate instant sensor reading using calibration data
  float calibrate( float sensor_reading)
  {
    return (sensor_reading - offset) * scale;
  }

  float offset; 	//!< sensor offset in sensor units
  float scale;  	//!< convert sensor-units into SI-data
  float variance; 	//!< measure of precision: sensor calibration parameter variance
};

//! this class maintains 3d magnetic calibration data
template <class sample_type, class evaluation_type> class compass_calibration_t
{
public:
  compass_calibration_t( void)
    : calibration_done( false)
  {}

  float3vector calibrate( const float3vector &in)
  {
    float3vector out;

    // shortcut while no data available
    if( !calibration_done)
      return in;

    for( unsigned i=0; i<3; ++i)
      out.e[i]=calibration[i].calibrate( in.e[i]);
    return out;
  }

  bool set_default (void)
  {
    float variance;
    calibration_done = true;
    for( unsigned i=0; i<3; ++i)
      {
        calibration[i].offset = 0.0f;
        calibration[i].scale = 1.0f;
        calibration[i].variance = 0.1f;
      }

    calibration_done = true;
    return false; // no error;
  }


  bool set_calibration_if_changed(
      linear_least_square_fit<sample_type, evaluation_type> mag_calibrator_right[3],
      linear_least_square_fit<sample_type, evaluation_type> mag_calibrator_left[3],
      float scale_factor)
  {
    if( ( mag_calibrator_right[0].get_count() < MINIMUM_MAG_CALIBRATION_SAMPLES) )
      return false;
    if( ( mag_calibrator_left[0].get_count() < MINIMUM_MAG_CALIBRATION_SAMPLES) )
      return false;

    // now we have enough entropy and evaluate our result
    linear_least_square_result< float> new_calibration_data_right[3];
    linear_least_square_result< float> new_calibration_data_left[3];
    single_axis_calibration_t calibration_candidate[3];

    for (unsigned i = 0; i < 3; ++i)
      {
	mag_calibrator_right[i].evaluate( new_calibration_data_right[i]);
	mag_calibrator_left[i].evaluate( new_calibration_data_left[i]);

	float vo_r = ONE; //new_calibration_data_right[i].variance_offset;
	float vo_l =  ONE; //new_calibration_data_left[i].variance_offset;
	float vs_r =  ONE; //new_calibration_data_right[i].variance_slope;
	float vs_l =  ONE; //new_calibration_data_left[i].variance_slope;

	calibration_candidate[i].refresh(
	    (new_calibration_data_right[i].y_offset * vo_l + new_calibration_data_left[i].y_offset * vo_r) / scale_factor / (vo_l + vo_r),
	    (new_calibration_data_right[i].slope * vs_l    + new_calibration_data_left[i].slope * vs_r)    / (vs_r + vs_l),
	    (new_calibration_data_right[i].variance_offset + new_calibration_data_left[i].variance_offset) / SQR(scale_factor) / TWO,
	    (new_calibration_data_right[i].variance_slope  + new_calibration_data_left[i].variance_slope)  / TWO);

	mag_calibrator_right[i].reset();
	mag_calibrator_left[i].reset();
      }

    if( ! parameters_changed_significantly ( calibration_candidate))
	return false; // we keep the old calibration

    // now we take the new calibration
    for (unsigned i = 0; i < 3; ++i)
      calibration[i] = calibration_candidate[i];

    write_into_EEPROM();

    return true;
  }

  bool isCalibrationDone () const
  {
    return calibration_done;
  }

  const single_axis_calibration_t *get_calibration(void) const
  {
    return calibration_done ? calibration : 0;
  }

  bool parameters_changed_significantly ( single_axis_calibration_t const *new_calibration) const
  {
    for( unsigned i=0; i<3; ++i)
      {
	if( fabs( new_calibration[i].offset - calibration[i].offset) > MAG_OFFSET_CHANGE_LIMIT)
	  return true;

	if( fabs( new_calibration[i].scale - calibration[i].scale) > MAG_SCALE_CHANGE_LIMIT)
	  return true;

	if( new_calibration[i].variance < calibration[i].variance)
	  return true;
}
    return false;
  }

  void write_into_EEPROM (void) const
  {
    if( calibration_done == false)
      return;

    EEPROM_initialize();

    float variance = 0.0f;
    for( unsigned i=0; i<3; ++i)
      {
        write_EEPROM_value( (EEPROM_PARAMETER_ID)(MAG_X_OFF   + 2*i), calibration[i].offset);
        write_EEPROM_value( (EEPROM_PARAMETER_ID)(MAG_X_SCALE + 2*i), calibration[i].scale);
        variance += calibration[i].variance;
      }
    write_EEPROM_value(MAG_STD_DEVIATION, SQRT( variance / 6.0f));
  }

  bool read_from_EEPROM (void)
  {
    float variance;
    calibration_done = false;
    if( true == read_EEPROM_value( MAG_STD_DEVIATION, variance))
      return true; // error
    variance = SQR( variance); // has been stored as STD DEV

    for( unsigned i=0; i<3; ++i)
      {
        if( true == read_EEPROM_value( (EEPROM_PARAMETER_ID)(MAG_X_OFF   + 2*i), calibration[i].offset))
  	    return true; // error

        if( true == read_EEPROM_value( (EEPROM_PARAMETER_ID)(MAG_X_SCALE + 2*i), calibration[i].scale))
  	    return true; // error

        calibration[i].variance = variance;
      }

    calibration_done = true;
    return false; // no error;
  }

  single_axis_calibration_t calibration[3];
  bool calibration_done;
};

#endif /* COMPASS_CALIBRATION_H_ */
