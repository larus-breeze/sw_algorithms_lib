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
#include "persistent_data_file.h"
#include "NAV_tuning_parameters.h"

#if UNIX != 1
#include "my_assert.h"
#endif

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
      assert( result.slope != 0.0f);
      scale = 1.0f / result.slope;
      variance = (result.variance_offset + result.variance_slope) * 0.5f; // use variance average
    }

  //! feed in new calibration from linear least square fit
  void refresh ( float _offset, float _slope, float _variance_offset, float _variance_slope)
    {
      offset = _offset;
      assert( _slope != 0.0f);
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

//! Maintains 3 axes magnetic calibration data
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
      out[i]=calibration[i].calibrate( in[i]);
    return out;
  }

  bool set_default (void)
  {
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
      float scale_factor,
      bool mag_calibration_poor)
  {
    bool have_left = false;
    bool have_right = false;

    if( ( mag_calibrator_right[0].get_count() > MINIMUM_MAG_CALIBRATION_SAMPLES) )
      have_right = true;
    if( ( mag_calibrator_left[0].get_count() > MINIMUM_MAG_CALIBRATION_SAMPLES) )
      have_left = true;

    // now we have enough entropy and evaluate our result
    linear_least_square_result< float> new_calibration_data_right[3];
    linear_least_square_result< float> new_calibration_data_left[3];
    single_axis_calibration_t calibration_candidate[3];

    for (unsigned i = 0; i < 3; ++i)
      {
	if( have_right)
	  mag_calibrator_right[i].evaluate( new_calibration_data_right[i]);
	if( have_left)
	  mag_calibrator_left[i].evaluate( new_calibration_data_left[i]);

	if( have_right && have_left)
	  {
		  calibration_candidate[i].refresh(
		    (new_calibration_data_right[i].y_offset + new_calibration_data_left[i].y_offset) / scale_factor / TWO,
		    (new_calibration_data_right[i].slope    + new_calibration_data_left[i].slope)    /  TWO,
		    (new_calibration_data_right[i].variance_offset + new_calibration_data_left[i].variance_offset) / SQR(scale_factor) / TWO,
		    (new_calibration_data_right[i].variance_slope  + new_calibration_data_left[i].variance_slope)  / TWO);

		    mag_calibrator_right[i].reset();
		    mag_calibrator_left[i].reset();
	  }
	else
	  {
	    if( mag_calibration_poor && have_right)
	      {
		  calibration_candidate[i].refresh(
		    (new_calibration_data_right[i].y_offset) / scale_factor,
		    new_calibration_data_right[i].slope,
		    new_calibration_data_right[i].variance_offset / SQR(scale_factor),
		    new_calibration_data_right[i].variance_slope);
	      }
	    else if( mag_calibration_poor && have_left)
	      {
		  calibration_candidate[i].refresh(
		    (new_calibration_data_left[i].y_offset) / scale_factor,
		    new_calibration_data_left[i].slope,
		    new_calibration_data_left[i].variance_offset / SQR(scale_factor),
		    new_calibration_data_left[i].variance_slope);
	      }
	    else
	      return false; // have none
	  }
      }

    for (unsigned i = 0; i < 3; ++i)
      {
	calibration[i].offset = 0.25f * calibration_candidate[i].offset + 0.75f * calibration[i].offset;
	calibration[i].scale = 0.25f * calibration_candidate[i].scale + 0.75f * calibration[i].scale;
      }
    return true;
  }

  bool available () const
  {
    return calibration_done;
  }

  const single_axis_calibration_t *get_calibration(void) const
  {
    return calibration_done ? calibration : 0;
  }

  void write_into_EEPROM (void) const
    {
      if (calibration_done == false)
	return;

      if (using_permanent_data_file)
	{
	  float variance = 0.0f;
	  for (unsigned i = 0; i < 3; ++i)
	      variance += calibration[i].variance;
	  variance *= 0.3333333f; // -> mean variance

	  float persistent_parameters[7] =
	      {
		  calibration[0].offset,
		  calibration[0].scale,
		  calibration[1].offset,
		  calibration[1].scale,
		  calibration[2].offset,
		  calibration[2].scale,
		  variance
	      };

	  permanent_data_file.store_data( MAG_SENSOR_CALIBRATION, 7, persistent_parameters);
	}
      else
	{
	  float variance = 0.0f;
	  for (unsigned i = 0; i < 3; ++i)
	    {
	      write_EEPROM_value ((EEPROM_PARAMETER_ID) (MAG_X_OFF + 2 * i), calibration[i].offset);
	      write_EEPROM_value ((EEPROM_PARAMETER_ID) (MAG_X_SCALE + 2 * i), calibration[i].scale);
	      variance += calibration[i].variance;
	    }
	  write_EEPROM_value (MAG_STD_DEVIATION, SQRT(variance * 0.3333333f));
	}
    }

    bool read_from_EEPROM (void)
    {
      if ( using_permanent_data_file)
	{
	  float persistent_parameters[7]={ 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.001f};
	  bool result = permanent_data_file.retrieve_data( MAG_SENSOR_CALIBRATION, sizeof(persistent_parameters)/sizeof( float32_t), persistent_parameters);
	  float mag_variance  = persistent_parameters[6];

	  calibration[0].offset=persistent_parameters[0];
	  calibration[0].scale=persistent_parameters[1];
	  calibration[0].variance=mag_variance;
	  calibration[1].offset=persistent_parameters[2];
	  calibration[1].scale=persistent_parameters[3];
	  calibration[1].variance=mag_variance;
	  calibration[2].offset=persistent_parameters[4];
	  calibration[2].scale=persistent_parameters[5];
	  calibration[2].variance=mag_variance;
	  calibration_done = true;
	  return not result;
	}
      else
	{
	  float variance;
	  calibration_done = false;
	  if (true == read_EEPROM_value (MAG_STD_DEVIATION, variance))
	    return true; // error
	  variance = SQR(variance); // has been stored as STD DEV

	  for (unsigned i = 0; i < 3; ++i)
	    {
	      if (true
		  == read_EEPROM_value (
		      (EEPROM_PARAMETER_ID) (MAG_X_OFF + 2 * i),
		      calibration[i].offset))
		return true; // error

	      if (true
		  == read_EEPROM_value (
		      (EEPROM_PARAMETER_ID) (MAG_X_SCALE + 2 * i),
		      calibration[i].scale))
		return true; // error

	      calibration[i].variance = variance;
	    }

	  calibration_done = true;
	  return false; // no error;
	}
    }

  single_axis_calibration_t calibration[3];
  bool calibration_done;
};

#endif /* COMPASS_CALIBRATION_H_ */
