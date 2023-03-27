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
class calibration_t
{
public:
  calibration_t( float _offset=0.0f, float slope=1.0f)
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
class compass_calibration_t
{
public:
  compass_calibration_t( void)
    : completeness( HAVE_NONE),
      calibration_done( false)
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

  bool set_default (void);

bool set_calibration_if_changed( linear_least_square_fit<float> mag_calibrator[3], bool turning_right)
  {
    if( turning_right)
      completeness |= HAVE_RIGHT;
    else
      completeness |= HAVE_LEFT;

    if( false == (completeness == HAVE_BOTH))
      return false;

    if( ( mag_calibrator[0].get_count() < MINIMUM_MAG_CALIBRATION_SAMPLES) )
      return false;

    // now we have enough entropy and evaluate our result
    linear_least_square_result<float> new_calibration[3];

    float variance = 0;
    for (unsigned i = 0; i < 3; ++i)
      {
	mag_calibrator[i].evaluate( new_calibration[i]);
	variance += new_calibration[i].variance_offset;
	variance += new_calibration[i].variance_slope;
        mag_calibrator[i].reset();
      }

    variance *= 0.1666666f; // gives us the mean value

    if( SQRT( variance) > configuration(MAG_STD_DEVIATION))
      return false; // measurement precision has not improved

    for (unsigned i = 0; i < 3; ++i)
      calibration[i].refresh ( new_calibration[i]);

    if( parameters_changed_significantly())
      {
      write_into_EEPROM();
      return true;
      }
    return false;
  }

  bool isCalibrationDone () const
  {
    return calibration_done;
  }

  float get_variance_average( void) const
  {
    float retv = 0.0f;
    for( unsigned i=0; i < 3; ++i)
      retv += calibration[i].variance;
    return retv * 0.33333333f;
  }

  const calibration_t *get_calibration(void) const
  {
    return calibration_done ? calibration : 0;
  }

  bool parameters_changed_significantly(void) const;
  void write_into_EEPROM( void) const;
  bool read_from_EEPROM( void); // false if OK
// private:
  enum completeness_type { HAVE_NONE=0, HAVE_RIGHT=1, HAVE_LEFT=2, HAVE_BOTH=3};
  unsigned completeness; // bits from completeness_type
  bool calibration_done;
  calibration_t calibration[3];
};

inline bool compass_calibration_t::parameters_changed_significantly (void) const
{
  float parameter_change_variance = 0.0f;
  for( unsigned i=0; i<3; ++i)
    {
      parameter_change_variance +=
	  SQR( configuration( (EEPROM_PARAMETER_ID)(MAG_X_OFF   + 2*i)) - calibration[i].offset) +
	  SQR( configuration( (EEPROM_PARAMETER_ID)(MAG_X_SCALE + 2*i)) - calibration[i].scale);
    }
  parameter_change_variance /= 6.0f; // => average
  return parameter_change_variance > MAG_CALIBRATION_CHANGE_LIMIT;
}

inline void compass_calibration_t::write_into_EEPROM (void) const
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

inline bool compass_calibration_t::read_from_EEPROM (void)
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

inline bool compass_calibration_t::set_default (void)
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

#endif /* COMPASS_CALIBRATION_H_ */
