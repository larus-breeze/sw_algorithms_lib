/*
 * compass_calibration.h
 *
 *  Created on: Apr 27, 2021
 *      Author: schaefer
 */

#ifndef COMPASS_CALIBRATION_H_
#define COMPASS_CALIBRATION_H_

#include "system_configuration.h"
#include "float3vector.h"
#include "Linear_Least_Square_Fit.h"
#include "persistent_data.h"

#if WRITE_MAG_CALIB_EEPROM
#include "FreeRTOS_wrapper.h"
extern Queue< linear_least_square_result<float>[3] > magnetic_calibration_queue;
#endif

class calibration_t
{
public:
  calibration_t( float _offset=0.0f, float slope=1.0f)
  : offset( _offset),
    scale( 1.0f / slope),
    variance ( 1.0e10f)
  {}
  bool is_initialized( void) const
  {
    return variance < 1.0e9f;
  }
  void refresh ( linear_least_square_result<float> &result)
    {
    if( ! is_initialized()) // first calibration coming in
	{
	  offset = result.y_offset;
	  scale = 1.0f / result.slope;
	  variance = result.variance_offset + result.variance_slope;
	}
      else
	{
	  offset   = offset   * MAG_CALIB_LETHARGY + ( result.y_offset          * (1.0f - MAG_CALIB_LETHARGY));
	  scale    = scale    * MAG_CALIB_LETHARGY + ( (1.0f / result.slope) * (1.0f - MAG_CALIB_LETHARGY)) ;
	  variance = variance * MAG_CALIB_LETHARGY + ( (result.variance_offset + result.variance_slope)
									 * (1.0f - MAG_CALIB_LETHARGY));
	}
    }

  float calibrate( float sensor_reading)
  {
    return (sensor_reading - offset) * scale;
  }

  float offset; 	//!< sensor offset in sensor units
  float scale;  	//!< convert sensor-units into SI-data
  float variance; 	//!< measure of precision: sensor calibration parameter variance
};

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
  void set_calibration( linear_least_square_fit<float> mag_calibrator[3], char id, bool turning_right, bool check_samples=true)
  {
    if( turning_right)
      completeness |= HAVE_RIGHT;
    else
      completeness |= HAVE_LEFT;

    if( check_samples && !
	( (completeness == HAVE_BOTH) && ( mag_calibrator[0].get_count() > MINIMUM_MAG_CALIBRATION_SAMPLES) )
	)
      return; // not enough entropy

    linear_least_square_result<float> new_calibration[3];
    new_calibration[0].id=id;

    for (unsigned i = 0; i < 3; ++i)
      {
	mag_calibrator[i].evaluate( new_calibration[i]);
        calibration[i].refresh ( new_calibration[i]);
        mag_calibrator[i].reset();
      }
    calibration_done = true; // at least one calibration has been done now
    completeness = HAVE_NONE; // restart acquisition

#if WRITE_MAG_CALIB_EEPROM
    magnetic_calibration_queue.send(new_calibration, 0);
#endif
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
    return retv * 0.33333f;
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
  write_EEPROM_value(MAG_STD_DEVIATION, SQRT( variance / 3.0f));
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

#endif /* COMPASS_CALIBRATION_H_ */
