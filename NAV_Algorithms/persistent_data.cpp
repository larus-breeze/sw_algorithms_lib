/***********************************************************************//**
 * @file 		persistent_data.cpp
 * @brief 		definitions for persistent data in EEPROM or config. file
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

#include "system_configuration.h"
#include "embedded_memory.h"
#include "embedded_math.h"
#include "persistent_data.h"
#include "string.h"

ROM persistent_data_t PERSISTENT_DATA[]=
    {
	{SENS_TILT_ROLL,"SensTilt_Roll",	true,  0.0f, 0}, 	//! IMU Sensor tilt angle signed / degrees front right down frame
	{SENS_TILT_PITCH,"SensTilt_Pitch",	true,  0.0f, 0}, 	//! IMU Sensor tilt angle signed
	{SENS_TILT_YAW, "SensTilt_Yaw",		true,  0.0f, 0},  	//! IMU Sensor tilt angle signed

	{PITOT_OFFSET,	"Pitot_Offset",		false,  0.0f, 0},	//! Pitot offset signed / Pa
	{PITOT_SPAN, 	"Pitot_Span",		false,  1.0f, 0},	//! Pitot Span signed (around 1.0f)
	{QNH_OFFSET, 	"QNH-delta",		false,  0.0f, 0},	//! Absolute pressure sensor offset signed / Pa

	{MAG_X_OFF,	"Mag_X_Off",		false,  0.0f, 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_X_SCALE,	"Mag_X_Scale",		false,  1.0f, 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_Y_OFF,	"Mag_Y_Off",		false,  0.0f, 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_Y_SCALE,	"Mag_Y_Scale", 		false,  1.0f, 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_Z_OFF,	"Mag_Z_Off",		false,  0.0f, 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_Z_SCALE,	"Mag_Z_Scale",		false,  1.0f, 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_STD_DEVIATION, "Mag_Calib_Err",	false,  1e-2f, 0},	//! Magnetic calibration STD deviation / ( 1 % / 65536 )
	{MAG_AUTO_CALIB, "Mag_Auto_Calib",	false,  2.0f, 0},	//! Magnetic calibration automatic { OFF=0, 2D=1, 3D=2 }

	{VARIO_TC,	"Vario_TC",		false, 2.0f, 0}, 	//! Vario time constant unsigned s / ( 100.0f / 65536 )
	{VARIO_INT_TC,	"Vario_Int_TC",		false, 30.0f, 0},	//! Vario integrator time constant unsigned s / ( 100.0f / 65536 )
	{WIND_TC,	"Wind_TC",		false, 5.0f, 0}, 	//! Wind fast time constant unsigned s / ( 100.0f / 65536 )
	{MEAN_WIND_TC,	"Mean_Wind_TC",		false, 30.0f, 0},	//! Wind slow time constant unsigned s / ( 100.0f / 65536 )
	{HORIZON,	"Horizon_active",	false, 1.0f, 0},	//! Horizon output is available
	{VARIO_P_TC,	"Vario_P_TC",		false, 5.0f, 0}, 	//! Pneumatic Vario time constant unsigned s / ( 100.0f / 65536 )

	{GNSS_CONFIGURATION, "GNSS_CONFIG",	false, 1.0f, 0},	//! type of GNSS system
	{ANT_BASELENGTH, "ANT_BASELEN",		false, 1.0f, 0},	//! Slave DGNSS antenna baselength / mm
	{ANT_SLAVE_DOWN, "ANT_SLAVE_DOWN",	false, 0.0f, 0},	//! Slave DGNSS antenna lower / mm
	{ANT_SLAVE_RIGHT,"ANT_SLAVE_RIGHT",	false, 0.0f, 0},	//! Slave DGNSS antenna more right /mm
    };

ROM unsigned PERSISTENT_DATA_ENTRIES = sizeof(PERSISTENT_DATA) / sizeof(persistent_data_t);

bool all_EEPROM_parameters_available( void)
{
  float dummy;
  for( 	const persistent_data_t * parameter = PERSISTENT_DATA;
	parameter < PERSISTENT_DATA + PERSISTENT_DATA_ENTRIES;
	++parameter)
    {
      if( true == read_EEPROM_value( parameter->id, dummy)) // parameter missing
	return false;
    }
  return true;
}

void ensure_EEPROM_parameter_integrity( void)
{
  float dummy;
  const persistent_data_t * parameter = PERSISTENT_DATA;
  while( parameter < PERSISTENT_DATA + PERSISTENT_DATA_ENTRIES)
    {
      if( true == read_EEPROM_value( parameter->id, dummy)) // parameter missing
	{
	  (void) write_EEPROM_value( parameter->id, parameter->default_value);
	}
      ++parameter;
    }
}

const persistent_data_t * find_parameter_from_name( char * name)
{
  for( const persistent_data_t *parameter = PERSISTENT_DATA; parameter < (PERSISTENT_DATA+PERSISTENT_DATA_ENTRIES); ++parameter )
    if( 0 == strncmp( parameter->mnemonic, name, strlen( parameter->mnemonic)))
      return parameter;
  return 0;
}

const persistent_data_t * find_parameter_from_ID( EEPROM_PARAMETER_ID id)
{
  for( const persistent_data_t *parameter = PERSISTENT_DATA; parameter < (PERSISTENT_DATA+PERSISTENT_DATA_ENTRIES); ++parameter )
    if( parameter->id == id)
      return parameter;
  return 0;
}

#if UNIX != 1

#include "eeprom.h"
#include "my_assert.h"

#define READ true
#define WRITE false

bool EEPROM_convert( EEPROM_PARAMETER_ID id, EEPROM_data_t & EEPROM_value, float & value , bool read )
{
  switch(id)
  {
    case BOARD_ID:
    case GNSS_CONFIGURATION:
    case MAG_AUTO_CALIB:
    case HORIZON:
      if( read)
	value = (float)(EEPROM_value.u16);
      else
	EEPROM_value.u16 = (uint16_t)round(value);
      break;
    case QNH_OFFSET:
    case PITOT_OFFSET:
      if( read)
	value = ( float)(EEPROM_value.i16);
      else
	EEPROM_value.i16 = (int16_t)value;
      break;
    case PITOT_SPAN:
    case MAG_X_SCALE:
    case MAG_Y_SCALE:
    case MAG_Z_SCALE:
      if( read)
	value = ( (float)(EEPROM_value.i16) / 32768.0f) + 1.0f;
      else
	{
	  int ivalue = (int)((value - 1.0f) * 32768.0f);
	  if( ivalue > 32767)
	    ivalue = 32767;
	  if( ivalue < -32768)
	    ivalue = -32768;
	  EEPROM_value.i16 = (int16_t)ivalue;
	}
      break;
    case MAG_STD_DEVIATION:
      if( read)
	value = (float)(EEPROM_value.u16) / 65536.0f * 1e-2f;
      else
	{
	  if( value >= 0.009999f || value < 0.0f)
	    EEPROM_value.u16 = 0xffff;
	  else
	    EEPROM_value.u16 = (uint16_t)(value * 1e2f * 65536.0f);
	}
      break;
    case MAG_X_OFF:
    case MAG_Y_OFF:
    case MAG_Z_OFF:
      if( read)
	value = ( (float)(EEPROM_value.i16) / 3276.8f);
      else
	EEPROM_value.i16 = (int16_t)(value * 3276.8f);
      break;
    case ANT_BASELENGTH: // max +/- 32.768 m
    case ANT_SLAVE_DOWN:
    case ANT_SLAVE_RIGHT:
      if( read)
	value = (float)(EEPROM_value.i16) * 0.001f;
      else
	EEPROM_value.i16 = (int16_t)(value * 1000.0f);
      break;
      break;
    case SENS_TILT_ROLL:
    case SENS_TILT_PITCH:
    case SENS_TILT_YAW:
      if( read)
	value = (float)(EEPROM_value.i16) / 32768.0f * M_PI_F;
      else
	{
	  if( value < -M_PI_F)
	    value += 2.0f * M_PI_F;
	  if( value >= M_PI_F)
	    value -= 2 * M_PI_F;
	  int ivalue = (int) round(value * 32768.0f / M_PI_F);
	  if( ivalue >= 32768)
	    ivalue = 32767;
	  EEPROM_value.i16 = (int16_t)ivalue;
	}
      break;
    case VARIO_TC:
    case VARIO_P_TC:
    case VARIO_INT_TC:
    case WIND_TC:
    case MEAN_WIND_TC:
      if( read)
	value = (float)(EEPROM_value.u16) / 655.36f;
      else
	EEPROM_value.u16 = (uint16_t)(value * 655.36f);
      break;
    case EEPROM_PARAMETER_ID_END:
    default:
	value = 0.0f; // just to be sure ...
	return true; // error
      break;
  }
  return false; // OK
}

bool write_EEPROM_value( EEPROM_PARAMETER_ID id, float value)
{
  EEPROM_data_t EEPROM_value;
  if( EEPROM_convert( id, EEPROM_value, value , WRITE))
      return true; // error

  return EE_WriteVariableBuffered( id, EEPROM_value.u16);
}

bool read_EEPROM_value( EEPROM_PARAMETER_ID id, float &value)
{
  uint16_t data;
  if( HAL_OK != EE_ReadVariableBuffered( id, (uint16_t*)&data))
    return true;
  return ( EEPROM_convert( id, (EEPROM_data_t &)data, value , READ));
}

float configuration( EEPROM_PARAMETER_ID id)
{
  float value;
  bool result = read_EEPROM_value( id, value);
  ASSERT( result == false);
  return value;
}

#endif
