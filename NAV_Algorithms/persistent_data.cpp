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

ROM persistent_data_t PERSISTENT_DATA[]=
    {
	{BOARD_ID, 	"Board_ID", 0},	 	//! Board ID Hash to avoid board confusion

	{SENS_TILT_ROLL,"SensTilt_Roll", 0}, 	//! IMU Sensor tilt angle signed / (pi / 32768) front right down frame
	{SENS_TILT_NICK,"SensTilt_Nick", 0}, 	//! IMU Sensor tilt angle signed / (pi / 32768)
	{SENS_TILT_YAW, "SensTilt_Yaw", 0},  	//! IMU Sensor tilt angle signed / (pi / 32768)

	{PITOT_OFFSET,	"Pitot_Offset", 0},	//! Pitot offset signed / ( ADC readings )
	{PITOT_SPAN, 	"Pitot_Span", 0},	//! Pitot Span signed ( scale-factor  = value / 32768 + 1.0f)
	{QNH_OFFSET, 	"QNH-delta", 0},	//! Absolute pressure sensor offset signed / Pa

	{MAG_X_OFF,	"Mag_X_Off", 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_X_SCALE,	"Mag_X_Scale", 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_Y_OFF,	"Mag_Y_Off", 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_Y_SCALE,	"Mag_Y_Scale", 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_Z_OFF,	"Mag_Z_Off", 0},	//! Induction sensor x offset signed / ( 10.0f / 32768 )
	{MAG_Z_SCALE,	"Mag_Z_Scale", 0},	//! Induction sensor x gain signed ( scale-factor = 1.0f + value / 32768 )
	{MAG_STD_DEVIATION, "Mag_Calib_Err", 0},//! Magnetic calibration STD deviation / ( 1 % / 65536 )

	{DECLINATION,	"Mag_Declination", 0}, 	//! Magnetic declination (east positive) signed / ( 180° / 32768)
	{INCLINATION,	"Mag_Inclination", 0}, 	//! Magnetic inclination (down positive) signed / ( 180° / 32768)

	{VARIO_TC,	"Vario_TC", 0},	 	//! Vario time constant unsigned s / ( 100.0f / 65536 )
	{VARIO_INT_TC,	"Vario_Int_TC", 0},	//! Vario integrator time constant unsigned s / ( 100.0f / 65536 )
	{WIND_TC,	"Wind_TC", 0},	 	//! Wind fast time constant unsigned s / ( 100.0f / 65536 )
	{MEAN_WIND_TC,	"Mean_Wind_TC", 0},	//! Wind slow time constant unsigned s / ( 100.0f / 65536 )
	{VETF,		"VrtclEnrgTuning", 0},	//! Vertical Energy tuning factor s / ( 1.0f / 65536 )

	{GNSS_CONFIGURATION, "GNSS_CONFIG", 0},	//! type of GNSS system
	{ANT_BASELENGTH, "ANT_BASELEN", 0},	//! Slave DGNSS antenna baselength / mm
	{ANT_SLAVE_DOWN, "ANT_SLAVE_DOWN", 0},	//! Slave DGNSS antenna lower / mm
	{ANT_SLAVE_RIGHT,"ANT_SLAVE_RIGHT", 0},	//! Slave DGNSS antenna more right /mm
    };

ROM unsigned PERSISTENT_DATA_ENTRIES = sizeof(PERSISTENT_DATA) / sizeof(persistent_data_t);

bool all_EEPROM_parameters_existing( void)
{
  float dummy;
  const persistent_data_t * parameter = PERSISTENT_DATA + 1; // skip BOARD_ID
  while( parameter < PERSISTENT_DATA + PERSISTENT_DATA_ENTRIES)
    {
      if( true == read_EEPROM_value( parameter->id, dummy))
	return false; // read error
      ++parameter;
    }
  return true;
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
      if( read)
	value = (float)(EEPROM_value.u16);
      else
	EEPROM_value.u16 = (uint16_t)(value + 0.5f); // rounding
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
	EEPROM_value.i16 = (int16_t)((value - 1.0f) * 32768.0f);
      break;
    case MAG_STD_DEVIATION:
      if( read)
	value = (float)(EEPROM_value.u16) / 65536.0f * 1e-2f;
      else
	EEPROM_value.u16 = (uint16_t)(value * 1e2f * 65536.0f);
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
    case DECLINATION:
    case INCLINATION:
    case SENS_TILT_ROLL:
    case SENS_TILT_NICK:
    case SENS_TILT_YAW:
      if( read)
	value = (float)(EEPROM_value.i16) / 32768.0f * 180.0;
      else
	{
	  if( value < -180.0f)
	    value += 360.0f;
	  if( value >= +180.0f)
	    value -= 360.0f;
	  int ivalue = value * 32768.0f / 180.0;
	  if( ivalue >= 32768)
	    ivalue = 32767;
	  EEPROM_value.i16 = (int16_t)ivalue;
	}
      break;
    case VARIO_TC:
    case VARIO_INT_TC:
    case WIND_TC:
    case MEAN_WIND_TC:
      if( read)
	value = (float)(EEPROM_value.u16) / 655.36f;
      else
	EEPROM_value.u16 = (uint16_t)(value * 655.36f);
      break;
    case VETF:
      if( read)
	value = (float)(EEPROM_value.u16) / 65536.0f;
      else
	unsigned uvalue = value * 65536.0f;
	if( uvalue >= 65536)
	  uvalue = 65535;
	EEPROM_value.u16 = (uint16_t)uvalue;
      break;
    case EEPROM_PARAMETER_ID_END:
    default:
	return true; // error
      break;
  }
  return false; // OK
}

bool lock_EEPROM( bool lockit)
{
  if( lockit)
    return HAL_FLASH_Lock();

  HAL_FLASH_Unlock();
  return (EE_Init());
}

bool write_EEPROM_value( EEPROM_PARAMETER_ID id, float value)
{
  EEPROM_data_t EEPROM_value;
  if( EEPROM_convert( id, EEPROM_value, value , WRITE))
      return true; // error

  EEPROM_data_t read_value;
  if( (HAL_OK != EE_ReadVariable( id, &read_value.u16))
      ||
      (read_value.u16 != EEPROM_value.u16) )
	return EE_WriteVariable( id, EEPROM_value.u16);
  return HAL_OK;
}

bool read_EEPROM_value( EEPROM_PARAMETER_ID id, float &value)
{
  uint16_t data;
  if( HAL_OK != EE_ReadVariable( id, (uint16_t*)&data))
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

bool EEPROM_initialize( void)
{
  unsigned status;

  status = HAL_FLASH_Unlock();
  ASSERT(status == HAL_OK);

  status = EE_Init();
  ASSERT(status == HAL_OK);

  return HAL_OK;
}

#endif
