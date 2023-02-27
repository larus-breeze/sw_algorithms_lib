/***********************************************************************//**
 * @file 		persistent_data.h
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

#ifndef INC_PERSISTENT_DATA_H_
#define INC_PERSISTENT_DATA_H_

#include "stdint.h"

#ifdef __cplusplus

typedef union
  {
    uint16_t u16;
    int16_t  i16;
  } EEPROM_data_t;

enum GNSS_configration_t
{
  GNSS_NONE, // not really useful
  GNSS_M9N, 	// single frequency module, usually on PCB
  GNSS_F9P_F9P, // D-GNSS using 2 * uBlox F9P, both on USART 3
  GNSS_F9P_F9H  // D-GNSS using F9P on USART 3 and F9H (heading) on USART4
};

enum EEPROM_PARAMETER_ID
{
  BOARD_ID = 0,

  SENS_TILT_ROLL = 1,
  SENS_TILT_NICK,
  SENS_TILT_YAW,

  PITOT_OFFSET=4,
  PITOT_SPAN,
  QNH_OFFSET,

  MAG_X_OFF=10,
  MAG_X_SCALE,
  MAG_Y_OFF,
  MAG_Y_SCALE,
  MAG_Z_OFF,
  MAG_Z_SCALE,
  MAG_STD_DEVIATION,

  DECLINATION=20,
  INCLINATION,

  VARIO_TC=30,
  VARIO_INT_TC,
  WIND_TC,
  MEAN_WIND_TC,

  GNSS_CONFIGURATION=40,
  ANT_BASELENGTH,
  ANT_SLAVE_DOWN,
  ANT_SLAVE_RIGHT,

  EEPROM_PARAMETER_ID_END // 1 behind last parameter ID
};

class persistent_data_t
{
public:
  enum { MNEMONIC_LENGTH=16};
  EEPROM_PARAMETER_ID id;
  char mnemonic[MNEMONIC_LENGTH];
  EEPROM_data_t value;
};

const persistent_data_t * find_parameter_from_ID( EEPROM_PARAMETER_ID id);

// standard function to read configuration data from EEPROM
float configuration( EEPROM_PARAMETER_ID id);
bool write_EEPROM_value( EEPROM_PARAMETER_ID id, float value);
bool read_EEPROM_value( EEPROM_PARAMETER_ID id, float &value);
bool lock_EEPROM( bool lockit);
bool EEPROM_initialize( void);
bool all_EEPROM_parameters_existing( void);

extern const persistent_data_t PERSISTENT_DATA[];
extern const unsigned PERSISTENT_DATA_ENTRIES;

#endif ///#ifdef __cplusplus

#endif /* INC_PERSISTENT_DATA_H_ */
