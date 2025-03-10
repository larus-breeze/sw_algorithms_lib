/***********************************************************************//**
 * @file		CAN_output.h
 * @brief		Portable interface to some CAN-bus driver
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

#ifndef SRC_CAN_OUTPUT_H_
#define SRC_CAN_OUTPUT_H_

#include "data_structures.h"

/* Larus CAN definitions */
#define  SYSWIDECONFIG_ITEM_ID_VOLUME 		0
#define  SYSWIDECONFIG_ITEM_ID_MC 		1
#define  SYSWIDECONFIG_ITEM_ID_BALLAST 		2
#define  SYSWIDECONFIG_ITEM_ID_BUGS 		3
#define  SYSWIDECONFIG_ITEM_ID_QNH 		4
#define  SYSWIDECONFIG_ITEM_ID_PILOT_KG 	5
#define  SYSWIDECONFIG_ITEM_ID_VARIO_MODE	6

#define  SENS_CONFIG_ITEM_ROLL			0x2000
#define  SENS_CONFIG_ITEM_PITCH			0x2001
#define  SENS_CONFIG_ITEM_YAW			0x2002
#define  SENS_CONFIG_ITEM_PIT_OFF		0x2003
#define  SENS_CONFIG_ITEM_PIT_SPAN		0x2004
#define  SENS_CONFIG_ITEM_QNH_OFF		0x2005
#define  SENS_CONFIG_MAG_CALIB			0x2006
#define  SENS_CONFIG_VARIO_TC			0x2007
#define  SENS_CONFIG_VARIO_INT_TC		0x2008
#define  SENS_CONFIG_WIND_TC			0x2009
#define  SENS_CONFIG_WIND_MEAN_TC		0x200a
#define  SENS_CONFIG_GNSS_CONFIG		0x200b
#define  SENS_CONFIG_ANT_BASELEN		0x200c
#define  SENS_CONFIG_ANT_SLAVE_DOWN		0x200d
#define  SENS_CONFIG_ANT_SLAVE_RIGHT		0x200e

#define  CMD_MEASURE_LEFT			0x3000
#define  CMD_MEASURE_RIGHT			0x3001
#define  CMD_MEASURE_LEVEL			0x3002
#define  CMD_CALCULATE				0x3003
#define  CMD_TUNE				0x3004

void CAN_output ( const output_data_t &x, bool horizon_activated);
void CAN_heartbeat( void);

#endif /* SRC_CAN_OUTPUT_H_ */
