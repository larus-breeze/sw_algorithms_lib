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


void CAN_output ( const output_data_t &x, bool horizon_activated);

#endif /* SRC_CAN_OUTPUT_H_ */
