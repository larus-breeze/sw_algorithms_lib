/***********************************************************************//**
 * @file		generic_CAN_driver.h
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

#ifndef GENERIC_CAN_DRIVER_H_
#define GENERIC_CAN_DRIVER_H_

#include "stdint.h"

//! basic CAN packet type
class CANpacket
{
public:
  CANpacket( uint16_t _id=0, uint16_t _dlc=0, uint64_t _data=0)
  : id(_id),
    dlc(_dlc),
    data_l(_data)
  {}
  bool operator ==(const CANpacket&right)
    {
      return 	(id == right.id) &&
		(dlc = right.dlc) &&
		(data_l == right.data_l);
    }
  uint16_t id; 	//!< identifier
  uint16_t dlc; //!< data length code
  union
  {
		uint8_t  data_b[8]; 	//!< data seen as 8 times uint8_t
		int8_t   data_sb[8]; 	//!< data seen as 8 times int8_t
		uint16_t data_h[4]; 	//!< data seen as 4 times uint16_t
		int16_t  data_sh[4]; 	//!< data seen as 4 times int16_t
		uint32_t data_w[2]; 	//!< data seen as 2 times uint32_t
		int32_t  data_sw[2];	//!< data seen as 2 times int32_t
		float    data_f[2]; 	//!< data seen as 2 times 32-bit floats
		uint64_t data_l;    	//!< data seen as 64-bit integer
  };
} ;

#pragma pack(push, 2)

//! CAN packet tunneled through USART gateway
class CAN_gateway_packet
{
public:
  CAN_gateway_packet( const CANpacket &p)
  : ID_checked(p.id),
    DLC_checksum(p.dlc),
    data(p.data_l)
  {
    uint16_t checksum = ~(p.id % 31);
    ID_checked |= checksum << 11;

    checksum = ~(p.data_l % 4095);
    DLC_checksum |= checksum << 4;
  }

  bool to_CANpacket( CANpacket &p)
  {
    uint16_t checksum = ~((ID_checked & 0x7ff) % 31) & 0x1f;
    if( ID_checked >> 11 != checksum)
      return false;

    checksum = ~(data % 4095) & 0xfff;
    if( DLC_checksum >> 4 != checksum)
      return false;

    p.id     = ID_checked & 0x7ff;
    p.dlc    = DLC_checksum & 0x0f;
    p.data_l = data;

    return true;
  }

public:
  uint16_t ID_checked;
  uint16_t DLC_checksum;
  uint64_t data; //
};

#pragma pack(pop)

//! Global CAN send procedure
bool CAN_send( const CANpacket &p, unsigned dummy);

#endif /* GENERIC_CAN_DRIVER_H_ */
