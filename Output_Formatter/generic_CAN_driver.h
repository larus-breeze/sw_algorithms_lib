/*
 * generic_CAN_driver.h
 *
 *  Created on: Sep 5, 2022
 *      Author: schaefer
 */

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

bool CAN_send( const CANpacket &p, unsigned dummy);

#endif /* GENERIC_CAN_DRIVER_H_ */
