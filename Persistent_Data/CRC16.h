#ifndef CRC16_H_
#define CRC16_H_

#include <CRC16.h>
#include <stdint.h>
#include "embedded_memory.h"

extern ROM uint16_t CRCtable[];

static inline uint16_t CRC16( const uint16_t input, uint16_t crc)
{
    uint8_t carry=(uint8_t)((crc >> 8) ^ input);
    crc = (crc << 8) ^ CRCtable[carry];
	return crc;
}

static inline uint16_t CRC16_blockcheck_bytes( const uint8_t *input, unsigned  length)
{
  uint16_t crc=0;
  while( length--)
    {
      crc = CRC16( (uint16_t)(*input++), crc);
    }
  return crc;
}

static inline uint16_t CRC16_blockcheck( const uint16_t *input, unsigned length)
{
  uint16_t crc=0;
  while( length--)
    {
      crc = CRC16( *input++, crc);
    }
  return crc;
}

#endif /* CRC16_H_ */
