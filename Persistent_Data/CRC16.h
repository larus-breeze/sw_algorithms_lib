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

//! CRC16 over both bytes of a 16 bit value (low byte first)
static inline uint16_t CRC16_word( const uint16_t input, uint16_t crc)
{
  crc = CRC16( input & 0xff, crc);
  return CRC16( input >> 8, crc);
}

static inline uint16_t CRC16_blockcheck( const uint16_t *input, unsigned length)
{
  uint16_t crc=0;
  while( length--)
    {
      crc = CRC16_word( *input++, crc);
    }
  return crc;
}

// Data written before the CRC fix was protected by CRC16() fed with 16 bit
// values, which only covers their low bytes. Such data is still accepted
// when reading if ACCEPT_LEGACY_CRC is set.
#ifndef ACCEPT_LEGACY_CRC
#define ACCEPT_LEGACY_CRC 1
#endif

static inline uint16_t CRC16_blockcheck_legacy( const uint16_t *input, unsigned length)
{
  uint16_t crc=0;
  while( length--)
    {
      crc = CRC16( *input++, crc); // low byte only
    }
  return crc;
}

#endif /* CRC16_H_ */
