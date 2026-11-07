#ifndef PERSISTENT_DATA_CRC32_H_
#define PERSISTENT_DATA_CRC32_H_

#include "stdint.h"
#include "embedded_memory.h"
extern ROM uint32_t crc32_table[];

#include "stdint.h"

class CRC32
{
public:
  CRC32(void)
  : crc(0xffffffff)
  {}

  //! match STM32F407 hardware CRC unit
  uint32_t update( uint32_t input)
  {
    uint8_t work;
    for( unsigned i=0; i < 4; ++i)
  	{
  	  work = input >> (8 * (3-i)); // little endian order
  	  crc = (crc << 8) ^ crc32_table[(crc >> 24) ^ work];
  	}
    return crc;
  }

  uint32_t update( const uint32_t *data, uint32_t len)
  {
    uint8_t work;
    while( len--)
      {
        for( unsigned i=0; i < 4; ++i)
  	{
  	  work = *data >> (8 * (3-i)); // little endian order
  	  crc = (crc << 8) ^ crc32_table[(crc >> 24) ^ work];
  	}
        ++data;
      }
    return crc;
  }

  void reset( void)
  {
    crc = 0xffffffff;
  }

  uint32_t get_value( void)
  {
    return crc;
  }

  uint16_t get_value_16bits( void)
  {
    uint16_t retv = crc ^ crc >> 16;
    return retv;
  }

  operator uint16_t( void)
    {
    return get_value_16bits();
    }

private:
  uint32_t crc;
};

#endif /* PERSISTENT_DATA_CRC32_H_ */
