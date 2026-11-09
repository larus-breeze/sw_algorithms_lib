#ifndef PERSISTENT_DATA_CRC32_H_
#define PERSISTENT_DATA_CRC32_H_

#include "stdint.h"
#include "embedded_memory.h"
extern ROM uint32_t crc32_table[];

#include "stdint.h"

class CRC32
{
public:
  CRC32(void);

  //! match STM32F407 hardware CRC unit
  uint32_t update( uint32_t input);

  uint32_t update( const uint32_t *data, uint32_t len);

  void reset( void);

  uint32_t get_value( void);

  uint16_t get_value_16bits( void);

  operator uint16_t( void)
    {
    return get_value_16bits();
    }

private:
  uint32_t crc;
};

#endif /* PERSISTENT_DATA_CRC32_H_ */
