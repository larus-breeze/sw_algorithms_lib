#ifndef NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_
#define NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_

#define DEBUG

#include "stdint.h"
#include "assert.h"

typedef uint8_t ID_t;
enum { ERASED_FLASH=0xff};

class node
{
public:
  node * next( void)
  {
    return this + size;
  }

  ID_t id;
  uint8_t size; // If = 1: direct data, otherwise size in 32 bit words + 1
  union
  {
    uint16_t direct_data_h;
    uint16_t crc; // optional
  }data;
};

node * find_end( node * work, node * tail,)
{
  while( work->id != ERASED_FLASH)
    {
      work = work->next();
      if( work > tail)
	return 0;
    }
  return work;
}

node * find_first_datum(  node * head, node * tail, ID_t id=ERASED_FLASH)
{
  for( node * work = head; work < tail && work->size != 0 && work->size != 0xff; work = work->next())
    {
      if( work->id == id)
	return work;
    }
  return 0;
}

node * find_last_datum(  node * head, node * tail, ID_t id=0xff)
{
  node * thisone = find_first_datum( head, tail, id);
  node * candidate=0;
  while( thisone != 0)
   {
     candidate = thisone;
     thisone = find_first_datum( candidate, tail, id);
   }
  return candidate;
}

node * store_data( node * head, node * tail, ID_t id, unsigned size_words, void * data)
{
  node * work = find_first_datum( head, tail, ERASED_FLASH);
  if( work == 0 || ( work + size_words + 1) > tail)
    return 0;

  work->id=id;
  work->size=size_words + 1; // including node object

  if( size_words == 1) // direct data
    tail->data.direct_data_h = *(uint16_t *) data;
  else
    {
      uint32_t *to = (uint32_t *)work +1;
      uint32_t *from = (uint32_t *)data;
      for( to = (uint32_t *)tail + 1; size_words > 0; --size_words)
	*to++ = *from++;
    }
  return work->next();
}

bool retrieve_data( node * head, node * tail, ID_t id, uint32_t * target, unsigned size_including_node)
{
  node * candidate = find_last_datum( head, tail, id);
  if( candidate == 0 || size_including_node != candidate->size)
    return false;

  if( candidate->size == 1)
    *target = candidate->data.direct_data_h;
  else
    {
      uint32_t from = (uint32_t *)candidate + 1;
      --size_including_node;
      do
      {
	  *target++ = *from++;
      }
      while( size_including_node--);
    }

  return true;
}

#endif /* NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_ */
