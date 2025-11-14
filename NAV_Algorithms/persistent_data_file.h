#ifndef NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_
#define NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_

#define DEBUG

#include "stdint.h"
#include "assert.h"
#include "CRC.h"

class node
{
public:
  enum{ DIRECT_8_BIT=0};

  typedef uint8_t ID_t;

  node * next( void)
  {
    return this + size;
  }

  ID_t id;
  uint8_t size; // size in 32-bit units of entry including sizeof( node)
  uint16_t data; // direct 8bit + checksum OR crc of 32bit data file
};

#ifdef DEBUG

void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words)
{
  memcpy( dest, source, n_words * sizeof( uint32_t));
}

#endif

class file_system
{
public:
  file_system( void * begin, void * behind_end)
  : head( (node *)begin),
    free_space( begin),
    end( behind_end)
  {
    free_space = find_end();
  }

  node * find_datum( node::ID_t id)
  {
    return find_last_datum( head, id);
  }

  bool store_data( node::ID_t id, unsigned data_size_words, const void * data)
  {
    node * next_entry = (node *)free_space;

    if( next_entry + data_size_words + 1 >= end)
      return false; // no more room !

    node temp_node;
    temp_node.id = id;
    temp_node.size = data_size_words + 1; // size including node itself

    if( data_size_words == node::DIRECT_8_BIT) // if direct 16bit data
      {
	uint16_t checked_datum = *(uint8_t *)data; // take only 8 bits
	checked_datum |= ( (~checked_datum) << 8);
	temp_node.data = checked_datum;

	FLASH_write( (uint32_t *)next_entry, (uint32_t *)&temp_node, 1);
      }
    else // : bunch of 32bit data
      {
	temp_node.data = CRC16_blockcheck( (uint16_t *)data,  data_size_words * sizeof( uint16_t));

	FLASH_write( (uint32_t *)next_entry, (uint32_t *)&temp_node, 1);
	FLASH_write( (uint32_t *)(next_entry + 1), (uint32_t *)data, data_size_words * sizeof( uint32_t));
      }

    free_space = (uint32_t *)free_space + data_size_words + 1;
    return true;
  }

  bool store_data( node::ID_t id, const uint8_t data)
  {
    return store_data( id, node::DIRECT_8_BIT, &data);
  }

  bool retrieve_data( node::ID_t id, uint32_t * target, unsigned data_size)
  {
    unsigned size_including_node = data_size +1; // ... including node itself
    node * candidate = find_last_datum( head, id);
    if( candidate == 0 || size_including_node != candidate->size)
      return false;

    uint32_t *from = (uint32_t *)candidate + 1;
    uint16_t crc = CRC16_blockcheck( (uint16_t *)from,  data_size * sizeof( uint16_t));

    if( candidate->data != crc)
      return false;

    do
    {
      *target++ = *from++;
      --data_size;
    }
    while( data_size != 0);

    return true;
  }

  bool retrieve_data( node::ID_t id, uint16_t & target)
  {
    node * my_node = find_last_datum( head, id);
    if( my_node == 0)
      return false;

    if( my_node->size != 1)
      return false;

    target = my_node->data;
    return true;
  }

#ifdef DEBUG

  void dump_all_entries( void)
  {
    node * the_node;
    for( node::ID_t id=1; id < 255; ++id)
      {
        the_node = find_last_datum( head, id);
        if( the_node != 0)
  	{
  	  printf( "ID: %d ", the_node->id);

  	  if( the_node->size == 1)
  	    printf( "val = %02x\n", the_node->data & 0xff);
  	  else
  	    {
  	      printf( "val =");
  	      for( unsigned i = 0; i < the_node->size - 1; ++i)
  		      printf( " %08x", * ((unsigned *) the_node + 1 + i));
  	      printf( "\n");
  	    }
  	}
      }
#endif
  }

  bool is_consistent(void)
  {
    //check all nodes for consistency
    for( node * work = head; *(uint32_t *)work != 0xffffffff && work < free_space; work = work->next())
      {
	if( work->size == 1) // we have found a dircect-data-entry
	  {
	    if( ((work->data & 0xff) ^ (work->data >> 8)) != 0xff)
	      return false; // invalid data pattern
	  }
	else
	  {
	    uint16_t crc = CRC16_blockcheck( (uint16_t *)work + 2, (work->size - 1) * 2);
	    if( work->data != crc)
	      return false; // found bad entry
	  }
      }
    return true;
  }

private:

  node * find_last_datum( node * start, node::ID_t id=0xff)
  {
    node * thisone = find_first_datum( start, id);
    node * candidate=0;
    while( thisone != 0)
     {
       candidate = thisone;
       thisone = find_first_datum( candidate->next(), id);
     }
    return candidate;
  }

  node * find_end( void)
  {
    node * work = head;

    while( (work < free_space) && (work->size != 0xff))
      {

	work = work->next();
	if( work >= end)
	  {
	    work = 0;
	    break;
	  }
      }

    if( work >= end)
      work = 0;

    return work;
  }

  node * find_first_datum( node * start, node::ID_t id)
  {
    for( node * work = start; (work < free_space) && (work->size != 0) && (work->size != 0xff); work = work->next())
      {
        if( work->id == id)
  	return work;
      }
    return 0;
  }

  node * head;
  void * free_space;
  void * end;
};

#ifdef DEBUG

#define STORAGE_SIZE 1024
uint32_t storage[STORAGE_SIZE];

void test_storage( void)
{
  bool success;
  memset( storage, 0xff, STORAGE_SIZE * sizeof(uint32_t));
  file_system file( storage, storage+STORAGE_SIZE);

  uint8_t datum1 = 0x34;
  file.store_data(1, node::DIRECT_8_BIT, &datum1);

  success = file.is_consistent();

  file.dump_all_entries();

  unsigned datum2[10]={ 1, 2, 3, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff};
  file.store_data(2, 3, datum2);

  success = file.is_consistent();
  file.dump_all_entries();

  datum1 = 0x21;
  file.store_data(1, datum1);

  success = file.is_consistent();
  file.dump_all_entries();

  uint16_t test = 0xff;
  success = file.retrieve_data( 1, test);

  datum2[0]=4;
  datum2[1]=5;
  datum2[2]=6;
  success = file.retrieve_data( 2, datum2, 3);

  success = file.is_consistent();
  file.dump_all_entries();

  do
    {
      ++datum2[0];
      success = file.store_data(2, 3, datum2);
    }
  while( success == true);

  success = file.is_consistent();
  file.dump_all_entries();

  datum2[0]=0;
  datum2[1]=0;
  datum2[2]=0;
  success = file.retrieve_data( 2, datum2, 3);

  success = file.is_consistent();
  file.dump_all_entries();

  do
    {
      ++datum1;
      success = file.store_data(1, 0, &datum1);
    }
  while( success);

  success = file.is_consistent();
  file.dump_all_entries();

  success = file.retrieve_data( 1, test);
  file.dump_all_entries();

  success = file.retrieve_data( 13, test);
  success = file.retrieve_data( 0xff, test);
}

#endif

#endif /* NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_ */
