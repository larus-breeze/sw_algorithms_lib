/***********************************************************************//**
 * @file		persistent_data_file.h
 * @brief		tiny file system for configuration data
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

#ifndef NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_
#define NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_

#define DEBUG

#include "embedded_math.h"
#include <CRC16.h>
#include "mutex_implementation.h"

#ifdef DEBUG
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#endif

#define ERASED_FLASH_BYTE 0xff

//!< generic call to write permanent data
void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words);

class EEPROM_file_system_node
{
public:
  enum{ DIRECT_8_BIT=0};

  typedef uint8_t ID_t;

  EEPROM_file_system_node * next( void)
  {
    return this + size;
  }

  ID_t id;
  uint8_t size; // size in 32-bit units of entry including node itself
  uint16_t data; // direct 8bit | (checksum << 8) OR crc16 of 32bit data file
};

class EEPROM_file_system
{
public:
  EEPROM_file_system( EEPROM_file_system_node * begin=0, EEPROM_file_system_node * behind_end=0)
  : head( (EEPROM_file_system_node *)begin),
    free_space( begin),
    tail( behind_end)
  {
    free_space = find_end();
  }

  bool in_use( void)
  {
    return head != 0 && tail != 0;
  }

  unsigned get_remaining_space_words( void) const
  {
    return tail - free_space;
  }

  //!< method for simulation only as it writes 0xff into flash to erase
  void initialize_memory_area( uint32_t *begin, uint32_t *behind_end)
  {
    head 	= (EEPROM_file_system_node * )begin;
    tail	= (EEPROM_file_system_node * )behind_end;
    free_space = head;
    memset( head, 0xff, (tail-head) * sizeof( uint32_t));
  }

  void set_memory_virgin( uint32_t *begin, uint32_t *behind_end)
  {
    head 	= (EEPROM_file_system_node * )begin;
    tail	= (EEPROM_file_system_node * )behind_end;
    free_space = head;
  }

  bool set_memory_to_existing_data( uint32_t *begin, uint32_t *behind_end)
  {
    head 	= (EEPROM_file_system_node * )begin;
    tail	= (EEPROM_file_system_node * )behind_end;

    EEPROM_file_system_node * work;
    for( work = head; work->id != ERASED_FLASH_BYTE; work = work->next())
	if( work >= tail)
	  return false;
    free_space = work;
      return is_consistent();
  }

  EEPROM_file_system_node * find_datum( EEPROM_file_system_node::ID_t id)
  {
    return find_last_datum( head, id);
  }

  uint32_t * find_data( EEPROM_file_system_node::ID_t id, unsigned data_size)
  {
      unsigned size_including_node = data_size + 1; // size including node itself
      EEPROM_file_system_node * candidate = find_last_datum( head, id);
      if( candidate == 0 || size_including_node != candidate->size)
        return 0;

      uint32_t *from = (uint32_t *)candidate + 1;
      uint16_t crc = CRC16_blockcheck( (uint16_t *)from,  data_size * sizeof( uint16_t));
      uint16_t protected_id_and_size = candidate->id + ((candidate->size) << 8);
      crc = CRC16( protected_id_and_size, crc);

      if( candidate->data != crc)
        return 0;
      return from;
  }

  bool store_data( EEPROM_file_system_node::ID_t id, unsigned data_size_words, const void * data)
  {
    LOCK_SECTION();

    // at first: check if this information is already stored identically
    if( data_size_words == EEPROM_file_system_node::DIRECT_8_BIT)
      {
	uint8_t data_read;
	bool ok = retrieve_data( id, data_read);
	if( ok && data_read == *(uint8_t *)data)
	  return true;
      }
    else
      {

	uint32_t * place_in_file = find_data( id, data_size_words);
	if( place_in_file != 0)
	  if( compare_words_block_binary( (uint32_t *)data, place_in_file, data_size_words))
	    return true; // same information found
      }

    EEPROM_file_system_node * next_entry = (EEPROM_file_system_node *)free_space;

    if( next_entry + data_size_words + 1 >= tail)
      return false; // no more room !

    EEPROM_file_system_node temp_node;
    temp_node.id = id;
    temp_node.size = (int8_t)(data_size_words + 1); // size including node itself

    if( data_size_words == EEPROM_file_system_node::DIRECT_8_BIT)
      {

	uint16_t checked_datum = *(uint8_t *)data; 	// take only 8 bits
	temp_node.data = check_and_pack_id_len_and_data( temp_node, checked_datum);

	FLASH_write( (uint32_t *)next_entry, (uint32_t *)&temp_node, 1);
      }
    else // : bunch of 32bit data
      {
	uint16_t crc = CRC16_blockcheck( (uint16_t *)data,  data_size_words * 2);
	uint16_t protected_id_and_size = temp_node.id + ((temp_node.size) << 8);
	temp_node.data = CRC16( protected_id_and_size, crc);
	FLASH_write( (uint32_t *)next_entry, (uint32_t *)&temp_node, 1);
	FLASH_write( (uint32_t *)(next_entry + 1), (uint32_t *)data, data_size_words);
      }

    free_space += data_size_words + 1;
    return true;
  }

  bool store_data( EEPROM_file_system_node::ID_t id, const uint8_t data)
  {
    return store_data( id, EEPROM_file_system_node::DIRECT_8_BIT, &data);
  }

  bool retrieve_data( EEPROM_file_system_node::ID_t id, unsigned data_size, void * target)
  {
    LOCK_SECTION();

    uint32_t *dest = (uint32_t *)target;
    unsigned size_including_node = data_size + 1; // size including node itself
    EEPROM_file_system_node * candidate = find_last_datum( head, id);
    if( candidate == 0 || size_including_node != candidate->size)
      return false;

    uint32_t *from = (uint32_t *)candidate + 1;
    uint16_t crc = CRC16_blockcheck( (uint16_t *)from,  data_size * sizeof( uint16_t));
    uint16_t protected_id_and_size = candidate->id + ((candidate->size) << 8);
    crc = CRC16( protected_id_and_size, crc);

    if( candidate->data != crc)
      return false;

    do
    {
      *dest++ = *from++;
      --data_size;
    }
    while( data_size != 0);

    return true;
  }

  bool retrieve_data( EEPROM_file_system_node::ID_t id, uint8_t & target)
  {
    LOCK_SECTION();

    EEPROM_file_system_node * my_node = find_last_datum( head, id);
    if( my_node == 0)
      return false;

    if( my_node->size != 1)
      return false;

    if( not short_node_is_consistent(*my_node))
      return false;

    target = my_node->data & 0xff;
    return true;
  }

#ifdef DEBUG

  void dump_all_entries (void)
  {
    EEPROM_file_system_node *the_node;
    for (EEPROM_file_system_node::ID_t id = 1; id < 255; ++id)
      {
	the_node = find_last_datum (head, id);
	if (the_node != 0)
	  {
	    printf ("ID: %d ", the_node->id);

	    if (the_node->size == 1)
	      printf ("val = %02x\n", the_node->data & 0xff);
	    else
	      {
		printf ("val =");
		for ( int i = 0; i < the_node->size - 1; ++i)
//		  printf( " %08x", * ((unsigned *) the_node + 1 + i));
		  printf (" %10e", *((float32_t*) the_node + 1 + i));
		printf ("\n");
	      }
	  }
      }
  }

#endif

  bool is_consistent(void)
  {
    EEPROM_file_system_node * work;
    //check all nodes for consistency
    for( work = head; (work < free_space) && (*(uint32_t *)work != 0xffffffff); work = work->next())
      {
	if( work->size == 1) // we have found a direct-data-entry
	  {
	    if( not short_node_is_consistent( *work))
	      return false; // invalid data pattern
	  }
	else
	  {
	    if( not long_node_is_consistent( work))
	      return false; // invalid data pattern
	  }
      }

    for( uint32_t * p = (uint32_t *)work; p < (uint32_t *)tail; ++p)
      if( *p != 0xffffffff)
	return false;

    return true;
  }

  void import_all_data (const EEPROM_file_system &source)
  {
    EEPROM_file_system_node *current_node;
    for (EEPROM_file_system_node::ID_t id = 1; id < ERASED_FLASH_BYTE; ++id)
      {
	current_node = source.find_last_datum ( source.head, id);
	if (current_node != 0)
	  {
	    switch (current_node->size)
	      {
	      case 0:
	      case ERASED_FLASH_BYTE:
		continue; // invalid size
	      case 1: // direct data node
		if (not short_node_is_consistent (*current_node))
		  continue;
		store_data (current_node->id, current_node->data & 0xff);
		break;
	      default: // data file
		if (not long_node_is_consistent (current_node))
		  continue;
		store_data (current_node->id, current_node->size-1,
			    (void*) (current_node + 1));
		break;
	      }
	  }
      }
  }

  void * get_head( void) const
  {
    return (void *)head;
  }

  unsigned get_size( void) const
  {
    return (free_space - head) * sizeof( uint32_t);
  }

private:
  bool compare_words_block_binary( const uint32_t *one,  const uint32_t *two, unsigned size) const
  {
    while( size--)
      {
      if( *one++ != *two++)
	return false;
      }
    return true;
  }

  uint16_t check_and_pack_id_len_and_data( EEPROM_file_system_node the_node, uint8_t datum)
  {
	uint16_t info = datum;		 // need 16bit data
	uint16_t crc = CRC16( info, 0); // data crc
	info = the_node.id + (the_node.size << 8);
	crc = CRC16( info, crc); 	 // plus node crc
	crc = (crc ^ (crc >> 8)) & 0xff;	 // fold crc into 8 bits
	return datum | (crc << 8);
  }

  bool short_node_is_consistent( EEPROM_file_system_node the_node)
  {
    uint16_t crc = CRC16( the_node.data & 0xff, 0); // data crc
    uint16_t info = the_node.id + (the_node.size << 8);
    crc = CRC16( info, crc); 	     // plus node crc
    crc = (crc ^ (crc >> 8)) & 0xff; // fold crc into 8 bits
    return (the_node.data >> 8) == crc;
  }

  bool long_node_is_consistent( EEPROM_file_system_node * work)
  {
    uint16_t crc = CRC16_blockcheck( (uint16_t *)work + 2, (work->size - 1) * 2);
    uint16_t protected_id_and_size = work->id + ((work->size) << 8);
    crc = CRC16( protected_id_and_size, crc);

    return work->data == crc;
  }

  EEPROM_file_system_node * find_last_datum( EEPROM_file_system_node * start, EEPROM_file_system_node::ID_t id=ERASED_FLASH_BYTE) const
  {
    EEPROM_file_system_node * thisone = find_first_datum( start, id);
    EEPROM_file_system_node * candidate=0;
    while( thisone != 0)
     {
       candidate = thisone;
       thisone = find_first_datum( candidate->next(), id);
     }
    return candidate;
  }

  EEPROM_file_system_node * find_end( void)
  {
    EEPROM_file_system_node * work = head;

    while( (work < free_space) && (work->size != 0xff) && (work->size != 0x00))
      {
	work = work->next();
	if( work >= tail)
	  {
	    work = 0;
	    break;
	  }
      }

    if( work >= tail)
      work = 0;

    return work;
  }

  EEPROM_file_system_node * find_first_datum( EEPROM_file_system_node * start, EEPROM_file_system_node::ID_t id) const
  {
    for( EEPROM_file_system_node * work = start; (work < free_space) && (work->size != 0) && (work->size != ERASED_FLASH_BYTE); work = work->next())
      {
        if( work->id == id)
  	return work;
      }
    return 0;
  }

  EEPROM_file_system_node * head;
  EEPROM_file_system_node * free_space;
  EEPROM_file_system_node * tail;
};

extern EEPROM_file_system permanent_data_file;

#endif /* NAV_ALGORITHMS_PERSISTENT_DATA_FILE_H_ */
