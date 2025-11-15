/***********************************************************************//**
 * @file		persistent_data_file.cpp
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
#include "persistent_data_file.h"

#ifdef DEBUG

void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words)
{
  memcpy( dest, source, n_words * sizeof( uint32_t));
}

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

  unsigned id=5;
  do
    {
      ++datum2[0];
      success = file.store_data( id, 3, datum2);
      ++id;
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

  success = file.retrieve_data( 13, test); // test non existing nodes
  success = file.retrieve_data( 0xff, test);
}

#endif






