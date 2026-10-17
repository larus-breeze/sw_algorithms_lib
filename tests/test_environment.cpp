// Replacements for what the sensor firmware provides to the library
// (configuration, EEPROM, flash, compass calibrator instances).
#include <string.h>
#include <stdint.h>
#include "abstract_EEPROM_storage.h"
#include "compass_calibrator_3D.h"

float configuration( EEPROM_PARAMETER_ID id)
{
  return id == ANT_BASELENGTH ? 1.0f : 0.0f;
}

EEPROM_file_system<LOWEST_UNUSED_EEPROM_ID> permanent_data_file;

void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words, bool)
{
  memcpy( dest, source, n_words * sizeof( uint32_t));
}

magnetic_calculation_data_t temporary_mag_calculation_data;
compass_calibrator_3D_t compass_calibrator_3D( temporary_mag_calculation_data);
compass_calibrator_3D_t external_compass_calibrator_3D( temporary_mag_calculation_data);
void trigger_compass_calibrator_3D_calculation( bool) {}
