// Minimal system configuration for building and testing the library outside
// the sensor firmware (CI). Values mirror sw_sensor where they affect the library.
#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

#include <assert.h>

#define GIT_TAG_DEC 0x12345678 // dummy

// 0 = like the sensor firmware, 1 = like the SIL (sensor_data_analyzer)
#ifndef DEVELOPMENT_ADDITIONS
#define DEVELOPMENT_ADDITIONS		0
#endif

#define USE_HARDWARE_EEPROM		0 // EEPROM content in RAM
#define DISABLE_SAT_COMPASS		0
#define PRINT_3D_MAG_PARAMETERS		0

#define LIMIT_DENSITY_CORRECTION( x)

#define ASSERT assert

#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
