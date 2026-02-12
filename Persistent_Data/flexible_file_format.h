#ifndef PERSISTENT_DATA_FLEXIBLE_FILE_FORMAT_H_
#define PERSISTENT_DATA_FLEXIBLE_FILE_FORMAT_H_

  enum flexible_log_file_record_type
  {
    FILE_FORMAT_VERSION = 1,	//!< Larus flexible file format version (enumerated)

    LARUS_DESCRIPTION = 10,	//!< individual hardware, firmware IDs and FLASH-content SHA256
    EEPROM_FILE = 11,		//!< persistent EEPROM data file
    EEPROM_FILE_RECORD = 12,	//!< single persistent EEPROM data record (on any change)
    RUNTIME_EVENT_MARKER = 13,	//!< significant event and optional explanation value
    SENSOR_STATUS = 14,		//!< 32 bits hardware- and software status

    BASIC_SENSOR_DATA = 20 + 0x100, 	//!< IMU-data, pressures, temperature todo patch
    EXTENDED_SENSOR_DATA = 21,	//!< basic data plus external magnetometer readings

    GNSS_DATA = 30,		//!< abstract GNSS receiver output
    D_GNSS_DATA = 31,		//!< abstract D-GNSS receiver output

    CRASH_LOG_REGISTERS = 40,	//!< processor register and exception data
    RTOS_TRACE_DATA = 41,	//!< Post-mortem snapshot of RTOS-events
    SYSTEM_STATE_VECTOR = 42,	//!< All significant state-vector variables
    FLASH_DUMP_SECTION = 43,	//!< raw FLASH section data (start-address, data ...)

    EXTENDED_RECORD = 255	//!< long records with 32bit id and length
  };

#endif /* PERSISTENT_DATA_FLEXIBLE_FILE_FORMAT_H_ */
