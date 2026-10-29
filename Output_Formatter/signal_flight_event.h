#ifndef SIGNAL_FLIGHT_EVENT_H_
#define SIGNAL_FLIGHT_EVENT_H_

enum event
{
  NO_EVENT,

  MAG_CALIBRATION_DONE,
  EXT_MAG_CALIBRATION_DONE,
  AIR_DENSITY_MODIFIED,

  EEPROM_CONFIGURATION_CHANGED,
  CAN_COMMAND_RECEIVED,
  DEBUGGER_DATA,

  ACCELERATION_CALIBRATION_DONE,
  GNSS_LOG,
};

typedef enum
{
  UNSPECIFIED_ERROR,
  GNSS_TIMEOUT_HALF,
  GNSS_TIMEOUT_FULL,
  GNSS_DATA_ERROR
} debug_points;

void signal_logger_event( uint32_t event);

inline void signal_debug_event( debug_points event, int16_t value = 0)
{
  signal_logger_event( (uint32_t)(DEBUGGER_DATA | event << 8) | ((uint32_t)value << 16));
}

inline void signal_GNSS_log_event( uint32_t explanation)
{
  signal_logger_event( (uint32_t)GNSS_LOG | explanation << 8);
}


#endif /* SIGNAL_FLIGHT_EVENT_H_ */
