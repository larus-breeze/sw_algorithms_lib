#ifndef SIGNAL_FLIGHT_EVENT_H_
#define SIGNAL_FLIGHT_EVENT_H_

#include "FreeRTOS_wrapper.h"

extern Queue <uint32_t> flight_event_queue;

enum event
{
  NO_EVENT,

  MAG_CALIBRATION_DONE,
  EXT_MAG_CALIBRATION_DONE,
  AIR_DENSITY_MODIFIED,

  EEPROM_CONFIGURATION_CHANGED,
};

inline void signal_event( uint32_t event)
{
  flight_event_queue.send_from_ISR(event);
}

#endif /* SIGNAL_FLIGHT_EVENT_H_ */
