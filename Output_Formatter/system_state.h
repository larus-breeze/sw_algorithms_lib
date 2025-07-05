/*
 * system_state.h
 *
 *  Created on: Jan 19, 2023
 *      Author: schaefer
 */

#ifndef INC_SYSTEM_STATE_H_
#define INC_SYSTEM_STATE_H_

enum availability_bits
{
	GNSS_AVAILABLE 		= 1,
	D_GNSS_AVAILABLE 	= 2,

	MTI_SENSOR_AVAILABE 	= 0x10,
	MS5611_STATIC_AVAILABLE = 0x80,
	PITOT_SENSOR_AVAILABLE 	= 0x200,
	AIR_SENSOR_AVAILABLE 	= 0x400,

	HORIZON_NOT_AVAILABLE	= 0x10000
};

extern uint32_t system_state; //!< bits collected from availability_bits

inline void update_system_state_set( unsigned value)
{
	__atomic_or_fetch ( &system_state, value, __ATOMIC_ACQUIRE);
}

inline void update_system_state_clear( unsigned value)
{
	__atomic_and_fetch ( &system_state, ~value, __ATOMIC_ACQUIRE);
}

inline bool essential_sensors_available( bool need_DGNSS)
    {
	uint32_t essential_sensors_mask =
	  GNSS_AVAILABLE |
	  MTI_SENSOR_AVAILABE |
	  MS5611_STATIC_AVAILABLE |
	  PITOT_SENSOR_AVAILABLE;

	if( need_DGNSS)
	  essential_sensors_mask |= D_GNSS_AVAILABLE;

	return( system_state & essential_sensors_mask) == essential_sensors_mask;
    }

#endif /* INC_SYSTEM_STATE_H_ */
