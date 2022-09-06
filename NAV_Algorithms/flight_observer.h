/** ***********************************************************************
 * @file		flight_observer.h
 * @brief		windspeed and vario update
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef FLIGHT_OBSERVER_H_
#define FLIGHT_OBSERVER_H_

#include "system_configuration.h"
#include "GNSS.h"
#include <differentiator.h>
#include "KalmanVario.h"
#include "KalmanVario_PVA.h"
#include "embedded_math.h"
#include "windobserver.h"

#if USE_HARDWARE_EEPROM	== 0
#include "EEPROM_emulation.h"
#endif

#include "pt2.h"
#include "HP_LP_fusion.h"

class flight_observer_t
{
public:
  flight_observer_t( void)
  :
  vario_averager_pressure( configuration( VARIO_TC)),
  vario_averager_GNSS( configuration( VARIO_TC)),
  windspeed_instant_observer( configuration( WIND_TC)),
  kinetic_energy_differentiator( 1.0f, 1.0f / 100.0f),
  speed_compensation_TAS( ZERO),
  speed_compensation_GNSS( ZERO),
  vario_uncompensated_GNSS( ZERO),
  vario_uncompensated_pressure( ZERO),
  speed_compensation_fusioner( 0.998f),
  KalmanVario_GNSS( 0.0f, 0.0f, 0.0f, -9.81f),
  KalmanVario_pressure( 0.0f, 0.0f, 0.0f, -9.81f)
  {
  };
	void update
	(
	    const float3vector &gnss_velocity,
	    const float3vector &gnss_acceleration,
	    const float3vector &ahrs_acceleration,
	    const float3vector &heading_vector,
	    float GNSS_altitude,
	    float pressure_altitude,
	    float TAS,
	    circle_state_t circle_state,
	    const float3vector &wind_average
	);

	void reset(float pressure_altitude, float GNSS_altitude);

	float get_pressure_altitude( void) const;

	float get_speed_compensation_TAS( void ) const
	{
		return speed_compensation_TAS;
	}

	float get_speed_compensation_INS( void ) const
	{
		return speed_compensation_GNSS;
	}

	float get_vario_uncompensated_GNSS( void ) const
	{
		return vario_uncompensated_GNSS;
	}

	float get_vario_pressure( void ) const
	{
		return (float)( vario_averager_pressure.get_output());
	}

	float get_vario_GNSS( void ) const
	{
		return vario_averager_GNSS.get_output();
	}

	const float3vector & get_instant_wind( void ) const
	{
		return windspeed_instant_observer.get_output();
	}

	float get_effective_vertical_acceleration( void) const
	{
		return KalmanVario_GNSS.get_x( KalmanVario_PVA_t::ACCELERATION_OBSERVED);
	}

private:
	pt2<float,float> vario_averager_pressure;
	pt2<float,float> vario_averager_GNSS;
	wind_observer_t windspeed_instant_observer;

	differentiator<float,float>kinetic_energy_differentiator;

	float speed_compensation_TAS;
	float speed_compensation_GNSS;
	float vario_uncompensated_GNSS;
	float vario_uncompensated_pressure;

	HP_LP_fusion <float> speed_compensation_fusioner;

	KalmanVario_PVA_t KalmanVario_GNSS;
	KalmanVario_t KalmanVario_pressure;
};

#endif /* FLIGHT_OBSERVER_H_ */
