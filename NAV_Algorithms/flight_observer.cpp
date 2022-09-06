/** ***********************************************************************
 * @file		flight_observer.cpp
 * @brief		windspeed and vario update
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#include "system_configuration.h"
#include "flight_observer.h"
#include "embedded_math.h"

#define ONE_DIV_BY_GRAVITY_TIMES_2 0.0509684f
#define RECIP_GRAVITY 0.1094f

// to be called at 100 Hz
void flight_observer_t::update (
    const float3vector &gnss_velocity,
    const float3vector &gnss_acceleration,
    const float3vector &ahrs_acceleration,
    const float3vector &heading_vector,
    float GNSS_negative_altitude,
    float pressure_altitude,
    float TAS,
    circle_state_t circle_state,
    const float3vector &wind_average
  )
{
  vario_uncompensated_pressure = KalmanVario_pressure.update ( pressure_altitude, ahrs_acceleration.e[DOWN]);
  speed_compensation_TAS = kinetic_energy_differentiator.respond( TAS * TAS * ONE_DIV_BY_GRAVITY_TIMES_2);
  vario_averager_pressure.respond( speed_compensation_TAS  - vario_uncompensated_pressure); 	// -> positive on positive energy gain

  if( isnan( gnss_acceleration.e[NORTH])) // no GNSS fix by now
    {
      // workaround for no GNSS fix: maintain GNSS vario with pressure data
      vario_uncompensated_GNSS = - KalmanVario_GNSS.update ( pressure_altitude, ZERO, ahrs_acceleration.e[DOWN]); // todo MURX: what can we do without vertical speed information ?
      // this time use pressure vario
      vario_averager_GNSS.respond( speed_compensation_TAS  - vario_uncompensated_pressure);
    }
  else
    {
      float3vector air_velocity = heading_vector * TAS;
      windspeed_instant_observer.update( gnss_velocity - air_velocity, heading_vector, circle_state);

      // non TEC compensated vario, negative if *climbing* !
#if GNSS_VERTICAL_SPEED_INVERTED
      vario_uncompensated_GNSS = - KalmanVario_GNSS.update ( GNSS_negative_altitude, - gnss_velocity.e[DOWN], ahrs_acceleration.e[DOWN]);
#else
      vario_uncompensated_GNSS = - KalmanVario_GNSS.update ( GNSS_negative_altitude, gnss_velocity.e[DOWN], ahrs_acceleration.e[DOWN]);
#endif
      float speed_compensation =
    		  (
    		      ((gnss_velocity.e[NORTH] - wind_average.e[NORTH]) * gnss_acceleration.e[NORTH]) +
    		      ((gnss_velocity.e[EAST]  - wind_average.e[EAST])  * gnss_acceleration.e[EAST])  +
    		      (KalmanVario_GNSS.get_x(KalmanVario_PVA_t::VARIO) * KalmanVario_GNSS.get_x(KalmanVario_PVA_t::ACCELERATION_OBSERVED))
    		   ) * RECIP_GRAVITY;

      speed_compensation_GNSS = speed_compensation_fusioner.respond( speed_compensation, speed_compensation_TAS);
      vario_averager_GNSS.respond( speed_compensation_GNSS + vario_uncompensated_GNSS);
    }
}

void flight_observer_t::reset(float pressure_negative_altitude, float GNSS_negative_altitude)
{
  KalmanVario_GNSS.reset( GNSS_negative_altitude, -9.81f);
  KalmanVario_pressure.reset( pressure_negative_altitude, -9.81f);
}
