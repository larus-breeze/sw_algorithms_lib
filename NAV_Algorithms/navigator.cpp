/** ***********************************************************************
 * @file		navigator.cpp
* @brief		maintain speed attitude and position data
  * @author		Dr. Klaus Schaefer
 **************************************************************************/

#include <navigator.h>

// to be called at 100 Hz
void navigator_t::update_IMU (
    const float3vector &acc,
    const float3vector &mag,
    const float3vector &gyro)
{
  ahrs.update( gyro, acc, mag,
	    GNSS_acceleration,
	    GNSS_heading,
	    GNSS_fix_type == (SAT_FIX | SAT_HEADING));

#if PARALLEL_MAGNETIC_AHRS
#if 1
  ahrs_magnetic.update_special(
	  gyro, acc, mag,
	  GNSS_acceleration);
#else
  ahrs_magnetic.update_compass(
	  gyro, acc, mag,
	  GNSS_acceleration);
#endif
#endif
  float3vector heading_vector;
  heading_vector[NORTH] = ahrs.get_north ();
  heading_vector[EAST]  = ahrs.get_east  ();
  heading_vector[DOWN]  = ahrs.get_down  (); // todo: do we need this one ?

  flight_observer.update (
      GNSS_velocity,
      GNSS_acceleration,
      ahrs.get_nav_acceleration (),
      heading_vector,
      GNSS_negative_altitude,
      atmosphere.get_negative_altitude(),
      TAS,
      ahrs.get_circling_state(),
      wind_average_observer.get_value()
      );
}

// to be called at 10 Hz
void navigator_t::update_GNSS (const coordinates_t &coordinates)
{
  if( coordinates.sat_fix_type == SAT_FIX_NONE) // presently no GNSS fix
      return; // todo needs to be improved

  GNSS_fix_type		= coordinates.sat_fix_type;
  GNSS_velocity 	= coordinates.velocity;
  GNSS_acceleration	= coordinates.acceleration;
  GNSS_heading 		= coordinates.relPosHeading;
  GNSS_negative_altitude= coordinates.position.e[DOWN];
  GNSS_speed 		= coordinates.speed_motion;

  wind_average_observer.update( flight_observer.get_instant_wind(), // do this here because of the update rate 10Hz
				ahrs.get_euler ().y,
				ahrs.get_circling_state ());

  float3vector relative_wind_NAV  = flight_observer.get_instant_wind() - wind_average_observer.get_value();
  float3vector relative_wind_BODY =  ahrs.get_body2nav().reverse_map(relative_wind_NAV);
  relative_wind_observer.update(relative_wind_BODY,
				ahrs.get_euler ().y,
				ahrs.get_circling_state ());

  float3vector wind_correction_nav    = ahrs.get_body2nav() * relative_wind_observer.get_value();
  wind_correction_nav.e[DOWN]=0.0f; // ignore vertical component as this van cause an underflow error
  float3vector instant_wind_corrected = flight_observer.get_instant_wind() - wind_correction_nav;

  if( ahrs.get_circling_state () == CIRCLING)
    corrected_wind_averager.respond( instant_wind_corrected);
  else
    corrected_wind_averager.respond( flight_observer.get_instant_wind()); // todo bad: cascaded lowpass filters !

  vario_integrator.update (flight_observer.get_vario_GNSS(), // here because of the update rate 10Hz
			   ahrs.get_euler ().y,
			   ahrs.get_circling_state ());
}

void navigator_t::report_data( output_data_t &d)
{
    d.TAS 			= TAS_averager.get_output();
    d.IAS 			= IAS;

    d.euler			= ahrs.get_euler();
    d.q				= ahrs.get_attitude();

#if PARALLEL_MAGNETIC_AHRS
    d.euler_magnetic		= ahrs_magnetic.get_euler();
    d.q_magnetic		= ahrs_magnetic.get_attitude();
#endif
    d.vario			= flight_observer.get_vario_GNSS(); // todo pick one vario
    d.vario_pressure		= flight_observer.get_vario_pressure();
    d.integrator_vario		= vario_integrator.get_value();
    d.vario_uncompensated 	= flight_observer.get_vario_uncompensated_GNSS();

    d.wind			= corrected_wind_averager.get_output(); // short-term avg
    d.wind_average		= wind_average_observer.get_value();  // smart long-term avg

    d.speed_compensation_TAS 	= flight_observer.get_speed_compensation_TAS();
    d.speed_compensation_INS 	= flight_observer.get_speed_compensation_INS();
    d.effective_vertical_acceleration
				= flight_observer.get_effective_vertical_acceleration();

    d.circle_mode 		= ahrs.get_circling_state();
    d.nav_correction		= ahrs.get_nav_correction();
    d.gyro_correction		= ahrs.get_gyro_correction();
    d.nav_acceleration_gnss 	= ahrs.get_nav_acceleration();
    d.nav_induction_gnss 	= ahrs.get_nav_induction();

#if PARALLEL_MAGNETIC_AHRS
    d.nav_acceleration_mag 	= ahrs_magnetic.get_nav_acceleration();
    d.nav_induction_mag 	= ahrs_magnetic.get_nav_induction();
#endif

    d.turn_rate			= ahrs.get_turn_rate();
    d.slip_angle		= ahrs.getSlipAngle();
    d.nick_angle		= ahrs.getNickAngle();
    d.G_load			= ahrs.get_G_load();
    d.HeadingDifferenceAhrsDgnss = ahrs.getHeadingDifferenceAhrsDgnss();
    d.QFF			= atmosphere.get_QFF();
    d.air_density		= atmosphere.get_density();
}

