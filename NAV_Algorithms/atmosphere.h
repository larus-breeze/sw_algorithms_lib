/***********************************************************************//**
 * @file		atmosphere.h
 * @brief		computes properties of earth's atmosphere
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * 			air density formula developed by Philipp Puetz
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

#ifndef APPLICATION_ATMOSPHERE_H_
#define APPLICATION_ATMOSPHERE_H_

#include "embedded_math.h"
#include <air_density_observer.h>
#include <NAV_tuning_parameters.h>
#include <pt2.h>

#define RECIP_STD_DENSITY_TIMES_2 1.632f

/*! The gas constant of dry air in J/kg/K */
#define GAS_CONST_DRY_AIR	287.058f
/*! The gas constant of water vapor in J/kg/K */
#define GAS_CONST_WATER_VAPOR	461.523f
/*! The ratio of the gas constant of dry air to the gas contant of water vapor */
#define ONE_MINUS_RATIO_GAS_CONSTANTS 0.378f
/*! The offest for the conversion from degree celsius to kelvin */
#define CELSIUS_TO_KELVIN_OFFSET 273.15f

//! this class maintains instant atmosphere data like pressure, density etc
class atmosphere_t
{
public:
  atmosphere_t( float p_abs)
  :
    have_ambient_air_data(false),
    pressure ( p_abs),
    temperature(20.0f),
    humidity( 0.0f),
    density_correction(1.0f),
    density_correction_averager(0.001f),
    QFF(101325)
  {
    density_correction_averager.settle(1.0f);
  }

  //! set density to ICAO std density at present altitude
  void normalize_density_correction( float GNSS_altitude)
  {
    float density_from_pressure = 1.0496346613e-5f * pressure + 0.1671546011f;
    float ICAO_density_from_altitude = 1.2250 + GNSS_altitude * -1.1659e-4 + GNSS_altitude * GNSS_altitude * 3.786e-9;
    density_correction = ICAO_density_from_altitude / density_from_pressure;
    density_correction_averager.settle(density_correction);
  }
  void update_density_correction( void)
  {
    density_correction_averager.respond(density_correction);
  }
  void initialize( float altitude)
  {
    density_QFF_calculator.initialize(altitude);
  }
  void set_pressure( float p_abs)
  {
    pressure = p_abs;
  }
  float get_pressure( void) const
  {
    return pressure;
  }
  float get_density( void) const
  {
    return  (1.0496346613e-5f * pressure + 0.1671546011f) * density_correction_averager.get_output();
  }
  float get_negative_pressure_altitude( void) const
  {
    float tmp = 8.104381531e-4f * pressure;
    return - tmp * tmp  + 0.20867299170f * pressure - 14421.43945f;
  }
  float get_TAS_from_dynamic_pressure( float dynamic_pressure) const
  {
    return SQRT( 2 * dynamic_pressure / get_density());
  }
  float get_IAS_from_dynamic_pressure( float dynamic_pressure) const
  {
    return SQRT( dynamic_pressure * RECIP_STD_DENSITY_TIMES_2);
  }
  void set_ambient_air_data( float temperature, float humidity)
  {
    this->temperature = temperature;
    this->humidity = humidity;
    have_ambient_air_data = true;
  }
  void disregard_ambient_air_data( void)
  {
    have_ambient_air_data = false;
  }

  float get_QFF () const
  {
    return QFF;
  }

  void feed_QFF_density_metering( float pressure, float MSL_altitude)
    {
    air_data_result result = density_QFF_calculator.feed_metering( pressure, MSL_altitude);
      if( result.valid)
	{
	  QFF = QFF * AIR_DENSITY_LETHARGY_FACTOR + (1.0f - AIR_DENSITY_LETHARGY_FACTOR) * result.QFF;
	  density_correction = density_correction * AIR_DENSITY_LETHARGY_FACTOR + (1.0f - AIR_DENSITY_LETHARGY_FACTOR) * result.density_correction;
	}
    }

private:
  float calculateGasConstantHumAir(
      float humidity, float pressure, float temperature);
  float calculateAirDensity(
      float humidity, float pressure, float temperature);
  float calculateSaturationVaporPressure(float temp);
  bool have_ambient_air_data;
  float pressure;
  float temperature;
  float humidity;
  float density_correction;
  pt2<float,float> density_correction_averager;
  float QFF;
  air_density_observer density_QFF_calculator;
};

#endif /* APPLICATION_ATMOSPHERE_H_ */
