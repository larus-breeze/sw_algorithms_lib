/***********************************************************************//**
 * @file		earth_induction_model.h
 * @brief		find position-dependent data for magnetic declination and magnetic inclination
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
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

#ifndef NAV_ALGORITHMS_EARTH_INDUCTION_MODEL_H_
#define NAV_ALGORITHMS_EARTH_INDUCTION_MODEL_H_

#include "embedded_memory.h"
#include "embedded_math.h"
#include "wmm_coefficients.h"

//! decimal year used when no date is known: the middle of the model's validity
#define WMM_DEFAULT_YEAR ( WMM_EPOCH + 2.5)

//! struct containing magnetic induction data for a point
typedef struct
{
  float declination; //!< degrees, positive to the east
  float inclination; //!< degrees, positive downward (northern hemisphere)
  bool valid;        //!< false only for invalid input (NaN, |latitude| > 90)
} induction_values;

//! Worldwide magnetic declination and inclination from the World Magnetic Model
//! (spherical harmonic model, coefficients in wmm_coefficients.h)
class earth_induction_model_t
{
public:
  earth_induction_model_t( void)
  {};

  //! declination and inclination at a geodetic position
  //! @param latitude     geodetic latitude, degrees north
  //! @param longitude    longitude, degrees east
  //! @param decimal_year date, e.g. 2026.75 (secular variation)
  //! @param altitude_km  height above the WGS84 ellipsoid in km
  induction_values get_induction_data_at( double latitude, double longitude,
					  double decimal_year = WMM_DEFAULT_YEAR,
					  double altitude_km = 0.0) const;
};

extern earth_induction_model_t earth_induction_model; //!< one singleton object of this type

#endif /* NAV_ALGORITHMS_EARTH_INDUCTION_MODEL_H_ */
