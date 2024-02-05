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

enum { N_AREAS=8, N_COEFFICIENTS=10};

typedef struct
{
  float declination; //!< positive to the east
  float inclination; //!< positive on the northern hemisphere
  bool valid;
} induction_values;

typedef struct
{
  double longitude_limit_west;
  double longitude_limit_east;
  double latitude__limit_south;
  double latitude__limit_north;
  double coefficients_declination[N_COEFFICIENTS];
  double coefficients_inclination[N_COEFFICIENTS];
} induction_model_area_t;

class earth_induction_model_t
{
private:
  static const induction_model_area_t induction_model_area[N_AREAS]; //!< 3rd order 2d polynome coefficients
public:
  earth_induction_model_t( void)
  {};

  induction_values get_induction_data_at( double longitude, double latitude);
};

extern earth_induction_model_t earth_induction_model; //!< one singleton object of this type

#endif /* NAV_ALGORITHMS_EARTH_INDUCTION_MODEL_H_ */
