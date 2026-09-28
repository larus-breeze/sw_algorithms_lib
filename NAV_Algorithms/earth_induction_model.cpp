/***********************************************************************//**
 * @file		earth_induction_model.cpp
 * @brief		magnetic declination and inclination from the World Magnetic Model
 * @author		Dominic Spreitz
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

// Evaluation of the World Magnetic Model as described in "The US/UK World
// Magnetic Model for 2025-2030: Technical Report" (NOAA NCEI), section 1.2:
// geodetic -> geocentric coordinates, Schmidt semi-normalized associated
// Legendre functions by recursion, field components, rotation back into the
// geodetic frame. Only scalar recursions are used, so no tables on the stack.
// The coordinate conversion runs once in double precision, the sum over the
// coefficients in float on the FPU: about 0.2 ms on the STM32F407 instead of
// about 4 ms with software double precision. Errors against the reference
// implementation: below 0.005 degrees (0.001 degrees between 80 S and 80 N).

#include <math.h>
#include "earth_induction_model.h"

static const double WMM_REFERENCE_RADIUS_KM = 6371.2;           // geomagnetic reference radius
static const double WGS84_A_KM              = 6378.137;         // semi-major axis
static const double WGS84_F                 = 1.0 / 298.257223563;
static const double WGS84_E2                = WGS84_F * ( 2.0 - WGS84_F);
static const double DEGREE                  = 3.14159265358979323846 / 180.0;
static const double MAX_LATITUDE_DEG        = 89.999;           // avoid the pole singularity of the model

induction_values earth_induction_model_t::get_induction_data_at( double latitude, double longitude,
								 double decimal_year, double altitude_km) const
{
  induction_values retv = { 0.0f, 0.0f, false };
  if( ! ( fabs( latitude) <= 90.0) || ! ( fabs( longitude) <= 360.0)
      || decimal_year != decimal_year || altitude_km != altitude_km) // NaN or out of range
    return retv;

  if( latitude > MAX_LATITUDE_DEG)
    latitude = MAX_LATITUDE_DEG;
  if( latitude < -MAX_LATITUDE_DEG)
    latitude = -MAX_LATITUDE_DEG;

  // geodetic -> geocentric spherical coordinates
  double sin_lat = sin( latitude * DEGREE), cos_lat = cos( latitude * DEGREE);
  double curvature_radius = WGS84_A_KM / sqrt( 1.0 - WGS84_E2 * sin_lat * sin_lat);
  double p = ( curvature_radius + altitude_km) * cos_lat;
  double z = ( curvature_radius * ( 1.0 - WGS84_E2) + altitude_km) * sin_lat;
  double r = sqrt( p * p + z * z);
  double geocentric_latitude = asin( z / r);

  // Legendre functions in x = cos(colatitude) = sin(geocentric latitude)
  float x = (float)( z / r);
  float s = (float)( p / r);  // sin(colatitude), > 0 away from the poles
  float recip_s = 1.0f / s;
  float years = (float)( decimal_year - WMM_EPOCH);
  float ratio = (float)( WMM_REFERENCE_RADIUS_KM / r);
  float cos_l = (float)cos( longitude * DEGREE);
  float sin_l = (float)sin( longitude * DEGREE);

  float north = 0.0f, east = 0.0f, down = 0.0f; // geocentric field components
  float p_mm = 1.0f;                            // P_m^m, Schmidt semi-normalized
  float ratio_m2 = ratio * ratio;               // ratio^(m+2)
  float cos_ml = 1.0f, sin_ml = 0.0f;           // cos(m*longitude), sin(m*longitude)
  for( unsigned m = 0; m <= WMM_DEGREE; ++m)
    {
      if( m > 0)
	{
	  p_mm *= ( m == 1) ? s : SQRT( ( 2.0f * m - 1.0f) / ( 2.0f * m)) * s;
	  float c = cos_ml * cos_l - sin_ml * sin_l;
	  sin_ml = sin_ml * cos_l + cos_ml * sin_l;
	  cos_ml = c;
	  ratio_m2 *= ratio;
	}

      float p_n1 = 0.0f;    // P_{n-1}^m
      float p_n = p_mm;     // P_n^m, starting with n = m
      float root_n1 = 0.0f; // sqrt((n-1)^2 - m^2)
      float ratio_n2 = ratio_m2;
      for( unsigned n = m; n <= WMM_DEGREE; ++n)
	{
	  float root_n = SQRT( (float)( n * n - m * m)); // sqrt(n^2 - m^2)
	  if( n > m) // recursion in n for fixed m
	    {
	      float p_next = ( ( 2.0f * n - 1.0f) * x * p_n - root_n1 * p_n1) / root_n;
	      p_n1 = p_n;
	      p_n = p_next;
	      ratio_n2 *= ratio;
	    }
	  root_n1 = root_n;
	  if( n == 0)
	    continue; // no monopole term

	  // derivative with respect to the colatitude: s * dP/dtheta = n x P_n^m - sqrt(n^2 - m^2) P_{n-1}^m
	  float dp_n = ( n * x * p_n - root_n * p_n1) * recip_s;

	  const wmm_coefficient_t &c = WMM_COEFFICIENTS[ n * ( n + 1) / 2 + m - 1];
	  float g = c.g + years * c.dg;
	  float h = c.h + years * c.dh;
	  float gh_cos = g * cos_ml + h * sin_ml;

	  north += ratio_n2 * gh_cos * dp_n;
	  east  += ratio_n2 * m * ( g * sin_ml - h * cos_ml) * p_n;
	  down  -= ( n + 1.0f) * ratio_n2 * gh_cos * p_n;
	}
    }
  east *= recip_s;

  // rotate from the geocentric into the geodetic frame
  double psi = geocentric_latitude - latitude * DEGREE;
  double north_geodetic = north * cos( psi) - down * sin( psi);
  double down_geodetic  = north * sin( psi) + down * cos( psi);

  retv.declination = (float)( atan2( (double)east, north_geodetic) / DEGREE);
  retv.inclination = (float)( atan2( down_geodetic, sqrt( north_geodetic * north_geodetic + (double)east * east)) / DEGREE);
  retv.valid = true;
  return retv;
}

earth_induction_model_t earth_induction_model; //!< one singleton object of this type
