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
// The model is evaluated rarely (at start and every 15 minutes), so it runs
// in double precision.

#include <math.h>
#include "earth_induction_model.h"

static const double WMM_REFERENCE_RADIUS_KM = 6371.2;           // geomagnetic reference radius
static const double WGS84_A_KM              = 6378.137;         // semi-major axis
static const double WGS84_F                 = 1.0 / 298.257223563;
static const double WGS84_E2                = WGS84_F * ( 2.0 - WGS84_F);
static const double DEGREE                  = 3.14159265358979323846 / 180.0;
static const double MAX_LATITUDE_DEG        = 89.999;           // avoid the pole singularity of the model

// coefficient of degree n, order m at the given decimal year
static inline void coefficient( unsigned n, unsigned m, double years_since_epoch, double &g, double &h)
{
  const wmm_coefficient_t &c = WMM_COEFFICIENTS[ n * ( n + 1) / 2 + m - 1];
  g = c.g + years_since_epoch * c.dg;
  h = c.h + years_since_epoch * c.dh;
}

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
  double x = sin( geocentric_latitude);
  double s = cos( geocentric_latitude); // sin(colatitude), > 0 away from the poles
  double years = decimal_year - WMM_EPOCH;
  double ratio = WMM_REFERENCE_RADIUS_KM / r;

  double north = 0.0, east = 0.0, down = 0.0; // geocentric field components
  double p_mm = 1.0;                          // P_m^m, Schmidt semi-normalized
  for( unsigned m = 0; m <= WMM_DEGREE; ++m)
    {
      if( m == 1)
	p_mm = s;
      else if( m > 1)
	p_mm *= sqrt( ( 2.0 * m - 1.0) / ( 2.0 * m)) * s;

      double cos_ml = cos( m * longitude * DEGREE);
      double sin_ml = sin( m * longitude * DEGREE);

      double p_n1 = 0.0;  // P_{n-1}^m
      double p_n = p_mm;  // P_n^m, starting with n = m
      double ratio_n2 = pow( ratio, m + 2);
      for( unsigned n = m; n <= WMM_DEGREE; ++n)
	{
	  if( n > m) // recursion in n for fixed m
	    {
	      double p_next;
	      if( n == m + 1)
		p_next = sqrt( 2.0 * m + 1.0) * x * p_n;
	      else
		p_next = ( ( 2.0 * n - 1.0) * x * p_n
			   - sqrt( (double)( n - 1) * ( n - 1) - (double)m * m) * p_n1)
			 / sqrt( (double)n * n - (double)m * m);
	      p_n1 = p_n;
	      p_n = p_next;
	      ratio_n2 *= ratio;
	    }
	  if( n == 0)
	    continue; // no monopole term

	  // derivative with respect to the colatitude: s * dP/dtheta = n x P_n^m - sqrt(n^2 - m^2) P_{n-1}^m
	  double dp_n = ( n * x * p_n - sqrt( (double)n * n - (double)m * m) * p_n1) / s;

	  double g, h;
	  coefficient( n, m, years, g, h);
	  double gh_cos = g * cos_ml + h * sin_ml;

	  north += ratio_n2 * gh_cos * dp_n;
	  east  += ratio_n2 * m * ( g * sin_ml - h * cos_ml) * p_n;
	  down  -= ( n + 1.0) * ratio_n2 * gh_cos * p_n;
	}
    }
  east /= s;

  // rotate from the geocentric into the geodetic frame
  double psi = geocentric_latitude - latitude * DEGREE;
  double north_geodetic = north * cos( psi) - down * sin( psi);
  double down_geodetic  = north * sin( psi) + down * cos( psi);

  retv.declination = (float)( atan2( east, north_geodetic) / DEGREE);
  retv.inclination = (float)( atan2( down_geodetic, sqrt( north_geodetic * north_geodetic + east * east)) / DEGREE);
  retv.valid = true;
  return retv;
}

earth_induction_model_t earth_induction_model; //!< one singleton object of this type
