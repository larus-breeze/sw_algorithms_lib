/***********************************************************************//**
 * @file		Linear_Least_Square_Fit.h
 * @brief		linear least square fit for arbitrary data
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

#ifndef LINEAR_LEAST_SQUARE_FIT_H_
#define LINEAR_LEAST_SQUARE_FIT_H_

#include "embedded_math.h"

//! maintains result and precision data from a linear least square fit
template<typename sample_type>
  class linear_least_square_result
  {
  public:
    linear_least_square_result(void)
    : y_offset( sample_type()),
      slope( sample_type()),
      variance_offset( sample_type()),
      variance_slope( sample_type()),
      id(0)
      {}
      sample_type y_offset;
    sample_type slope;
    sample_type variance_offset;
    sample_type variance_slope;
    uint32_t id; //!< channel identifier (for logging)
  };

//! @brief linear least square fit for  y = a + b * x
template<typename sample_type, typename evaluation_type=sample_type>
  class linear_least_square_fit
  {
  public:
    linear_least_square_fit (void)
    : sum_x(ZERO),
      sum_y(ZERO),
      sum_xx(ZERO),
      sum_yy(ZERO),
      sum_xy(ZERO),
      n(ZERO)
    {}
    void
    add_value (const sample_type x, const sample_type y)
    {
      sum_x += x;
      sum_xx += x * x;
      sum_y += y;
      sum_yy += y * y;
      sum_xy += x * y;
      ++n;
    }
    void
    reset (void)
    {
      sum_x = sum_xx = sum_y = sum_yy = sum_xy = n = ZERO;
    }
    void
    evaluate (evaluation_type &a, evaluation_type &b, evaluation_type &variance_a, evaluation_type &variance_b) const
    {
      evaluation_type inv_n = (evaluation_type)ONE / n;

      evaluation_type x_mean = (evaluation_type)sum_x * inv_n;
      evaluation_type Qx = (evaluation_type)sum_xx - inv_n * sum_x * sum_x;
      evaluation_type invQx = (evaluation_type)ONE / Qx;
      evaluation_type Qy = (evaluation_type)sum_yy - inv_n * sum_y * sum_y;
      evaluation_type Qxy = (evaluation_type)sum_xy - inv_n * sum_x * sum_y;

//      ASSERT( n > TWO);
      evaluation_type Vyx = (Qy - Qxy * Qxy / Qx) / (n - TWO);

      b = Qxy * invQx;
      a = sum_y * inv_n - b * x_mean;

      variance_a = Vyx * (inv_n + SQR( x_mean) * invQx);
      variance_b = Vyx * invQx;
    }
    void
    evaluate (linear_least_square_result<evaluation_type> &r) const
    {
      evaluate (r.y_offset, r.slope, r.variance_offset, r.variance_slope);
    }
    unsigned
    get_count (void) const
    {
      return (unsigned) n;
    }
    sample_type get_mean_y( void) const
    {
      return sum_y / n;
    }
    sample_type get_mean_x( void) const
    {
      return sum_x / n;
    }
  private:
    sample_type sum_x;
    sample_type sum_y;
    sample_type sum_xx;
    sample_type sum_yy;
    sample_type sum_xy;
    sample_type n;
  };

#endif /* LINEAR_LEAST_SQUARE_FIT_H_ */
