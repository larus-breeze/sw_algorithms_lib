/***********************************************************************//**
 * @file		Quadratic_Least_Square_Fit.h
 * @brief		Quadratic least square fit for float data
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2026 Dr. Klaus Schaefer. All rights reserved.
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

#ifndef GENERIC_ALGORITHMS_QUADRATICLEASTSQUAREFIT_H_
#define GENERIC_ALGORITHMS_QUADRATICLEASTSQUAREFIT_H_

#include "embedded_math.h"

template <typename acquisition_t, typename computation_t>  class Quadratic_Least_Square_Fit
{
public:
  Quadratic_Least_Square_Fit ()
  {
    reset();
  }

  void add_observation_point( acquisition_t abs, acquisition_t ord)
  {
    ++N;

    S1 += abs;
    S2 += abs * abs;
    S3 += abs * abs * abs;
    S4 += abs * abs * abs * abs;

    T0 += ord;
    T1 += ord * abs;
    T2 += abs * abs * ord;

    U0 += ord * ord;
  }


  void reset( void)
  {
    N=0;
    S1=S2=S3=S4=T0=T1=T2=U0 = ZERO;
    for( unsigned i=0; i<3; ++i)
      coefficient[i] = ZERO;
  }

  void calculate(void)
  {
  // evaluate determinants
    computation_t D =  N * (S2*S4-S3*S3)-S1*(S1*S4-S2*S3)+S2*(S1*S3-S2*S2);
    computation_t Da = T0* (S2*S4-S3*S3)-S1*(T1*S4-S3*T2)+S2*(T1*S3-S2*T2);
    computation_t Db = N * (T1*S4-S3*T2)-T0*(S1*S4-S2*S3)+S2*(S1*T2-T1*S2);
    computation_t Dc = N * (S2*T2-T1*S3)-S1*(S1*T2-T1*S2)+T0*(S1*S3-S2*S2);

    coefficient[0] = Da / D;
    coefficient[1] = Db / D;
    coefficient[2] = Dc / D;

    variance = ( U0 - coefficient[0] * T0 - coefficient[1] * T1 - coefficient[1] * T2) / (computation_t)(N - 3);
  }

  computation_t get_coefficient( int n)
  {
    if( n < 3)
      return coefficient[n];
    else
      {
        if( n == -1)
          return variance;
        else
          return ZERO;
      }
  }

  unsigned get_count( void)
  {
    return N;
  }

private:
  unsigned N;

  acquisition_t S1;
  acquisition_t S2;
  acquisition_t S3;
  acquisition_t S4;

  acquisition_t T0;
  acquisition_t T1;
  acquisition_t T2;

  acquisition_t U0;

  computation_t coefficient[3];
  computation_t variance;
};

#endif /* GENERIC_ALGORITHMS_QUADRATICLEASTSQUAREFIT_H_ */
