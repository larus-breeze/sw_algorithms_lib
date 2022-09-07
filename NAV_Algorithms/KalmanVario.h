/** ***********************************************************************
 * @file		KalmanVario.h
 * @brief		Kalman Filter for vertical navigation (i.e. altitude)
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef APPLICATION_KALMANVARIO_H_
#define APPLICATION_KALMANVARIO_H_

#include "embedded_memory.h"
#include "embedded_math.h"
#include <stdint.h>
#include "system_configuration.h"

class KalmanVario_t
{
private:

  // constants
  enum
  {
    N = 4,  //!< size of state vector x = { altitude, vario, vertical-acceleration, acceleration-offset }
    L = 2  //!< number of measurement channels = { altitude, vertical_acceleration_measurement }
  };
  static constexpr float Ta = 0.01f; 			//!< sampling rate
  static constexpr float Ta_s_2 = Ta * Ta / 2.0f; 	//!< sampling rate
  static ROM float Gain[N][L];				//!< Pre-computed Kalman Gain

  // variables
  float x[N];	//!< state vector: altitude, vario, acceleration, acceleration offset

public:
  typedef enum// state vector components
  {
    ALTITUDE, VARIO, ACCELERATION_OBSERVED, ACCELERATION_OFFSET
  }  state;

  KalmanVario_t ( float _x=ZERO, float v=ZERO, float a=ZERO, float a_offset=ZERO)
    : x{_x, v, a, a_offset}
  {}

  void reset(  const float altitude, const float acceleration_offset)
  {
    x[0] = altitude;
    x[1] = 0.0f;
    x[2] = 0.0f;
    x[3] = acceleration_offset;
  }

  float update( const float altitude, const float acceleration);

  inline float get_x( state index) const
  {
    if( index <= ACCELERATION_OFFSET)
      return x[index];
    else
      return x[ACCELERATION_OBSERVED] + x[ACCELERATION_OFFSET]; // = acceleration minus offset
  };
};

#endif /* APPLICATION_KALMANVARIO_H_ */
