/** ***********************************************************************
 * @file		KalmanVario.cpp
 * @brief		Kalman Filter for vertical navigation (i.e. altitude)
 * @author		Dr. Klaus Schaefer
 * *Sensor fusion observer*
 * Blend vertical acceleration inclusive gravity with altitude data
 * to provide UN-compensated variometer plus vertical net acceleration w/o gravity
 **************************************************************************/

#include <KalmanVario.h>

ROM float KalmanVario_t::Gain[N][L]= //!< Kalman Gain for 100Hz sampling rate
    {
	   0.022706480781195,   0.000238300640696,
	   0.026080120255934,   0.008557096024865,
	   0.012200483136450,   0.282217429952530,
	  -0.011857330213848,   0.000264240373951
    };

float KalmanVario_t::update( const float altitude, const float acceleration)
{
  // predict x[] by propagating it through the system model
  float x_est_0 = x[0] + Ta * x[1] + Ta_s_2 * x[2];
  float x_est_1 = x[1] + Ta * x[2];
  float x_est_2 = x[2];
  float x_est_3 = x[3];

  float innovation_x = altitude     - x_est_0;
  float innovation_a = acceleration - x_est_2 - x_est_3;

  // x[] correction
  x[0] = x_est_0 + Gain[0][0] * innovation_x + Gain[0][1] * innovation_a;
  x[1] = x_est_1 + Gain[1][0] * innovation_x + Gain[1][1] * innovation_a;
  x[2] = x_est_2 + Gain[2][0] * innovation_x + Gain[2][1] * innovation_a;
  x[3] = x_est_3 + Gain[3][0] * innovation_x + Gain[3][1] * innovation_a;

  return x[1]; // return velocity
}
