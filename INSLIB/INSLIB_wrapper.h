#ifndef INSLIB_INSLIB_WRAPPER_H_
#define INSLIB_INSLIB_WRAPPER_H_

#include "ins.h"
#include "geodetic_toolbox.h"
#include "GNSS.h"
#include "data_structures.h"

class INSLIB_wrapper
{
public:
  INSLIB_wrapper()
  :  ins({0}),
     t_us(0),
     new_GNSS_record_received(false)
  {

  }

  void update( D_GNSS_coordinates_t coordinates, measurement_data_t observations, const float3vector &mag, bool GNSS_valid)
  {
    ins_measurements_t m = {0};

    if( GNSS_valid)
      {
	t_us = (ins_time_us_t)
	    (coordinates.hour   * 3600000000.0 +
	     coordinates.minute * 60000000.0 +
	     coordinates.second * 1000000.0 +
	     coordinates.nano   / 1000);

	new_GNSS_record_received = true;

	m.timestamp        = t_us;
	m.strapdown_dt_sec = 0.01;

	m.gnss_pos.is_valid   = true;
	m.gnss_pos.llh[0]     = coordinates.latitude * M_PI / 180.0;
	m.gnss_pos.llh[1]     = coordinates.longitude * M_PI / 180.0;
	m.gnss_pos.llh[2]     = coordinates.GNSS_MSL_altitude;

	m.gnss_pos.Qll_ned[0] = 1.0f;
	m.gnss_pos.Qll_ned[4] = 1.0f;
	m.gnss_pos.Qll_ned[8] = 1.0f;

	m.gnss_vel.is_valid = true;
	m.gnss_vel.vel_ned[0] = coordinates.velocity[NORTH];
	m.gnss_vel.vel_ned[1] = coordinates.velocity[EAST];
	m.gnss_vel.vel_ned[2] = coordinates.velocity[DOWN];

	m.gnss_vel.Qll_ned[0] = SQR(coordinates.speed_acc);
	m.gnss_vel.Qll_ned[4] = SQR(coordinates.speed_acc);
	m.gnss_vel.Qll_ned[8] = SQR(coordinates.speed_acc * 2.0);

//	m.gnss_delay_ms = 20;
      }
    else
      {
	t_us += new_GNSS_record_received ? 5000 :10000;
	new_GNSS_record_received = false;

	m.timestamp        = t_us;
	m.strapdown_dt_sec = 0.01;

	m.acc.is_valid = true;
	m.acc.data[0]  = observations.acc[FRONT];
	m.acc.data[1]  = observations.acc[RIGHT];
	m.acc.data[2]  = observations.acc[BOTTOM];

	m.gyr.is_valid = true;
	m.gyr.data[0]  = observations.gyro[FRONT];
	m.gyr.data[1]  = observations.gyro[RIGHT];
	m.gyr.data[2]  = observations.gyro[BOTTOM];

	m.mag.is_valid = true;
	m.mag.data[FRONT]  = mag[FRONT] * 50.0f;
	m.mag.data[RIGHT]  = mag[RIGHT] * 50.0f;
	m.mag.data[BOTTOM] = mag[BOTTOM] * 50.0f;
      }

    ins_update( &ins, &m);
  }

  int initialize( const D_GNSS_coordinates_t & coordinates)
  {
    int retv;

    ins_init_t init = {0};

    t_us = (ins_time_us_t)
	(coordinates.hour * 3600000000.0 +
	 coordinates.minute * 60000000.0 +
	 coordinates.second * 1000000.0 +
	 coordinates.nano * 0.001);

    init.llh[0] = coordinates.latitude * M_PI / 180.0;
    init.llh[1] = coordinates.longitude * M_PI / 180.0;
    init.llh[2] = coordinates.GNSS_MSL_altitude;

    init.pos_init_stddev_m = 5.0f; /* [m]   trust the first fix ~5 m  */
    init.vel_init_stddev_mps = 1.0f; /* [m/s] trust in init. velocity */

    init.rpy_init_stddev_rad[0] = 0.1f;
    init.rpy_init_stddev_rad[1] = 0.1f;
    init.rpy_init_stddev_rad[2] = 0.1f;

    init.time = t_us;

    ins_options_t opt = {0};
    opt.auto_init = true;
    opt.estimate_mag_bias = false;

    retv = ins_init( &ins, &init, &opt);
    ins_set_magnetic_model_from_position( &ins, init.llh[0], init.llh[1], 2025.0);
    return retv;
  }

  bool get_rpy( eulerangle<float> &retv)
  {
    return ins_get_rpy( &ins, &(retv.roll), &(retv.pitch), &(retv.yaw));
  }

private:
  ins_t ins;
  ins_time_us_t t_us;
  bool new_GNSS_record_received;
};



#endif /* INSLIB_INSLIB_WRAPPER_H_ */
