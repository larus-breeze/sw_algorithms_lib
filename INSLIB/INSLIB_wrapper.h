#ifndef INSLIB_INSLIB_WRAPPER_H_
#define INSLIB_INSLIB_WRAPPER_H_

#include "ins.h"
#include "geodetic_toolbox.h"
#include "GNSS.h"
#include "data_structures.h"
#include <fstream>
#include <string>
#include <iomanip>

#define MAG_UPDATE_COUNT 100000

class INSLIB_wrapper
{
public:
  INSLIB_wrapper()
  :  ins({0}),
     t_us(0),
     old(0),
     magnetic_update_countdown(MAG_UPDATE_COUNT),
     imu_values({0}),
     m({0})

#if LOG_INSLIB_CALLS
     ,
     out_file( "inslog.csv", ios::out | ios::binary | ios::ate)
#endif

  {
#if LOG_INSLIB_CALLS
    if( not out_file.is_open ())
      {
        cout << "Unable to open output file\n";
        exit( 1);
      }
#endif
  }

#if LOG_INSLIB_CALLS
  ~INSLIB_wrapper(void)
  {
    out_file.close();
  }
#endif

  void update( const D_GNSS_coordinates_t &coordinates, const state_vector_t &calibrated_data, float D_GNSS_heading, bool GNSS_valid)
  {
    if( GNSS_valid)
      {
	t_us =
	      (coordinates.hour   * (int64_t)3600000000000.0 +
	       coordinates.minute * (int64_t)60000000000.0 +
	       coordinates.second * (int64_t)1000000000.0 +
	       (int64_t)(coordinates.nano)) / 1000;

//	if( t_us < old)
//	  {
//	    m.timestamp = old;
//	    printf("[Warn] time step: %d\r", t_us - old);
//	  }
//	else
	  m.timestamp        = t_us;

	imu_values = 0;
	old = m.timestamp;

	m.strapdown_dt_sec = 0.01;

	m.gnss_pos.is_valid   = coordinates.sat_fix_type > 0;

	m.gnss_pos.llh[0]     = coordinates.latitude  * M_PI / 180.0;
	m.gnss_pos.llh[1]     = coordinates.longitude * M_PI / 180.0;

	m.gnss_pos.llh[2]     = coordinates.GNSS_MSL_altitude;

	m.gnss_pos.Qll_ned[0] = SQR(1.0f);
	m.gnss_pos.Qll_ned[4] = SQR(1.0f);
	m.gnss_pos.Qll_ned[8] = SQR(1.5f);

	m.gnss_vel.is_valid = coordinates.sat_fix_type > 0;

	m.gnss_vel.vel_ned[0] = coordinates.velocity[NORTH];
	m.gnss_vel.vel_ned[1] = coordinates.velocity[EAST];
	m.gnss_vel.vel_ned[2] = coordinates.velocity[DOWN];

	m.gnss_vel.Qll_ned[0] = SQR(coordinates.speed_acc);
	m.gnss_vel.Qll_ned[4] = SQR(coordinates.speed_acc * 1.5);
	m.gnss_vel.Qll_ned[8] = SQR(coordinates.speed_acc * 2.0);

	m.gnss_delay_ms = 80;

#if LOG_INSLIB_CALLS
	out_file << "GNSS" << ',' << t_us << ',' <<
		std::fixed << std::setprecision(9) <<
		coordinates.latitude  * M_PI / 180.0
		<< ',' <<
		coordinates.longitude * M_PI / 180.0
		<< ',' <<
		std::fixed << std::setprecision(5) <<
		coordinates.GNSS_MSL_altitude
		<< ',' <<
		coordinates.velocity[NORTH]
		<< ',' <<
		coordinates.velocity[EAST]
		<< ',' <<
		coordinates.velocity[DOWN]
		<< endl;
#endif

	if( --magnetic_update_countdown == 0 )
	  {
	    magnetic_update_countdown = MAG_UPDATE_COUNT;

	    float year = coordinates.year;
	    if( year < 2025.0f)
	      year = 2025.0f;

	    ins_set_magnetic_model_from_position( &ins, coordinates.latitude * M_PI / 180.0, coordinates.longitude * M_PI / 180.0, year);
	  }

	ins_update( &ins, &m);
	memset( &m, 0, sizeof(m));

      }
    else
      {
	m.timestamp        = t_us;
	old = m.timestamp;
	++imu_values;

	m.strapdown_dt_sec = 0.01;

	m.acc.is_valid = true;
	m.acc.data[0]  = calibrated_data.body_acc[FRONT];
	m.acc.data[1]  = calibrated_data.body_acc[RIGHT];
	m.acc.data[2]  = calibrated_data.body_acc[BOTTOM];

	m.gyr.is_valid = true;
	m.gyr.data[0]  = calibrated_data.body_gyro[FRONT];
	m.gyr.data[1]  = calibrated_data.body_gyro[RIGHT];
	m.gyr.data[2]  = calibrated_data.body_gyro[BOTTOM];

	if( coordinates.sat_fix_type & 2)
	  {
	    m.yaw.is_valid = true;
	    m.yaw.stddev_rad = 0.5 * M_PI / 180.0;
	    m.yaw_delay_ms = 80;
	    m.yaw.yaw_rad = D_GNSS_heading;
	  }

	m.mag.is_valid = true;
	float strength = ins.mag_field_expected_uT;
	m.mag.Qll_diag[0]=m.mag.Qll_diag[1]=m.mag.Qll_diag[2]=SQR( strength * (0.01));
	m.mag.data[FRONT]  = calibrated_data.body_induction[FRONT]  * strength;
	m.mag.data[RIGHT]  = calibrated_data.body_induction[RIGHT]  * strength;
	m.mag.data[BOTTOM] = calibrated_data.body_induction[BOTTOM] * strength;

	t_us += 10000; // IMU sample rate in microseconds

	if( imu_values < 8)
	  {
	    ins_update( &ins, &m);
	    memset( &m, 0, sizeof(m));
	  }

#if LOG_INSLIB_CALLS
	out_file << "IMU" << ',' << t_us << ',' <<
		calibrated_data.body_acc[FRONT]
		<< ',' <<
		calibrated_data.body_acc[RIGHT]
		<< ',' <<
		calibrated_data.body_acc[DOWN]
		<< ',' <<
		calibrated_data.body_gyro[FRONT]
		<< ',' <<
		calibrated_data.body_gyro[RIGHT]
		<< ',' <<
		calibrated_data.body_gyro[DOWN]
		<< ',' <<
		calibrated_data.body_induction[FRONT] * strength
		<< ',' <<
		calibrated_data.body_induction[RIGHT] * strength
		<< ',' <<
		calibrated_data.body_induction[DOWN] * strength
		<< endl;
#endif

      }
  }

  int initialize( const D_GNSS_coordinates_t & coordinates)
  {
    int retv;

    ins_init_t init = {0};

    init.llh[0] = coordinates.latitude * M_PI / 180.0;
    init.llh[1] = coordinates.longitude * M_PI / 180.0;
    init.llh[2] = coordinates.GNSS_MSL_altitude;

    init.pos_init_stddev_m = 5.0f; /* [m]   trust the first fix ~5 m  */
    init.vel_init_stddev_mps = 1.0f; /* [m/s] trust in init. velocity */

    init.rpy_init_stddev_rad[0] = 0.1f;
    init.rpy_init_stddev_rad[1] = 0.1f;
    init.rpy_init_stddev_rad[2] = 1.0f;

    int64_t GNSS_time_micro_s =
	  (coordinates.hour   * (int64_t)3600000000000.0 +
	   coordinates.minute * (int64_t)60000000000.0 +
	   coordinates.second * (int64_t)1000000000.0 +
	   (int64_t)(coordinates.nano)) / 1000;

    t_us = GNSS_time_micro_s;

    init.time = t_us;

    ins_options_t opt = {0};
    opt.auto_init = true;
    opt.estimate_mag_bias = false;
    opt.gnss_start_max_horizontal_vel_stddev_mps = 0.5;
    opt.gnss_start_max_vertical_vel_stddev_mps = 0.5;

    retv = ins_init( &ins, &init, &opt);
    ins_set_magnetic_model_from_position( &ins, init.llh[0], init.llh[1], 2025.0);
    magnetic_update_countdown = MAG_UPDATE_COUNT;
    return retv;
  }

  bool get_rpy( eulerangle<float> &retv)
  {
    return ins_get_rpy( &ins, &(retv.roll), &(retv.pitch), &(retv.yaw));
  }

  bool get_quaternion( quaternion <float> &out)
  {
    float data[4];
    bool retv = ins_get_quaternion( &ins, data);
    quaternion <float> tmp( data);
    out = tmp;
    return retv;
  }

  float get_latitude( void)
  {
    double llh[3];
    ins_get_latlonh(&ins, llh);
    return llh[0];
  }

  float get_longitude( void)
  {
    double llh[3];
    ins_get_latlonh(&ins, llh);
    return llh[1];
  }

private:
  ins_t ins;
  ins_time_us_t t_us;
  ins_time_us_t old;
  unsigned magnetic_update_countdown;
  unsigned imu_values;
  ins_measurements_t m;
#if LOG_INSLIB_CALLS
  ofstream out_file;
#endif

};



#endif /* INSLIB_INSLIB_WRAPPER_H_ */
