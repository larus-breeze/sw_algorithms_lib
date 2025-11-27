#include "float3vector.h"
#include "float3matrix.h"
#include "quaternion.h"
#include "AHRS.h"

const float ldi[]={0.1f, +0.25f, -10.0f}; // left wing down
const float rdi[]={0.1f, -0.15f, -10.0f}; // right wing down
const float hi[] ={0.0f, 0.0f, -10.0f};   // horizontal

const float test1[] ={ 1.0f, 0.0f, 0.0f};
const float test2[] ={ 0.57735f, 0.57735f, -0.57735f};
const float test3[] ={ 0.57735f, -0.57735f, 0.57735f};

void setup_tester(void)
{
  // create measurements in aircraft body frame
  float3vector left_down_body( ldi);
  float3vector right_down_body( rdi);
  float3vector horiz_body( hi);

  // setup the sensor orientation to test
  quaternion<float>q_sensor_2_body;
//  q_sensor_2_body.from_euler( 0 * M_PI/180.0, 0 * M_PI/180.0, 13 * M_PI/180.0);
  q_sensor_2_body.from_euler( 90 * M_PI/180.0, 180.0 * M_PI/180.0, -30 * M_PI/180.0);
//  sensor_q.from_euler( 45.0 * M_PI/180.0, 135.0 * M_PI/180.0, -30.0 * M_PI/180.0);
//  sensor_q.from_euler( -135.0 * M_PI/180.0, 45.0 * M_PI/180.0, 150.0 * M_PI/180.0);
//  sensor_q.from_euler( -10.0 * M_PI/180.0, 75.0 * M_PI/180.0, -45.0 * M_PI/180.0);
//  sensor_q.from_euler( 10.0 * M_PI/180.0, -75.0 * M_PI/180.0, +125.0 * M_PI/180.0);

  float3matrix m_sensor_2_body_assumption;
  q_sensor_2_body.get_rotation_matrix( m_sensor_2_body_assumption);

  //check if we get the rotation angles using the matrix given
  quaternion<float> q_sensor_2_body_assumption;
  q_sensor_2_body_assumption.from_rotation_matrix(m_sensor_2_body_assumption);
  eulerangle<float> euler_in = q_sensor_2_body_assumption;
  euler_in.roll  *= 180/M_PI;
  euler_in.pitch *= 180/M_PI;
  euler_in.yaw   *= 180/M_PI;

  // map measurements into the sensor frame
  float3vector v_left_down_sensor	= m_sensor_2_body_assumption.reverse_map( left_down_body);
  float3vector v_right_down_sensor= m_sensor_2_body_assumption.reverse_map( right_down_body);
  float3vector v_horiz_sensor 	= m_sensor_2_body_assumption.reverse_map( horiz_body);

  // resolve sensor orientation using measurements
  float3vector front_down_sensor_helper = v_right_down_sensor.vector_multiply(v_left_down_sensor);
  float3vector u_right_sensor = front_down_sensor_helper.vector_multiply(v_horiz_sensor);
  u_right_sensor.normalize();

  float3vector u_down_sensor = v_horiz_sensor * -1.0f;
  u_down_sensor.normalize();

  float3vector u_front_sensor=u_right_sensor.vector_multiply(u_down_sensor);
  u_front_sensor.normalize();

  // calculate the rotation matrix using our calibration data
  float3matrix m_sensor_2_body;
  m_sensor_2_body.e[FRONT][0]=u_front_sensor[0];
  m_sensor_2_body.e[FRONT][1]=u_front_sensor[1];
  m_sensor_2_body.e[FRONT][2]=u_front_sensor[2];

  m_sensor_2_body.e[RIGHT][0]=u_right_sensor[0];
  m_sensor_2_body.e[RIGHT][1]=u_right_sensor[1];
  m_sensor_2_body.e[RIGHT][2]=u_right_sensor[2];

  m_sensor_2_body.e[DOWN][0]=u_down_sensor[0];
  m_sensor_2_body.e[DOWN][1]=u_down_sensor[1];
  m_sensor_2_body.e[DOWN][2]=u_down_sensor[2];

  // test if the rotation matrix recovers the body frame data
  float3vector test_left_down_body = m_sensor_2_body * v_left_down_sensor;
  (void)test_left_down_body;
  float3vector test_right_down_body = m_sensor_2_body * v_right_down_sensor;
  (void)test_right_down_body;
  float3vector test_horiz_body = m_sensor_2_body * v_horiz_sensor;
  (void)test_horiz_body;

  // test if the matrix provides pure rotation
  float3vector vtest1 = m_sensor_2_body * float3vector( test1);
  float test1_abs=vtest1.abs();
  (void)test1_abs;
  float3vector vtest2 = m_sensor_2_body * float3vector( test2);
  float test2_abs=vtest2.abs();
  (void)test2_abs;
  float3vector vtest3 = m_sensor_2_body * float3vector( test3);
  float test3_abs=vtest3.abs();
  (void)test3_abs;

  //check if we get the rotation angles using the matrix we have calculated
  quaternion<float> q;
  q.from_rotation_matrix(m_sensor_2_body);
  eulerangle<float> euler = q;
  euler.roll  *= 180/M_PI;
  euler.pitch *= 180/M_PI;
  euler.yaw   *= 180/M_PI;

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // quaternion chaining test
  quaternion<float> first;
  first.from_euler( ZERO, ZERO, 90.0f * M_PI_F / 180.0f);

  quaternion<float> second;
  second.from_euler( ZERO, 10.0f * M_PI_F / 180.0f, ZERO);

  quaternion<float> combi;

  combi = second * first;

  euler = combi;
  euler.roll  *= 180/M_PI;
  euler.pitch *= 180/M_PI;
  euler.yaw   *= 180/M_PI;

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // pitch and roll axis fine tuning

  float sensor_orientation_roll  = 10.0f * M_PI_F / 180;
  float sensor_orientation_pitch = 120.0f * M_PI_F / 180;
  float sensor_orientation_yaw   = -30.0f * M_PI_F / 180;

  float pitch_angle_delta = 3.0  * M_PI_F / 180; 	// sensor nose is higher than configured
  float roll_angle_delta  = -4.0 * M_PI_F / 180; 	// sensor is looking more left than configured

  quaternion<float> q_sensor_orientation_configured;
  q_sensor_orientation_configured.from_euler( sensor_orientation_roll, sensor_orientation_pitch, sensor_orientation_yaw);

  quaternion<float> q_sensor_2_body_real;
  q_sensor_2_body_real.from_euler( sensor_orientation_roll + roll_angle_delta, sensor_orientation_pitch + pitch_angle_delta, sensor_orientation_yaw);

  quaternion<float> q_body_2_sensor_real = q_sensor_2_body_real.inverse();

  float airframe_orientation_roll  = 0.0f;
  float airframe_orientation_pitch = 0.0f;
  float airframe_orientation_yaw   = 123.4f;
  quaternion<float> q_airframe_orientation;
  q_airframe_orientation.from_euler( airframe_orientation_roll, airframe_orientation_pitch, airframe_orientation_yaw);

  quaternion<float> q_world_to_misaligned_sensor;
  q_world_to_misaligned_sensor = q_body_2_sensor_real * q_airframe_orientation;
  float3matrix m_world_to_misaligned_sensor;
  q_world_to_misaligned_sensor.get_rotation_matrix( m_world_to_misaligned_sensor);

  eulerangle<float> euler_misaligned_sensor = q_world_to_misaligned_sensor;
  euler_misaligned_sensor.roll  *= 180/M_PI;
  euler_misaligned_sensor.pitch *= 180/M_PI;
  euler_misaligned_sensor.yaw   *= 180/M_PI;

  // make a measurement of the gravity vector,
  // assuming no velocity change at this moment
  float3vector gravity;
  gravity[DOWN] = -9.81;

  // compute what the sensor will see
  float3vector gravity_measurement_sensor;
  gravity_measurement_sensor  = m_world_to_misaligned_sensor * gravity;

  // map this to the airframe using the configured sensor orientation
  float3matrix m_sensor_orientation_configured;
  q_sensor_orientation_configured.get_rotation_matrix( m_sensor_orientation_configured);

  float3vector gravity_measurement_body;
  gravity_measurement_body = m_sensor_orientation_configured * gravity_measurement_sensor;

  // correct for "gravity pointing to minus "down" "
  gravity_measurement_body.negate();

  // evaluate observed "down" direction in the body frame
  float3vector unity_vector_down_body;
  unity_vector_down_body = gravity_measurement_body;
  unity_vector_down_body.normalize();

  // find two more unity vectors defining the corrected coordinate system
  float3vector unity_vector_front_body;
  unity_vector_front_body[FRONT] = unity_vector_down_body[DOWN];
  unity_vector_front_body[DOWN]  = unity_vector_down_body[FRONT];
  unity_vector_front_body.normalize();

  float3vector unity_vector_right_body;
#if 1
  unity_vector_right_body = unity_vector_down_body.vector_multiply( unity_vector_front_body);
#else
  unity_vector_right_body[RIGHT] = unity_vector_down_body[DOWN];
  unity_vector_right_body[DOWN]  = - unity_vector_down_body[RIGHT];
#endif
  unity_vector_right_body.normalize();

  // fine tune the front vector using the other ones
  unity_vector_front_body = unity_vector_right_body.vector_multiply( unity_vector_down_body);

  // calculate the rotation matrix using our calibration data
  float3matrix observed_correction_matrix;
  observed_correction_matrix.e[FRONT][0]=unity_vector_front_body[0];
  observed_correction_matrix.e[FRONT][1]=unity_vector_front_body[1];
  observed_correction_matrix.e[FRONT][2]=unity_vector_front_body[2];

  observed_correction_matrix.e[RIGHT][0]=unity_vector_right_body[0];
  observed_correction_matrix.e[RIGHT][1]=unity_vector_right_body[1];
  observed_correction_matrix.e[RIGHT][2]=unity_vector_right_body[2];

  observed_correction_matrix.e[DOWN][0]=unity_vector_down_body[0];
  observed_correction_matrix.e[DOWN][1]=unity_vector_down_body[1];
  observed_correction_matrix.e[DOWN][2]=unity_vector_down_body[2];

  quaternion<float> q_observed_correction;
  q_observed_correction.from_rotation_matrix(observed_correction_matrix);

  eulerangle<float> correction_euler = q_observed_correction;

  correction_euler.roll  *= 180/M_PI;
  correction_euler.pitch *= 180/M_PI;
  correction_euler.yaw   *= 180/M_PI;

  // check if correction combined with old orientation setting fits
  quaternion <float> q_sensor_orientation_corrected;
  q_sensor_orientation_corrected = q_observed_correction * q_sensor_orientation_configured;

  float3matrix m_sensor_orientation_corrected;
  q_sensor_orientation_corrected.get_rotation_matrix( m_sensor_orientation_corrected);

  // now we check if gravity points down using the corrected sensor orientation
  gravity_measurement_body = m_sensor_orientation_corrected * gravity_measurement_sensor;

  // now we check if the north unit vector stays north
  float3vector north;
  north[NORTH] = 1.0f;

  float3vector attitude_measurement_sensor;
  attitude_measurement_sensor  = m_world_to_misaligned_sensor * north;
  float3vector attitude_measurement_body;
  attitude_measurement_body = m_sensor_orientation_corrected * attitude_measurement_sensor;

  float3matrix m_world_2_body;
  q_airframe_orientation.get_rotation_matrix( m_world_2_body);

  float3vector attitude_measurement_world;
  attitude_measurement_world = m_world_2_body.reverse_map( attitude_measurement_body);
}


