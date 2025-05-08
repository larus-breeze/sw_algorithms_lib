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
  float3vector test_right_down_body = m_sensor_2_body * v_right_down_sensor;
  float3vector test_horiz_body = m_sensor_2_body * v_horiz_sensor;

  // test if the matrix provides pure rotation
  float3vector vtest1 = m_sensor_2_body * float3vector( test1);
  float test1_abs=vtest1.abs();
  float3vector vtest2 = m_sensor_2_body * float3vector( test2);
  float test2_abs=vtest2.abs();
  float3vector vtest3 = m_sensor_2_body * float3vector( test3);
  float test3_abs=vtest3.abs();

  //check if we get the rotation angles using the matrix we have calculated
  quaternion<float> q;
  q.from_rotation_matrix(m_sensor_2_body);
  eulerangle<float> euler = q;
  euler.roll  *= 180/M_PI;
  euler.pitch *= 180/M_PI;
  euler.yaw   *= 180/M_PI;

  ////////////////////////////////////////////////////////////////////////////////////////////////

  // pitch and roll axis fine tuning
  float pitch_angle = 0.5f; 	// sensor nose is higher than configured
  float roll_angle =  -0.3f; 	// sensor is looking more left than configured
  quaternion<float>q_sensor_ideal_2_misaligned;
  q_sensor_ideal_2_misaligned.from_euler( roll_angle, pitch_angle, ZERO);

  quaternion<float>q_body_2_world; // looking NE horizontally
  q_body_2_world.from_euler( ZERO, ZERO, 45.0f * M_PI_F / 180.0f);
//  q_body_2_world.from_euler( ZERO, ZERO, -145.0f * M_PI_F / 180.0f);

  // provide simulated measurement
  quaternion <float> q_misaligned_sensor_2_world = q_sensor_ideal_2_misaligned;
  q_misaligned_sensor_2_world *= q_sensor_2_body;
  q_misaligned_sensor_2_world *= q_body_2_world;

  euler = q_misaligned_sensor_2_world;
  euler.roll  *= 180/M_PI;
  euler.pitch *= 180/M_PI;
  euler.yaw   *= 180/M_PI;

  float3matrix m_misaligned_sensor_to_world;
  q_misaligned_sensor_2_world.get_rotation_matrix( m_misaligned_sensor_to_world);

  // simulate measurement
  float3vector v_down;
  v_down[DOWN] = -10.0f;
  float3vector v_measurement_sensor = m_misaligned_sensor_to_world.reverse_map( v_down);

  q_sensor_2_body.get_rotation_matrix( m_sensor_2_body);

  float3vector v_measurement_body = m_sensor_2_body * v_measurement_sensor;

  float3matrix m_body_2_world;
  q_body_2_world.get_rotation_matrix(m_body_2_world);
  float3vector v_measurement_world = m_body_2_world * v_measurement_body;

  float pitch_angle_measured = ATAN2( -v_measurement_world[0], -v_measurement_world[2]);
  float roll_angle_measured = ATAN2( -v_measurement_world[1], -v_measurement_world[2]);

  // compute corrected sensor alignment quaternion
  quaternion <float> q_correction;
  q_correction.from_euler( roll_angle_measured, pitch_angle_measured, ZERO);

  quaternion<float> q_corrected_sensor_orientation;
}


