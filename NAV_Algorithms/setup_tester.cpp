#include "float3vector.h"
#include "float3matrix.h"
#include "quaternion.h"

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
  quaternion<float>sensor_q;
//  sensor_q.from_euler( 45.0 * M_PI/180.0, 135.0 * M_PI/180.0, -30.0 * M_PI/180.0);
//  sensor_q.from_euler( -135.0 * M_PI/180.0, 45.0 * M_PI/180.0, 150.0 * M_PI/180.0);
//  sensor_q.from_euler( -10.0 * M_PI/180.0, 75.0 * M_PI/180.0, -45.0 * M_PI/180.0);
  sensor_q.from_euler( 10.0 * M_PI/180.0, -75.0 * M_PI/180.0, +125.0 * M_PI/180.0);

  float3matrix sensor_2_body_assumption;
  sensor_q.get_rotation_matrix(sensor_2_body_assumption);

  //check if we get the rotation angles using the matrix given
  quaternion<float> qin;
  qin.from_rotation_matrix(sensor_2_body_assumption);
  eulerangle<float> euler_in = qin;
  euler_in.r *= 180/M_PI;
  euler_in.p *= 180/M_PI;
  euler_in.y *= 180/M_PI;

  // map measurements into the sensor frame
  float3vector left_down_sensor	= sensor_2_body_assumption.reverse_map( left_down_body);
  float3vector right_down_sensor= sensor_2_body_assumption.reverse_map( right_down_body);
  float3vector horiz_sensor 	= sensor_2_body_assumption.reverse_map( horiz_body);

  // resolve sensor orientation using measurements
  float3vector front_down_sensor_helper = right_down_sensor.vector_multiply(left_down_sensor);
  float3vector u_right_sensor = front_down_sensor_helper.vector_multiply(horiz_sensor);
  u_right_sensor.normalize();

  float3vector u_down_sensor = horiz_sensor * -1.0f;
  u_down_sensor.normalize();

  float3vector u_front_sensor=u_right_sensor.vector_multiply(u_down_sensor);
  u_front_sensor.normalize();

  // calculate the rotation matrix using our calibration data
  float3matrix sensor_2_body;
  sensor_2_body.e[0][0]=u_front_sensor[0];
  sensor_2_body.e[0][1]=u_front_sensor[1];
  sensor_2_body.e[0][2]=u_front_sensor[2];

  sensor_2_body.e[1][0]=u_right_sensor[0];
  sensor_2_body.e[1][1]=u_right_sensor[1];
  sensor_2_body.e[1][2]=u_right_sensor[2];

  sensor_2_body.e[2][0]=u_down_sensor[0];
  sensor_2_body.e[2][1]=u_down_sensor[1];
  sensor_2_body.e[2][2]=u_down_sensor[2];

  // test if the rotation matrix recovers the body frame data
  float3vector test_left_down_body = sensor_2_body * left_down_sensor;
  float3vector test_right_down_body = sensor_2_body * right_down_sensor;
  float3vector test_horiz_body = sensor_2_body * horiz_sensor;

  // test if the matrix provides pure rotation
  float3vector vtest1 = sensor_2_body * float3vector( test1);
  float test1_abs=vtest1.abs();
  float3vector vtest2 = sensor_2_body * float3vector( test2);
  float test2_abs=vtest2.abs();
  float3vector vtest3 = sensor_2_body * float3vector( test3);
  float test3_abs=vtest3.abs();

  //check if we get the rotation angles using the matrix we have calculated
  quaternion<float> q;
  q.from_rotation_matrix(sensor_2_body);
  eulerangle<float> euler = q;
  euler.r *= 180/M_PI;
  euler.p *= 180/M_PI;
  euler.y *= 180/M_PI;
}


