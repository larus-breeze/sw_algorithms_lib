#include "data_structures.h"
#include "navigator.h"
#include "sensor_orientation_setup.h"

//! compute sensor orientation relative to airframe front/right/down system
//  and make changes permanent
  void update_sensor_orientation_data( const vector_average_collection_t & values)
  {
    // resolve sensor orientation using measurements
    float3vector front_down_sensor_helper = values.acc_observed_right.vector_multiply( values.acc_observed_left);
    float3vector u_right_sensor = front_down_sensor_helper.vector_multiply( values.acc_observed_level);
    u_right_sensor.normalize();

    float3vector u_down_sensor = values.acc_observed_level * -1.0f;
    u_down_sensor.normalize();

    float3vector u_front_sensor=u_right_sensor.vector_multiply(u_down_sensor);
    u_front_sensor.normalize();

    // calculate the new rotation matrix using our calibration data
    float3matrix new_sensor_mapping;
    new_sensor_mapping.e[0][0]=u_front_sensor[0];
    new_sensor_mapping.e[0][1]=u_front_sensor[1];
    new_sensor_mapping.e[0][2]=u_front_sensor[2];

    new_sensor_mapping.e[1][0]=u_right_sensor[0];
    new_sensor_mapping.e[1][1]=u_right_sensor[1];
    new_sensor_mapping.e[1][2]=u_right_sensor[2];

    new_sensor_mapping.e[2][0]=u_down_sensor[0];
    new_sensor_mapping.e[2][1]=u_down_sensor[1];
    new_sensor_mapping.e[2][2]=u_down_sensor[2];

    quaternion<float> q;
    q.from_rotation_matrix( new_sensor_mapping);
    eulerangle<float> euler = q;

    // make the change permanent
    write_EEPROM_value( SENS_TILT_ROLL,  euler.roll);
    write_EEPROM_value( SENS_TILT_PITCH, euler.pitch);
    write_EEPROM_value( SENS_TILT_YAW,   euler.yaw);
  }

//! fine tuning of the sensor orientation angles relative to the airframes body
// taking the attitude of a straight-and-level flight period
  void fine_tune_sensor_orientation( const vector_average_collection_t & values, const float3matrix &sensor_mapping)
  {
    float3vector gravity_measurement_body = sensor_mapping * values.acc_observed_level;

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
    unity_vector_right_body = unity_vector_down_body.vector_multiply( unity_vector_front_body);
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

    quaternion<float> q_present_setting;
    q_present_setting.from_euler (
	configuration (SENS_TILT_ROLL),
	configuration (SENS_TILT_PITCH),
	configuration (SENS_TILT_YAW));

    quaternion <float> q_sensor_orientation_corrected;
    q_sensor_orientation_corrected = q_observed_correction * q_present_setting;

    quaternion<float> q_new_setting;
    q_new_setting = q_observed_correction * q_present_setting;

    eulerangle<float> new_euler = q_new_setting;

    // make the change permanent
    write_EEPROM_value( SENS_TILT_ROLL,  new_euler.roll);
    write_EEPROM_value( SENS_TILT_PITCH, new_euler.pitch);
    write_EEPROM_value( SENS_TILT_YAW,   new_euler.yaw);
  }





