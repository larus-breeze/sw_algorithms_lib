// Regression tests for the Larus algorithm library.
// Runs on the host and, via QEMU, on a Cortex-M4F (see tests/Makefile).

#include <string.h>
#include <math.h>
#include <stdint.h>

#include "doctest.h"
#include "embedded_math.h"
#include "float3matrix.h"
#include "quaternion.h"
#include "NAV_tuning_parameters.h" // Linear_Least_Square_Fit.h needs it but does not include it
#include "Linear_Least_Square_Fit.h"
#include "ascii_support.h"
#include "abstract_EEPROM_storage.h"
#include "flexible_log_file.h"

// ---- log file output into RAM -----------------------------------------------
static uint32_t log_buffer[512];
static unsigned log_words = 0;
bool write_block( uint32_t * begin, uint32_t size_words)
{
  memcpy( log_buffer + log_words, begin, size_words * sizeof( uint32_t));
  log_words += size_words;
  return true;
}

class test_log_file_t : public flexible_log_file_t
{
public:
  test_log_file_t( void) : flexible_log_file_t( log_buffer, 512) {}
  bool open( char *) { return true; }
  bool close( void) { return true; }
};

// ---- tests ---------------------------------------------------------------------
TEST_CASE( "EEPROM file system: store, update, reboot, corruption")
{
  static uint32_t storage[256];
  memset( storage, 0xff, sizeof storage);
  permanent_data_file.set_memory_to_existing_data( storage, storage + 256);

  float calibration[6] = { 0.01f, 1.01f, -0.02f, 0.99f, 0.03f, 1.02f };
  CHECK( permanent_data_file.store_data( ACCELEROMETER_CALIBRATION, 6, calibration) != nullptr);
  CHECK( permanent_data_file.store_data( BOARD_ID, (uint8_t)42));
  calibration[0] = 0.5f; // newer version of the same entry
  CHECK( permanent_data_file.store_data( ACCELEROMETER_CALIBRATION, 6, calibration) != nullptr);

  EEPROM_file_system<LOWEST_UNUSED_EEPROM_ID> after_reboot;
  CHECK( after_reboot.set_memory_to_existing_data( storage, storage + 256));
  float read_back[6] = { 0 };
  CHECK( after_reboot.retrieve_data( ACCELEROMETER_CALIBRATION, 6, read_back));
  CHECK( memcmp( read_back, calibration, sizeof calibration) == 0);
  uint8_t board_id = 0;
  CHECK( after_reboot.retrieve_data( BOARD_ID, board_id));
  CHECK( board_id == 42);
  CHECK_FALSE( after_reboot.retrieve_data( MAG_SENSOR_XFER_MATRIX, 12, read_back));

  SUBCASE( "a corrupted older entry does not hide the newer one")
  {
    storage[2] ^= 0x00000001;
    EEPROM_file_system<LOWEST_UNUSED_EEPROM_ID> corrupted;
    corrupted.set_memory_to_existing_data( storage, storage + 256);
    CHECK( corrupted.retrieve_data( ACCELEROMETER_CALIBRATION, 6, read_back));
  }
}

TEST_CASE( "log file: record headers")
{
  test_log_file_t log;
  uint32_t data[3] = { 1, 2, 3 };
  log_words = 0;
  log.append_record( (flexible_log_file_record_type)7, data, 3);
  CHECK( flexible_log_file_t::verify_record_get_size( log_buffer[0]) == 3);
  CHECK( flexible_log_file_t::verify_record_get_size( log_buffer[0] ^ 0x00010000) == 0); // corrupted CRC

  static uint32_t long_data[300]; // more than 254 words -> extended record
  log_words = 0;
  log.append_record( EEPROM_FILE, long_data, 300);
  CHECK( flexible_log_file_t::verify_record_get_size( log_buffer[0]) == 255);
  CHECK( flexible_log_file_t::verify_extended_record_get_size( log_buffer[0], log_buffer[1], log_buffer[2]) == 300);
  CHECK( log_buffer[1] == EEPROM_FILE);
}

TEST_CASE( "quaternion: euler and rotation matrix round trips")
{
  const float angles[][3] = { { 0.1f, -0.2f, 0.3f }, { -1.0f, 0.5f, 2.5f }, { 0.0f, 0.0f, -3.0f } };
  for( auto &a : angles)
    {
      CAPTURE( a[0]); CAPTURE( a[1]); CAPTURE( a[2]);
      quaternion<float> q;
      q.from_euler( a[0], a[1], a[2]);
      eulerangle<float> e = q;
      CHECK( e.roll  == doctest::Approx( a[0]).epsilon( 1e-5));
      CHECK( e.pitch == doctest::Approx( a[1]).epsilon( 1e-5));
      CHECK( e.yaw   == doctest::Approx( a[2]).epsilon( 1e-5));

      float3matrix m;
      q.get_rotation_matrix( m);
      quaternion<float> q2;
      q2.from_rotation_matrix( m);
      eulerangle<float> e2 = q2;
      CHECK( fabsf( e2.roll  - a[0]) < 1e-4f);
      CHECK( fabsf( e2.pitch - a[1]) < 1e-4f);
      CHECK( fabsf( e2.yaw   - a[2]) < 1e-4f);
    }
}

TEST_CASE( "least squares: exact line recovered")
{
  linear_least_square_fit<float> fit;
  for( int i = 0; i < 20; ++i)
    fit.add_value( (float)i, 3.0f + 0.5f * (float)i); // y = 3 + 0.5 x
  linear_least_square_result<float> r;
  bool evaluated = fit.evaluate( r);
  CHECK( evaluated);
  if( ! evaluated)
    return;
  CHECK( fabsf( r.y_offset - 3.0f) < 1e-4f);
  CHECK( fabsf( r.slope - 0.5f) < 1e-5f);
}

TEST_CASE( "ascii: to_ascii_n_decimals")
{
  // values that are formatted identically with truncation and with rounding
  struct { float value; unsigned decimals; const char * expected; } cases[] =
    { { 3.5f, 1, "3.5" }, { -12.5f, 1, "-12.5" }, { 0.25f, 2, "0.25" }, { 1013.25f, 2, "1013.25" }, { 42.0f, 3, "42.000" } };
  for( auto &c : cases)
    {
      char buffer[32];
      char * p = buffer;
      to_ascii_n_decimals( c.value, c.decimals, p);
      *p = 0;
      CHECK( doctest::String( buffer) == doctest::String( c.expected));
    }
}
