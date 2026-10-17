// Regression tests for the Larus algorithm library.
// Runs on the host and, via QEMU, on a Cortex-M4F (see tests/Makefile).
// Returns 0 if all checks pass.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "embedded_math.h"
#include "float3matrix.h"
#include "quaternion.h"
#include "NAV_tuning_parameters.h" // Linear_Least_Square_Fit.h needs it but does not include it
#include "Linear_Least_Square_Fit.h"
#include "ascii_support.h"
#include "abstract_EEPROM_storage.h"
#include "flexible_log_file.h"

// ---- environment stubs --------------------------------------------------
EEPROM_file_system<LOWEST_UNUSED_EEPROM_ID> permanent_data_file;
void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words, bool)
{
  memcpy( dest, source, n_words * sizeof( uint32_t));
}

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

// ---- test helpers --------------------------------------------------------
static unsigned checks = 0, failures = 0;
static void check( bool ok, const char * what)
{
  ++checks;
  if( ! ok)
    {
      ++failures;
      printf( "FAIL: %s\n", what);
    }
}

// ---- tests ---------------------------------------------------------------
static void test_eeprom_file_system( void)
{
  static uint32_t storage[256];
  memset( storage, 0xff, sizeof storage);
  permanent_data_file.set_memory_to_existing_data( storage, storage + 256);

  float calibration[6] = { 0.01f, 1.01f, -0.02f, 0.99f, 0.03f, 1.02f };
  check( permanent_data_file.store_data( ACCELEROMETER_CALIBRATION, 6, calibration) != 0, "EEPROM: store data file");
  check( permanent_data_file.store_data( BOARD_ID, (uint8_t)42), "EEPROM: store direct value");
  calibration[0] = 0.5f; // newer version of the same entry
  check( permanent_data_file.store_data( ACCELEROMETER_CALIBRATION, 6, calibration) != 0, "EEPROM: store update");

  EEPROM_file_system<LOWEST_UNUSED_EEPROM_ID> after_reboot;
  check( after_reboot.set_memory_to_existing_data( storage, storage + 256), "EEPROM: consistent after reboot");
  float read_back[6] = { 0 };
  check( after_reboot.retrieve_data( ACCELEROMETER_CALIBRATION, 6, read_back)
	 && memcmp( read_back, calibration, sizeof calibration) == 0, "EEPROM: newest data file read back");
  uint8_t board_id = 0;
  check( after_reboot.retrieve_data( BOARD_ID, board_id) && board_id == 42, "EEPROM: direct value read back");
  check( ! after_reboot.retrieve_data( MAG_SENSOR_XFER_MATRIX, 12, read_back), "EEPROM: missing entry reported");

  storage[2] ^= 0x00000001; // corrupt the first data file
  EEPROM_file_system<LOWEST_UNUSED_EEPROM_ID> corrupted;
  corrupted.set_memory_to_existing_data( storage, storage + 256);
  check( corrupted.retrieve_data( ACCELEROMETER_CALIBRATION, 6, read_back), "EEPROM: newer entry still valid after corrupting an older one");
}

static void test_log_record_headers( void)
{
  test_log_file_t log;
  uint32_t data[3] = { 1, 2, 3 };
  log_words = 0;
  log.append_record( (flexible_log_file_record_type)7, data, 3);
  check( flexible_log_file_t::verify_record_get_size( log_buffer[0]) == 3, "log: record size");
  check( flexible_log_file_t::verify_record_get_size( log_buffer[0] ^ 0x00010000) == 0, "log: corrupted CRC rejected");

  static uint32_t long_data[300]; // more than 254 words -> extended record
  log_words = 0;
  log.append_record( EEPROM_FILE, long_data, 300);
  check( flexible_log_file_t::verify_record_get_size( log_buffer[0]) == 255, "log: extended record marker");
  check( flexible_log_file_t::verify_extended_record_get_size( log_buffer[0], log_buffer[1], log_buffer[2]) == 300, "log: extended record size");
  check( log_buffer[1] == EEPROM_FILE, "log: extended record id");
}

static void test_quaternion( void)
{
  const float angles[][3] = { { 0.1f, -0.2f, 0.3f }, { -1.0f, 0.5f, 2.5f }, { 0.0f, 0.0f, -3.0f } };
  for( auto &a : angles)
    {
      quaternion<float> q;
      q.from_euler( a[0], a[1], a[2]);
      eulerangle<float> e = q;
      check( fabsf( e.roll - a[0]) < 1e-5f && fabsf( e.pitch - a[1]) < 1e-5f && fabsf( e.yaw - a[2]) < 1e-5f,
	     "quaternion: euler round trip");

      float3matrix m;
      q.get_rotation_matrix( m);
      quaternion<float> q2;
      q2.from_rotation_matrix( m);
      eulerangle<float> e2 = q2;
      check( fabsf( e2.roll - a[0]) < 1e-4f && fabsf( e2.pitch - a[1]) < 1e-4f && fabsf( e2.yaw - a[2]) < 1e-4f,
	     "quaternion: rotation matrix round trip");
    }
}

static void test_least_square_fit( void)
{
  linear_least_square_fit<float> fit;
  for( int i = 0; i < 20; ++i)
    fit.add_value( (float)i, 3.0f + 0.5f * (float)i); // y = 3 + 0.5 x
  linear_least_square_result<float> r;
  check( fit.evaluate( r), "least squares: evaluation possible");
  check( fabsf( r.y_offset - 3.0f) < 1e-4f && fabsf( r.slope - 0.5f) < 1e-5f, "least squares: exact line recovered");
}

static void test_ascii_formatting( void)
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
      check( strcmp( buffer, c.expected) == 0, "ascii: to_ascii_n_decimals");
    }
}

int main( void)
{
  test_eeprom_file_system();
  test_log_record_headers();
  test_quaternion();
  test_least_square_fit();
  test_ascii_formatting();

  printf( "%u checks, %u failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
