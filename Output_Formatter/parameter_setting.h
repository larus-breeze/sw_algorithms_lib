/** ***********************************************************************
 * @file		parameter_setting.h
 * @brief		interface to set parameters via CAN and NMEA
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

typedef enum
{
  MC_CREADY,
  BALLAST,
  BUGS,
  QNH,
  VARIO_MODE
}
parameter_type;

typedef struct
{
  parameter_type type;
  float32_t value;
}
parameter_setting_message;

