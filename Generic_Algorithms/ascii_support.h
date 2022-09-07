/** ***********************************************************************
 * @file		ascii_support.h
 * @brief		Simple and fast ASCII converters
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef ASCII_SUPPORT_H_
#define ASCII_SUPPORT_H_

#include <stdint.h>

char * my_itoa( char * target, int value);
char * my_ftoa( char * target, float value);

#ifdef __cplusplus

char * utox(uint32_t value, char* result, uint8_t nibbles = 8);
char * lutox(uint64_t value, char* result);

extern "C"
 {
#endif /* __cplusplus */

float string2float(char *input);
char* ftoa(float Value, char* Buffer);
char* itoa(int value, char* result, int base=10);

inline char * format_2_digits( char * target, uint32_t data)
{
  data %= 100;
  *target++ = (char)(data / 10 + '0');
  *target++ = (char)(data % 10 + '0');
  *target = 0; // just be sure string is terminated
  return target;
}

inline char *append_string( char *target, const char *source)
{
  while( *source)
      *target++ = *source++;
  *target = 0; // just to be sure :-)
  return target;
}

#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif /* ASCII_SUPPORT_H_ */
