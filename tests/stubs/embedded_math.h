// Platform math for building the library outside the sensor firmware (CI).
// The firmware uses CMSIS-DSP functions here; the standard library is used instead.
#ifndef INC_EMBEDDED_MATH_H_
#define INC_EMBEDDED_MATH_H_

#include <math.h>
#include <stdint.h>
#include <float.h>
#include <assert.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ftype float
typedef float float32_t;

#define ZERO 0.0f
#define ONE 1.0f
#define TWO 2.0f
#define HALF 0.5f
#define QUARTER 0.25f
#define M_PI_F 3.14159265358979323846f
#define EPSILON 1e-12f

#define SQR(x) ((x)*(x))
#define SQRT(x) sqrtf(x)
#define COS(x) cosf(x)
#define SIN(x) sinf(x)
#define ASIN(x) asinf(x)
#define ATAN2(y, x) atan2f(y, x)

template <typename type> type CLIP( type x, type min, type max)
{
  return x < min ? min : x > max ? max : x;
}

#endif /* INC_EMBEDDED_MATH_H_ */
