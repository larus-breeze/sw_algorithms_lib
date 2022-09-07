#include "atmosphere.h"
#include "embedded_memory.h"

/**
 * Parameters for the approximation method.
 *
 * @details The parameters are based on the approximation of the saturation
 * 	pressure by the Antoine equation from the book Thermodynamics (2016) by Hans
 * 	Dieter Baehr and Stephan Kabelac. This was then approximated by a Taylor
 * 	series.
 */
static ROM struct
{
	float range1Begin = 223.15f;
	float range2Begin = 273.16f;
	float rangeEnd = 333.15f;
	float s[2][6] =
	    {
		{ 24.8925f, 2.6766f, 0.1327f, 0.0040f, 8.1294e-5f, 1.1624e-6f },
		{ 3.3640e3f, 198.8920f, 5.1247f, 0.0741f, 6.364e-4f, 3.0197e-6f }
	    };
	float a[2] = { 239.15f, 299.15f };
} PWS;

/*! Routines for exponentiation */
#define POT_2(x)		((x)*(x))
#define POT_3(x)		((x)*(x)*(x))
#define POT_4(x)		((x)*(x)*(x)*(x))
#define POT_5(x)		((x)*(x)*(x)*(x)*(x))

/**
 * Calculate the saturation vapor pressure as a function of temperature
 *
 * @param temperature Temperature in degree celsius.
 *
 * @return the saturation vapor pressure in Pa.
 */
float atmosphere_t::calculateSaturationVaporPressure(float temp)
{
	if (PWS.range1Begin <= temp && temp < PWS.range2Begin)
	  {
		return (PWS.s[0][0] + PWS.s[0][1] * ((temp - PWS.a[0]))
				+ PWS.s[0][2] * POT_2((temp - PWS.a[0]))
				+ PWS.s[0][3] * POT_3((temp - PWS.a[0]))
				+ PWS.s[0][4] * POT_4((temp - PWS.a[0]))
				+ PWS.s[0][5] * POT_5((temp - PWS.a[0])));
	  }
	else if (PWS.range2Begin <= temp && temp <= PWS.rangeEnd)
	  {
		return (PWS.s[1][0] + PWS.s[1][1] * ((temp - PWS.a[1]))
				+ PWS.s[1][2] * POT_2((temp - PWS.a[1]))
				+ PWS.s[1][3] * POT_3((temp - PWS.a[1]))
				+ PWS.s[1][4] * POT_4((temp - PWS.a[1]))
				+ PWS.s[1][5] * POT_5((temp - PWS.a[1])));
	  }
	return 0.0f;
}

/**
 * Calculate the gas constant of humid air as a function of temperature,
 * pressure and relative humidity.
 *
 * @param humidity Humidity in relative humidity (0..1).
 *
 * @param pressure Pressure in Pascal.
 *
 * @param temperature Temperature in degree celsius.
 *
 * @return the gas constant of humid air in J/Kg/K.
 */
float atmosphere_t::calculateGasConstantHumAir(
    float humidity, float pressure, float temperature)
{
	float satVapPressure = calculateSaturationVaporPressure(temperature);
	float var1 =
	    humidity * satVapPressure / pressure * ONE_MINUS_RATIO_GAS_CONSTANTS;

	return (GAS_CONST_DRY_AIR / (1 - var1));
}

/**
 * Calculate the density of air as a function of temperature, pressure and
 * relative humidity.
 *
 * @param humidity Humidity in % relative humidity x1000.
 *
 * @param pressure Pressure in Pascal.
 *
 * @param temperature Temperature in degree celsius x100.
 *
 * @return the air density in kg/m^3.
 */
float atmosphere_t::calculateAirDensity(
    float humidity, float pressure, float temperature)
{
	float abs_temp = (CELSIUS_TO_KELVIN_OFFSET + temperature);
	float gasConst = calculateGasConstantHumAir(humidity, pressure, abs_temp);
	return pressure / gasConst / temperature;
}
