#include "earth_induction_model.h"

ROM induction_model_area_t earth_induction_model_t::induction_model_area[N_AREAS]=
    {
	{ // europe
	    -15.0, 30.0, // left, right
	    25.0, 70,   // bottom, top
	    { // declination
		5.121784702900829e-05,
		2.624098034879970e-04,
		-9.419552924210621e-03,
		5.343918890002500e-05,
		-1.631811030378127e-02,
		4.986530342769596e-01,
		1.713583700195904e-05,
		-5.343808350939929e-03,
		4.744005367709492e-01,
		-6.919556947352653e+00
	    },
	    { // inclination
		 2.238116112917459e-04,
		 2.208613842153091e-05,
		 -4.838904340786659e-02,
		 -2.229902111583445e-05,
		 -4.774809334262235e-03,
		 4.023802474337383e+00,
		 -1.682245906466733e-05,
		 2.375176076700740e-03,
		 2.306371479408610e-01,
		 -4.296960683967160e+01
	    },
	},
	{ // africa
	    -20.0, 60.0, // left, right
	    -35.0, 40,   // bottom, top
	    { // declination
		 1.120219493186473e-04,
		 -1.225698952752873e-04,
		 -7.129961078385496e-03,
		 4.931655990117274e-05,
		 1.665170371891766e-03,
		 2.727567451221284e-01,
		 2.910501686112947e-05,
		 -7.035073356032582e-03,
		 3.204208774791076e-01,
		 -3.647158038091194e+00
	    },
	    { // inclination
		-5.433127444135837e-04,
		-1.137350438537858e-04,
		1.706516490386623e-02,
		-4.511218757889591e-05,
		2.879877325512916e-03,
		2.267568526704758e+00,
		-1.087813989819037e-04,
		8.725656228245442e-03,
		9.743230657776267e-02,
		-2.939792941166480e+01
	    },
	},
	{ // australia
	    110.0, 155.0,   // left, right
	    -40.0, -10.0,   // bottom, top
	    { // declination
		5.546403635333835e-05,
		5.314915232219207e-04,
		-6.849943856443287e-02,
		2.958100087894962e-04,
		-6.622137658688818e-02,
		3.447689949284927e+00,
		-1.756972044942521e-05,
		1.504216579312397e-02,
		-2.809034281885659e+00,
		1.496901011293250e+02
	    },
	    { // inclination
		1.515805621426717e-04,
		-6.343899008870405e-05,
		3.528378997526102e-02,
		1.696309522522108e-05,
		-9.470688732882490e-03,
		3.205317922018218e+00,
		7.111255870608101e-05,
		-2.686844631767804e-02,
		3.351232420712531e+00,
		-1.530444949539504e+02
	    },
	},
	{ // usa
	    -125.00, -70.0,   // left, right
	    25.0, 70.0,   // bottom, top
	    { // declination
		-2.224233201399186e-04,
		-2.670316634833714e-04,
		-3.074590220552326e-04,
		2.130731708230263e-05,
		1.579370819172795e-02,
		3.789060809334363e-01,
		3.173990524545548e-04,
		8.721041713129403e-02,
		7.211047791637301e+00,
		1.825984054809143e+02
	    },
	    { // inclination
		-3.849560141354113e-06,
		-2.177788276836292e-05,
		-7.379103859208931e-03,
		5.034913907563424e-05,
		1.093250782229599e-02,
		1.821453913909042e+00,
		-4.095350585248441e-05,
		-1.878072768345651e-02,
		-2.436546108993555e+00,
		-7.074741045769397e+01
	    },
	},
	{ // south america
	    -80.0, -35.0,   // left, right
	    -40.0, 0.0,   // bottom, top
	    { // declination
		-8.916044734793926e-05,
		1.908992736525847e-04,
		1.652392774122049e-02,
		1.398940528909979e-04,
		3.388131087247316e-02,
		1.583093751030807e+00,
		1.856740161715831e-04,
		4.598409447651635e-02,
		3.046289860366367e+00,
		4.088154472938876e+01
	    },
	    { // inclination
		-1.286255971474346e-04,
		2.781668984274968e-04,
		2.629059668574044e-02,
		-1.218543392258984e-04,
		-1.875065502578563e-02,
		1.252153998646948e+00,
		3.690416840339815e-04,
		5.105259785530584e-02,
		1.123874609725794e+00,
		-2.658596083317213e+01
	    },
	}
    };

induction_values earth_induction_model_t::get_induction_data_at( double longitude, double latitude)
  {
    induction_values retv={ 0.0, 0.0, false};

    // try to find parameter set for given coordinates
    for( int parameter_set = 0; parameter_set < N_AREAS; ++parameter_set)
    {
	if( latitude < induction_model_area[parameter_set].latitude__limit_south)
	  continue;
	if( latitude > induction_model_area[parameter_set].latitude__limit_north)
	  continue;
	if( longitude < induction_model_area[parameter_set].longitude_limit_west)
	  continue;
	if( longitude > induction_model_area[parameter_set].longitude_limit_east)
	  continue;

	// valid parameter set found, now evaluate induction data
	retv.declination =
	    induction_model_area[parameter_set].coefficients_declination[9] +
	    induction_model_area[parameter_set].coefficients_declination[8] * longitude +
	    induction_model_area[parameter_set].coefficients_declination[7] * longitude * longitude +
	    induction_model_area[parameter_set].coefficients_declination[6] * longitude * longitude * longitude +
	    induction_model_area[parameter_set].coefficients_declination[5] * latitude +
	    induction_model_area[parameter_set].coefficients_declination[4] * latitude * longitude +
	    induction_model_area[parameter_set].coefficients_declination[3] * latitude * longitude * longitude +
	    induction_model_area[parameter_set].coefficients_declination[2] * latitude * latitude +
	    induction_model_area[parameter_set].coefficients_declination[1] * latitude * latitude * longitude +
	    induction_model_area[parameter_set].coefficients_declination[0] * latitude * latitude * latitude;

	retv.inclination =
	    induction_model_area[parameter_set].coefficients_inclination[9] +
	    induction_model_area[parameter_set].coefficients_inclination[8] * longitude +
	    induction_model_area[parameter_set].coefficients_inclination[7] * longitude * longitude +
	    induction_model_area[parameter_set].coefficients_inclination[6] * longitude * longitude * longitude +
	    induction_model_area[parameter_set].coefficients_inclination[5] * latitude +
	    induction_model_area[parameter_set].coefficients_inclination[4] * latitude * longitude +
	    induction_model_area[parameter_set].coefficients_inclination[3] * latitude * longitude * longitude +
	    induction_model_area[parameter_set].coefficients_inclination[2] * latitude * latitude +
	    induction_model_area[parameter_set].coefficients_inclination[1] * latitude * latitude * longitude +
	    induction_model_area[parameter_set].coefficients_inclination[0] * latitude * latitude * latitude;

	 retv.valid = true;
	 return retv;
    }

    // no area found
    retv.valid = false;
    return retv;
  }

earth_induction_model_t earth_induction_model; //!< one singleton object of this type
