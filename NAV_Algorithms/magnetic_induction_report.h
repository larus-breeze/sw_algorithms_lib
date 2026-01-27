/*
 * induction_report.h
 *
 *  Created on: Mar 26, 2023
 *      Author: schaefer
 */

#ifndef MAGNETIC_INDUCTION_REPORT_H_
#define MAGNETIC_INDUCTION_REPORT_H_

#include "compass_calibrator_3D.h"
#include "float3vector.h"
#include "NAV_tuning_parameters.h"

//! helper struct containing magnetic calibration data
struct magnetic_induction_report_t
{
  compass_calibrator_3D_t * calibration;
  bool valid;
};

void report_magnetic_calibration_has_changed( magnetic_induction_report_t *p_magnetic_induction_report, char type);

#endif /* MAGNETIC_INDUCTION_REPORT_H_ */
