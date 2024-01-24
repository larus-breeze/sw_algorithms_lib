/*
 * induction_report.h
 *
 *  Created on: Mar 26, 2023
 *      Author: schaefer
 */

#ifndef MAGNETIC_INDUCTION_REPORT_H_
#define MAGNETIC_INDUCTION_REPORT_H_

#include "compass_calibration.h"
#include "float3vector.h"

struct magnetic_induction_report_t
{
  single_axis_calibration_t calibration[3];
  float3vector nav_induction;
  float nav_induction_std_deviation;
};

void report_magnetic_calibration_has_changed( magnetic_induction_report_t *p_magnetic_induction_report, char type);

#endif /* MAGNETIC_INDUCTION_REPORT_H_ */
