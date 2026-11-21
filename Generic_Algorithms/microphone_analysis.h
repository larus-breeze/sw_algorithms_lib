/***********************************************************************//**
 * @file		microphone_analysis.h
 * @brief		IGC Environmental Noise Level (ENL) and band levels
 *			from the microphone sample stream
 * @license 		This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/
#ifndef MICROPHONE_ANALYSIS_H_
#define MICROPHONE_ANALYSIS_H_

#include <stdint.h>
#include <math.h>

// Signal chain, record layout and the IGC requirements behind the numbers:
// sw_sensor documentation/microphone_ENL.md

#define MIC_SAMPLE_RATE_HZ		8000.0f
#define MIC_RECORD_SAMPLES		2000	// one record every 250 ms
#define MIC_SETTLING_SAMPLES		4000	// microphone start-up and filter transients

#define ENL_CENTER_FREQUENCY_HZ		150.0f	// IGC: peak sensitivity 100..200 Hz
#define ENL_FILTER_Q			1.2f	// per stage, two identical stages
#define ENL_WINDOW_RECORDS		4	// ENL = RMS over the last 1 s
#define ENL_BASELINE			10	// IGC: "010" = serviceable, no noise
#define ENL_MAXIMUM			999
#define ENL_UNSERVICEABLE		0	// IGC: "000" = system not working
// Uncalibrated default: ENL 900 at an in-band level of -26 dBFS (RMS
// 0.03544), i.e. 94 dB SPL with the ICS-43434's sensitivity.
#define ENL_DEFAULT_GAIN		(890.0f / 0.03544f)

// third-octave bands, base-2 centers 1000 Hz * 2^(k/3): 24.8 Hz .. 2520 Hz
// (nominal 25 .. 2500 Hz), the highest band edge stays below fs/2
#define MIC_BANDS			21
#define MIC_BAND_1KHZ			16	// index of the 1 kHz band
#define MIC_BAND_SECTIONS		3	// 6th order Butterworth band-pass
#define MIC_BANDS_PER_OCTAVE		3
#define MIC_OCTAVES			(MIC_BANDS / MIC_BANDS_PER_OCTAVE)
#define MIC_DECIMATOR_SECTIONS		4	// 8th order Butterworth low-pass
#define MIC_DECIMATOR_CORNER		0.205f	// relative to the octave's input rate

#define MIC_STATUS_NO_SIGNAL		0x0001	// constant samples: microphone missing or stuck
#define MIC_STATUS_LOST_BLOCKS_SHIFT	16	// DMA blocks not processed in time (set by driver)

//! LRSX record MICROPHONE_DATA, 27 words. Levels in dBFS: 0 dBFS = sine
//! with full-scale peak.
typedef struct
{
  uint32_t record_count;		//!< +1 per record, shows gaps
  uint32_t ENL;				//!< 0..999, 1 s window
  uint32_t status;			//!< MIC_STATUS_xxx
  float ENL_band_dBFS;			//!< ENL filter output, 250 ms RMS
  float broadband_dBFS;			//!< DC removed, 250 ms RMS
  float peak_dBFS;			//!< largest raw sample in 250 ms, 0 = clipping
  float band_dBFS[MIC_BANDS];		//!< third-octave bands, 250 ms RMS
} microphone_record_t;

//! Direct form I biquad: robust in float for the low f/fs ratios used here
class mic_biquad_t
{
public:
  void setup_bandpass (float f0, float q, float fs); //!< RBJ, 0 dB peak gain
  void setup_lowpass (float fc, float q, float fs);  //!< RBJ
  float filter (float x)
  {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
  }
private:
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
};

//! Third-octave band-pass: 3rd order Butterworth low-pass prototype,
//! low-pass to band-pass transform, bilinear transform (band edges
//! prewarped). Every section has the numerator 1 - z^-2; the overall gain
//! is applied to the mean square only, saving a multiplication per sample.
//! Cascaded direct form I sections share their state between neighbours.
class mic_third_octave_t
{
public:
  void setup (float center, float fs);

  //! @return unscaled output, multiply its mean square by gain_squared
  float filter (float x)
  {
    float v = x;
    for (unsigned k = 0; k < MIC_BAND_SECTIONS; ++k)
      {
	float y = v - state[k][1] - a1[k] * state[k + 1][0] - a2[k] * state[k + 1][1];
	state[k][1] = state[k][0];
	state[k][0] = v;
	v = y;
      }
    state[MIC_BAND_SECTIONS][1] = state[MIC_BAND_SECTIONS][0];
    state[MIC_BAND_SECTIONS][0] = v;
    return v;
  }

  float gain_squared;

private:
  float a1[MIC_BAND_SECTIONS], a2[MIC_BAND_SECTIONS];
  float state[MIC_BAND_SECTIONS + 1][2]; // node k: last two values
};

//! Implementation in microphone_analysis.cpp: compiled optimized even
//! where the including code is not (sw_sensor builds Communication/ -O0)
class microphone_analysis_t
{
public:
  microphone_analysis_t (float ENL_center_frequency = ENL_CENTER_FREQUENCY_HZ,
			 float ENL_gain = ENL_DEFAULT_GAIN);

  //! nominal centers 1000 Hz * 2^(k/3)
  static float band_center_frequency (unsigned band);

  //! restart after a gap in the sample stream, record_count keeps counting
  void reset (void);

  //! @param raw left-justified 32-bit sample, full scale = 2^31
  //! @return true when a new record is available
  bool update (int32_t raw);

  const microphone_record_t & get_record (void) const	{ return record; }
  uint16_t get_ENL (void) const				{ return (uint16_t) record.ENL; }

private:
  void clear_accumulators (void);
  void finish_record (void);

  // Octave o (0 = highest) runs at MIC_SAMPLE_RATE_HZ / 2^o: each
  // decimator halves the rate for the next lower octave.
  mic_biquad_t ENL_stage[2];
  mic_third_octave_t band[MIC_BANDS];
  mic_biquad_t decimator[MIC_OCTAVES - 1][MIC_DECIMATOR_SECTIONS];
  bool decimator_phase[MIC_OCTAVES - 1];
  float ENL_gain;
  uint32_t record_count;
  unsigned settling;
  float dc_x1, dc_y1;
  unsigned count;
  unsigned octave_count[MIC_OCTAVES];
  int32_t raw_min, raw_max;
  float sum_broadband, sum_ENL, sum_band[MIC_BANDS];
  float ENL_mean_square[ENL_WINDOW_RECORDS];
  unsigned filled_records;
  unsigned next_ENL_slot;
  microphone_record_t record;
};

#endif /* MICROPHONE_ANALYSIS_H_ */
