/***********************************************************************//**
 * @file		microphone_analysis.cpp
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
#include "microphone_analysis.h"

void mic_biquad_t::setup_bandpass (float f0, float q, float fs)
{
  float w0 = 2.0f * (float) M_PI * f0 / fs;
  float alpha = sinf (w0) / (2.0f * q);
  float a0 = 1.0f + alpha;
  b0 = alpha / a0;
  b1 = 0.0f;
  b2 = -alpha / a0;
  a1 = -2.0f * cosf (w0) / a0;
  a2 = (1.0f - alpha) / a0;
  x1 = x2 = y1 = y2 = 0.0f;
}

void mic_biquad_t::setup_lowpass (float fc, float q, float fs)
{
  float w0 = 2.0f * (float) M_PI * fc / fs;
  float alpha = sinf (w0) / (2.0f * q);
  float a0 = 1.0f + alpha;
  float c = cosf (w0);
  b0 = (1.0f - c) / 2.0f / a0;
  b1 = (1.0f - c) / a0;
  b2 = b0;
  a1 = -2.0f * c / a0;
  a2 = (1.0f - alpha) / a0;
  x1 = x2 = y1 = y2 = 0.0f;
}

void mic_third_octave_t::setup (float center, float fs)
{
  struct cplx
  {
    float re, im;
    cplx operator+ (cplx b) const { return { re + b.re, im + b.im }; }
    cplx operator- (cplx b) const { return { re - b.re, im - b.im }; }
    cplx operator* (cplx b) const { return { re * b.re - im * b.im, re * b.im + im * b.re }; }
    cplx operator/ (cplx b) const
    {
      float d = b.re * b.re + b.im * b.im;
      return { (re * b.re + im * b.im) / d, (im * b.re - re * b.im) / d };
    }
    cplx root (void) const
    {
      float r = sqrtf (hypotf (re, im)), phi = atan2f (im, re) / 2.0f;
      return { r * cosf (phi), r * sinf (phi) };
    }
  };

  // edges of the third-octave band, prewarped for the bilinear transform
  float w1 = tanf ((float) M_PI * center / powf (2.0f, 1.0f / 6.0f) / fs);
  float w2 = tanf ((float) M_PI * center * powf (2.0f, 1.0f / 6.0f) / fs);
  float w0 = sqrtf (w1 * w2);
  float bw = w2 - w1;

  // upper half-plane band-pass poles: two from the prototype pole at
  // 120 degrees, one from the real prototype pole
  cplx p_complex = { cosf (2.0f * (float) M_PI / 3.0f) * bw / 2.0f, sinf (2.0f * (float) M_PI / 3.0f) * bw / 2.0f };
  cplx p_real = { -bw / 2.0f, 0.0f };
  cplx w0_squared = { w0 * w0, 0.0f };
  cplx d = (p_complex * p_complex - w0_squared).root ();
  cplx poles[MIC_BAND_SECTIONS] = { p_complex + d, p_complex - d, p_real + (p_real * p_real - w0_squared).root () };

  // section response at the center frequency to normalize the gain
  float omega0 = 2.0f * atanf (w0);
  cplx z_inv = { cosf (omega0), -sinf (omega0) };
  cplx z_inv2 = z_inv * z_inv;
  float gain = 1.0f;
  cplx one = { 1.0f, 0.0f };

  for (unsigned k = 0; k < MIC_BAND_SECTIONS; ++k)
    {
      cplx s = poles[k];
      if (s.im < 0.0f)
	s.im = -s.im; // the conjugate belongs to the same section
      cplx z = (one + s) / (one - s);
      float c1 = -2.0f * z.re;
      float c2 = z.re * z.re + z.im * z.im;
      a1[k] = c1;
      a2[k] = c2;
      cplx denominator = one + cplx { c1, 0.0f } * z_inv + cplx { c2, 0.0f } * z_inv2;
      cplx response = (one - z_inv2) / denominator;
      gain /= hypotf (response.re, response.im);
    }
  gain_squared = gain * gain;

  for (unsigned k = 0; k <= MIC_BAND_SECTIONS; ++k)
    state[k][0] = state[k][1] = 0.0f;
}

static inline unsigned octave_of_band (unsigned band)
{
  return (MIC_BANDS - 1 - band) / MIC_BANDS_PER_OCTAVE;
}

static inline float octave_sample_rate (unsigned octave)
{
  return MIC_SAMPLE_RATE_HZ / (float) (1 << octave);
}

microphone_analysis_t::microphone_analysis_t (float ENL_center_frequency, float ENL_gain)
: ENL_gain (ENL_gain),
  record_count (0)
{
  ENL_stage[0].setup_bandpass (ENL_center_frequency, ENL_FILTER_Q, MIC_SAMPLE_RATE_HZ);
  ENL_stage[1].setup_bandpass (ENL_center_frequency, ENL_FILTER_Q, MIC_SAMPLE_RATE_HZ);

  for (unsigned i = 0; i < MIC_BANDS; ++i)
    band[i].setup (band_center_frequency (i), octave_sample_rate (octave_of_band (i)));

  // 8th order Butterworth as four sections, pole angles (2m-1) * pi / 16
  for (unsigned octave = 0; octave < MIC_OCTAVES - 1; ++octave)
    for (unsigned m = 0; m < MIC_DECIMATOR_SECTIONS; ++m)
      {
	float q = 1.0f / (2.0f * cosf ((float) (2 * m + 1) * (float) M_PI / (4.0f * MIC_DECIMATOR_SECTIONS)));
	float fs = octave_sample_rate (octave);
	decimator[octave][m].setup_lowpass (MIC_DECIMATOR_CORNER * fs, q, fs);
      }
  reset ();
}

float microphone_analysis_t::band_center_frequency (unsigned band)
{
  return 1000.0f * powf (2.0f, ((float) band - (float) MIC_BAND_1KHZ) / 3.0f);
}

void microphone_analysis_t::reset (void)
{
  settling = MIC_SETTLING_SAMPLES;
  dc_x1 = dc_y1 = 0.0f;
  for (unsigned octave = 0; octave < MIC_OCTAVES - 1; ++octave)
    decimator_phase[octave] = false;
  clear_accumulators ();
  filled_records = 0;
  next_ENL_slot = 0;
  record.ENL = ENL_UNSERVICEABLE;
  record.status = 0;
}

void microphone_analysis_t::clear_accumulators (void)
{
  count = 0;
  raw_min = INT32_MAX;
  raw_max = INT32_MIN;
  sum_broadband = sum_ENL = 0.0f;
  for (unsigned i = 0; i < MIC_BANDS; ++i)
    sum_band[i] = 0.0f;
  for (unsigned octave = 0; octave < MIC_OCTAVES; ++octave)
    octave_count[octave] = 0;
}

bool microphone_analysis_t::update (int32_t raw)
{
  float x = (float) raw * (1.0f / 2147483648.0f);

  // DC blocker, corner ~2.5 Hz: keeps the 25 Hz band unattenuated
  float hp = x - dc_x1 + 0.998f * dc_y1;
  dc_x1 = x;
  dc_y1 = hp;

  float enl = ENL_stage[1].filter (ENL_stage[0].filter (hp));
  bool accumulate = (settling == 0);

  // octave filter bank: three bands per octave, then low-pass and keep
  // every second sample for the next lower octave
  float v = hp;
  for (unsigned octave = 0; octave < MIC_OCTAVES; ++octave)
    {
      unsigned top = MIC_BANDS - 1 - octave * MIC_BANDS_PER_OCTAVE;
      for (unsigned j = 0; j < MIC_BANDS_PER_OCTAVE; ++j)
	{
	  float y = band[top - j].filter (v);
	  if (accumulate)
	    sum_band[top - j] += y * y;
	}
      if (accumulate)
	++octave_count[octave];

      if (octave == MIC_OCTAVES - 1)
	break;
      for (unsigned m = 0; m < MIC_DECIMATOR_SECTIONS; ++m)
	v = decimator[octave][m].filter (v);
      decimator_phase[octave] = ! decimator_phase[octave];
      if (decimator_phase[octave])
	break;
    }

  if ( ! accumulate)
    {
      --settling;
      return false;
    }

  if (raw < raw_min)
    raw_min = raw;
  if (raw > raw_max)
    raw_max = raw;
  sum_broadband += hp * hp;
  sum_ENL += enl * enl;

  if (++count < MIC_RECORD_SAMPLES)
    return false;

  finish_record ();
  return true;
}

// +3 dB: RMS relative to a full-scale sine, as usual for dBFS
static inline float dBFS (float sum, unsigned n)
{
  return 10.0f * log10f (2.0f * sum / (float) (n ? n : 1) + 1e-20f);
}

void microphone_analysis_t::finish_record (void)
{
  ENL_mean_square[next_ENL_slot] = sum_ENL / (float) MIC_RECORD_SAMPLES;
  next_ENL_slot = (next_ENL_slot + 1) % ENL_WINDOW_RECORDS;
  if (filled_records < ENL_WINDOW_RECORDS)
    ++filled_records;

  float mean_square = 0.0f;
  for (unsigned i = 0; i < filled_records; ++i)
    mean_square += ENL_mean_square[i];
  mean_square /= (float) filled_records;

  // a working microphone always shows self-noise
  bool no_signal = (raw_min == raw_max);

  if (no_signal)
    record.ENL = ENL_UNSERVICEABLE;
  else
    {
      float value = (float) ENL_BASELINE + ENL_gain * sqrtf (mean_square);
      record.ENL = value >= (float) ENL_MAXIMUM ? ENL_MAXIMUM : (uint32_t) value;
    }
  record.record_count = ++record_count;
  record.status = no_signal ? MIC_STATUS_NO_SIGNAL : 0;
  record.ENL_band_dBFS = dBFS (sum_ENL, MIC_RECORD_SAMPLES);
  record.broadband_dBFS = dBFS (sum_broadband, MIC_RECORD_SAMPLES);

  // |INT32_MIN| does not fit into int32_t
  float peak = fmaxf (-(float) raw_min, (float) raw_max) * (1.0f / 2147483648.0f);
  record.peak_dBFS = 20.0f * log10f (peak + 1e-10f);

  // the lower octaves see 1/2, 1/4 ... of the samples, not always an integer
  for (unsigned i = 0; i < MIC_BANDS; ++i)
    record.band_dBFS[i] = dBFS (sum_band[i] * band[i].gain_squared, octave_count[octave_of_band (i)]);

  clear_accumulators ();
}
