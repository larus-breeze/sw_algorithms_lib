# World Magnetic Model

`NAV_Algorithms/earth_induction_model.cpp` evaluates the official
[World Magnetic Model](https://www.ncei.noaa.gov/products/world-magnetic-model)
(NOAA NCEI / British Geological Survey, public domain) on the sensor: a
spherical harmonic model of degree 12, valid worldwide, with secular variation.

| File | Content |
|---|---|
| `WMM.COF` | the coefficient file as published by NOAA (currently WMM-2025, valid 2025.0–2030.0) |
| `make_wmm_coefficients.py` | converts `WMM.COF` into `NAV_Algorithms/wmm_coefficients.h` |

When a new model is published (every five years, next WMM2030 around
December 2029), replace `WMM.COF` with the new file and run, from the
repository root:

    python3 World_Magnetic_Model_design/make_wmm_coefficients.py \
        World_Magnetic_Model_design/WMM.COF > NAV_Algorithms/wmm_coefficients.h

## Source and license

The World Magnetic Model is produced by the NOAA National Centers for
Environmental Information (NCEI) and the British Geological Survey (BGS)
for the U.S. National Geospatial-Intelligence Agency and the UK Defence
Geographic Centre.

- `WMM.COF` is the unmodified coefficient file from
  <https://www.ncei.noaa.gov/products/world-magnetic-model>.
  `NAV_Algorithms/wmm_coefficients.h` contains the same coefficients,
  converted by `make_wmm_coefficients.py`.
- NOAA states: "The WMM source code is in the public domain and not
  licensed or under copyright. The information and software may be used
  freely by the public."
- Notice according to 17 U.S.C. 403: `WMM.COF` and
  `NAV_Algorithms/wmm_coefficients.h` consist of U.S. Government material
  (the WMM coefficients), which is not subject to copyright protection.
  The GPL-3.0 of this project does not apply to these coefficients.
  The code that evaluates the model (`earth_induction_model.cpp`) and the
  scripts in this directory are part of this project and licensed under
  GPL-3.0.

Citation, as requested by NOAA:

> NOAA NCEI Geomagnetic Modeling Team; British Geological Survey. 2024:
> World Magnetic Model 2025. NOAA National Centers for Environmental
> Information. https://doi.org/10.25921/aqfd-sd83

Technical report: A. Chulliat et al., 2025. The US/UK World Magnetic Model
for 2025-2030: Technical Report, NOAA NCEI. https://doi.org/10.25923/prbc-s316
