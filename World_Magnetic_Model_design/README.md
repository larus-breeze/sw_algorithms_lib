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
