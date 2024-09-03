# Sensor algorithms
This module contains the algorithms for the Larus soaring flight sensor.

The algorithms are used as submodules for the Larus ARM Cortex M4 data acquisition system 
and for the [LARUS Software-In-The-Loop simulator - Sensor Data Analyzer](https://github.com/larus-breeze/sw_tools)

The algorithms include:

- A quaternion-based Attitude and Heading Reference System (**AHRS**)
- **Real-time wind-measurement** with 10Hz sampling-rate
- A **Kalman-filter** fusioning altitude, vertical speed and vertical acceleration for an **ultra-fast variometer**
- A **GNSS / INS-based speed-compensation** for the variometer
- A **D-GNSS-based satellite-compass** with sub-degree accuracy (optional)
- A self-calibrating **3d magnetic compass** using a worldwide NOAA magnetic induction model
- **Air-density measurement** by observing pressure over GNSS-altitude 
- **NMEA-stream output** for flight-management systems like [XCsoar](https://github.com/XCSoar/)
- Output of application-specific data to a **CAN-bus** to feed cockpit-instruments 

The sensor output as a set of NMEA sentences ist defined in our subproject [larus-NMEA-protocol](https://github.com/larus-breeze/doc_larus/blob/master/documentation/Larus_NMEA_Protocol.md)

To get an idea about the software structure and the algorithms used you may want to browse the [Doxygen-generated documentation of the library.](https://schaefer.eit.h-da.de/Larus_SIL)

# This library is designed to be imported into another project via a .gitmodules file.

Add as submodule to repository:

     git submodule add git@github.com:larus-breeze/sw_sensor_algorithms.git lib

Init and Update submodule

     git submodule init 
     git submodule update

Open submodule folder "lib" and run "git pull" to update the submodule to the latest commit.

Clone repository including the submodules: 

      git clone --recursive URL git://github.com/foo/example.git

Update submodule from using repository

Just cd into the submodules folder and use git commit, push etc.

# Data formats
The libary works with binary data formats. There is a description here: [Data Formats](https://github.com/larus-breeze/sw_tools/blob/master/analysis/dataformats.py)
