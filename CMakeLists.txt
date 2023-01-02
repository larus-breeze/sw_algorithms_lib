project(sw_sensor_algorithms)

set(SOURCE_FILES
    Generic_Algorithms/ascii_support.cpp
    Generic_Algorithms/serial_io.cpp
    NAV_Algorithms/AHRS.cpp
    NAV_Algorithms/air_density_observer.cpp
    NAV_Algorithms/atmosphere.cpp
    NAV_Algorithms/flight_observer.cpp
    NAV_Algorithms/KalmanVario.cpp
    NAV_Algorithms/KalmanVario_PVA.cpp
    NAV_Algorithms/navigator.cpp
    NAV_Algorithms/persistent_data.cpp
    Output_Formatter/CAN_output.cpp
    Output_Formatter/NMEA_format.cpp
)

set(HEADER_FILES
    Generic_Algorithms/ascii_support.h
    Generic_Algorithms/delay_line.h
    Generic_Algorithms/differentiator.h
    Generic_Algorithms/euler.h
    Generic_Algorithms/float3matrix.h
    Generic_Algorithms/float3vector.h
    Generic_Algorithms/HP_LP_fusion.h
    Generic_Algorithms/integrator.h
    Generic_Algorithms/Linear_Least_Square_Fit.h
    Generic_Algorithms/matrix.h
    Generic_Algorithms/pt2.h
    Generic_Algorithms/quaternion.h
    Generic_Algorithms/ringbuffer.h
    Generic_Algorithms/serial_io.h
    Generic_Algorithms/trigger.h
    Generic_Algorithms/vector.h
    NAV_Algorithms/AHRS.h
    NAV_Algorithms/air_density_observer.h
    NAV_Algorithms/atmosphere.h
    NAV_Algorithms/compass_calibration.h
    NAV_Algorithms/data_structures.h
    NAV_Algorithms/flight_observer.h
    NAV_Algorithms/GNSS.h
    NAV_Algorithms/KalmanVario.h
    NAV_Algorithms/KalmanVario_PVA.h
    NAV_Algorithms/navigator.h
    NAV_Algorithms/NAV_tuning_parameters.h
    NAV_Algorithms/organizer.h
    NAV_Algorithms/persistent_data.h
    NAV_Algorithms/soaring_flight_averager.h
    NAV_Algorithms/windobserver.h
    Output_Formatter/CAN_output.h
    Output_Formatter/generic_CAN_driver.h
    Output_Formatter/NMEA_format.h
)


include_directories(
    ../src
    Generic_Algorithms
    NAV_Algorithms
    Output_Formatter
)

add_library(larus_lib
  ${SOURCE_FILES}
  ${HEADER_FILES}
)


