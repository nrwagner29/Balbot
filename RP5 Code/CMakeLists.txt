cmake_minimum_required(VERSION 3.5.0)
project(Balbot VERSION 0.1.0 LANGUAGES C CXX)

#main.cpp

add_executable(Balbot  Motortelemetrytest.cpp
lib/brushless_drive_client.hpp
lib/client_communication.cpp
lib/client_communication.hpp
lib/coil_temperature_estimator_client.hpp
lib/esc_propeller_input_parser_client.hpp
lib/generic_interface.hpp
lib/generic_interface.cpp
lib/iquart_flight_controller_interface_client.hpp
lib/persistent_memory_client.hpp
lib/power_monitor_client.hpp
lib/power_safety_client.hpp
lib/propeller_motor_control_client.hpp
lib/pulsing_rectangular_input_parser_client.hpp
lib/serial_interface_client.hpp
lib/system_control_client.hpp
lib/temperature_estimator_client.hpp
lib/temperature_monitor_uc_client.hpp
lib/voltage_superposition_client.hpp
lib/byte_queue.c
lib/byte_queue.h
lib/crc_helper.c
lib/crc_helper.h
lib/generic_interface.cpp
lib/packet_finder.c
lib/packet_finder.h
lib/wiringPi.h
lib/wiringPiI2C.h
lib/SerialPort.h
lib/SerialStream.h
lib/SerialPortConstants.h
lib/SerialStreamBuf.h
)

# Find the libserial library
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED libserial)

find_library(WIRINGPI_LIB wiringPi)

# Include the libserial directories
include_directories(${LibSerial_INCLUDE_DIRS})

# Link the libserial library
target_link_libraries(Balbot ${LibSerial_LIBRARIES})
target_link_libraries(Balbot ${WIRINGPI_LIB})
include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

