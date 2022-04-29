cmake_minimum_required(VERSION 2.8.3)
project(epson_imu_uart_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Specify compiler options
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Default to C99
if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_C_FLAGS "-std=gnu99 ${CMAKE_C_FLAGS}")
    endif ()
else ()
    set (CMAKE_C_STANDARD 99)
endif ()

# Specify DEBUG macro to enable any debug code by adding "-DDEBUG" in add_definitions()
#add_definitions(-DDEBUG)

# Refer to the readme.txt inside the src folder for more details
# Uncomment the desired imu_model to build
set(imu_model "G365PDF0")
#set(imu_model "G365PDF1")
#set(imu_model "G365PDC0")
#set(imu_model "G365PDC1")
#set(imu_model "G370PDF0")
#set(imu_model "G370PDF1")
#set(imu_model "G325PDF0")
#set(imu_model "G325PDF1")
#set(imu_model "G354")
#set(imu_model "G364PDCA")
#set(imu_model "G364PDC0")
#set(imu_model "G320")
#set(imu_model "V340")
add_definitions(-D${imu_model})
message("---- Building for IMU Model: ${imu_model}")

# Setting macro NATIVE_QUAT enables hardware quaternion output function in G325/G365 only 
# and has no effect for any other IMU models
# If not defined then quaternion output is emulated by converting from G325/G365 Euler output using ROS tf2 library
# NATIVE_QUAT is recommended over software conversion from euler output
add_definitions(-DNATIVE_QUAT)

# Setting macro PUB_RPY enables publishing Roll Pitch Yaw output (non-moving frame) 
# in topic /epson_imu_rpy
# The PUB_RPY macro and euler output is only supported on G325/G365 only and not supported in other IMU model
# NOTE: Euler output data is emulated by converting from G325/G365 quaternion output using ROS tf2 library
# add_definitions(-DPUB_RPY)

# Create filelist of sources for C library 
set(lib_sources 
  src/hcl_gpio.c
  src/hcl_linux.c
  src/hcl_uart.c
  src/sensor_epsonCommon.c
  src/sensor_epsonUart.c
)

# Add IMU model specific source to library 
if (imu_model STREQUAL "G365PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDC0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDC1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G370PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (imu_model STREQUAL "G370PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (imu_model STREQUAL "G325PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG325.c)
elseif (imu_model STREQUAL "G325PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG325.c)
elseif (imu_model STREQUAL "G354")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG354.c)
elseif (imu_model STREQUAL "G364PDCA")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (imu_model STREQUAL "G364PDC0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (imu_model STREQUAL "G320")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG320.c)
elseif (imu_model STREQUAL "V340")
  set(lib_sources ${lib_sources}
    src/sensor_epsonV340.c)
else()
  message([FATAL_ERROR] "**** Invalid IMU Model")
endif()

# Declare a library for Epson IMU functions from C sources
add_library(epson_imu_uart_driver_lib
  ${lib_sources}
)

# Declare a C++ executable
add_executable(epson_imu_uart_driver_node src/epson_imu_uart_driver_node.cpp)

# Link epson C library to executable target ROS node
target_link_libraries(epson_imu_uart_driver_node
 ${catkin_LIBRARIES}
  epson_imu_uart_driver_lib
)

# Debugging
#message(INFO "${catkin_LIBRARIES}")
#message(INFO "${catkin_INCLUDE_DIRS}")
#message(INFO "${wiringPi_LIB}")
#message(INFO "${lib_sources}")

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS epson_imu_uart_driver_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

