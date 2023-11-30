** This repo is deprecated and read-only **

** Please use ess_imu_driver instead **

# README for Epson IMU Driver using UART interface for ROS

<!---toc start-->

- [README for Epson IMU Driver using UART interface for ROS](#readme-for-epson-imu-driver-using-uart-interface-for-ros)
  - [What is this repository for?](#what-is-this-repository-for)
  - [What kind of hardware or software will I likely need?](#what-kind-of-hardware-or-software-will-i-likely-need)
  - [How do I use the driver?](#how-do-i-use-the-driver)
  - [How do I use the driver if usleep() is not supported for time delays?](#how-do-i-use-the-driver-if-usleep-is-not-supported-for-time-delays)
  - [How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?](#how-do-i-use-the-driver-with-gpios-to-control-imu-reset-drdy-ext-pins)
  - [How do I build, install, run this ROS1 package?](#how-do-i-build-install-run-this-ros1-package)
    - [Example console output of *catkin_make* build for G366PDG0:](#example-console-output-of-catkin_make-build-for-g366pdg0)
    - [Example console output of launching ROS1 node for G366PDG0:](#example-console-output-of-launching-ros1-node-for-g366pdg0)
  - [What does this ROS1 IMU Node Publish as messages?](#what-does-this-ros1-imu-node-publish-as-messages)
    - [Without Quaternion Output](#without-quaternion-output)
      - [ROS Topic Message *data_raw*](#ros-topic-message-data_raw)
    - [With Quaternion Output](#with-quaternion-output)
      - [ROS Topic Message data](#ros-topic-message-data)
  - [Why am I seeing high latencies or slower than expected IMU data rates when using USB-UART bridges?](#why-am-i-seeing-high-latencies-or-slower-than-expected-imu-data-rates-when-using-usb-uart-bridges)
    - [Modifying *latency_timer* by sysfs mechanism](#modifying-latency_timer-by-sysfs-mechanism)
    - [Modifying *low_latency* flag using *setserial* utility](#modifying-low_latency-flag-using-setserial-utility)
  - [Package Contents](#package-contents)
  - [License](#license)
  - [References](#references)

<!---toc end-->

## What is this repository for?

- This code provides interface between Epson IMU and ROS using the UART interface.
- The UART connection can be either direct or by USB-serial converter such as FTDI bridge ICs.
- The src/epson_imu_uart_driver_node.cpp is the ROS C++ wrapper used to communicate with ROS
- The other source files in src/ are based on the Linux C driver originally released by Epson:
  [Epson IMU UART-only Linux User-space Driver Example](https://vdc.epson.com/imu-products/imu-inertial-measurement-units)
- Information about ROS, ROS packages, and tutorials can be found: [ROS.org](https://www.ros.org)

## What kind of hardware or software will I likely need?

This will likely require the following:

- Epson USB evaluation board or equivalent FTDI USB-Serial interface to connect to ROS host (tty/serial) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
- Epson IMU (G320/G330/G354/G364/G365/G366/G370/V340) [IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
- ROS Indigo or later (via download) [ROS.org](https://www.ros.org)

## How do I use the driver?

- This code assumes that the user is familiar with building ROS packages using the catkin build process.
- **NOTE:** This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS driver.
- Please refer to the ROS.org website for more detailed instructions on configuring the ROS environment & the ROS package build process. [ROS.org](https://wiki.ros.org/ROS/Tutorials)
- **NOTE:** At bare minimum, you must modify the *CMakeLists.txt*, or any time you change to a different IMU model, then build this package running "catkin_make".
- If the IMU model is unchanged, then subsequent changes to IMU settings can be done by instead editing the IMU model specific .launch file located in the launch/ folder.
- The IMU model specific launch file should only be used in conjunction with the same catkin-built executable of the same matching the IMU model.
- **NOTE:** Do not just switch .launch files without modifying the CMakeList.txt & rebuilding the executable to match the IMU model. Do not mix IMU model launch files without the matching IMU model catkin built binaries.

## How do I use the driver if usleep() is not supported for time delays?

- **NOTE:** In the hcl_linux.c, there are empty functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
- On embedded Linux platforms, the user may need modify and redirect to platform specific delay routines if usleep() is not supported.
- For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
- If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

- Because this driver connects to the IMU using the UART interface, the use of GPIO pins for connecting to the IMU RESET#, EXT, or DRDY is purely optional, and mainly intended for embedded Linux platforms (non-PC based).
- When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization, for better robustness.
- Although this code does not implement GPIO functions, this code is structured for the user to easily redirect GPIO control to low-level hardware GPIO function calls for ease of implementation.
- There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
  src/hcl_linux.c
  src/hcl_gpio.c
  src/hcl_gpio.h
```

- Typically, an external library needs to be invoked to initialize & enable GPIO HW functions on the user's embedded platform.

- This typically requires changes to hcl_linux.c

  - add #include to external library near the top of hcl_linux.c
  - add the initialization call inside the seInit() function in hcl_linux.c

For example on an Raspberry Pi, the following changes can be made to hcl_linux.c:

```
...

  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }

...
```

- Typically, the GPIO pins need to be assigned according to pin numbering specific to the embedded HW platform.
- This typically requires changes to hcl_gpio.h

For example on an Raspberry Pi, the following changes to hcl_gpio.h with the following pin mapping:

```
    Epson IMU                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
```

```
...

  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);

  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);

  #define RPI_GPIO_P1_15              22                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added

  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
...
```

- Typically, the external library will have GPIO pin control functions such as set_output(), set_input(), set(), reset(), read_pin_level(), etc...

- This requires changes to hcl_gpio.c

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.
  - For example on an Raspberry Pi, the following changes to hcl_gpio.c:

```
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library

...

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }

...

  int gpioRelease(void)
  {
    return OK;
  }

...

  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  ...

  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  ...

  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  ...
```

## How do I build, install, run this ROS1 package?

The Epson IMU ROS1 driver is designed for building in the standard ROS catkin build environment.
Therefore, a functional catkin workspace in ROS1 is a prerequisite.
Refer to the ROS1 Tutorials for more info: [ROS1 Tutorial](https://wiki.ros.org/ROS/Tutorials)

For more information on ROS & catkin setup refer to:
[Installing and Configuring ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

1. Place this package (including folders) into a new folder within your catkin workspace "src" folder.
   For example, we recommend using the folder name "ess_imu_ros1_uart_driver"

```
   <catkin_workspace>/src/ess_imu_ros1_uart_driver/
```

2. Modify the *CMakeLists.txt* to select the desired Epson IMU model that is attached to the ROS system.
   Refer to the comment lines inside the *CMakeLists.txt* for additional info.
   **NOTE:** You *MUST* re-build using *catkin_make* when changing IMU models or after any changes in the *CMakeLists.txt*

3. From the catkin workspace folder run *catkin_make* to build all ROS1 packages located in the \<catkin_workspace>/src/ folder.
   Re-run the above *catkin_make* command to rebuild the driver after making any changes to the *CMakeLists.txt*, any of the .c or .cpp or .h source files.

```
   <catkin_workspace>/catkin_make
```

**NOTE:** It is not necessary to re-run *catkin_make* if changes are only made to the launch files.

**NOTE:** It is recommended to change IMU settings by editing the parameters in the launch file, wherever possible, instead of modifying the .c or .cpp source files directly

4. Reload the current ROS environment variables that may have changed after the catkin build process.

```
   From the <catkin_workspace>: source devel/setup.bash
```

5. Modify the appropriate launch file for the IMU model in the launch/ folder to set your desired IMU configure parameter options at runtime:

| Launch Parameter | Comment                                                                                                                                                                       |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ext_sel          | specifies the function of the GPIO2 pin (GPIO2, External Trigger, External Counter Reset). **NOTE:** Must be set to External Counter Reset when time_correction is enabled.   |
| ext_pol          | specifies the polarity of the GPIO2 pin when *ext_sel* is External Trigger or External Counter Reset                                                                          |
| drdy_on          | specifies to enable DRDY output function on GPIO1                                                                                                                             |
| drdy_pol         | specifies the polarity of the DRDY output pin when enabled                                                                                                                    |
| dout_rate        | specifies the IMU output data rate                                                                                                                                            |
| filter_sel       | specifies the IMU filter setting                                                                                                                                              |
| flag_out         | specifies to enable or disable ND_FLAG status in IMU output data (not used by ROS)                                                                                            |
| temp_out         | specifies to enable or disable TempC sensor in IMU output data (not used by ROS)                                                                                              |
| gyro_out         | specifies to enable or disable Gyro sensor in IMU output data (**must be enabled**)                                                                                           |
| accel_out        | specifies to enable or disable Accl sensor in IMU output data (**must be enabled**)                                                                                           |
| gyro_delta_out   | specifies to enable or disable DeltaAngle in IMU output data (not used by ROS)                                                                                                |
| accel_delta_out  | specifies to enable or disable DeltaVelocity in IMU output data (not used by ROS)                                                                                             |
| qtn_out          | specifies to enable or disable Quaternion in IMU output data (**only supported for G330/G365/G366**)                                                                          |
| atti_out         | specifies to enable or disable Attitude in IMU output data (not used by ROS)                                                                                                  |
| gpio_out         | specifies to enable or disable GPIO in IMU output data (not used by ROS)                                                                                                      |
| count_out        | specifies to enable or disable counter in IMU output data (**must be enabled when time_correction is enabled**)                                                               |
| checksum_out     | specifies to enable or disable checksum in IMU output data (when enabled checksum errors are detected)                                                                        |
| temp_bit         | specifies to 16 or 32 bit resolution in TempC output data (not used by ROS)                                                                                                   |
| gyro_bit         | specifies to 16 or 32 bit resolution in Gyro output data (**recommended to be enabled**)                                                                                      |
| accel_bit        | specifies to 16 or 32 bit resolution in Accl output data (**recommended to be enabled**)                                                                                      |
| gyro_delta_bit   | specifies to 16 or 32 bit resolution in DeltaAngle output data (not used by ROS)                                                                                              |
| accel_delta_bit  | specifies to 16 or 32 bit resolution in DeltaVelocity output data (not used by ROS)                                                                                           |
| qtn_bit          | specifies to 16 or 32 bit resolution in Quaternion output data (**only supported for G330/G365/G366**)                                                                        |
| invert_xgyro     | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_ygyro     | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_zgyro     | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_xaccel    | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_yaccel    | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_zaccel    | specifies to reverse polarity of this sensor axis                                                                                                                             |
| atti_mode        | specifies the attitude mode as 0=inclination or 1=euler (**only supported for G330/G365/G366**)                                                                               |
| atti_profile     | specifies the attitude motion profile (**only supported for G330/G365/G366**)                                                                                                 |
| time_correction  | enables time correction function using External Counter Reset function & external 1PPS connected to IMU GPIO2/EXT pin. **NOTE:** Must have ext_sel=1 (External Counter Reset) |

**NOTE:** The ROS1 launch file passes IMU configuration settings to the IMU at runtime.
Therefore, rebuilding with *catkin_make* when changing the launch file is not required

6. To start the Epson IMU ROS1 driver use the appropriate launch file (located in launch/) from console. All parameters are described in the inline comments of the launch file. The launch file contains parameters for configuring IMU settings at runtime.

   For example, for the Epson G366PDG0 IMU:

```
   <catkin_workspace>/roslaunch ess_imu_ros1_uart_driver epson_g330_g365_g366.launch
```

| Launch File                     | Description                                                                                                                  |
| ------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| epson_g320_g354_g364.launch     | For G320PDG0/G354PDH0/G364DC0/G364PDCA, outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)   |
| epson_g330_g365_g366.launch     | For G330PDG0/G365PDx1/G366PDG0, outputs to ROS topic imu/data (**gyro, accel data, including quaternion orientation**)       |
| epson_g330_g365_g366_raw.launch | For G330PDG0/G365PDx1/G366PDG0, outputs to ROS topic imu/data_raw (**gyro, accel data, but no quaternion orientation**)      |
| epson_g370.launch               | For G370PDF1/G370PDS0/G370PDG0/G370PDT0 , outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**) |
| epson_v340.launch               | For V340PDD0, outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)                             |

### Example console output of *catkin_make* build for G366PDG0:

```
user@user-VirtualBox:~/catkin_ws$ catkin_make
Base path: /home/guest/catkin_ws
Source space: /home/guest/catkin_ws/src
Build space: /home/guest/catkin_ws/build
Devel space: /home/guest/catkin_ws/devel
Install space: /home/guest/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/guest/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/guest/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/guest/catkin_ws/devel;/opt/ros/noetic
-- This workspace overlays: /home/guest/catkin_ws/devel;/opt/ros/noetic
-- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3")
-- Using PYTHON_EXECUTABLE: /usr/bin/python3
-- Using Debian Python package layout
-- Using empy: /usr/lib/python3/dist-packages/em.py
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/guest/catkin_ws/build/test_results
-- Forcing gtest/gmock from source, though one was otherwise available.
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python3 (found version "3.8.10")
-- Using Python nosetests: /usr/bin/nosetests3
-- catkin 0.8.10
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - ess_imu_ros1_uart_driver
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'ess_imu_ros1_uart_driver'
-- ==> add_subdirectory(ess_imu_ros1_uart_driver)
-- Building for IMU Model: G366PDG0
-- Configuring done
-- Generating done
-- Build files have been written to: /home/guest/catkin_ws/build
####
#### Running command: "make -j1 -l1" in "/home/guest/catkin_ws/build"
####
Scanning dependencies of target ess_imu_ros1_uart_driver_lib
[ 11%] Building C object ess_imu_ros1_uart_driver/CMakeFiles/ess_imu_ros1_uart_driver_lib.dir/src/hcl_gpio.c.o
[ 22%] Building C object ess_imu_ros1_uart_driver/CMakeFiles/ess_imu_ros1_uart_driver_lib.dir/src/hcl_linux.c.o
[ 33%] Building C object ess_imu_ros1_uart_driver/CMakeFiles/ess_imu_ros1_uart_driver_lib.dir/src/hcl_uart.c.o
[ 44%] Building C object ess_imu_ros1_uart_driver/CMakeFiles/ess_imu_ros1_uart_driver_lib.dir/src/sensor_epsonCommon.c.o
[ 55%] Building C object ess_imu_ros1_uart_driver/CMakeFiles/ess_imu_ros1_uart_driver_lib.dir/src/sensor_epsonUart.c.o
[ 66%] Building C object ess_imu_ros1_uart_driver/CMakeFiles/ess_imu_ros1_uart_driver_lib.dir/src/sensor_epsonG330_G366.c.o
[ 77%] Linking C shared library /home/guest/catkin_ws/devel/lib/libess_imu_ros1_uart_driver_lib.so
[ 77%] Built target ess_imu_ros1_uart_driver_lib
Scanning dependencies of target ess_imu_ros1_uart_driver_node
[ 88%] Building CXX object ess_imu_ros1_uart_driver/CMakeFiles/ess_imu_ros1_uart_driver_node.dir/src/epson_imu_uart_driver_node.cpp.o
[100%] Linking CXX executable /home/guest/catkin_ws/devel/lib/ess_imu_ros1_uart_driver/ess_imu_ros1_uart_driver_node
[100%] Built target ess_imu_ros1_uart_driver_node

```

### Example console output of launching ROS1 node for G366PDG0:

```
guest@guest-VirtualBox:~/catkin_ws$ roslaunch ess_imu_ros1_uart_driver epson_g330_g365_g366.launch
... logging to /home/guest/.ros/log/22ed5260-ef68-11ed-83dc-970fe52b86b9/roslaunch-guest-VirtualBox-6915.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://guest-VirtualBox:35703/

SUMMARY
========

PARAMETERS
 * /ess_imu_ros1_uart_driver_node/accel_bit: 1
 * /ess_imu_ros1_uart_driver_node/accel_delta_bit: 1
 * /ess_imu_ros1_uart_driver_node/accel_delta_out: 0
 * /ess_imu_ros1_uart_driver_node/accel_out: 1
 * /ess_imu_ros1_uart_driver_node/atti_bit: 1
 * /ess_imu_ros1_uart_driver_node/atti_conv: 0
 * /ess_imu_ros1_uart_driver_node/atti_mode: 1
 * /ess_imu_ros1_uart_driver_node/atti_out: 0
 * /ess_imu_ros1_uart_driver_node/atti_profile: 0
 * /ess_imu_ros1_uart_driver_node/checksum_out: 1
 * /ess_imu_ros1_uart_driver_node/count_out: 1
 * /ess_imu_ros1_uart_driver_node/dout_rate: 4
 * /ess_imu_ros1_uart_driver_node/drdy_on: 0
 * /ess_imu_ros1_uart_driver_node/drdy_pol: 0
 * /ess_imu_ros1_uart_driver_node/ext_pol: 0
 * /ess_imu_ros1_uart_driver_node/ext_sel: 0
 * /ess_imu_ros1_uart_driver_node/filter_sel: 5
 * /ess_imu_ros1_uart_driver_node/flag_out: 1
 * /ess_imu_ros1_uart_driver_node/gpio_out: 0
 * /ess_imu_ros1_uart_driver_node/gyro_bit: 1
 * /ess_imu_ros1_uart_driver_node/gyro_delta_bit: 1
 * /ess_imu_ros1_uart_driver_node/gyro_delta_out: 0
 * /ess_imu_ros1_uart_driver_node/gyro_out: 1
 * /ess_imu_ros1_uart_driver_node/invert_xaccel: 0
 * /ess_imu_ros1_uart_driver_node/invert_xgyro: 0
 * /ess_imu_ros1_uart_driver_node/invert_yaccel: 0
 * /ess_imu_ros1_uart_driver_node/invert_ygyro: 0
 * /ess_imu_ros1_uart_driver_node/invert_zaccel: 0
 * /ess_imu_ros1_uart_driver_node/invert_zgyro: 0
 * /ess_imu_ros1_uart_driver_node/port: /dev/ttyUSB0
 * /ess_imu_ros1_uart_driver_node/qtn_bit: 1
 * /ess_imu_ros1_uart_driver_node/qtn_out: 1
 * /ess_imu_ros1_uart_driver_node/temp_bit: 1
 * /ess_imu_ros1_uart_driver_node/temp_out: 1
 * /ess_imu_ros1_uart_driver_node/time_correction: 0
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
  /
    ess_imu_ros1_uart_driver_node (ess_imu_ros1_uart_driver/ess_imu_ros1_uart_driver_node)

auto-starting new master
process[master]: started with pid [6934]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 22ed5260-ef68-11ed-83dc-970fe52b86b9
process[rosout-1]: started with pid [6944]
started core service [/rosout]
process[ess_imu_ros1_uart_driver_node-2]: started with pid [6951]
[ INFO] [1683746607.165191043]: Initializing HCL layer...
[ INFO] [1683746607.166467878]: Initializing GPIO interface...
[ INFO] [1683746607.166573693]: Initializing UART interface...
Attempting to open port.../dev/ttyUSB0

...sensorDummyWrite.[ INFO] [1683746607.418517188]: Checking sensor NOT_READY status...
...done.[ INFO] [1683746608.276623939]: Initializing Sensor...
[ INFO] [1683746608.308601926]: Epson IMU initialized.
[ INFO] [1683746608.308927341]: Compiled for:	G366PDG0
[ INFO] [1683746608.309360891]: Reading device info...
[ INFO] [1683746608.375192888]: PRODUCT ID:	G366PDG0
[ INFO] [1683746608.436259989]: SERIAL ID:	T0000008
[ INFO] [1683746608.500317431]: OK: Build matches detected device

...Sensor start.[ INFO] [1683746608.502846688]: Quaternion Output: Native.
```

## What does this ROS1 IMU Node Publish as messages?

The Epson IMU ROS1 driver will publish IMU messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

- For IMU models such as G320/G354/G364/G370/V340, the IMU messages will only update the fields for angular rate (gyro) and linear acceleration (accel) data.
- For IMU models G330/G365/G366, it depends on the enabling/disabling of the internal attitude function with quaternion output:
  - IMU messages will only update *angular_velocity* (gyro) and *linear_acceleration* (accel) fields when quaternion output is *disabled*
  - IMU messages will update update *angular_velocity* (gyro), *linear_acceleration* (accel), and *orientation* field using the internal extended Kalman Filter when the quaternion output is *enabled*

### Without Quaternion Output

For non-quaternion output models, the ROS1 driver will publish to the following ROS topic:

```
/epson_imu/data_raw <-- orientation field will not contain valid data & should be ignored
```

**NOTE** The launch file will remap the ROS message to publish on /imu/data_raw

#### ROS Topic Message *data_raw*

```
---
header:
  seq: 34983
  stamp:
    secs: 1601590579
    nsecs: 771273325
  frame_id: "imu_link"
orientation:
  x: 2.4786295434e+33
  y: 1.1713334935e+38
  z: 1.17130631507e+38
  w: 1.17130956026e+38
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.00435450254008
  y: 0.000734272529371
  z: -9.40820464166e-05
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.730921983719
  y: -1.54766368866
  z: 9.72711181641
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```

### With Quaternion Output

- For quaternion output models, the ROS1 driver will publish to the following ROS topic:

```
/epson_imu/data <-- orientation, angular_velocity, linear_acceleration fields will be updating
```

**NOTE** The launch file will remap the ROS message to publish on /imu/data

#### ROS Topic Message data

```
---
header:
  seq: 10608
  stamp:
    secs: 1601575987
    nsecs: 673387357
  frame_id: "imu_link"
orientation:
  x: -0.0801454782486
  y: 0.0367396138608
  z: 0.00587898213416
  w: 0.996088504791
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: -0.00118702522013
  y: 0.000320095219649
  z: -0.00014466587163
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.727666378021
  y: -1.5646469593
  z: 9.69056034088
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

## Why am I seeing high latencies or slower than expected IMU data rates when using USB-UART bridges?

- If your connection between the Epson IMU UART and the Linux host is by FTDI (or similar USB-UART bridge devices,
  the default *latency_timer* setting in the FTDI driver may be too large (i.e. typically 16msec).
- There are 2 recommended methods to reduce this value to 1msec.

### Modifying *latency_timer* by sysfs mechanism

The example below reads the *latency_timer* setting for /dev/ttyUSB0 which returns 16msec.
Then, the *latency_timer* is set to 1msec, and confirmed by readback.

**NOTE**: May require root (sudo su) access on your system to modify.

```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

### Modifying *low_latency* flag using *setserial* utility

The example below sets the *low_latency* flag for /dev/ttyUSB0.
This will have the same effect as setting the *latency_timer* to 1msec.
This can be confirmed by running the *setserial* command again.

```
user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user-VirtualBox:~$ setserial /dev/ttyUSB0 low_latency

user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```

## Package Contents

The Epson IMU ROS1 driver-related sub-folders & root files are:

```
   launch/        <== various example launch files for Epson IMU models
   src/           <== source code for ROS node C++ wrapper, IMU Linux C driver, and additional README_src.md specifically for building and using the IMU Linux C driver as stand-alone (without ROS support)
   CHANGELOG.rst  <== summarizes major changes
   CMakeLists.txt <== cmake build script for catkin_make
   LICENSE.txt    <== description of the applicable licenses
   package.xml    <== catkin package description
   README.md      <== general README for the ROS1 driver
```

## License

Refer to *LICENSE.txt* in this package

## References

1. https://index.ros.org
