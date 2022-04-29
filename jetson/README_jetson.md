# Epson IMU Usage on Nvidia Jetson TK1

This is a general guide for installing and configuring the *Nvidia Jetson TK1* with Linux for Tegra (L4T) and *ROS Indigo* before making use of the Epson IMU UART ROS driver.

Using the Epson IMU ROS driver on an embedded platform requires some additional steps for setting up *Ubuntu 14* and *ROS Indigo* environment compared 
to PC Ubuntu + ROS Indigo/Kinetic system which is straight forward.

## Getting Started

These instructions are a compilation from various web sources to get the Jetson TK1 configured from scratch to execute the Epson IMU ROS driver in ROS Indigo.

### Prerequisites

This will require the following:

- Epson USB evaluation board or equivalent FTDI USB-Serial interface [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
- Epson IMU (V340/G320/G325/G354/G364/G365/G370) [See IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
- PC running native [Ubuntu 14 or higher](https://ubuntu.com/download/desktop)
- Nvidia Jetson TK1 + USB Micro host cable [See here](https://www.nvidia.com/object/jetson-tk1-embedded-dev-kit.html)
- Nvidia Jetpack 2.3.1 software [See here](https://developer.nvidia.com/embedded/jetpack-2_3_1)
- L4T v21.5 kernel sources [Download from here](http://developer.download.nvidia.com/embedded/L4T/r21_Release_v5.0/source/kernel_src.tbz2)
- ROS Indigo (via download) [See here](https://wiki.ros.org/indigo/Installation/UbuntuARM)


### Installing L4T on Jetson TK1

This will assume that the starting point is to install a fresh copy of the latest Linux for Tegra (L4T) on the Jetson TK1.

The recommended method is to download the Jetpack 2.3.1 from NVIDIA website on to a PC running Ubuntu. [From here](https://developer.nvidia.com/embedded/jetpack-2_3_1)

The Jetpack is an all-in-one software package that installs/runs on a host PC Ubuntu host.
The PC Ubuntu host communicates to the Jetson TK1 via USB micro cable.
The file system image is prepared on the host PC and then transferred to the Jetson TK1 onboard eMMC.

*NOTE: This will reflash the filesystem on the Jetson TK1 onboard eMMC. You will lose all your existing
data on the Jetson TK1.*

Follow the instructions included on the Jetpack documentation to unarchive and execute the Jetpack on a native Ubuntu 14 PC.
This will be part of the flashing process to install a fresh copy of the Linux file system which should include cuda toolkit and opencv4tegra.

*NOTE: Running Jetpack in Ubuntu 14 inside a VirtualBox can cause stability problems with USB communication between the PC host and Jetson TK1 during flashing process.
It is recommended to run the Jetpack software directly on native Ubuntu PC, instead of Virtual Box or VMWare.*

### Update L4T on Jetson TK1

Once L4T is installed properly, it is important to update Ubuntu 14 with the latest fixes and packages on the Jetson TK1.

From a console running on the Jetson TK1, run the commands:

```
  sudo apt-get update
  sudo apt-get upgrade
```

### Update L4T FTDI USB Serial kernel driver on Jetson TK1
By default, the L4T kernel will not support FTDI USB serial converters (these are needed for the Epson USB Evalboards), so the kernel module needs to be built and enabled.

The procedure to enable the FTDI USB kernel driver is also described in https://elinux.org/Jetson/Tutorials/Program_An_Arduino

An automated shell script can also be found here (not tested by author) https://github.com/jetsonhacks/buildJetsonFTDIModule


- Download the kernel sources:
```
wget http://developer.download.nvidia.com/embedded/L4T/r21_Release_v5.0/source/kernel_src.tbz2
```

- Extract to your home directory:
```
tar -xvf kernel_src.tbz2
```

- Extract a copy current kernel configuration to extracted source folder:
```
zcat /proc/config.gz > ~/kernel/.config
```

- Execute menuconfig to start process of enabling the USB FTDI Serial Driver:
```
sudo apt-get install ncurses-bin libncurses5-dev
make menuconfig
```

- In menuconfig, choose [M] in Device Drivers -> USB Support -> USB Serial Converter Support -> USB FTDI Single Port Serial Driver

- Determine the CONFIG_LOCALVERSION run the command and write down the return value for future steps (CONFIG_LOCALVERSION i.e. "-gdacac96"):

```
uname -a
```

- The return value is appended to the kernel module DURING the compilation to guarantee that the version of the kernel and the kernel module driver match.

- To set the CONFIG_LOCALVERSION in the kernel module to the same as the current kernel, to the following:

- In menuconfig, go to the original menu, and select General Setup->Local Version.

- Type in the Local Version from above step with *dash included* i.e. "-gdacac96"

- Save changes and exit from menuconfig

- Build the kernel module:
```
make prepare
make modules_prepare
make M=drivers/usb/serial/
```

- Copy the kernel module to the system location:
```
sudo cp drivers/usb/serial/ftdi_sio.ko /lib/modules/$(uname -r)/kernel
sudo depmod -a
```

- Add your username to the dialout group to allow read/write permission to serial ports:
```
sudo usermod -a -G dialout <username>
```
- Reboot the Jetson TK1.

### Installing ROS Indigo on L4T

After updating Ubuntu 14, and enabling FTDI kernel module, the next step is to install ROS Indigo or ROS Kinetic. This is fairly straight forward.

Refer to the standard procedure for ARM platforms in the following link:
[ROS Installation, Ubuntu ARM](http://wiki.ros.org/indigo/Installation/UbuntuARM)


### Building and Installing Epson IMU ROS Driver

The Epson IMU ROS driver is designed to build in the ROS catkin build environment. 
Therefore, an functional catkin workspace in ROS is a prerequisite. 
Refer to the ROS Tutorials Wiki for more info: [ROS Tutorial Wiki](https://wiki.ros.org/ROS/Tutorials#Beginner_Level)

For more information on ROS & catkin setup refer to 
[Installing and Configuring ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

To install the Epson IMU ROS driver package:
1. Place files (including folders) into a new folder within your catkin workspace "src" folder.
   For example, we recommend using the folder name "epson_imu_uart_driver"
```
   <catkin_workspace>/src/epson_imu_uart_driver/ <-- place files here
```
2. Modify the CMakeLists.txt to select the correct Epson IMU model that is attached to the ROS system.
   Refer to the comment lines inside the CMakeLists.txt for additional info.

3. From the catkin_workspace folder run "catkin_make" to build any ROS packages located in the <catkin_workspace>/src/ folder.
```
   <catkin_workspace>/catkin_make
```

The following is example of the output of the catkin_make:
```
user@user-VirtualBox:~/catkin_ws$ catkin_make
Base path: /home/user/catkin_ws
Source space: /home/user/catkin_ws/src
Build space: /home/user/catkin_ws/build
Devel space: /home/user/catkin_ws/devel
Install space: /home/user/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/user/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/user/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/user/catkin_ws/devel;/opt/ros/kinetic
-- This workspace overlays: /home/user/catkin_ws/devel;/opt/ros/kinetic
-- Using PYTHON_EXECUTABLE: /usr/bin/python
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/user/catkin_ws/build/test_results
-- Found gmock sources under '/usr/src/gmock': gmock will be built
-- Found gtest sources under '/usr/src/gmock': gtests will be built
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.7.14
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 5 packages in topological order:
-- ~~  - imu_tools (metapackage)
-- ~~  - epson_imu_uart_driver
-- ~~  - imu_complementary_filter
-- ~~  - imu_filter_madgwick
-- ~~  - rviz_imu_plugin
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin metapackage: 'imu_tools'
-- ==> add_subdirectory(imu_tools/imu_tools)
-- +++ processing catkin package: 'epson_imu_uart_driver'
-- ==> add_subdirectory(imu_ros_uart)
INFO/opt/ros/kinetic/lib/libroscpp.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_signals.so;/opt/ros/kinetic/lib/librosconsole.so;/opt/ros/kinetic/lib/librosconsole_log4cxx.so;/opt/ros/kinetic/lib/librosconsole_backend_interface.so;/usr/lib/x86_64-linux-gnu/liblog4cxx.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/opt/ros/kinetic/lib/libxmlrpcpp.so;/opt/ros/kinetic/lib/libroscpp_serialization.so;/opt/ros/kinetic/lib/librostime.so;/opt/ros/kinetic/lib/libcpp_common.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so
-- +++ processing catkin package: 'imu_complementary_filter'
-- ==> add_subdirectory(imu_tools/imu_complementary_filter)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- +++ processing catkin package: 'imu_filter_madgwick'
-- ==> add_subdirectory(imu_tools/imu_filter_madgwick)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Boost version: 1.58.0
-- Found the following Boost libraries:
--   system
--   thread
--   signals
--   chrono
--   date_time
--   atomic
-- +++ processing catkin package: 'rviz_imu_plugin'
-- ==> add_subdirectory(imu_tools/rviz_imu_plugin)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Using Qt5 based on the rviz_QT_VERSION: 5.5.1
-- Configuring done
-- Generating done
-- Build files have been written to: /home/user/catkin_ws/build
####
#### Running command: "make -j2 -l2" in "/home/user/catkin_ws/build"
####
Scanning dependencies of target epson_imu_uart_driver_lib
[  3%] Building C object imu_ros_uart/CMakeFiles/epson_imu_uart_driver_lib.dir/src/hcl_gpio.c.o
[ 13%] Built target complementary_filter
[ 17%] Built target imu_filter_madgwick_gencfg
[ 20%] Automatic moc for target rviz_imu_plugin
[ 20%] Built target rviz_imu_plugin_automoc
[ 24%] Building C object imu_ros_uart/CMakeFiles/epson_imu_uart_driver_lib.dir/src/hcl_linux.c.o
[ 27%] Building C object imu_ros_uart/CMakeFiles/epson_imu_uart_driver_lib.dir/src/sensor_epsonCommon.c.o
[ 31%] Building C object imu_ros_uart/CMakeFiles/epson_imu_uart_driver_lib.dir/src/hcl_uart.c.o
[ 34%] Building C object imu_ros_uart/CMakeFiles/epson_imu_uart_driver_lib.dir/src/sensor_epsonUart.c.o
[ 41%] Built target complementary_filter_node
[ 44%] Linking C shared library /home/user/catkin_ws/devel/lib/libepson_imu_uart_driver_lib.so
[ 58%] Built target imu_filter
[ 58%] Built target epson_imu_uart_driver_lib
Scanning dependencies of target epson_imu_uart_driver_node
[ 79%] Built target rviz_imu_plugin
[ 82%] Building CXX object imu_ros_uart/CMakeFiles/epson_imu_uart_driver_node.dir/src/epson_imu_uart_driver_node.cpp.o
[ 89%] Built target imu_filter_nodelet
[ 96%] Built target imu_filter_node
[100%] Linking CXX executable /home/user/catkin_ws/devel/lib/epson_imu_uart_driver/epson_imu_uart_driver_node
[100%] Built target epson_imu_uart_driver_node
user@user-VirtualBox:~/catkin_ws$  
```

4. Reload the current ROS environment variables that may have changed after the catkin build process.
```
   source <catkin_workspace>/devel/setup.bash
```

5. To start the Epson IMU ROS driver run the appropriate launch file (located in launch/) from console.
   Modify the launch file as necessary for your desired setting such as dout_rate, filter_sel, etc...
   For example, for the Epson G365 IMU:
```
   roslaunch epson_imu_uart_driver epson_g325_g365.launch
```
Below is an example of console output when the Epson IMU ROS driver is correctly launched:

```
user@user-VirtualBox:~/catkin_ws$ roslaunch epson_imu_uart_driver epson_g325_g365.launch 
... logging to /home/user/.ros/log/0c01e56e-9e88-11e9-ad70-0800277645e4/roslaunch-user-VirtualBox-3004.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://user-VirtualBox:41555/

SUMMARY
========

PARAMETERS
 * /epson_imu_uart_driver_node/accel_bit: 1
 * /epson_imu_uart_driver_node/accel_delta_bit: 1
 * /epson_imu_uart_driver_node/accel_delta_out: 0
 * /epson_imu_uart_driver_node/accel_out: 1
 * /epson_imu_uart_driver_node/atti_bit: 1
 * /epson_imu_uart_driver_node/atti_conv: 0
 * /epson_imu_uart_driver_node/atti_mode: 1
 * /epson_imu_uart_driver_node/atti_out: 1
 * /epson_imu_uart_driver_node/checksum_out: 1
 * /epson_imu_uart_driver_node/count_out: 1
 * /epson_imu_uart_driver_node/dout_rate: 9
 * /epson_imu_uart_driver_node/drdy_on: 0
 * /epson_imu_uart_driver_node/drdy_pol: 0
 * /epson_imu_uart_driver_node/ext_pol: 0
 * /epson_imu_uart_driver_node/ext_sel: 1
 * /epson_imu_uart_driver_node/filter_sel: 9
 * /epson_imu_uart_driver_node/flag_out: 1
 * /epson_imu_uart_driver_node/gpio_out: 0
 * /epson_imu_uart_driver_node/gyro_bit: 1
 * /epson_imu_uart_driver_node/gyro_delta_bit: 1
 * /epson_imu_uart_driver_node/gyro_delta_out: 0
 * /epson_imu_uart_driver_node/gyro_out: 1
 * /epson_imu_uart_driver_node/invert_xaccel: 0
 * /epson_imu_uart_driver_node/invert_xgyro: 0
 * /epson_imu_uart_driver_node/invert_yaccel: 0
 * /epson_imu_uart_driver_node/invert_ygyro: 0
 * /epson_imu_uart_driver_node/invert_zaccel: 0
 * /epson_imu_uart_driver_node/invert_zgyro: 0
 * /epson_imu_uart_driver_node/port: /dev/ttyUSB0
 * /epson_imu_uart_driver_node/temp_bit: 1
 * /epson_imu_uart_driver_node/temp_out: 1
 * /epson_imu_uart_driver_node/time_correction: 0
 * /rosdistro: indigo
 * /rosversion: 1.11.20

NODES
  /
    epson_imu_uart_driver_node (epson_imu_uart_driver/epson_imu_uart_driver_node)

auto-starting new master
process[master]: started with pid [3016]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 0c01e56e-9e88-11e9-ad70-0800277645e4
process[rosout-1]: started with pid [3029]
started core service [/rosout]
process[epson_imu_driver_node-2]: started with pid [3032]
[ INFO] [1562264277.355674173]: Initializing HCL layer...
[ INFO] [1562264277.355817808]: Initializing GPIO interface...
[ INFO] [1562264277.355952854]: Initializing UART interface...
Attempting to open port.../dev/ttyUSB0

...sensorDummyWrite.[ INFO] [1562264277.617467874]: Checking sensor NOT_READY status...
...done.[ INFO] [1562264278.468729865]: Initializing Sensor...
[ INFO] [1562264278.505628435]: ...Epson IMU initialized.

```

## License

The Epson IMU ROS Driver and related code is licensed under the BSD-3 License - see the [3-Clause BSD](https://opensource.org/licenses/BSD-3-Clause) for details

