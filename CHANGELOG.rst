^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ess_imu_ros1_uart_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2023-05-24)
------------------
* adds support for G330PDG0, G366PDG0, G370PDS0, G370PDG0, G370PDT0, remove G325, G365PDx0, G370PDF0
* adds output message at start of execution to display the device model that the executable is built for
* adds output message to read the device model and warn if there is a mismatch with executable build

1.3.2 (2022-08-03)
------------------
* update cmake_minimum_required to avoid unstable build on buildfarm due to CMP0048 warning
* update README.md for name change in package.xml, CMakeLists.txt
* add dependency geometry_msgs and tf2
* fix for name change in package.xml, CMakeLists.txt, launch files
* Cleanup description of applicable licensing for files BSD-3 and Public Domain

1.3.1 (2022-07-19)
------------------
* release 1st time
* renamed from package from epson_imu_uart_driver
