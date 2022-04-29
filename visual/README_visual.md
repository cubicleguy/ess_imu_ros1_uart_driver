# ROS Python 3D Visualization

## Additional Python Libraries for Visualization
For testing or evaluating or visualizing using the Epson IMU in a ROS Python 2 environment, then these packages will help:
- [Pyserial](https://pythonhosted.org/pyserial/) and 
- [Visual Python](https://www.vpython.org/) (only for Python 2).

To install run the following:
```
sudo apt-get install python-serial
sudo apt-get install python-visual python-wxgtk2.8
```

## Description of ROS Visualization

These ROS 3D visualization python scripts are modified from [ROS razor_imu_9dof](https://wiki.ros.org/razor_imu_9dof).

These scripts currently work only with Python2 and only with python package Visual Python 5.

These python scripts reads fused IMU orientation messages that are published on ROS topic /imu/data or /epson_imu_rpy.

1. For orientation (quaternion) field, the python script will subscribe to /imu/data
2. For Euler messages in geometry_msgs/Vector3 format, the script will subscribe to /epson_imu_rpy

*NOTE:*

The python 3D visualization will not work with IMUs that only publish gyroscope & accelerometer messages (not orientation messages) such as those published in /imu/data_raw
   
Choose the correct python script based on the available ROS topic:

- display_3D_epson_quat.py - Subscribes to /imu/data to get orientation field (quaternions)
- display_3D_epson_euler.py - Subscribes to /epson_imu_rpy to get 3 vector Euler fields


*NOTE:*

Epson IMU models such as G365 include attitude output (Euler mode) function, which output non-standard Euler rotation order. 
The Euler angle rotation order for the Epson IMU (yaw->roll->pitch) differs from the ROS convention (yaw->pitch->roll).
Therefore, when the attitude output function (atti_out) is enabled, the Epson IMU ROS node will convert
the non-standard Euler angles to Quaternion orientation messages using the ROS tf2 transform library.
This also means that when using the G365 attitude output function, it will suffer from Euler ambiguity when roll
exceeds +/- 90 degrees*

## License

Refer to copyright in the python source header