# DART-5
In this repository, we're upgrading the DART robot for traffic light detection. DART is a robot, created at the TU Delft and instructions for building your own DART can be found [here](https://github.com/Lorenzo-Lyons/DART). 
In this repository, the following physical changes are made to the DART:
+ The LiDAR is connected directly to the Jetson Nano Expansion Board rather than to the Jetson Nano itself. This setup resolves the issue of the LiDAR drawing too much current from the Jetson Nano.
+ A new camera setup is added to enable the DART to perform traffic light detection.
You can 
