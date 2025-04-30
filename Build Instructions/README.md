# Build Instructions
These build instructions assume you already have a DART, according to [Lorenzo Lyon's Github](https://github.com/Lorenzo-Lyons/DART) and want to upgrade it with a camera that's suitable for almost all object detection applications. 

For all applications of DART, even if you don't want to upgrade the camera, it's best practice to follow the 'changing the power supply of the LiDar' part of this repository to not overheat the Jetson Nano.

# Required Extra Components
The required components for the seperate power supply of the LiDar are:

+ a micro USB cable
+ The updated baseboard [dxf file link](https://github.com/Semthart28/DART-5/tree/main/Build%20Instructions/DXF%20files) (Lorenzo probably updated his repo so you might not have to update the baseboard).
  
The required components for the new camera setup are:

+ The Intel RealSense Depth Camera D455 [link](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d455.html)
+ 1/4‑20 UNC bolt, 5/8 inch long [link](https://www.montagetechniek.nl/bouten/unc-bouten/unc-inbusbouten/iso-7380-unc-inbus-bolkop/iso-7380-rvs/per-stuk/1-4--20-unc-inbus-bolkop/1-4-20-x-5-8-rvs)

# Changing the LiDar's power supply
In order to change the LiDar's power supply, you want to make sure your baseboard is the updated version. This version has an extra gap so the micro-USB can be soldered to the battery pack. Strip your micro-USB to the correct length and attach the power supply to the LiDar. Solder the stripped end of the micro-USB to the labeled '5V' and 'GND' on the right-hand-side of the display. Now the LiDar is directly connected to the battery pack.

# Adding the new camera setup
