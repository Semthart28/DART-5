# Build Instructions
These build instructions assume you already have a DART, according to [Lorenzo Lyon's Github](https://github.com/Lorenzo-Lyons/DART) and want to upgrade it with a camera that's suitable for object detection applications. 

For all applications of DART, even if you don't want to upgrade the camera, it's best practice to follow the rest of the building instructions in this repository. Changing the LiDar's power supply will prevent the Jetson Nano from overheating. Changing the upperboard and baseboard with the 3D-printed standoffs will lower the center of mass as much as possible. The DART is a really fast robot and could tilt easily in a sharp corner. Especially now with a heavy camera on it. So lowering the center of mass is preferred. Additionally, the IMU and USB Adapter Board for the LiDar can now be mounted to the baseboard.

# Required Extra Components
+ a micro USB cable for the LiDar's power supply
+ The updated baseboard [dxf file link](https://github.com/Semthart28/DART-5/tree/main/Build%20Instructions/DXF%20files)
  
The required components for the new camera setup are:

+ The Intel RealSense Depth Camera D455 [link](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d455.html)
+ 1/4‑20 UNC bolt, 5/8 inch long [link](https://www.montagetechniek.nl/bouten/unc-bouten/unc-inbusbouten/iso-7380-unc-inbus-bolkop/iso-7380-rvs/per-stuk/1-4--20-unc-inbus-bolkop/1-4-20-x-5-8-rvs)
+ The updated upperboard [dxf file link](https://github.com/Semthart28/DART-5/tree/main/Build%20Instructions/DXF%20files)

# Changing the LiDar's power supply
In order to change the LiDar's power supply, you want to make sure your baseboard is the updated version. This version has an extra gap so the micro-USB can be soldered to the battery pack. Strip your micro-USB to the correct length and attach the power supply to the LiDar. Solder the stripped end of the micro-USB to the labeled '5V' and 'GND' on the right-hand-side of the display. Now the LiDar is directly connected to the battery pack.

# Adding the new camera setup
A close-up of the camera setup is shown here below:

<p align="center">
  <img src="images/camera-setup.jpg" width="250">
</p>

You'll need to replace the upper board with the updated version to leave a gap for attaching the camera and enough space to plug in the cable. Plug the other end of the cable into a USB port on the Jetson Nano and your camera-setup is done!
