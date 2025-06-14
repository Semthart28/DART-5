# Build Instructions
These build instructions assume you already have a DART, according to [Lorenzo Lyon's Github](https://github.com/Lorenzo-Lyons/DART), and want to upgrade it with a camera that's suitable for object detection applications. 

For all applications of DART, even if you don't want to upgrade the camera, it's best practice to follow the rest of the building instructions in this repository. Changing the LiDar's power supply will prevent the Jetson Nano from overheating. Changing the upperboard and baseboard with the 3D-printed standoffs will lower the center of mass as much as possible. The DART is a really fast robot and could tilt easily around a sharp corner. Especially now with a heavy camera on it. So, lowering the center of mass is preferred. Additionally, the Arduino, IMU, and USB Adapter Board for the LiDar can now be mounted to the baseboard.

At the bottom, you can find how we built our 3D-printed traffic light.

# Required Extra Components
+ a micro USB cable for the LiDAR's power supply
+ The updated baseboard and upperboard [dxf files](https://github.com/Semthart28/DART-5/tree/main/Build%20Instructions/DXF%20files)
+ The Intel RealSense Depth Camera D455 [link](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d455.html)
+ The new 3D-printed parts [STL files](https://github.com/Semthart28/DART-5/tree/main/Build%20Instructions/STL%20files) (Carefully read how many times you need to print each file)
+ Some bolts and screws:
    + 12x M2.5 8mm Stainless Steel Screws
    + 16x M2.5 Stainless Steel Nuts
    + 8x M2.5 6mm-6mm brass female-male
    + 12x M2.5 ~14mm screws
    + 2x M4 ~8mm screws (Intel camera mounts)
    + 2x M3 ~10mm screws
    + 2x M3 nuts
    + 4x M1.6 ~10mm screws 
    + 4x M1.6 nuts
+ This small Arduino LED display for the traffic light [link](https://nl.aliexpress.com/item/1005006161998547.html?spm=a2g0o.order_list.order_list_main.5.454a79d2ccuUTg&gatewayAdapt=glo2nld)
+ Any Arduino-compatible microcontroller to power and control the timing of the lights of the traffic light



# Changing the LiDar's power supply
In order to change the LiDar's power supply, you need to strip a micro-USB cable. Attach the micro-USB port to the USB adapter board of the LiDAR and solder the stripped end of the cable directly onto the JetRacer Pro Expansionboard as shown in the image below:

<p align="center">
  <img src="images/Back_of_ExpansionBoard.JPG" width="250">
</p>

Solder the red cable of the micro-USB to the labeled '5V' and the black cable to the labeled 'GND' on the right-hand side of the display. Now the LiDar is directly connected to the battery pack.

# Replacing the Baseboard
On the new baseboard, you can attach the IMU and the USB Adapterboard of the LiDar as shown in this image below (left):

<p align="center">
  <img src="images/Baseboard_1.jpg" width="350">
  <img src="images/Arduino_nano.jpeg" width="350">
</p>

On the right, you can see how to attach the Arduino Nano to the slots on the baseboard with the 3D printed pieces.

Attach the blue standoffs and the black standoffs that belong to the LiDar's setup, and then attach the baseboard onto the expansion board.

# Adding the new camera setup
A side-view of the DART is shown below:

<p align="center">
  <img src="images/Zijkant_DART.jpg" width="350">
</p>

Attach the 3D-printed camera mounts to the back of the camera with M4 bolts and to the upperboard with M3 bolts. Plug the USB cable into the Jetson Nano, and your camera setup is done!

# Replacing the Brass spacers of the suspension with 3D-printed spacers (Optional)
If you want to, you can replace the brass spacers of the suspension with the 3D printed 'Bus' STL file.  We did this so we didn't have to buy the brass rings and can print them easily with the other 3D parts.

# 3D-printed traffic light
We designed a small 3D-printed traffic light that resembles a realistic Dutch traffic light. You will need the three 'traffic light' STL files, the LED Arduino display, any Arduino-compatible microcontroller, and the [Arduino code](Google.com).

For printing the white edge of the traffic light, we made a separate 3D model for the white edge, which we merged in the slicer software with the rest. By having two separate STLs, you can assign different colors to the different parts of the traffic light:
<p align="center">
  <img src="images/Screenshot from 2025-05-27 12-10-28.png" width="350">
</p>





