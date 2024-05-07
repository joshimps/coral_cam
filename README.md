# Coral Cam
This package is designed to be run and manage the different components of the underwater PAM system under development at UTS Rapido. \
The following hardware is required for operation. 

- UpBoard Squared V2
- Intel Realsense D405
- PWM Buttons

## Features

- **Parameterised**: Highly configurable using Ros Parameters, edit the launch file to change startup settings for the buttons, GUI, Realsense Camera and more
- **GUI Interface**: Enjoy a simple Qt GUI for easy interaction and monitoring
- **File Output**: Leverages the PCL library to output pcd files for easy capture anaylsis

## Installation
Assuming a fresh install of Ubuntu 22.04 has been installed on the UpBoard Squared V2.

1. Install the pin control driver for the Upboard following the instructions found [here](https://github.com/up-division/pinctrl-upboard).
   
3. Install ROS 2 Humble from debian following the instructions found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#id2).

4. Intall the latest Realsense SDK 2.0 following the instructions found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).

5. Install LGPIO following the instructions found [here](https://abyz.me.uk/lg/download.html)
   
6. Install PCL following the instructions found [here](https://pointclouds.org/downloads/#linux)

7. Create a Ros Workspace and clone this package to it

   `mkdir -p ~/pam_ws/src` \
   `cd ~/pam_ws/src` \
   `https://github.com/joshimps/coral_cam.git`

8. Resolve the depenencies for the coral_cam package

   `cd ~/pam_ws/` \
   `rosdep install -i --from-path src --rosdistro humble -y`
   
## Setup and Running


