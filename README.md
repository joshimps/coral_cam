# Coral Cam
This package is designed to run and manage the different components of the underwater PAM system under development at UTS Rapido. 

## Features

- **Parameterised**: Highly configurable using Ros Parameters, edit the launch file to change startup settings for the buttons, GUI, Realsense Camera and more
- **GUI Interface**: Enjoy a simple Qt GUI for easy interaction and monitoring
- **File Output**: Leverages the PCL library to output pcd files for easy capture anaylsis

## Installation
Assuming a fresh install of Ubuntu 22.04 has been installed on the UpBoard Squared V2.

1. Install the pin control driver for the Upboard following the instructions found [here](https://github.com/up-division/pinctrl-upboard).
   
2. Install ROS 2 Humble from debian following the instructions found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#id2).

3. Intall the latest Realsense SDK 2.0 following the instructions found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).

4. Install LGPIO following the instructions found [here](https://abyz.me.uk/lg/download.html)
   
5. Install PCL following the instructions found [here](https://pointclouds.org/downloads/#linux)

6. Install QT 5.15 folllowing the instructions found below ([here](https://wiki.qt.io/Building_Qt_5_from_Git#Getting_the_source_code) is used as a reference)

   Choose a location for the source files to live

   `mkdir ~/git` \
   `cd ~/git` \
   `git clone git://code.qt.io/qt/qt5.git` \
   `cd qt5` \
   `git checkout 5.15` \
   `perl init-repository` 

   Create a location for the build files to live 
   
   `export LLVM_INSTALL_DIR=/usr/llvm` \
   `cd ~` \
   `mkdir qt5-build` \
   `cd qt5-build` \
   '~/git/qt5/configure -prefix /opt/Qt15 -opensource -nomake examples -nomake tests' \

   Make the modules we need, they should be installed to /opt/Qt15 as specified earlier \

   `make module-qtbase` \
   `make module-qtdeclarative` \
   `make install`
   
   
7. Create a git folder (if not done in previous step) and clone this package to it. Create a ros workspace and then symbolic link this package to the src of the workspace.

   `cd ~/git` \
   `git clone https://github.com/joshimps/coral_cam.git` \
   `mkdir -p ~/pam_ws/src` \
   `cd ~/pam_ws/src` \
   `ln -s ~/git/coral_cam/ .` 

8. Source the ROS overlay, you may want to add this to your .bashrc

    `source /opt/ros/humble/setup.bash` 
   
9. Resolve the depenencies for the coral_cam package

   `cd ~/pam_ws/` \
   `rosdep install -i --from-path src --rosdistro humble -y`

10. Build the package

    `cd ~/pam_ws/` \
    `colcon build`

11. Source the underlay, you may want to add this to your .bashrc

    `cd ~/pam_ws/` \
    `source install/local_setup.bash`
   
## Setup and Running


