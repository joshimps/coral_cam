# Coral Cam
This package is designed to run and manage the different components of the underwater PAM system under development at UTS Rapido. 

## Features

- **Parameterised**: Highly configurable using Ros Parameters, edit the launch file to change startup settings for the buttons, GUI, Realsense Camera and more
- **GUI Interface**: Enjoy a simple Qt GUI for easy interaction and monitoring
- **File Output**: Leverages the PCL library to output pcd files for easy capture anaylsis

## UpBoard Squared V2 Setup
1. Install a fresh install of Ubuntu 22.04
2. Install the pin control driver for the Upboard following the instructions found [here](https://github.com/up-division/pinctrl-upboard).
3. Connect the Waveshare 3.5 inch screen using HDMI
4. Set the resolution through the GNOME settings to 800x480
5. Connect the Intel Real Sense D405 using USB
6. Connect all GPIO peripherals according to the pinout table below

   ![gpio](https://github.com/up-board/up-community/raw/main/images/up2/up2pinout_04092021.jpg)

## Package Installation
Assuming a fresh install of Ubuntu 22.04 has been installed on the UpBoard Squared V2.

1. Install ROS 2 Humble from debian following the instructions found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#id2).

2. Install LGPIO following the instructions found [here](https://abyz.me.uk/lg/download.html)

3. Create a git folder and clone this package to it. Create a ros workspace and then symbolic link this package to the src of the workspace.

   `mkdir ~/git` \
   `cd ~/git` \
   `git clone git@github.com:joshimps/coral_cam.git` \
   `mkdir -p ~/pam_ws/src` \
   `cd ~/pam_ws/src` \
   `ln -s ~/git/coral_cam/ .`


4. Source the ROS overlay, you may want to add this to your .bashrc

    `source /opt/ros/humble/setup.bash`
   
5. Resolve the depenencies for the coral_cam package

   `cd ~/pam_ws/` \
   `rosdep install -i --from-path src --rosdistro humble -y`

6. Build the package

    `cd ~/pam_ws/` \
    `colcon build`

7. Source the underlay, you may want to add this to your .bashrc

    `cd ~/pam_ws/` \
    `source install/local_setup.bash`

## Parameters

### GPIO Settings
- **gpio_number = 5** GPIO register which the peripherals are attached to, on the UpBoard GPIO use register 5
- **capture_button_pin = 27** pin number in Raspberry Pi GPIO numbering for the capture button to be connected to
- **debounce_time_us = 10000** number of nanoseconds to be used for button debounce time

### Real Sense Node Settings
- **number_of_real_sense_captures = 10** Number of point clouds the real sense camera should write after the capture button is pressed
- **point_cloud_path = "/home/pam/git/coral_cam/clouds"** Location the point cloud captures will be written to

### Industrial Node Settings
- **number_of_industrial_captures = 10** Number of photos the industrial camera should write after the capture button is pressed

### Light Settings
- **blue_flash_length_ms = 1000** Time in milliseconds the blue light should remain on for

### Temperature Settings
- **cuttoff_temperature = 100** Maximum temperature in degrees celsius before automatic shutdown

### Image Processing Settings (Best If Left Alone)
- **desired_width = 800** Width of the image displayed on the GUI 
- **desired_height = 360** Height of the image displayed on the GUI 
- **input_image = '/real_sense/color/image_rect_raw'** Topic of input image to resize
- **input_info = '/real_sense/color/camera_info'** ATopic of input image info to resize
- **output_image = '/real_sense/color/image_rect_resized'** Topic Of Resized output image
- **output_info = '/real_sense/color/camera_info_resized'** Topic Of Resized output image info

### Real Sense Image Settings
For the real sense image settings please refer to the documentation found on the [Intel Real Sense ROS Github](https://github.com/IntelRealSense/realsense-ros)
   
## Setup and Running

1. Unfortunately for the moment LGPIO needs sudo priviledges to access the GPIO devices. Start a new sudo command line with
   `sudo -i`
2. If you added sources to your .bashrc simply source .bashrc now
   `source /home/pam/.bashrc` \
   If you did not add these sources to your .bashrc source them now
   `source /opt/ros/humble/setup.bash` \
   `source /home/pam/pam_ws/install/local_setup.bash`
3. After configuring your parameters in coral_cam_launch.py launch the file
   `ros2 launch coral_cam coral_cam_launch.py`




