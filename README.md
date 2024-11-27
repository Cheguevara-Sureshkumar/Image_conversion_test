# Image_conversion_test

# ROS2 Image Conversion Package (C++ Version)

## Prerequisites
- ROS2 Humble
- OpenCV
- usb_cam package

## Dependencies
Install dependencies:
```bash
sudo apt install ros-humble-usb-cam ros-humble-cv-bridge ros-humble-image-transport
```

## Build Instructions
```bash
cd ~/<your_workspace>_ws/src          #change it to your workspace name
git clone <repository-url>            #clone the repository from the github
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select image_conversion_pkg    #build the clone package
source install/setup.bash      
```
## Launch Instructions
Continue on the working terminal:
```bash
ros2 launch image_conversion_pkg image_conversion_launch.py
```

## Service Calls to Change Mode

Open a new terminal to run:
```bash
source install/setup.bash
ros2 service call /image_conversion/change_mode std_srvs/SetBool "{data: true}"      # Change to Grayscale Mode
```
```bash
source install/setup.bash
ros2 service call /image_conversion/change_mode std_srvs/SetBool "{data: false}"     # Change to Color Mode
```
## Node Details
- Subscribes to input camera topic
- Provides service to switch between color and grayscale modes
- Publishes converted images to output topic

