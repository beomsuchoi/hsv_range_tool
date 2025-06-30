# HSV Range Tool

A user-friendly Qt-based HSV color range calibration tool for ROS2

## Environment Setup

### Prerequisites
- **Ubuntu 22.04 LTS**
- **ROS2 Humble**
- **USB Camera**

### Dependencies Installation
```bash
# Install ROS2 packages
sudo apt update
sudo apt install ros-humble-usb-cam ros-humble-cv-bridge

# Install system dependencies
sudo apt install qtbase5-dev libopencv-dev
```

### Build Instructions
```bash
# Navigate to workspace
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/beomsuchoi/hsv_range_tool.git

# Build package
cd ~/ros2_ws
colcon build --packages-select hsv_range_tool
source install/setup.bash
```

## Usage

### Method 1: Separate Terminal Execution
```bash
# Terminal 1: Start camera
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Start HSV tool
ros2 launch hsv_range_tool hsv_range_tool.launch.py
```

### Method 2: Direct Execution
```bash
# Terminal 1: Start camera
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Start HSV tool directly
ros2 run hsv_range_tool hsv_range_tool
```

## Interface

- **Original Image**: Live camera feed
- **HSV Color**: Selected color range spectrum  
- **Masked White**: Binary mask (white = detected area)
- **Masked Raw**: Filtered image (detected area only)
- **Result Values**: HSV range parameters for OpenCV

## Output Usage

Use the displayed Lower/Upper values in your OpenCV code:
```cpp
cv::Scalar lower_bound(h_min, s_min, v_min);
cv::Scalar upper_bound(h_max, s_max, v_max);
cv::inRange(hsv_image, lower_bound, upper_bound, mask);
```

## Troubleshooting

**Camera not found:**
```bash
ls /dev/video*  # Check available cameras
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
```

**GUI not showing:**
```bash
echo $DISPLAY
xhost +local:
```

**Build errors:**
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
```
