# HSV Range Tool

<div align="center">

![HSV Range Tool](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)
![Qt5](https://img.shields.io/badge/Qt5-GUI-green?style=for-the-badge&logo=qt)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-red?style=for-the-badge&logo=opencv)
![C++](https://img.shields.io/badge/C++-11-blue?style=for-the-badge&logo=cplusplus)

**A real-time, user-friendly HSV color range calibration tool for ROS2**

</div>

## 📖 Overview

HSV Range Tool is a professional-grade Qt-based application designed for real-time HSV (Hue, Saturation, Value) color space calibration and filtering. Built specifically for ROS2 environments, it provides an intuitive interface for computer vision applications requiring precise color detection and object segmentation.

### 🎯 Key Features

- **Real-time Processing**: Live camera feed processing with instant HSV filtering results
- **Quad-View Display**: Simultaneous visualization of original, HSV spectrum, binary mask, and filtered images
- **Interactive Controls**: Slider-based HSV range adjustment with numerical feedback
- **OpenCV Integration**: Seamless integration with OpenCV's `cv::inRange()` function
- **ROS2 Native**: Built for ROS2 ecosystem with standard sensor_msgs/Image support
- **Thread-Safe**: Robust multi-threaded architecture for stable performance

## 🖼️ Interface Layout

```
┌─────────────────────────────────────────────────────────────┐
│  Original Image    │  Binary Mask       │                  │
│  (Camera Feed)     │  (White/Black)     │   HSV Controls   │
├─────────────────────────────────────────────┤                  │
│  HSV Spectrum      │  Filtered Result   │   • H: 0-179     │
│  (Color Range)     │  (Masked Image)    │   • S: 0-255     │
│                    │                    │   • V: 0-255     │
└─────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Prerequisites

- **ROS2 Humble** (or compatible distribution)
- **Qt5** development libraries
- **OpenCV 4.x**
- **USB Camera** or compatible image source

### Installation

1. **Clone the repository**
   ```bash
   cd ~/ros2_ws/src
   git clone <your-repository-url> hsv_range_tool
   ```

2. **Install dependencies**
   ```bash
   sudo apt update
   sudo apt install ros-humble-usb-cam ros-humble-cv-bridge
   sudo apt install qtbase5-dev libopencv-dev
   ```

3. **Build the package**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select hsv_range_tool
   source install/setup.bash
   ```

### Usage

1. **Start the camera node**
   ```bash
   ros2 run usb_cam usb_cam_node_exe
   ```

2. **Launch HSV Range Tool**
   ```bash
   ros2 launch hsv_range_tool hsv_range_tool.launch.py
   ```

3. **Adjust HSV parameters** using the sliders to isolate your target color

4. **Copy the result values** from the Lower/Upper display for use in your code:
   ```cpp
   cv::Scalar lower_bound(h_min, s_min, v_min);
   cv::Scalar upper_bound(h_max, s_max, v_max);
   cv::inRange(hsv_image, lower_bound, upper_bound, mask);
   ```

## 🛠️ Technical Specifications

### System Requirements
- **OS**: Ubuntu 22.04 LTS (or ROS2 Humble compatible)
- **RAM**: Minimum 4GB, Recommended 8GB+
- **CPU**: Multi-core processor recommended for real-time processing
- **Camera**: USB 2.0/3.0 compatible camera

### Dependencies
```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>qtbase5-dev</depend>
<depend>libopencv-dev</depend>
```

### Supported Image Formats
- **Input**: sensor_msgs/Image (BGR8, RGB8, MONO8)
- **Processing**: OpenCV Mat (8UC1, 8UC3, 8UC4)
- **Output**: Real-time Qt display + HSV parameters

## 📚 API Reference

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/usb_cam/image_raw` | sensor_msgs/Image | Input camera stream |

### HSV Parameter Ranges

| Parameter | Range | Description |
|-----------|-------|-------------|
| **Hue (H)** | 0-179 | Color component (OpenCV format) |
| **Saturation (S)** | 0-255 | Color intensity |
| **Value (V)** | 0-255 | Brightness level |

## 🔧 Configuration

### Camera Parameters
Modify launch parameters for different camera setups:
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video1 \
  -p image_width:=1280 \
  -p image_height:=720
```

### Custom Launch
Create custom launch files in the `launch/` directory for specific configurations.

## 🧪 Testing

### Unit Tests
```bash
cd ~/ros2_ws
colcon test --packages-select hsv_range_tool
```

### Manual Testing
1. **Color Detection**: Test with various colored objects
2. **Lighting Conditions**: Verify performance under different lighting
3. **Performance**: Monitor frame rate and system resource usage

### Expected Results
- **Frame Rate**: 30 FPS (640x480) on modern hardware
- **Latency**: <100ms from camera to display
- **Accuracy**: Precise color isolation with proper HSV tuning

## 🤝 Contributing

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### Development Guidelines
- Follow **ROS2 coding standards**
- Include **unit tests** for new features
- Update **documentation** for API changes
- Ensure **thread safety** for all Qt operations

## 🐛 Troubleshooting

### Common Issues

**Q: GUI window doesn't appear**
```bash
# Check display environment
echo $DISPLAY
xhost +local:
```

**Q: Camera not detected**
```bash
# List available cameras
ls /dev/video*
# Test camera access
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```

**Q: Build errors**
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --packages-select hsv_range_tool --cmake-clean-cache
```

### Performance Optimization
- **Reduce image resolution** for better frame rates
- **Close unnecessary applications** to free system resources
- **Use dedicated GPU** for OpenCV operations (if available)

## 📄 License

This project is licensed under the **BSD 3-Clause License** - see the [LICENSE](LICENSE) file for details.

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/your-username/hsv_range_tool/issues)
- **Documentation**: [Wiki](https://github.com/your-username/hsv_range_tool/wiki)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/hsv_range_tool/discussions)

## 🎯 Use Cases

- **Object Detection**: Isolate specific colored objects in images
- **Color Calibration**: Fine-tune HSV parameters for vision algorithms
- **Educational**: Learn computer vision and color space concepts
- **Prototyping**: Rapid development of color-based detection systems
- **Research**: Academic and industrial computer vision projects

---

<div align="center">

**Made with ❤️ for the ROS2 Community**

⭐ **Star this repo** if you find it useful!

</div>
