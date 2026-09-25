<div align="center">
  <img src="https://capsule-render.vercel.app/api?type=rect&color=timeGradient&height=150&section=header&text=Task%201A:%20Ore%20Detection&fontSize=40&fontAlignY=50" />
</div>

<h1 align="center">🔍 OpenCV Color & Ore Detection</h1>

<div align="center">
  <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white" />
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white" />
</div>

---

## 🎯 Task Overview

Welcome to the **Task 1A** branch for e-Yantra 2026. This module focuses on the **Computer Vision** subsystem of our StrataCobot. The objective of this task is to utilize a live camera feed to reliably detect and classify different types of "ores" placed in the environment using their HSV color profiles and geometric contours.

### 🏜️ The Kepler Colony Environment
<div align="center">
  <img src="images/kepler_overview.png" alt="Kepler Colony Overview" width="800"/>
  <p><i>A wide view of the simulated Kepler Colony on Mars, featuring our robotic setup, terrain, rovers, and operational infrastructure.</i></p>
</div>

The simulation takes place in the custom `eyantra_kepler_world`. As seen above, the environment is a rugged Martian terrain where our UR7e arm is tasked with interacting with various scattered rocks and ores.

### 🔍 Ore Detection Setup
<div align="center">
  <img src="images/ore_setup.png" alt="Ore Detection Setup" width="800"/>
  <p><i>Close-up of the UR7e robotic arm alongside the conveyor belt and collection bins, with labeled ores (Malachite, Azurite, Vanadinite) ready for detection.</i></p>
</div>

This image demonstrates the core of Task 1A. The ores are spawned on the conveyor belt. Our OpenCV pipeline processes the camera feed to detect the bright green (Malachite), blue (Azurite), and orange (Vanadinite) ores against the dark background of the conveyor belt.

---

## 🛠️ Implementation Details

Our solution is contained in the `ore_detector.py` node. Here is a breakdown of our pipeline:

1. **Image Acquisition**: 
   - We subscribe to the raw camera topic from Gazebo using ROS 2 `sensor_msgs/msg/Image`.
   - We utilize `cv_bridge` to seamlessly convert ROS messages into OpenCV `cv2.Mat` objects.

2. **Color Filtering (HSV)**:
   - We convert the incoming RGB/BGR stream into the HSV (Hue, Saturation, Value) color space. 
   - HSV is far more resilient to lighting variations in the Gazebo simulation than pure RGB filtering.
   - We defined precise lower and upper boundaries to mask out specific ore colors.

3. **Contour Detection & Analysis**:
   - We apply morphological operations (like `cv2.erode` and `cv2.dilate`) to the masks to reduce noise.
   - `cv2.findContours` is used to locate the boundaries of the ores.
   - We filter contours based on total pixel area to reject false positives (e.g., tiny specks of color in the background).

4. **Localization (Centroid Extraction)**:
   - For every valid ore, we calculate image moments (`cv2.moments`) to pinpoint the precise `(Cx, Cy)` center pixel coordinate.
   - These coordinates are broadcasted back into the ROS network for the robotic arm to consume in later tasks!

## 🚀 Running the Node

You can launch the ore detector node via ROS 2 by running:

```bash
ros2 run algorithms ore_detector.py
```

*Ensure the Gazebo simulation is active and publishing to the camera topic before running the script!*

---
<div align="center">
  <a href="https://github.com/TejasRaut15k/e-yantra/tree/main">🔙 Return to Main Repository</a>
</div>
