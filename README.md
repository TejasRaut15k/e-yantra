<div align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=timeGradient&height=150&section=header&text=Full%20ROS%202%20Workspace&fontSize=40&fontAlignY=38" />
</div>

<h1 align="center">📦 Full e-Yantra 2026 ROS 2 Workspace</h1>

<div align="center">
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/Gazebo-Ignition-FFB300?style=for-the-badge&logo=gazebo&logoColor=white" />
</div>

---

## 🌟 Overview

Welcome to the **`full-workspace`** branch. This branch contains the **complete `src/` directory** of our ROS 2 workspace, encompassing all necessary packages (including the official competition simulation packages and our custom algorithm packages) to run the e-Yantra Kepler Colony simulation entirely out-of-the-box.

### 📁 Included Packages
This workspace contains the following ROS 2 packages:
* **`algorithms`**: Our custom logic containing the OpenCV ore detection node (Task 1A) and the 6-state Waypoint Navigation Controller (Task 1B).
* **`eyantra_kepler_colony`**: The official competition world and environment definitions.
* **`ebot_description`**: The official robot model and URDF for the eBot rover.
* **`ur_description`**: The Universal Robots UR7e description, meshes, and kinematics.

---

## 🚀 How to Build and Run

To set up this full workspace on your local machine, run the following commands:

### 1. Clone the repository
```bash
mkdir -p ~/eyantra_ws
cd ~/eyantra_ws
git clone -b full-workspace https://github.com/TejasRaut15k/e-yantra.git src
```

### 2. Install Dependencies
```bash
cd ~/eyantra_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace
```bash
colcon build
source install/setup.bash
```

Once built, you can follow the execution instructions detailed in the [Task 1A](https://github.com/TejasRaut15k/e-yantra/tree/task-1a) and [Task 1B](https://github.com/TejasRaut15k/e-yantra/tree/task-1b) branches!

---
<div align="center">
  <a href="https://github.com/TejasRaut15k/e-yantra/tree/main">🔙 Return to Main Repository</a>
</div>
