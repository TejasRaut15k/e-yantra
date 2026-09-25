<div align="center">
  <img src="https://capsule-render.vercel.app/api?type=rect&color=timeGradient&height=150&section=header&text=Task%201B:%20UR7e%20Waypoint%20Navigation&fontSize=40&fontAlignY=50" />
</div>

<h1 align="center">🦾 UR7e Arm Waypoint Navigation Controller</h1>

<div align="center">
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/Gazebo-Ignition-FFB300?style=for-the-badge&logo=gazebo&logoColor=white" />
</div>

---

## 🎯 Task Overview

Welcome to the **Task 1B** branch for e-Yantra 2026. This module focuses on the **Kinematics & Control** subsystem of our StrataCobot. The objective is to navigate the **Universal Robots UR7e Arm** through a strict sequence of 5 waypoints across the table without triggering any joint singularities or protective stops. 

Our custom controller, completely built from scratch without MoveIt2, achieves a flawless **40/40 score**, reaching all waypoints with less than `0.03m` of error.

---

## 🏆 Final Execution Results

The custom controller has successfully achieved a flawless, continuous run across all 5 waypoints from a fresh simulation.

**Final Waypoint Accuracy:**
* **WP1:** `0.0114m` ✅
* **WP2:** `0.0054m` ✅
* **WP3:** `0.0134m` ✅
* **WP4:** `0.0113m` ✅
* **WP5:** `0.0113m` ✅

> [!TIP]
> The exact final output log matches the requirement exactly: **no CRITICAL_SINGULARITY, no protective stop, no joint-limit events, and no command-stream failures!**

---

## 🧩 Key Challenges Overcome

The main hurdle in this task was that the UR7e arm, if instructed via simple Cartesian velocity without orientation management, would naturally lock its elbow out and enter a **Critical Singularity** (Status 21/22) when reaching for far waypoints (like WP2 and WP3 at horizontal ranges > 0.8m). If it crossed $q_2 \approx 0$ (a straight elbow), the internal protective stop would latch and instantly fail the task. 

Here is how we redesigned the pipeline to achieve 100% reliability:

### 1. Granular 6-State Controller
Instead of mixing translation, rotation, and elevation, we strictly decoupled every motion into 6 distinct steps per waypoint:
1. **LIFT**: Move straight up on the Z-axis to 0.55m altitude.
2. **ROTATE**: Use the `JointJog` (`delta_joint_controller`) exclusively to spin the shoulder pan to exactly face the target.
3. **ORIENT**: *Crucial step.* Tilt the end-effector to a predetermined safety angle BEFORE starting horizontal translation, ensuring the elbow never hits 0.
4. **TRANSLATE**: Translate via `TwistStamped` (`delta_twist_controller`) strictly along the XY plane to the target radius. 
5. **DROP**: Descend purely on the Z-axis down to the exact waypoint.
6. **HOLD**: Command strict `0.0` velocities to firmly lock the tool for 2.5 seconds. 

### 2. Radial Retraction on Lift
When LIFTing from a very far waypoint (like WP3 at r=0.91m), commanding a pure Z-velocity pushes the elbow towards 0. To counter this, if the arm is extended beyond `r > 0.75m`, the LIFT step artificially commands negative radial velocity to pull the tool inwards towards the base as it rises. This acts like a human bending their elbow back into their chest before reaching for the next target, totally eliminating the lift-singularity.

### 3. Dynamic Tool Tilt 
If we kept the end-effector perfectly perpendicular to the table, WP2 and WP3 were unreachable. We added an outward tilt heuristic based on distance:
* **r > 0.78m:** Strong outward tilt to artificially stretch the arm's reach (`Z-vector = [tx, ty, -0.3]`).
* **r > 0.60m:** Gentle outward tilt (`Z-vector = [tx, ty, -1.0]`).
* **r < 0.60m:** Straight down (`Z-vector = [0.0, 0.0, -1.0]`).

### 4. The Drop Curl
When descending to a far waypoint, maintaining a strong outward tilt during the DROP step forced the arm to remain at its extreme reach limit. Instead, the DROP phase forces the tool to gently transition back to pointing straight down (`[0, 0, -1]`). As it descends, this causes the wrist to curl inwards naturally, safely bending the elbow out of the singularity zone.

### 5. `use_sim_time=True` Timestamping Fix
The arm was previously dropping commands (status 91) due to timing mismatches. We hardcoded `use_sim_time=True` directly into the Node's parameter override during `__init__`, ensuring the published `header.stamp` perfectly aligned with Gazebo's simulated clock.

### 6. Graceful Controller Switching
We moved from `done_callback` switching to continuous asynchronous polling via `SwitchController.Request.BEST_EFFORT`. This allowed the controller to publish 0-velocity dead-man packets while waiting for the controllers to activate in the background, preventing `stream stopped` warnings from latching the arm.

## 🚀 Running the Controller

```bash
# Terminal 1: Launch the simulation
ros2 launch eyantra_kepler_colony task1b.launch.py

# Terminal 2: Run our robust waypoint controller
ros2 run algorithms arm_waypoints.py
```

---
<div align="center">
  <a href="https://github.com/TejasRaut15k/e-yantra/tree/main">🔙 Return to Main Repository</a>
</div>
