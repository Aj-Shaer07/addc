# SAEISS ADDC - Complete Commands & Operations Guide

This document lists all commands to build, simulate, test, and run the ADDC autonomous flight software stack on **Ubuntu 24.04 (ROS 2 Jazzy)**, **Ubuntu 22.04 (ROS 2 Humble)**, and the **Raspberry Pi 4B/5**.

---

## 1. System Setup & Dependencies

### Ubuntu 24.04 LTS (ROS 2 Jazzy)
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-ros-gz \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-shapely \
    python3-fastapi \
    python3-uvicorn \
    python3-requests

# Geographic datasets required by MAVROS
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

### Ubuntu 22.04 LTS (ROS 2 Humble / Raspberry Pi)
```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-cv-bridge \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-shapely \
    python3-fastapi \
    python3-uvicorn \
    python3-requests

# Geographic datasets required by MAVROS
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

---

## 2. Workspace Build Commands

```bash
# Navigate to your colcon workspace (e.g. ~/ros2_ws)
cd ~/ros2_ws

# Build the autonomy package
colcon build --symlink-install --packages-select addc_autonomy

# Source the overlay in your terminal
source install/setup.bash
```

---

## 3. Automated Test Execution

### Direct Python Unit Tests (Zero Sim/Robot Overhead)
Run these anywhere without running ROS 2 daemons or Gazebo:
```bash
# Vision detection, 4-tier cascade & centroid error test
python3 src/addc/addc_autonomy/test/test_sitl_qr_vision.py

# Path planner, boundary containment & tree obstacle avoidance test
python3 src/addc/addc_autonomy/test/test_sitl_search_grid.py

# HMI REST portal & runner ROI preemption test
python3 src/addc/addc_autonomy/test/test_sitl_roi_preemption.py
```

### Via Colcon Test Runner
```bash
colcon test --packages-select addc_autonomy
colcon test-result --all --verbose
```

---

## 4. Standalone Node Execution (CLI Commands)

### A. Vision Node (`qr_ros`)
* **In Gazebo Simulation (subscribes to `/camera/image_raw`):**
  ```bash
  ros2 run addc_autonomy qr_ros --ros-args -p use_sim_time:=true -p enable_debug_window:=true
  ```
* **On Physical Raspberry Pi (opens `Picamera2` directly):**
  ```bash
  # Headless flight (No GUI, zero crashes, maximum CPU savings)
  ros2 run addc_autonomy qr_ros --ros-args -p use_sim_time:=false -p enable_debug_window:=false

  # Ground testing with RealVNC display enabled:
  ros2 run addc_autonomy qr_ros --ros-args -p use_sim_time:=false -p enable_debug_window:=true
  ```

### B. Search Node (`search_node`)
* **In Simulation:**
  ```bash
  ros2 run addc_autonomy search_node --ros-args -p use_sim_time:=true -p search_altitude:=3.0 -p lane_spacing:=2.0
  ```
* **On Physical Raspberry Pi:**
  ```bash
  ros2 run addc_autonomy search_node --ros-args -p use_sim_time:=false -p search_altitude:=3.0 -p lane_spacing:=2.0
  ```

### C. Master Orchestrator (`mission_control`)
* **In Simulation:**
  ```bash
  ros2 run addc_autonomy mission_control --ros-args -p use_sim_time:=true
  ```
* **On Physical Raspberry Pi:**
  ```bash
  ros2 run addc_autonomy mission_control --ros-args -p use_sim_time:=false
  ```

### D. Precision Landing Node (`precision_landing`)
* **In Simulation:**
  ```bash
  ros2 run addc_autonomy precision_landing --ros-args -p use_sim_time:=true
  ```
* **On Physical Raspberry Pi:**
  ```bash
  ros2 run addc_autonomy precision_landing --ros-args -p use_sim_time:=false
  ```

### E. HMI Bridge Node (`hmi_bridge`)
* **Starts the Runner Phone Web Portal on port 5000:**
  ```bash
  ros2 run addc_autonomy hmi_bridge --ros-args -p http_port:=5000
  ```
  *(Access via runner smartphone browser: `http://<tailscale-ip>:5000`)*

---

## 5. Full Pipeline Launch Commands

### Simulation Full Launch (Gazebo SITL)
```bash
# 1. Start Gazebo with ADDC Arena world
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/addc/addc_autonomy/models
gazebo --verbose ~/ros2_ws/src/addc/addc_autonomy/worlds/addc_arena.world

# 2. In another terminal, launch all autonomy nodes together
source ~/ros2_ws/install/setup.bash
ros2 launch addc_autonomy sim_mission.launch.py enable_gui:=true search_altitude:=3.0
```

### Physical Flight Bringup (Raspberry Pi + Pixhawk)
```bash
# Single command launch for all flight nodes
source ~/ros2_ws/install/setup.bash
ros2 launch addc_autonomy flight_bringup.launch.py enable_gui:=false search_altitude:=3.0
```

---

## 6. MAVROS Communication Diagnostic Commands

Check if Pixhawk and telemetry links are communicating properly:
```bash
# Check active ROS 2 topics
ros2 topic list

# Echo current drone flight state (Armed, GUIDED mode, etc.)
ros2 topic echo /mavros/state

# Echo current drone local coordinates (X, Y, Z altitude)
ros2 topic echo /mavros/local_position/pose

# Echo visual target offset errors from camera
ros2 topic echo /addc/vision/target_offset

# Echo decoded access digits
ros2 topic echo /addc/vision/decoded_digits
```
