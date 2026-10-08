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

# Python MAVLink dependency for GCS script
pip install pymavlink
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

# Python MAVLink dependency for GCS script
pip install pymavlink
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

## 3. Full Pipeline Launch Commands (Gazebo SITL)

To run the full autonomous mission in Gazebo SITL with ArduPilot, open **6 separate terminals**:

**Terminal 1: Start Gazebo Simulator**
```bash
source ~/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros2_ws/src/ardupilot_gazebo/models:$HOME/ros2_ws/src/addc/addc_autonomy/models
gz sim -r -v 4 ~/ros2_ws/src/addc/addc_autonomy/worlds/addc_arena.world
```
*(Leave this running. It hosts the 3D physics and camera).*

**Terminal 2: Start ArduPilot SITL**
```bash
cd ~/ros2_ws/src/addc
./launch_ardupilot_sitl.sh
```
*(Connects the Pixhawk firmware to Gazebo).*

**Terminal 3: Ground Control Station (GCS) - Generate & Upload Mission**
```bash
cd ~/ros2_ws/src/addc
python3 search_area_setup.py
```
*(A GUI will open. Enter your telemetry connection string (e.g., `tcp:127.0.0.1:5760` for SITL), load your Mission Planner `.poly` boundary, and click "Generate & Upload". This dynamically calculates the optimal FOV-based lawnmower grid and injects it into the Pixhawk).*

**Terminal 4: Launch the ROS 2 Autonomy Stack**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch addc_autonomy sim_mission.launch.py
```
*(Starts MAVROS, Orchestrator, Search, Vision, Landing, and HMI Bridge. It automatically detects the SITL environment, bypasses physical hardware drivers, and auto-ingests the GPS mission uploaded in Terminal 3).*

**Terminal 5: Launch the Flutter HMI (Runner Dashboard)**
```bash
cd ~/ros2_ws/src/addc/addc_hmi
flutter run -d web-server
```

**Terminal 6: View the Drone's Live Camera Feed**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
*(Select the base `/camera` topic from the dropdown, NOT the compressed one).*

---

## 4. Physical Flight Bringup (Raspberry Pi + Pixhawk)

### Step 1: On the Ground Control Station (Laptop)
1. Draw your flight boundary in Mission Planner and save the `.poly` file.
2. Run the GCS script to generate the exact FOV footprint and upload the mission to the drone via Telemetry:
   ```bash
   python3 ~/ros2_ws/src/addc/search_area_setup.py
   ```
3. Enter your radio's COM port (e.g., `COM3,57600` for Windows or `/dev/ttyUSB0,57600` for Linux) and click Upload.

### Step 2: On the Drone (Raspberry Pi SSH)
```bash
# Single command launch for all flight nodes
source ~/ros2_ws/install/setup.bash
ros2 launch addc_autonomy flight_bringup.launch.py enable_gui:=false
```
*(The physical launch file automatically disables `use_sim_time`, activating the physical PiCamera2 module and the Canny Edge precision landing hardware routines).*

---

## 5. MAVROS Communication Diagnostic Commands

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
