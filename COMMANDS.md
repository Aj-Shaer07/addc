# SAEISS ADDC - Complete Commands & Operations Guide

This document lists all commands to build, simulate, test, and run the ADDC autonomous flight software stack on **Ubuntu 24.04 (ROS 2 Jazzy)**, **Ubuntu 22.04 (ROS 2 Humble)**, and the **Raspberry Pi 4B/5**.

---

## 1. System Setup & Dependencies

### Ubuntu 24.04 LTS (ROS 2 Jazzy) - Laptop & Raspberry Pi
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

# Source ROS 2 Jazzy globally
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. Workspace Build Commands

```bash
# Navigate to your colcon workspace (e.g. ~/ros2_ws)
cd ~/ros2_wssudo apt update
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


# Build the autonomy package
colcon build --symlink-install --packages-select addc_autonomy

# Source the overlay in your terminal
source install/setup.bash
```

---

## 3. Automated Test Execution

### A. Direct Python Unit Tests (SITL / Zero Overhead)
Run these anywhere without running ROS 2 daemons or Gazebo:
```bash
# Vision detection, 4-tier cascade & centroid error test
python3 src/addc/addc_autonomy/test/test_sitl_qr_vision.py

# Path planner, boundary containment & tree obstacle avoidance test
python3 src/addc/addc_autonomy/test/test_sitl_search_grid.py

# HMI REST portal & runner ROI preemption test
python3 src/addc/addc_autonomy/test/test_sitl_roi_preemption.py

# Precision Landing state machine and fallback logic test
python3 src/addc/addc_autonomy/test/test_sitl_precision_landing.py

# Mission Control Orchestrator state machine test
python3 src/addc/addc_autonomy/test/test_sitl_mission_control.py
```

### B. Hardware-In-The-Loop (HITL) Physical Raspberry Pi Tests
Run these strictly on the physical Raspberry Pi while it is connected to the Pixhawk and Arducam.

**1. Vision & Arducam Test**
*(Verifies OpenCV detects the physical QR code through the IMX296 sensor)*
```bash
# Terminal 1: Start the camera driver with GUI
ros2 run addc_autonomy qr_ros --ros-args -p use_sim_time:=false -p enable_debug_window:=true

# Terminal 2: Run the test
python3 src/addc/addc_autonomy/test/test_rpi_camera.py
```

**2. UART Telemetry / Pixhawk Test**
*(Verifies the Pi is receiving MAVLink heartbeats & IMU on `/dev/ttyAMA0`)*
```bash
# Terminal 1: Start MAVROS
ros2 launch mavros apm.launch fcu_url:=/dev/ttyAMA0:921600

# Terminal 2: Run the test
python3 src/addc/addc_autonomy/test/test_rpi_telemetry.py
```

**3. Precision Landing / Visual Servoing Test**
*(Verifies the Canny Edge detector locks onto the physical pad and outputs `cmd_vel`)*
```bash
# Terminal 1: Start the landing node with camera
ros2 run addc_autonomy precision_landing --ros-args -p use_sim_time:=false -p enable_debug_window:=true

# Terminal 2: Run the test
python3 src/addc/addc_autonomy/test/test_rpi_precision_landing.py
```

**4. Ground-to-Air Telemetry Mission Ingestion Test**
*(Verifies the Pi downloads the mission sent from your laptop)*
```bash
# Terminal 1: Start the search node
ros2 run addc_autonomy search_node --ros-args -p use_sim_time:=false

# Terminal 2: Run the test
python3 src/addc/addc_autonomy/test/test_rpi_search_ingestion.py
```

**5. Tailscale & Flutter HMI Test**
*(Verifies the Android App can hit the Pi over the mesh VPN)*
```bash
# Terminal 1: Start the HMI Bridge
ros2 run addc_autonomy hmi_bridge

# Terminal 2: Run the test
python3 src/addc/addc_autonomy/test/test_rpi_hmi_roi.py
```

---

## 4. Full Pipeline Launch Commands (Gazebo SITL)

To run the full autonomous mission in Gazebo SITL with ArduPilot, open **6 separate terminals**:

**Terminal 1: Start Gazebo Simulator**
```bash
source ~/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/install/ardupilot_gazebo/share:$HOME/ros2_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$HOME/ros2_ws/src/addc/addc_autonomy/models:${GZ_SIM_RESOURCE_PATH}
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo:${GZ_SIM_SYSTEM_PLUGIN_PATH}
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

## 5. Physical Flight Bringup (Raspberry Pi + Pixhawk)

### Step 1: On the Ground Control Station (Laptop)
1. Connect to the drone via Mission Planner on your standard COM port.
2. Press `Ctrl + F`, click **Mavlink**, and open a **UDP Client** on `127.0.0.1:14550`.
3. Run the GCS script to generate the exact FOV footprint and tunnel it to the drone:
   ```bash
   python3 ~/ros2_ws/src/addc/search_area_setup.py
   ```
4. Enter `udp,14550` as your connection string and click Upload. Mission Planner will instantly forward the waypoints to the drone!

### Step 2: On the Drone (Raspberry Pi SSH)
```bash
# High-Performance Competition Launch (Mutes high-frequency ROS 2 logs, leaves critical phases on)
source ~/ros2_ws/install/setup.bash
ros2 launch addc_autonomy flight_bringup.launch.py enable_gui:=false competition_mode:=true

# Standard Testing Launch (Shows full verbose logs for debugging)
ros2 launch addc_autonomy flight_bringup.launch.py enable_gui:=false competition_mode:=false
```
*(The physical launch file automatically enables the Fault-Tolerant Watchdog heartbeat, dynamically pulls waypoints, and uses PyZBar via the Picamera2 module).*

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
