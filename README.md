# SAEISS ADDC Autonomous Drone Reconnaissance & HMI System

Autonomy software stack for the SAEISS Autonomous Drone Development Challenge (ADDC). Designed for **ROS 2 Humble** and **ArduPilot (Pixhawk 6C Mini + Raspberry Pi 4B/5)** with full **Gazebo SITL simulation** support.

---

## Architecture Overview

```
[ Runner Phone ] (Tailscale Mesh VPN)
       │
       │ HTTP POST /set_roi (Priority Region of Interest)
       ▼
[ hmi_bridge_node ] (FastAPI / HTTP Server on port 5000)
       │ Topic: /addc/hmi/priority_roi
       ▼
[ search_node ] (Boustrophedon Sweep + Obstacle Avoidance + ROI Stack)
       │ Topic: /addc/search/waypoint
       ▼
[ mission_control_node ] (Master FSM Orchestrator)
       │ MAVROS Setpoints (/mavros/setpoint_position/local, /mavros/setpoint_velocity/cmd_vel)
       ▼
[ Pixhawk 6C Mini / ArduPilot SITL ] (GUIDED Flight Mode)
       ▲
       │ /addc/vision/target_offset (ex, ey, lock=1.0)
       │ /addc/vision/decoded_digits (2 digits)
       ▼
[ qr_ros ] (Picamera2 / Gazebo camera + 4-tier cascaded OpenCV decode)
       │
       ▼ (Once target acquired & RTL triggered)
[ precision_landing_node ] (Centering & two-stage descent down to LAND touchdown)
```

---

## Repository Structure

```
├── addc_autonomy/                  # Main ROS 2 package
│   ├── addc_autonomy/              # Python node modules
│   │   ├── __init__.py
│   │   ├── qr_ros.py               # Vision node: Picamera2 + Gazebo + 4-tier decode
│   │   ├── search_node.py          # Lawnmower grid + tree avoidance + ROI stack
│   │   ├── mission_control_node.py # Master FSM + visual servoing centering
│   │   ├── precision_landing_node.py # Vision-guided pad recovery & descent
│   │   └── hmi_bridge_node.py      # Runner phone Tailscale web endpoint
│   ├── launch/
│   │   ├── sim_mission.launch.py   # Gazebo SITL simulation launch
│   │   └── flight_bringup.launch.py# Real hardware flight launch
│   ├── worlds/
│   │   └── addc_arena.world        # Gazebo simulation world with pad, cache, trees
│   ├── models/                     # Gazebo 3D SDF models
│   │   ├── landing_pad/
│   │   ├── intelligence_cache/
│   │   └── tree_obstacle/
│   ├── test/                       # Unit & SITL integration test suite
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── scripts/
│   ├── start_mission.sh            # One-click master flight launcher
│   └── mission_planner_setup.py    # GCS pre-flight boundary/obstacle tool
└── initial_codes/                  # Legacy reference scripts
```

---

## Setup on Ubuntu Laptop (Testing & Simulation)

### Supported Environments
- **Ubuntu 24.04 LTS**: Native **ROS 2 Jazzy Jalisco** + **Gazebo Harmonic**
- **Ubuntu 22.04 LTS**: Native **ROS 2 Humble Hawksbill** + **Gazebo Garden/Fortress**

### 1. Prerequisites (ROS 2 Jazzy on Ubuntu 24.04)
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
    python3-shapely

# Install geographic datasets required by MAVROS
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Install Python HMI and networking utilities
sudo apt install -y python3-fastapi python3-uvicorn python3-requests
```

*(Note for Ubuntu 22.04: Replace `jazzy` with `humble` in the package names above).*

### 2. Clone and Build
```bash
# In your ROS 2 workspace (e.g. ~/ros2_ws/src)
cd ~/ros2_ws/src
git clone <YOUR_REPO_URL>
cd ~/ros2_ws

# Build the package
colcon build --symlink-install --packages-select addc_autonomy
source install/setup.bash
```

### 3. Gazebo Simulation Setup (ardupilot_gazebo)
Ensure you have `ardupilot_gazebo` installed per [ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo):
```bash
# Export the package models to Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/addc_autonomy/models

# Run Gazebo with the ADDC Arena world
gazebo --verbose ~/ros2_ws/src/addc_autonomy/worlds/addc_arena.world
```

---

## Testing Matrix

Run the automated test suite locally:
```bash
# In ~/ros2_ws
colcon test --packages-select addc_autonomy
colcon test-result --all --verbose
```

---

## Deployment on Physical Drone (Raspberry Pi 4B/5 + Pixhawk 6C)

1. Connect Raspberry Pi 5 to Pixhawk 6C via `TELEM2` UART (`/dev/ttyAMA0` @ 921,600 baud).
2. Install Tailscale on the Raspberry Pi and the runner's smartphone:
   ```bash
   curl -fsSL https://tailscale.com/install.sh | sh
   sudo tailscale up
   ```
3. Run the one-click flight bringup:
   ```bash
   bash scripts/start_mission.sh
   ```
   *(For ground testing with RealVNC display: `bash scripts/start_mission.sh --gui`)*
