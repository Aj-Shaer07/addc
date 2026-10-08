# Gazebo SITL Simulation Guide (ADDC Autonomy)

This document is the definitive guide to setting up and running the Gazebo Harmonic (SITL) simulation environment for the ADDC Autonomy stack on **Ubuntu 24.04 (ROS 2 Jazzy)**. 

If you are cloning this repository for the first time on a fresh Linux machine, follow every step precisely.

---

## 1. Core Prerequisites Installation

### ROS 2 Jazzy & Gazebo Harmonic
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
    python3-requests \
    git \
    curl
```

### Create Workspace & Clone ADDC Repository
You must place this repository inside a standard ROS 2 workspace `src` folder:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone this repository (ADDC) into the src folder
git clone <YOUR_ADDC_REPO_URL> addc
```

### Install ArduPilot SITL
Follow the official ArduPilot documentation to install the SITL environment:
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

### Install `ardupilot_gazebo` Plugin
This plugin bridges the ArduPilot SITL firmware with the Gazebo Harmonic physics engine.
```bash
cd ~/ros2_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
sudo make install
```

---

## 2. Common Errors & Troubleshooting

When setting up this simulation for the first time, you may encounter several critical errors. Here is exactly how to fix them:

### Error 1: MAVROS Crash (`GeographicLib exception: File not found`)
**Symptom:** When you run `sim_mission.launch.py`, the `mavros_node` immediately crashes complaining about missing geoid datasets.
**The Fix:** You must install the GeographicLib datasets for MAVROS manually:
```bash
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

### Error 2: Ground Control Script Crash (`No module named 'serial'`)
**Symptom:** When trying to run `search_area_setup.py`, Python throws an `ImportError` for `serial`.
**The Fix:** Even though the library is called `pymavlink`, it strictly depends on `pyserial`. Install both:
```bash
pip install pyserial pymavlink
```

### Error 3: MAVROS Odometry Drops (QoS Mismatch)
**Symptom:** You echo `/mavros/local_position/pose` and see data, but the `precision_landing_node` or `search_node` receives absolutely nothing.
**The Fix:** MAVROS publishes odometry using `BEST_EFFORT` reliability. Standard ROS 2 subscribers default to `RELIABLE`. This causes a QoS mismatch and ROS 2 silently drops the packets. 
*Note: This is already patched in our repository! Our nodes explicitly subscribe using `qos_profile_sensor_data`.*

### Error 4: Drone attempts to fly to Space (Altitude 584m)
**Symptom:** During the global search, the drone suddenly pitches up and attempts to fly to `Z=584.3` meters instead of `3.0` meters.
**The Cause:** In Gazebo, the "ground level" for ArduPilot SITL defaults to roughly 584 meters Above Mean Sea Level (AMSL). If a MAVLink mission is uploaded poorly, the drone interprets the GPS AMSL altitude as a Local ENU coordinate.
**The Fix:** *This is already patched in `search_node.py`.* Our code explicitly intercepts the MAVLink mission, ignores Sequence 0 (Home), and dynamically overrides the Z-coordinate with the `search_altitude` ROS parameter.

### Error 5: Gazebo Model Missing (`[GUI] [Err] ... model.sdf`)
**Symptom:** Gazebo launches, but the drone is invisible or throws plugin errors.
**The Fix:** You must export the ROS 2 workspace paths to Gazebo so it can find the `iris_with_ardupilot` mesh and our custom ADDC models (landing pad, cache).
Add this to your `~/.bashrc`:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros2_ws/src/ardupilot_gazebo/models:$HOME/ros2_ws/src/addc/addc_autonomy/models
```

---

## 3. Running the Simulation

Once the prerequisites are installed and the errors are patched, follow the execution guide in **[COMMANDS.md](COMMANDS.md)** to run the 6 terminal pipeline!
