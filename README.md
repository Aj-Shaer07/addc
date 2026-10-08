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
│   └── start_mission.sh            # One-click master flight launcher
├── search_area_setup.py            # GCS GUI tool for generating dynamic search grids
└── initial_codes/                  # Legacy reference scripts
```

---

## Documentation & Simulation

- **[COMMANDS.md](COMMANDS.md)**: A complete, copy-paste-friendly cheat sheet for all colcon, ros2, and Flutter run commands.
- **[GAZEBO.md](GAZEBO.md)**: The comprehensive guide for setting up ArduPilot SITL, Gazebo Harmonic, and troubleshooting simulation issues from scratch.

---

## Deployment on Physical Drone (Raspberry Pi 4B/5 + Pixhawk 6C)

1. Connect Raspberry Pi to Pixhawk via `TELEM2` UART (`/dev/ttyAMA0` @ 921,600 baud).
2. Install Tailscale on the Raspberry Pi and the runner's smartphone:
   ```bash
   curl -fsSL https://tailscale.com/install.sh | sh
   sudo tailscale up
   ```
3. Run the GCS GUI on your laptop (`python3 search_area_setup.py`) to upload the boundary mission.
4. Run the one-click flight bringup on the Raspberry Pi:
   ```bash
   bash scripts/start_mission.sh
   ```
   *(For ground testing with RealVNC display: `bash scripts/start_mission.sh --gui`)*
