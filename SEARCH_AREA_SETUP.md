# Ground Control Station: Dynamic Search Grid Setup

This guide explains how to use the `search_area_setup.py` tool to dynamically draw a boustrophedon (lawnmower) search grid and instantly beam it to the drone via Mission Planner, **without disconnecting your live telemetry.**

## 1. Prerequisites
- The drone is powered on and connected to Mission Planner via the physical telemetry radio on COM5.
- You have Python 3 installed on your Windows laptop.
- Install dependencies on your laptop:
  ```bash
  pip install pymavlink
  ```

## 2. Bypassing the Disconnect Delay (UDP Tunneling)
Mission Planner can only connect to one COM port at a time. However, it can act as a **Router** to tunnel commands from our Python script directly to the drone in the sky.

1. **Connect to the Drone:** Open Mission Planner and connect to COM5 (57600) as usual.
2. **Open the Developer Menu:** Press `Ctrl + F` on your keyboard.
3. **Start the Tunnel:** Click the **Mavlink** button.
4. **Configure the Tunnel:**
   - Select **UDP Client**.
   - IP: `127.0.0.1`
   - Port: `14550`

Mission Planner is now listening for our Python script while simultaneously streaming live telemetry and logs from the drone!

## 3. Running the Generator
Open a Windows Terminal and run the setup script:
```bash
python3 src/addc/search_area_setup.py
```

### GUI Inputs:
1. **Connection String:** Type `udp,14550` (Do NOT type COM5).
2. **FOV:** Set your camera's Field of View (e.g., 62 for Picamera2).
3. **Altitude:** Set the cruise altitude (e.g., 3.0m).
4. **Draw Grid:** Click exactly 4 points on the map to define your competition search arena.
5. **Upload:** Click the "Upload" button.

### What Happens Next:
1. The script automatically requests the drone's active `HOME_POSITION` through the tunnel.
2. It sets Mission Item #1 (`MAV_CMD_NAV_TAKEOFF`) exactly at the Home Position.
3. It generates a 100% overlapping search grid that entirely avoids the `tree_obstacles` defined in the code.
4. It beams the 50+ waypoints through Mission Planner directly into the Pixhawk's RAM in < 0.5 seconds.
5. The `search_node` on the Raspberry Pi will automatically detect the upload and pull the fresh waypoints into its autonomy engine.

You are now instantly ready to launch the mission!
