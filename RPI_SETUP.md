# Raspberry Pi 4B/5 Hardware Setup Guide

This document is the definitive guide for setting up the physical Raspberry Pi companion computer from a completely blank SD card to a fully flight-ready state. **There is NO Gazebo or Simulation installed here—this is pure hardware.**

---

## 1. Operating System & UART Configuration
1. Flash **Ubuntu 24.04 Server (64-bit)** onto your SD card using the Raspberry Pi Imager.
2. Boot the Pi, connect to Wi-Fi/Ethernet, and update packages:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```
3. Enable the Hardware UART and I2C/CSI Camera Interfaces by editing the boot config:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
   Add the following lines at the bottom:
   ```text
   # Enable UART for Pixhawk Telemetry
   enable_uart=1
   dtoverlay=uart0

   # Enable Arducam IMX296 Camera
   camera_auto_detect=0
   dtoverlay=imx296
   ```
4. Add your user to the `dialout` and `video` groups to grant permissions to the Serial port and Camera:
   ```bash
   sudo usermod -aG dialout $USER
   sudo usermod -aG video $USER
   ```
5. Reboot the Pi: `sudo reboot`

---

## 2. ROS 2 Jazzy & MAVROS Installation
Install the ROS 2 Jazzy base and MAVROS packages.
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    ros-jazzy-cv-bridge \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-pyzbar \
    python3-shapely \
    python3-fastapi \
    python3-uvicorn \
    python3-requests

# Install MAVROS GeographicLib Datasets (CRITICAL)
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```
Source ROS 2 in your bashrc:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Picamera2 Installation (Arducam Support)
The ADDC vision stack bypasses V4L2 and directly uses libcamera via the official `Picamera2` Python library for maximum performance on global shutter cameras.

```bash
sudo apt install -y python3-libcamera python3-kmsxx libcamera-tools
pip install git+https://github.com/raspberrypi/picamera2.git
```
*Test the camera connection:*
```bash
libcamera-hello --list-cameras
```
*(You should see the imx296 sensor listed).*

---

## 4. Tailscale VPN Installation (HMI Bridge)
The Flutter App (Runner Phone) talks to the Pi over Tailscale.
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```
*(Log in and note the Pi's Tailscale IP Address).*

---

## 5. Clone and Build ADDC Autonomy
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <YOUR_REPO_URL> addc
cd ~/ros2_ws
colcon build --symlink-install --packages-select addc_autonomy
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. Physical Wiring (Pixhawk to RPi)
Connect the Pi to the Pixhawk 6C's `TELEM2` port using a standard JST-GH to Dupont cable:
*   **Pixhawk TELEM2 TX** -> **RPi RX (Pin 10 / GPIO 15)**
*   **Pixhawk TELEM2 RX** -> **RPi TX (Pin 8 / GPIO 14)**
*   **Pixhawk GND** -> **RPi GND (Pin 6)**

*Do NOT connect the 5V VCC pin between them. Power the Pi and Pixhawk independently.*

Your Raspberry Pi is now flight-ready! Proceed to run the physical tests located in `COMMANDS.md`.
