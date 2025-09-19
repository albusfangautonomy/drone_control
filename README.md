# PX4 + Gazebo (SITL) + ROS 2 Bridge Quickstart

This guide brings up **PX4 SITL** with **Gazebo**, exposes PX4 topics to **ROS 2** via **Micro XRCE-DDS**, and starts a **MAVLink** UDP stream to a Ground Control Station (GCS) or your own tool.

---

## Prerequisites

- Ubuntu 22.04 (recommended)
- Git, CMake, Python 3, build-essential
- ROS 2 (Humble or newer) installed and sourced
- Gazebo (pick one):
  - **Modern Gazebo** (a.k.a. Ignition/Garden/Harmonic)
  - **Gazebo Classic** (legacy; still widely used with PX4 ≤ v1.13)
- Micro XRCE-DDS Agent
- PX4 toolchain & deps

### Install PX4 deps (Ubuntu 22.04)
```bash
# Clone PX4
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# PX4 setup script (installs sim deps, Gazebo bindings, etc.)
bash ./Tools/setup/ubuntu.sh
```
## Install Micro XRCE-DDS Agent

1. Using apt (common):
```
sudo apt-get update
sudo apt-get install micro-xrce-dds-agent
```
Now you should have `MicroXRCEAgent` on PATH

## From the PX4-Autopilot root:
```
make px4_sitl gz_x500
MicroXRCEAgent udp4 -p 8888
```

## inside pxh>
```
mavlink start -x -u 14550 -t 10.0.0.169
```

---

## Typical Data Flows

### PX4 uORB → ROS 2 subscriber
```
PX4 (uORB) → Micro XRCE client → UDP:8888 → MicroXRCEAgent → DDS → ROS 2 topic (/fmu/out/*)
```

### ROS 2 publisher → PX4 uORB
```
ROS 2 topic (/fmu/in/*) → DDS → MicroXRCEAgent → UDP:8888 → PX4 Micro XRCE client → uORB
```

### PX4 → GCS (MAVLink)
```
PX4 MAVLink → UDP 14550 → Target IP (e.g., QGC)
```