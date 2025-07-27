# [BlueOS ROS2 Extension](https://github.com/itskalvik/blueos-ros2)

<p align="center">
<img src=".assets/logo.png" width="50%"/> 
</p>

The BlueOS ROS2 Extension bridges the [BlueOS](https://bluerobotics.com/blueos-conversion/) and [ROS2](https://github.com/ros2) ecosystems, enabling advanced robotic applications and research on ArduPilot-based vehicles such as the BlueBoat and BlueROV2.

This extension provides drivers and tools for controlling vehicles and visualizing onboard sensor data. It simplifies the process of setting up a ROS 2 environment for marine robotics by offering pre-configured packages for sonar, cameras, and vehicle control—ready to use out of the box.

In addition, it includes a web-based terminal for convenient access to the ROS 2 environment and integrates seamlessly with Foxglove for browser-based data visualization.

# 🚀 Features

This extension includes the following ROS 2 packages:

- **[mavros_control](https://github.com/itskalvik/mavros_control)**

  A Python-based control interface using [MAVROS](https://github.com/mavlink/mavros). Supports GPS waypoint navigation (BlueBoat) and RC control (BlueROV2). Includes methods for arming/disarming, takeoff/landing, home location setting, and waypoint following. Skip the boilerplate—subclass or reuse it directly!

- **[gscam2](https://github.com/clydemcqueen/gscam2)**

  A ROS 2 camera driver for GStreamer-based video streams. Supports intra-process communication for performance.

- **[bluerobotics_sonar](https://github.com/itskalvik/bluerobotics_sonar)**

  Drivers for the Blue Robotics Ping1D altimeter and Ping360 scanning sonar. Includes nodes for hardware interfacing and data visualization.

- **[sonoptix_sonar](https://github.com/itskalvik/sonoptix_sonar)**

  Provides drivers and data processing for the Sonoptix Echo sonar. Captures raw data and converts it into polar images for visualization.

- **[ros-foxglove-bridge](https://app.foxglove.dev/)**

  A WebSocket-based bridge to visualize ROS 2 data in the Foxglove web interface. Compatible with the BlueOS Foxglove plugin.

## 📋 Prerequisites

- A 64-bit version of [BlueOS](https://github.com/bluerobotics/BlueOS) is required. 
Get the latest image for Raspberry Pi from the [BlueOS releases](https://github.com/bluerobotics/BlueOS/releases/).


## 🧰 Installation
You can install the ROS 2 Extension directly from the BlueOS App Store.

## ⚙️ Usage
- BlueOS automatically launches the extension on boot.

- The extension's terminal is accessible from the left-hand panel of the BlueOS interface.

- The extension mounts the host directory ```/usr/blueos/extensions/ros2/``` to the container path ```/home/persistent_ws/```
  Use this folder to store files that should persist across reboots, such as custom ROS 2 workspaces or configurations.

## 🛠️ Building the Docker Container Locally
To build the container for multiple architectures (`arm64`, `amd64`), follow these steps:

### 1. Set up a multi-architecture builder:

```bash
docker buildx create --name multi-arch \
  --platform "linux/arm64,linux/amd64" \
  --driver "docker-container"
docker buildx use multi-arch
```

### 2. Clone the repository and build the container:

```bash
git clone --recurse-submodules https://github.com/itskalvik/blueos-ros2
cd blueos-ros2
docker compose build
```