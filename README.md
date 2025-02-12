# [BlueOS ROS2 Extension](https://github.com/itskalvik/blueos-ros2)

This extension enables controlling an ArduPilot-based vehicle (BlueBoat, BlueROV) via [ROS2](https://github.com/ros2).

The extension includes the following packages:

- [MAVROS](https://github.com/mavlink/mavros): Includes a launch file to communicate with the flight controller (vehicle)

- [mavros_control](https://github.com/itskalvik/mavros_control): Includes a python script to control the vehicle using GPS-based waypoints (BlueBoat) and RC-control (BlueROV2) via mavros

- [Foxglove](https://docs.foxglove.dev/docs): Web-based RViz for visualizing ROS2 data

The extension also mounts the following folder on the computer running [BlueOS](https://blueos.cloud/) to the home directory folder in the extension's Docker container, which can be used to store files that need to persist across reboots, such as a ROS2 workspace with custom code:

```plaintext
/usr/blueos/extensions/ros2/  ->  /home/persistent_ws/
```

## Getting Started
The [mavros_control](https://github.com/itskalvik/mavros_control) package includes a demo launch file that starts the `controller` node. The node arms the vehicle, moves it, and then disarms it. Run the following command in the extension's terminal to start it:

```bash
ros2 launch mavros_control demo.launch.py
```

## Parameters
You can control the following extension parameters by running the following command in the terminal provided by the ros2 extension:

```bash
export <parameter_name>=<parameter_value>
```

The parameters reset to their default values after rebooting. They can be made permanent by configuring the parameters using the app environment variables on the BlueOS extensions page in pirate mode.

### Available Parameters

* **`NAVIGATION_TYPE`** (default: `0`):
    - If `0`: Uses global position based navigation (assumes access to GPS)
    - If `1`: Uses raw RC controls for navigation

* **`FOXGLOVE`** (default: `True`):
    - Starts Foxglove bridge when set to `True`
    - You can access it from a web browser at [https://app.foxglove.dev/](https://app.foxglove.dev/). Use the open connection feature and change the address from `localhost` to the `IP address` of the vehicle

## Building the Docker Container Locally
First, setup buildx to build the containers for both arm64 and amd64 platforms:

```bash
docker buildx create --name multi-arch \
  --platform "linux/arm64,linux/amd64" \
  --driver "docker-container"
docker buildx use multi-arch
```

Next, clone the repo and build the container, replacing `<tag>` with your own tag:

```bash
git clone --recurse-submodules https://github.com/itskalvik/blueos-ros2
cd blueos-ros2
docker build --platform linux/amd64,linux/arm64 -t <tag> . --push
```

## FAQ

- **RC-Control Setup:**
  - To control the vehicle using RC-control, please ensure that the `SYSID_MYGCS` parameter is set to `1` through the BlueOS Autopilot Parameters Tab. Note that once the `SYSID_MYGCS` is changed, you won't be able to control the vehicle using Cockpit anymore until you reset the parameter to its default value: `255`.
  - **Important:** In addition to setting `SYSID_MYGCS`, ensure that the **SERVOx_Function** in the vehicle setup tab is set to **RCPassThru** in order to use RC-control.

