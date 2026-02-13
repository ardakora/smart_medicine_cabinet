# smart_medicine_cabinet

# Smart Medicine Cabinet (ROS 2)

This project implements a smart medicine cabinet system using **ROS 2 Jazzy** and **Python**.
The system combines camera-based scanning and RFID identification to assist with medicine
recognition and monitoring.

The project is developed and tested on **Raspberry Pi 5** running **Ubuntu**.

---

## Overview

The system is composed of multiple ROS 2 Python nodes running concurrently.
A camera stream is visualized using **RQT** to improve the readability of medicine codes,
while RFID is used for identification.

---

## Features

- ROS 2 Python package structure
- Camera node for real-time image acquisition
- Scanner node for medicine code detection
- RFID node for tag-based identification
- RQT visualization for camera output
- Designed for embedded Linux (Raspberry Pi)

---

## System Requirements

- Ubuntu (tested on Raspberry Pi 5)
- ROS 2 Jazzy
- Python 3
- Camera module compatible with ROS 2
- RFID module (MFRC522)

---

## Build Instructions

Source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
````

Build the workspace:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

---

## Running the System

The system is designed to run using **four separate terminals**.

### Terminal 1 – Scanner Node

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run my_camera_pkg scanner_node
```

### Terminal 2 – Camera Node

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run my_camera_pkg cam_node
```

### Terminal 3 – RQT Interface

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rqt
```

RQT is used to visualize the camera stream and improve the readability of medicine codes.

### Terminal 4 – RFID Node

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run my_camera_pkg rfid_node
```

---

## External Libraries

This project uses the following external libraries:

* **libdtmx**
  Used for camera-related data handling and processing.

* **MFRC522-python** (by mxgxw)
  Used for RFID communication and tag reading.
  [https://github.com/mxgxw/MFRC522-python](https://github.com/mxgxw/MFRC522-python)

All other dependencies (such as `spidev` and ROS 2 core packages) are considered standard
and are assumed to be available in the ROS 2 environment.

---

## Project Status

This project is under active development and is intended for educational and experimental use.

---

## License

This project is licensed under the MIT License.
