# 🚁 EV-Pose: Event camera-enhanced Visual Positioning Service

[![ROS](https://img.shields.io/badge/ROS-Melodic-blue.svg)](#)
[![C++](https://img.shields.io/badge/C++-14-green.svg)](#)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](#)

**EV-Pose** is the first event camera-enhanced Visual Positioning Service (VPS) designed to deliver drift-free, absolute global coordinates at ultra-high frequencies for drones. This repository provides the reference implementation for evaluation.

> **⚠️ Implementation Note (Please Read First):** >  Please be aware that this code is actively used for academic research and is subject to further optimization and changes.

---

## 📦 1. Prerequisites

We strongly recommend using **Ubuntu 18.04 / 20.04** with a properly configured **ROS** environment.

### Quick Installation via `apt` (Recommended)
For Debian/Ubuntu-based systems, you can install most of the required dependencies using the command line:

```bash
sudo apt-get update
# Install OpenCV and Eigen3
sudo apt-get install libopencv-dev python3-opencv libeigen3-dev
# Install PCL (Point Cloud Library)
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-dev
```

### Install Pangolin (From Source)
Pangolin is required for trajectory and point cloud visualization. Please install it from the source:

```bash
git clone [https://github.com/stevenlovegrove/Pangolin.git](https://github.com/stevenlovegrove/Pangolin.git)
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
*(Note: If you encounter issues or prefer custom builds for OpenCV/PCL, please refer to their respective official documentation.)*

---

## ⚙️ 2. Build the Project

Clone the repository and build the ROS workspace using the provided script:

```bash
# Clone the repository
git clone [https://github.com/xxxx/EV-Pose.git](https://github.com/xxxx/EV-Pose.git)
cd EV-Pose

# Grant execution permission and build
chmod +x build_all.sh
source build_all.sh
```

---

## 📝 3. Configuration Guide

Before running the evaluation, you must configure the parameters in your `.yaml` config file. Below are the key parameters you need to modify:

| Parameter | Description |
| :--- | :--- |
| `pcd_path` | The absolute path to your global 3D point cloud map directory or file. |
| `bag_path` | The absolute path to the ROS bag containing the event stream and IMU data. |
| `start_time` | The starting timestamp to initialize the pose. It represents the timestamp of the first shot when building the map. *(Note: We provide a series of `start_time` reference values in the `config/` folder for public datasets).* |

---

## 🏃 4. Run EV-Pose

We take the **VECtor dataset** as an example. Ensure your ROS environment is sourced correctly before proceeding.

**Step 1: Start the ROS master**
Open a new terminal and run:
```bash
roscore
```

**Step 2: Launch EV-Pose**
Open another terminal, navigate to your workspace, and run the main node with your configured yaml file:
```bash
rosrun demo run /path/to/your/config.yaml
```

**Step 3: Terminate the program**
To forcefully stop the evaluation, you can use:
```bash
killall -9 run
```

