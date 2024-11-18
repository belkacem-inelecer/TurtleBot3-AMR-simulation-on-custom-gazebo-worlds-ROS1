# TurtleBot3-AMR-simulation-on-custom-gazebo-worlds-ROS1

## Overview

This project simulates the **TurtleBot3 Autonomous Mobile Robot (AMR)** in custom environments using **Gazebo** and **ROS Noetic**. The goal is to perform **SLAM (Simultaneous Localization and Mapping)** in the **Willow Garage world**, collect occupancy grid maps, and visualize the data.

---

## Features
- Simulate TurtleBot3 in Gazebo with a custom **Willow Garage world**.
- Perform **SLAM** using the **GMapping algorithm**.
- Save generated maps as `.pgm` and `.yaml` files.
- Teleoperate the robot manually using a keyboard.
- Customize simulation parameters for specific needs.

---

## Prerequisites

### 1. Install ROS Noetic
Follow the [ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation) for your operating system.

### 2. Install Required Packages
Install the necessary ROS packages for TurtleBot3 and Gazebo:

```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-navigation
```

### 3. Clone and Build the Workspace
Set up the workspace and clone this repository:

```bash
mkdir -p ~/Desktop/process_LiDAR_data/project_garage/src
cd ~/Desktop/process_LiDAR_data/project_garage/src
git clone <repository-url>
cd ~/Desktop/process_LiDAR_data/project_garage
catkin_make
source devel/setup.bash
```

---

## How to Run the Simulation

### Terminal 1: Launch Gazebo with Willow Garage World
Start Gazebo with the Willow Garage world and spawn the TurtleBot3 robot:

```bash
source ~/Desktop/process_LiDAR_data/project_garage/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_custom_launch turtlebot3_willowgarage.launch
```

### Terminal 2: Run GMapping SLAM
Run the GMapping SLAM algorithm to generate occupancy grid maps:

```bash
source ~/Desktop/process_LiDAR_data/project_garage/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_custom_launch turtlebot3_gmapping_custom.launch
```

### Terminal 3: Teleoperate the Robot (Optional)
Control the TurtleBot3 manually using keyboard teleoperation:

```bash
source ~/Desktop/process_LiDAR_data/project_garage/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Terminal 4: Save the Generated Map
Save the occupancy grid map (`.pgm` and `.yaml` files) to your Desktop:

```bash
source ~/Desktop/process_LiDAR_data/project_garage/devel/setup.bash
rosrun map_server map_saver -f ~/Desktop/willowgarage_map
```

---

## File Structure

- **`turtlebot3_custom_launch/launch/turtlebot3_willowgarage.launch`**: Launches Gazebo with the Willow Garage world.
- **`turtlebot3_custom_launch/launch/turtlebot3_gmapping_custom.launch`**: Runs the GMapping SLAM algorithm with custom parameters.
- **`turtlebot3_custom_launch/config/gmapping_params.yaml`**: Configuration file for SLAM parameters.
- **`willowgarage_map.pgm` and `willowgarage_map.yaml`**: Generated occupancy grid maps.

---

## Customization

### 1. Adjust Lidar Range
To modify the lidar range, update the URDF file:

```xml
<gazebo reference="base_scan">
  <sensor type="ray" name="laser">
    <range>
      <min>0.1</min>
      <max>15.5</max> <!-- Set desired lidar range -->
    </range>
  </sensor>
</gazebo>
```

### 2. Customize GMapping Parameters
Edit `gmapping_params.yaml` to fine-tune mapping settings:

```yaml
maxRange: 15.5
maxUrange: 15.5
delta: 0.05
linearUpdate: 0.1
angularUpdate: 0.05
map_update_interval: 2.0
```

Place the updated file in the `turtlebot3_custom_launch/config/` directory.

---

## Visual Demonstrations

### Gazebo Simulation
![Gazebo Simulation](https://github.com/user-attachments/assets/04f53bf4-bb1e-451e-864e-7a87a6d81f0d)

### Mapping in RViz
![Mapping in RViz](https://github.com/user-attachments/assets/80c98d5c-02fc-4655-ae3a-dcfdd86ed93c)

### Occupancy Grid Map
![Occupancy Grid Map](https://github.com/belkacem-inelecer/TurtleBot3-AMR-simulation-on-custom-gazebo-worlds-ROS1/blob/main/project_garage/occluded_willowgarage_map.pgm)
![Occupancy Grid Map](https://github.com/user-attachments/assets/e0a838c3-d0b3-4402-8dfc-6a912a2458d6)



### Video Demonstration
Watch the full simulation here: [**Video Link**](video/demo.mp4)

---


#### 2. Map Not Saving
Ensure the `map_saver` node is running correctly:

```bash
rosrun map_server map_saver -f ~/Desktop/willowgarage_map
```

#### 3. Lidar Range Not Reflected
Verify the URDF and SLAM parameters are updated, and relaunch the simulation.

---

## Future Work

- Integrate advanced SLAM algorithms like **Cartographer** or **Hector SLAM**.
- Automate the teleoperation using the ROS navigation stack.
- Enhance map accuracy with additional sensor fusion techniques.

