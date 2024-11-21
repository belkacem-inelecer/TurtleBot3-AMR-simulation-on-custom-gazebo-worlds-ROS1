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
Watch the full simulation here: [**Video Link**](https://youtu.be/N9f_IfdwZNY?feature=shared)


---


#### 2. Map Not Saving
Ensure the `map_saver` node is running correctly:

```bash
rosrun map_server map_saver -f ~/Desktop/willowgarage_map
```

#### 3. Lidar Range Not Reflected
Verify the URDF and SLAM parameters are updated, and relaunch the simulation.

---

## How to Generate a `.PGM` and `.YAML` Submap from a `.BAG` File

### Step 1: Use the Extraction Script
Make the `extract_maps.sh` script executable:

```bash
chmod +x extract_maps.sh
```

Run the script:

```bash
./extract_maps.sh
```

### Step 2: Interactive Prompts
You’ll be asked to input:
1. The path to the `.bag` file containing SLAM data.
2. The directory where the extracted map files should be saved.

Example:

```bash
Enter the full path to the BAG file (e.g., /home/user/maps/turtlebot3_slam_data.bag): /home/user/maps/turtlebot3_slam_data.bag
Enter the full path to the output directory (e.g., /home/user/maps): /home/user/maps
```

### Step 3: Script Process
The script will:
1. Launch a SLAM node to process the bag file.
2. Play back the bag file to generate mapping data.
3. Save the map as `.pgm` and `.yaml` files in the specified directory.

### Step 4: Verify the Output
After the script completes, check the output directory for the generated files. For example:

```bash
turtlebot3_slam_data.pgm
turtlebot3_slam_data.yaml
```

---

### Example Run
```bash
$ ./extract_maps.sh
Enter the full path to the BAG file (e.g., /home/user/maps/turtlebot3_slam_data.bag): /home/user/maps/turtlebot3_slam_data.bag
Enter the full path to the output directory (e.g., /home/user/maps): /home/user/maps

Launching SLAM node...
Playing bag file: /home/user/maps/turtlebot3_slam_data.bag...
Bag file playback completed.
Saving map for turtlebot3_slam_data...
Shutting down SLAM node...
Map for turtlebot3_slam_data saved successfully in /home/user/maps.
```
---

## How to Modify an Existing `.PGM` Map (OCCLUSION)

### Step 1: Use the Modification Script
To edit an existing map, use the `modify_map.py` script. Ensure the script is executable:

```bash
chmod +x modify_map.py
```

Run the script in a terminal:

```bash
python3 modify_map.py
```

### Step 2: Input File Paths
The script will prompt you to enter:
1. The path to the `.pgm` map you want to modify.
2. The output directory to save the modified map.

Example:

```bash
Enter the path to the .pgm file you want to modify: /home/user/maps/map.pgm
Enter the directory where you want to save the results: /home/user/maps/modified/
```

### Step 3: Modify the Map
An OpenCV window named **"Map Editor"** will open. Use the following controls:
- **`t`**: Select Rectangle Tool. Draw rectangles by left-clicking and dragging.
- **`c`**: Select Circle Tool. Add circular obstacles.
- **`o`**: Select Occlusion Tool. Click to add random occlusions.
- **`r`**: Reset the map to its original state.
- **`s`**: Save the modified map.
- **`q`**: Quit the editor.

### Step 4: Save the Map
When you save the map, the modified `.pgm` file and its corresponding `.yaml` file will be saved in the specified directory with names like:

```bash
map_modified_1.pgm
map_modified_1.yaml
```

### Step 5: Visualize the Modified Map
To visualize the map in RViz:
1. Run `roscore` to start the ROS core:
    ```bash
    roscore
    ```
2. Use `map_server` to serve the modified map:
    ```bash
    rosrun map_server map_server /home/user/maps/modified/map_modified_1.yaml
    ```
3. Launch RViz in another terminal:
    ```bash
    rosrun rviz rviz
    ```
4. Add the `/map` topic in RViz to view the map.

---



### Troubleshooting
- Ensure the `.bag` file contains relevant topics such as `/scan`, `/tf`, `/odom`, and `/map`.
- Verify the output directory has write permissions.
- If the map is not saved, check the completeness of the `.bag` file data.
