#!/bin/bash


'''
# Directories
BAG_DIR="/home/hedi/Desktop/process_LiDAR_data/project_garage"
OUTPUT_DIR="/home/hedi/Desktop/process_LiDAR_data/project_garage"
LAUNCH_DIR="/home/hedi/Desktop/process_LiDAR_data/project_garage/src/turtlebot3_custom_launch/launch"

# List of bag files
BAG_FILES=("turtlebot3_slam_data_submap_1.bag" "turtlebot3_slam_data_submap_2.bag")

# Playback speed
PLAYBACK_SPEED="1.0"  # Adjust if needed (1.0 = real-time)

# Iterate through each bag file
for BAG in "${BAG_FILES[@]}"
do
    # Extract submap name
    SUBMAP_NAME=$(basename "$BAG" .bag)

    echo "Processing $BAG_DIR/$BAG..."

    # Launch the SLAM node
    echo "Launching SLAM node..."
    roslaunch turtlebot3_custom_launch turtlebot3_gmapping_custom.launch &
    SLAM_PID=$!
    sleep 5  # Give time for the SLAM node to initialize

    # Play the bag file
    echo "Playing bag file: $BAG_DIR/$BAG..."
    rosbag play "$BAG_DIR/$BAG" --clock --rate "$PLAYBACK_SPEED" &
    ROSBAG_PID=$!

    # Wait for the rosbag playback to finish
    wait $ROSBAG_PID
    echo "Bag file playback completed."

    # Save the map
    echo "Saving map for $SUBMAP_NAME..."
    rosrun map_server map_saver -f "$OUTPUT_DIR/$SUBMAP_NAME"

    # Shut down the SLAM node
    echo "Shutting down SLAM node..."
    kill $SLAM_PID
    wait $SLAM_PID 2>/dev/null

    echo "Map for $SUBMAP_NAME saved successfully."
done

echo "All maps processed and saved to $OUTPUT_DIR."
'''
######################################################################

#!/bin/bash

# Prompt the user for the full path to the bag file directory
read -p "Enter the full path to the BAG file (e.g., /home/hedi/Desktop/process_LiDAR_data/project_garage/turtlebot3_slam_data_submap_2.bag): " BAG_FILE

# Prompt the user for the output directory
read -p "Enter the full path to the output directory (e.g., /home/hedi/Desktop/process_LiDAR_data/project_garage): " OUTPUT_DIR

# Ensure the BAG file exists
if [ ! -f "$BAG_FILE" ]; then
    echo "Error: BAG file not found at $BAG_FILE"
    exit 1
fi

# Ensure the output directory exists, or create it
if [ ! -d "$OUTPUT_DIR" ]; then
    echo "Output directory does not exist. Creating $OUTPUT_DIR..."
    mkdir -p "$OUTPUT_DIR"
fi

# Extract submap name from the bag file
SUBMAP_NAME=$(basename "$BAG_FILE" .bag)

# Launch the SLAM node
echo "Launching SLAM node..."
roslaunch turtlebot3_custom_launch turtlebot3_gmapping_custom.launch &
SLAM_PID=$!
sleep 5  # Give time for the SLAM node to initialize

# Play the bag file
echo "Playing bag file: $BAG_FILE..."
rosbag play "$BAG_FILE" --clock --rate "1.0" &
ROSBAG_PID=$!

# Wait for the rosbag playback to finish
wait $ROSBAG_PID
echo "Bag file playback completed."

# Save the map
echo "Saving map for $SUBMAP_NAME..."
rosrun map_server map_saver -f "$OUTPUT_DIR/$SUBMAP_NAME"

# Shut down the SLAM node
echo "Shutting down SLAM node..."
kill $SLAM_PID
wait $SLAM_PID 2>/dev/null

echo "Map for $SUBMAP_NAME saved successfully in $OUTPUT_DIR."


