'''

import cv2
import numpy as np

# Map file and metadata
map_path = "/home/hedi/Desktop/process_LiDAR_data/project_garage/turtlebot3_slam_data_submap_2.pgm"
output_path = "/home/hedi/Desktop/process_LiDAR_data/project_garage/modified_map.pgm"

# Load the map
map_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

if map_image is None:
    print("Error: Could not load the map file.")
    exit()

# Display the original map
cv2.imshow("Original Map", map_image)
cv2.waitKey(500)  # Show the map for 500ms

# Step 1: Add a rectangle (representing a wall or obstacle)
# Specify rectangle coordinates (x_start, y_start, x_end, y_end)
rectangle_start = (500, 500)  # Top-left corner
rectangle_end = (700, 700)    # Bottom-right corner
cv2.rectangle(map_image, rectangle_start, rectangle_end, (0,), -1)  # Filled black rectangle (value 0)

# Step 2: Add a circle (representing an obstacle)
# Specify circle center and radius
circle_center = (1000, 1000)  # x, y coordinates of the circle center
circle_radius = 50           # Radius of the circle
cv2.circle(map_image, circle_center, circle_radius, (0,), -1)  # Filled black circle (value 0)

# Step 3: Add a semi-occupied region (gray area)
# Specify rectangle coordinates for gray area
gray_start = (1500, 1500)
gray_end = (1700, 1700)
cv2.rectangle(map_image, gray_start, gray_end, (128,), -1)  # Gray rectangle (value 128)

# Step 4: Save the modified map
cv2.imwrite(output_path, map_image)

print(f"Modified map saved to: {output_path}")

# Display the modified map
cv2.imshow("Modified Map", map_image)
cv2.waitKey(0)  # Wait for a key press to close the window
cv2.destroyAllWindows()
'''


'''
import cv2
import numpy as np

# Load the original map
map_path = "/home/hedi/Desktop/process_LiDAR_data/project_garage/turtlebot3_slam_data_submap_2.pgm"
map_yaml_path = "/home/hedi/Desktop/process_LiDAR_data/project_garage/turtlebot3_slam_data_submap_2.yaml"
map_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

if map_image is None:
    print("Error: Unable to load the map.")
    exit()

# Initialize variables
modified_map = map_image.copy()
drawing = False  # True if the user is drawing
shape = "rectangle"  # Options: "rectangle", "circle"
start_x, start_y = -1, -1

# Mouse callback function
def draw_shape(event, x, y, flags, param):
    global drawing, start_x, start_y, modified_map

    if event == cv2.EVENT_LBUTTONDOWN:  # Start drawing
        drawing = True
        start_x, start_y = x, y

    elif event == cv2.EVENT_MOUSEMOVE:  # Drawing in progress
        if drawing:
            temp_map = modified_map.copy()
            if shape == "rectangle":
                cv2.rectangle(temp_map, (start_x, start_y), (x, y), (0,), -1)
            elif shape == "circle":
                radius = int(((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5)
                cv2.circle(temp_map, (start_x, start_y), radius, (0,), -1)
            cv2.imshow("Map Editor", temp_map)

    elif event == cv2.EVENT_LBUTTONUP:  # End drawing
        drawing = False
        if shape == "rectangle":
            cv2.rectangle(modified_map, (start_x, start_y), (x, y), (0,), -1)
        elif shape == "circle":
            radius = int(((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5)
            cv2.circle(modified_map, (start_x, start_y), radius, (0,), -1)
        cv2.imshow("Map Editor", modified_map)

# Display the map
cv2.namedWindow("Map Editor")
cv2.setMouseCallback("Map Editor", draw_shape)

while True:
    cv2.imshow("Map Editor", modified_map)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('r'):  # Switch to rectangle mode
        shape = "rectangle"
        print("Shape: Rectangle")
    elif key == ord('c'):  # Switch to circle mode
        shape = "circle"
        print("Shape: Circle")
    elif key == ord('s'):  # Save the modified map
        modified_map_path = "/home/hedi/Desktop/process_LiDAR_data/project_garage/modified_map.pgm"
        cv2.imwrite(modified_map_path, modified_map)
        print(f"Modified map saved to {modified_map_path}")
    elif key == ord('q'):  # Quit
        break

cv2.destroyAllWindows()


'''

##########################################################################################
'''
import cv2
import numpy as np

# Initialize global variables
current_map = None
modified_map = None
save_counter = 1
tool_selected = "none"  # Can be "occlusion" or "eraser"
display_scale = 0.3  # Scale factor for display

# Map parameters from the YAML file
map_path = "/home/hedi/Desktop/process_LiDAR_data/project_garage/turtlebot3_slam_data_submap_2.pgm"
map_yaml_path = "/home/hedi/Desktop/process_LiDAR_data/project_garage/turtlebot3_slam_data_submap_2.yaml"

# Load the map
current_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
if current_map is None:
    print("Error: Unable to load the map.")
    exit()

# Resize map for display purposes
def resize_for_display(image, scale):
    height, width = image.shape
    new_width = int(width * scale)
    new_height = int(height * scale)
    return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

# Rescale coordinates for editing
def scale_coordinates(x, y, scale, inverse=False):
    if inverse:
        return int(x / scale), int(y / scale)
    return int(x * scale), int(y * scale)

# Initialize map
modified_map = current_map.copy()
display_map = resize_for_display(modified_map, display_scale)

# Mouse callback function
drawing = False
start_x, start_y = -1, -1
shape = "rectangle"

def draw_shape(event, x, y, flags, param):
    global drawing, start_x, start_y, modified_map, display_map

    # Scale coordinates to match original map size
    scaled_x, scaled_y = scale_coordinates(x, y, display_scale, inverse=True)

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_x, start_y = scaled_x, scaled_y

    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        temp_map = modified_map.copy()
        if shape == "rectangle":
            cv2.rectangle(temp_map, (start_x, start_y), (scaled_x, scaled_y), (0,), -1)
        elif shape == "circle":
            radius = int(((scaled_x - start_x) ** 2 + (scaled_y - start_y) ** 2) ** 0.5)
            cv2.circle(temp_map, (start_x, start_y), radius, (0,), -1)
        display_map = resize_for_display(temp_map, display_scale)
        cv2.imshow("Map Editor", display_map)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if shape == "rectangle":
            cv2.rectangle(modified_map, (start_x, start_y), (scaled_x, scaled_y), (0,), -1)
        elif shape == "circle":
            radius = int(((scaled_x - start_x) ** 2 + (scaled_y - start_y) ** 2) ** 0.5)
            cv2.circle(modified_map, (start_x, start_y), radius, (0,), -1)
        display_map = resize_for_display(modified_map, display_scale)
        cv2.imshow("Map Editor", display_map)

# Create the window
cv2.namedWindow("Map Editor", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Map Editor", draw_shape)

# Display the map
while True:
    cv2.imshow("Map Editor", display_map)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('r'):  # Rectangle mode
        shape = "rectangle"
        print("Shape: Rectangle")
    elif key == ord('c'):  # Circle mode
        shape = "circle"
        print("Shape: Circle")
    elif key == ord('s'):  # Save map
        save_path = f"/home/hedi/Desktop/process_LiDAR_data/project_garage/modified_map_{save_counter}.pgm"
        cv2.imwrite(save_path, modified_map)
        print(f"Map saved as {save_path}")
        save_counter += 1
    elif key == ord('q'):  # Quit
        print("Exiting...")
        break

cv2.destroyAllWindows()
'''

##################################################################################
'''
import cv2
import numpy as np
import random

# Initialize global variables
current_map = None
modified_map = None
save_counter = 1
tool_selected = "none"  # Can be "rectangle", "circle", or "occlusion"

# Map parameters from the YAML file
map_resolution = 0.05  # meters per pixel
map_origin = [-100.0, -100.0, 0.0]  # x, y, z origin
occupied_thresh = 0.65
free_thresh = 0.196

# Random point spacing parameters
min_spacing = int(0.03 / map_resolution)  # Minimum spacing in pixels
max_spacing = int(0.15 / map_resolution)  # Maximum spacing in pixels

start_point = None  # Start point for drawing shapes


def click_event(event, x, y, flags, param):
    """Handle mouse events for drawing or adding occlusions."""
    global start_point, tool_selected, modified_map

    if event == cv2.EVENT_LBUTTONDOWN:
        # Start point for drawing shapes
        start_point = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE and start_point:
        # Preview the shape being drawn
        temp_map = modified_map.copy()
        if tool_selected == "rectangle":
            cv2.rectangle(temp_map, start_point, (x, y), (0,), -1)
        elif tool_selected == "circle":
            radius = int(np.linalg.norm(np.array(start_point) - np.array((x, y))))
            cv2.circle(temp_map, start_point, radius, (0,), -1)
        elif tool_selected == "occlusion":
            add_realistic_occlusion_preview(temp_map, x, y)
        cv2.imshow("Map Editor", temp_map)
    elif event == cv2.EVENT_LBUTTONUP:
        # Place the shape or occlusion
        if tool_selected == "rectangle":
            cv2.rectangle(modified_map, start_point, (x, y), (0,), -1)
        elif tool_selected == "circle":
            radius = int(np.linalg.norm(np.array(start_point) - np.array((x, y))))
            cv2.circle(modified_map, start_point, radius, (0,), -1)
        elif tool_selected == "occlusion":
            add_realistic_occlusion(x, y)
        start_point = None
        cv2.imshow("Map Editor", modified_map)


def add_realistic_occlusion(x, y):
    """Simulate a realistic occlusion with randomly distributed points on borders where intersecting with 254 pixels."""
    global modified_map
    height, width = modified_map.shape

    # Randomly determine the size of the occlusion
    rect_width = random.randint(20, 50)
    rect_height = random.randint(20, 50)

    # Ensure the occlusion stays within bounds
    x_start = max(0, x - rect_width // 2)
    y_start = max(0, y - rect_height // 2)
    x_end = min(width, x + rect_width // 2)
    y_end = min(height, y + rect_height // 2)

    # Draw the interior as gray (205)
    cv2.rectangle(modified_map, (x_start, y_start), (x_end, y_end), (205,), -1)

    # Draw randomly spaced points on borders
    current_x = x_start
    while current_x <= x_end:
        current_y = y_start
        while current_y <= y_end:
            # Check if it's a border pixel
            is_border = (current_x == x_start or current_x == x_end or current_y == y_start or current_y == y_end)
            if is_border and current_map[current_y, current_x] == 254:  # Border on `254` pixels only
                # Draw a point (circle) at the position
                modified_map[current_y, current_x] = 0  # Set pixel to black for a single dot
            # Randomize the next Y position
            current_y += random.randint(min_spacing, max_spacing)
        # Randomize the next X position
        current_x += random.randint(min_spacing, max_spacing)


def add_realistic_occlusion_preview(temp_map, x, y):
    """Preview the occlusion before placing it."""
    global current_map
    height, width = temp_map.shape

    # Randomly determine the size of the occlusion
    rect_width = random.randint(20, 50)
    rect_height = random.randint(20, 50)

    # Ensure the occlusion stays within bounds
    x_start = max(0, x - rect_width // 2)
    y_start = max(0, y - rect_height // 2)
    x_end = min(width, x + rect_width // 2)
    y_end = min(height, y + rect_height // 2)

    # Draw the interior as gray (205)
    cv2.rectangle(temp_map, (x_start, y_start), (x_end, y_end), (205,), -1)

    # Draw randomly spaced points on borders
    current_x = x_start
    while current_x <= x_end:
        current_y = y_start
        while current_y <= y_end:
            is_border = (current_x == x_start or current_x == x_end or current_y == y_start or current_y == y_end)
            if is_border and current_map[current_y, current_x] == 254:
                temp_map[current_y, current_x] = 0
            current_y += random.randint(min_spacing, max_spacing)
        current_x += random.randint(min_spacing, max_spacing)


def update_ui():
    """Update the UI with buttons for selecting tools."""
    global modified_map
    ui = modified_map.copy()
    # Draw buttons
    cv2.rectangle(ui, (10, 10), (110, 60), (255,), -1)  # Rectangle button
    cv2.putText(ui, "Rect", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,), 1)
    cv2.rectangle(ui, (120, 10), (220, 60), (255,), -1)  # Circle button
    cv2.putText(ui, "Circle", (130, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,), 1)
    cv2.rectangle(ui, (230, 10), (330, 60), (255,), -1)  # Occlusion button
    cv2.putText(ui, "Occ", (250, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,), 1)
    cv2.imshow("Map Editor", ui)


def save_map():
    """Save the modified map to a .pgm file."""
    global save_counter, modified_map
    file_name = f"modified_map_{save_counter}.pgm"
    cv2.imwrite(file_name, modified_map)
    print(f"Map saved as {file_name}")
    save_counter += 1


def load_map(file_path):
    """Load the .pgm map file."""
    global current_map, modified_map
    current_map = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    if current_map is None:
        print("Error: Could not load map.")
        return False
    modified_map = current_map.copy()
    return True


def reset_map():
    """Reset the map to its original state."""
    global modified_map, current_map
    modified_map = current_map.copy()
    update_ui()


if __name__ == "__main__":
    # Load the map file
    map_path = "/home/hedi/Desktop/process_LiDAR_data/project_garage/willowgarage_map.pgm"
    if not load_map(map_path):
        exit()

    # Print debug info
    print(f"Map Resolution: {map_resolution} meters/pixel")
    print(f"Map Origin: {map_origin}")
    print(f"Occupied Threshold: {occupied_thresh}")
    print(f"Free Threshold: {free_thresh}")

    # Create the window
    cv2.namedWindow("Map Editor", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Map Editor", click_event)
    update_ui()

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):  # Save map
            save_map()
        elif key == ord('r'):  # Reset map
            reset_map()
        elif key == ord('q'):  # Quit
            print("Exiting...")
            break
        elif key == ord('o'):  # Select occlusion tool
            tool_selected = "occlusion"
            print("Tool selected: Occlusion")
        elif key == ord('c'):  # Select circle tool
            tool_selected = "circle"
            print("Tool selected: Circle")
        elif key == ord('t'):  # Select rectangle tool
            tool_selected = "rectangle"
            print("Tool selected: Rectangle")

    cv2.destroyAllWindows()
    
'''

########################################################################################

import cv2
import numpy as np
import os
import random

# Initialize global variables
current_map = None
modified_map = None
save_counter = 1
tool_selected = "none"  # Can be "rectangle", "circle", or "occlusion"

# Map parameters
map_resolution = 0.05  # meters per pixel
map_origin = [-100.0, -100.0, 0.0]  # x, y, z origin
occupied_thresh = 0.65
free_thresh = 0.196

# Random point spacing parameters
min_spacing = int(0.03 / map_resolution)  # Minimum spacing in pixels
max_spacing = int(0.15 / map_resolution)  # Maximum spacing in pixels

start_point = None  # Start point for drawing shapes


def click_event(event, x, y, flags, param):
    """Handle mouse events for drawing or adding occlusions."""
    global start_point, tool_selected, modified_map

    if event == cv2.EVENT_LBUTTONDOWN:
        start_point = (x, y)  # Start point for drawing shapes
    elif event == cv2.EVENT_MOUSEMOVE and start_point:
        # Preview the shape being drawn
        temp_map = modified_map.copy()
        if tool_selected == "rectangle":
            cv2.rectangle(temp_map, start_point, (x, y), (0,), -1)
        elif tool_selected == "circle":
            radius = int(np.linalg.norm(np.array(start_point) - np.array((x, y))))
            cv2.circle(temp_map, start_point, radius, (0,), -1)
        elif tool_selected == "occlusion":
            add_realistic_occlusion_preview(temp_map, x, y)
        cv2.imshow("Map Editor", temp_map)
    elif event == cv2.EVENT_LBUTTONUP:
        # Place the shape or occlusion
        if tool_selected == "rectangle":
            cv2.rectangle(modified_map, start_point, (x, y), (0,), -1)
        elif tool_selected == "circle":
            radius = int(np.linalg.norm(np.array(start_point) - np.array((x, y))))
            cv2.circle(modified_map, start_point, radius, (0,), -1)
        elif tool_selected == "occlusion":
            add_realistic_occlusion(x, y)
        start_point = None
        cv2.imshow("Map Editor", modified_map)


def add_realistic_occlusion(x, y):
    """Simulate a realistic occlusion with randomly distributed points on borders."""
    global modified_map
    height, width = modified_map.shape

    rect_width = random.randint(20, 50)
    rect_height = random.randint(20, 50)
    x_start = max(0, x - rect_width // 2)
    y_start = max(0, y - rect_height // 2)
    x_end = min(width, x + rect_width // 2)
    y_end = min(height, y + rect_height // 2)

    # Draw the interior as gray (205)
    cv2.rectangle(modified_map, (x_start, y_start), (x_end, y_end), (205,), -1)

    # Draw randomly spaced points on borders
    current_x = x_start
    while current_x <= x_end:
        current_y = y_start
        while current_y <= y_end:
            is_border = (current_x == x_start or current_x == x_end or current_y == y_start or current_y == y_end)
            if is_border and current_map[current_y, current_x] == 254:
                modified_map[current_y, current_x] = 0  # Set pixel to black
            current_y += random.randint(min_spacing, max_spacing)
        current_x += random.randint(min_spacing, max_spacing)


def add_realistic_occlusion_preview(temp_map, x, y):
    """Preview the occlusion before placing it."""
    height, width = temp_map.shape
    rect_width = random.randint(20, 50)
    rect_height = random.randint(20, 50)
    x_start = max(0, x - rect_width // 2)
    y_start = max(0, y - rect_height // 2)
    x_end = min(width, x + rect_width // 2)
    y_end = min(height, y + rect_height // 2)
    cv2.rectangle(temp_map, (x_start, y_start), (x_end, y_end), (205,), -1)


def update_ui():
    """Update the UI with buttons for selecting tools."""
    global modified_map
    ui = modified_map.copy()
    cv2.rectangle(ui, (10, 10), (110, 60), (255,), -1)
    cv2.putText(ui, "Rect", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,), 1)
    cv2.rectangle(ui, (120, 10), (220, 60), (255,), -1)
    cv2.putText(ui, "Circle", (130, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,), 1)
    cv2.rectangle(ui, (230, 10), (330, 60), (255,), -1)
    cv2.putText(ui, "Occ", (250, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,), 1)
    cv2.imshow("Map Editor", ui)


def save_map(output_dir, file_name):
    """Save the modified map and create a .yaml file."""
    global save_counter, modified_map

    base_name, ext = os.path.splitext(file_name)
    modified_file_name = f"{base_name}_modified_{save_counter}{ext}"
    yaml_file_name = f"{base_name}_modified_{save_counter}.yaml"

    # Save the .pgm file
    cv2.imwrite(os.path.join(output_dir, modified_file_name), modified_map)

    # Save the .yaml file
    with open(os.path.join(output_dir, yaml_file_name), "w") as yaml_file:
        yaml_file.write(f"image: {modified_file_name}\n")
        yaml_file.write(f"resolution: {map_resolution}\n")
        yaml_file.write(f"origin: {map_origin}\n")
        yaml_file.write(f"negate: 0\n")
        yaml_file.write(f"occupied_thresh: {occupied_thresh}\n")
        yaml_file.write(f"free_thresh: {free_thresh}\n")

    print(f"Map saved as {modified_file_name} and {yaml_file_name}")
    save_counter += 1


def load_map(file_path):
    """Load the .pgm map file."""
    global current_map, modified_map
    current_map = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    if current_map is None:
        print("Error: Could not load map.")
        return False
    modified_map = current_map.copy()
    return True


def reset_map():
    """Reset the map to its original state."""
    global modified_map, current_map
    modified_map = current_map.copy()
    update_ui()


if __name__ == "__main__":
    # Ask user for input and output paths
    input_file = input("Enter the path to the .pgm file you want to modify: ")
    output_dir = input("Enter the directory where you want to save the results: ")

    if not load_map(input_file):
        exit()

    # Ensure the output directory exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Create the window
    cv2.namedWindow("Map Editor", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Map Editor", click_event)
    update_ui()

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):  # Save map
            save_map(output_dir, os.path.basename(input_file))
        elif key == ord('r'):  # Reset map
            reset_map()
        elif key == ord('q'):  # Quit
            print("Exiting...")
            break
        elif key == ord('o'):  # Select occlusion tool
            tool_selected = "occlusion"
            print("Tool selected: Occlusion")
        elif key == ord('c'):  # Select circle tool
            tool_selected = "circle"
            print("Tool selected: Circle")
        elif key == ord('t'):  # Select rectangle tool
            tool_selected = "rectangle"
            print("Tool selected: Rectangle")

    cv2.destroyAllWindows()


