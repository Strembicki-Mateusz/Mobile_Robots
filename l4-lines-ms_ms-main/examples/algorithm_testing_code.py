import numpy as np
import matplotlib.pyplot as plt
import json

def read_json_file(filepath):
    with open(filepath, 'r') as file:
        return json.load(file)

def pol2cart(rho, theta):
    x = rho * np.cos(theta)
    y = rho * np.sin(theta)
    return x, y

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    return rho, theta

# Hough function to line detection
def hough_transform(image):
    rows, cols = image.shape
    diag_len = int(np.sqrt(rows**2 + cols**2))
    rhos = np.linspace(-diag_len, diag_len, 2 * diag_len)
    thetas = np.deg2rad(np.arange(-90, 90))
    accumulator = np.zeros((len(rhos), len(thetas)), dtype=np.int32)

    y_idxs, x_idxs = np.nonzero(image)  # find non-zero points
    for x, y in zip(x_idxs, y_idxs):
        for theta_idx, theta in enumerate(thetas):
            rho = int(x * np.cos(theta) + y * np.sin(theta))
            rho_idx = np.searchsorted(rhos, rho)
            accumulator[rho_idx, theta_idx] += 1

    return accumulator, thetas, rhos

# Function to find line intersection
def line_intersection(line1, line2):
    (theta1, rho1), (theta2, rho2) = line1, line2
    a1, b1 = np.cos(theta1), np.sin(theta1)
    a2, b2 = np.cos(theta2), np.sin(theta2)

    determinant = a1 * b2 - a2 * b1
    if np.isclose(determinant, 0):
        return None  # Lines are parallel

    x = (b2 * rho1 - b1 * rho2) / determinant
    y = (a1 * rho2 - a2 * rho1) / determinant
    return x, y

# Localization of the robot based on detected lines
def localize_robot(pose, detected_lines, scan, angles):
    intersections = []
    for i, line1 in enumerate(detected_lines):
        for line2 in detected_lines[i + 1:]:
            intersection = line_intersection(line1, line2)
            if intersection is not None:
                intersections.append(intersection)

    if not intersections:
        return None, None

    intersections = np.array(intersections)
    
    # Oblicz średnią z punktów przecięcia linii
    x_robot = np.mean(intersections[:, 0]) /1000
    y_robot = np.mean(intersections[:, 1]) /1000

    # Uwzględnij przesunięcie na podstawie początkowego "pose"
    x_robot += pose[0]
    y_robot += pose[1]

    x_points, y_points = pol2cart(scan, angles)

    # Szukaj linii, do której robot jest najbliżej (porównanie odległości)
    best_match = None
    min_distance = float('inf')

    for line in detected_lines:
        theta, rho = line
        # Przekształcenie współrzędnych punktu przecięcia na układ współrzędnych robota
        a, b = np.cos(theta), np.sin(theta)

        m_perp = -b / a if a != 0 else np.inf  # Slope of the perpendicular line
        b_perp = y_robot - m_perp * x_robot  # Oblicz b dla prostej prostopadłej

        # Znajdź przecięcie z wykrytą linią
        x_int, y_int = line_intersection((m_perp, b_perp), (np.cos(theta), np.sin(theta)))

        if x_int is not None and y_int is not None:
            # Oblicz odległość robota od punktu przecięcia
            distance = np.sqrt((x_int - x_robot) ** 2 + (y_int - y_robot) ** 2)
            if abs(distance - np.mean(scan)) < min_distance:
                min_distance = abs(distance - np.mean(scan))
                best_match = (x_int, y_int)

    if best_match is not None:
        x_robot, y_robot = best_match

    # Zwróć ostateczną pozycję robota
    return x_robot, y_robot, intersections

def calculate_orientation(robot_x, robot_y, detected_lines):
    line1, line2 = detected_lines[0], detected_lines[1]
    
    # Calculate the angles of these lines with respect to the axis
    theta1, rho1 = line1
    theta2, rho2 = line2

    # Calculate the robot's orientation as the average of the line angles
    theta_robot = (theta1 - theta2) / 2
    return theta_robot

filepath = './json/line_detection_1.json'
#filepath = './json/line_detection_2.json'
#filepath = './json/line_localization_1.json'
data = read_json_file(filepath)

pose = np.array(data[0]['pose'])  # [x, y, theta]
scan = np.array(data[0]['scan'])  # distances

angles = np.linspace(-np.pi / 2, np.pi / 2, len(scan))  

# Removing inf values
valid_indices = np.isfinite(scan)
angles = angles[valid_indices]
scan = scan[valid_indices]

x_points, y_points = pol2cart(scan, angles)

# Creating points image
image_size = 500
image = np.zeros((image_size, image_size), dtype=np.uint8)

# Drawing points into the image
x_scaled = ((x_points - x_points.min()) / (x_points.max() - x_points.min()) * (image_size - 1)).astype(int)
y_scaled = ((y_points - y_points.min()) / (y_points.max() - y_points.min()) * (image_size - 1)).astype(int)

for x, y in zip(x_scaled, y_scaled):
    image[y, x] = 255 

hspace, angles_hough, dists = hough_transform(image)

# Robot localization based on Hough transform
detected_lines = list(zip(angles_hough, dists))
x_robot, y_robot, intersections = localize_robot(pose, detected_lines, scan, angles)

fig, axes = plt.subplots(1, 2, figsize=(15, 6))
ax = axes.ravel()

# Settings of image
ax[0].imshow(image, cmap='gray')
ax[0].set_title('Points')
ax[0].set_axis_off()

ax[1].imshow(image, cmap='gray')
ax[1].set_title('Lines and robot position')
ax[1].set_axis_off()

for rho_idx, theta_idx in zip(*np.where(hspace > np.max(hspace) * 0.5)):
    rho = dists[rho_idx]
    theta = angles_hough[theta_idx]
    a, b = np.cos(theta), np.sin(theta)
    x0, y0 = a * rho, b * rho
    x1, y1 = int(x0 + 1000 * (-b)), int(y0 + 1000 * (a))
    x2, y2 = int(x0 - 1000 * (-b)), int(y0 - 1000 * (a))
    ax[1].plot((x1, x2), (y1, y2), '-r')

# Add robot position on the image
if x_robot is not None and y_robot is not None:
    robot_x_scaled = int((x_robot - x_points.min()) / (x_points.max() - x_points.min()) * (image_size - 1))
    robot_y_scaled = int((y_robot - y_points.min()) / (y_points.max() - y_points.min()) * (image_size - 1))
    ax[1].plot(robot_x_scaled, robot_y_scaled, 'bo', label='Robot')

    # Calculate and plot robot orientation
    theta_robot = calculate_orientation(x_robot, y_robot, detected_lines)
    orientation_length = 50  # Length of orientation line in pixels
    orientation_x = robot_x_scaled + orientation_length * np.cos(theta_robot)
    orientation_y = robot_y_scaled + orientation_length * np.sin(theta_robot)
    ax[1].arrow(robot_x_scaled, robot_y_scaled, orientation_x - robot_x_scaled, orientation_y - robot_y_scaled, 
                color='green', head_width=10, label='Orientation')

    ax[1].legend()


plt.tight_layout()
plt.show()

# Displaying robot position in the terminal
if x_robot is not None and y_robot is not None:
    print(f"Rob_pose: x = {x_robot}, y = {y_robot}, theta = {theta_robot}")
else:
    print("Failed to locate the robot")
