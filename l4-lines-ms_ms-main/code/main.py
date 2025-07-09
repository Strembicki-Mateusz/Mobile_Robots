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

def merge_similar_lines(lines, rho_threshold=10, theta_threshold=np.deg2rad(1)):
    merged_lines = []
    for rho, theta in lines:
        found_similar = False
        for i, (merged_rho, merged_theta) in enumerate(merged_lines):
            if abs(rho - merged_rho) < rho_threshold and abs(theta - merged_theta) < theta_threshold:
                merged_lines[i] = ((merged_rho + rho) / 2, (merged_theta + theta) / 2)
                found_similar = True
                break
        if not found_similar:
            merged_lines.append((rho, theta))
    return merged_lines

def intersection(line1, line2):
    a1, b1, c1 = line1
    a2, b2, c2 = line2
    det = a1 * b2 - a2 * b1
    if det == 0:
        return None
    x = (b2 * c1 - b1 * c2) / det / 1000  
    y = (a1 * c2 - a2 * c1) / det / 1000  
    return (x, y)

def line_to_coefficients(rho, theta):
    a = np.cos(theta)
    b = np.sin(theta)
    c = -rho
    return a, b, c

# Loading data
filepath = './json/line_detection_1.json'
#filepath = './json/line_detection_2.json'
#filepath = './json/line_localization_1.json'
data = read_json_file(filepath)

# Iterating through all scans
for scan_data in data:
    scan = np.array(scan_data['scan'])
    angles = np.linspace(-np.pi / 2, np.pi / 2, len(scan))


    valid_indices = np.isfinite(scan)
    angles = angles[valid_indices]
    scan = scan[valid_indices]

    x_points, y_points = pol2cart(scan, angles)

    # Creating an image of points
    image_size = 500
    image = np.zeros((image_size, image_size), dtype=np.uint8)

    # Drawing points
    x_scaled = (((x_points - x_points.min()) / (x_points.max() - x_points.min()) * (image_size - 1)) / 2).astype(int)
    y_scaled = (((y_points - y_points.min()) / (y_points.max() - y_points.min()) * (image_size - 1))).astype(int)

    for x, y in zip(x_scaled, y_scaled):
        image[y, x] = 255

    hspace, angles_hough, dists = hough_transform(image)

    lines = []
    for rho_idx, theta_idx in zip(*np.where(hspace > np.max(hspace) * 0.5)):
        rho = dists[rho_idx]
        theta = angles_hough[theta_idx]
        lines.append((rho, theta))

    # Merging similar lines
    rho_threshold = 20
    theta_threshold = np.deg2rad(5)
    merged_lines = merge_similar_lines(lines, rho_threshold=rho_threshold, theta_threshold=theta_threshold)

    # Calculating the average angle of robot orientation (average angle of detected lines)
    angles_of_lines = [theta for _, theta in merged_lines]
    average_angle = np.mean(angles_of_lines)  # average angle of all lines
    print(f"Average robot orientation angle: {np.degrees(average_angle):.2f}°")

    # Robot position (average of detected points)
    avg_x = np.mean(x_points)
    avg_y = np.mean(y_points)


    plt.figure(figsize=(12, 6))


    plt.subplot(1, 2, 1)
    plt.imshow(image, cmap='gray')
    plt.title("Original image")
    plt.axis('off')


    plt.subplot(1, 2, 2)
    plt.imshow(image, cmap='gray')
    plt.title("Image with localization")
    plt.axis('off')

    for rho, theta in merged_lines:
        a, b = np.cos(theta), np.sin(theta)
        x0, y0 = a * rho, b * rho
        x1, y1 = int(x0 + 1000 * (-b)), int(y0 + 1000 * (a))
        x2, y2 = int(x0 - 1000 * (-b)), int(y0 - 1000 * (a))
        plt.plot((x1, x2), (y1, y2), '-r')

    # Calculating intersection points
    intersection_points = []
    for i in range(len(merged_lines)):
        for j in range(i + 1, len(merged_lines)):
            line1 = line_to_coefficients(*merged_lines[i])
            line2 = line_to_coefficients(*merged_lines[j])
            intersection_point = intersection(line1, line2)
            if intersection_point:
                intersection_points.append(intersection_point)

    if intersection_points:
        avg_x = np.mean([point[0] for point in intersection_points])
        avg_y = np.mean([point[1] for point in intersection_points])

        # Scaling robot position to pixels (converting from meters to pixels)
        avg_x_scaled = int(((avg_x - x_points.min()) / (x_points.max() - x_points.min()) * (image_size - 1)) / 2)
        avg_y_scaled = int(((avg_y - y_points.min()) / (y_points.max() - y_points.min()) * (image_size - 1)))


        print(f"Average robot position based on first intersection: {avg_x:.2f} m, {avg_y:.2f} m")

        # Drawing robot orientation as an arrow (in pixels)
        orientation_length = 50  # arrow length in pixels
        robot_orientation_x = avg_x_scaled + orientation_length * np.cos(average_angle)
        robot_orientation_y = avg_y_scaled + orientation_length * np.sin(average_angle)


        plt.arrow(avg_x_scaled, avg_y_scaled, 
                  robot_orientation_x - avg_x_scaled, 
                  robot_orientation_y - avg_y_scaled, 
                  head_width=20, head_length=30, fc='green', ec='green')

        plt.plot(avg_x_scaled, avg_y_scaled, 'bo', markersize=7)

    plt.tight_layout()
    plt.show()
