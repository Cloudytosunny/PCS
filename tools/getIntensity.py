import sys
import numpy as np
import pyntcloud

def load_pcd(file_path):
    cloud = pyntcloud.PyntCloud.from_file(file_path)
    points = cloud.points[['x', 'y', 'z', 'intensity']].values
    return points

def find_nearest_point(points, x, y, z):
    min_dist = float('inf')
    nearest_point = None
    same_intensity_count = 0
    for point in points:
        dist = np.sqrt((point[0] - x) ** 2 + (point[1] - y) ** 2 + (point[2] - z) ** 2)
        if dist < min_dist:
            min_dist = dist
            nearest_point = point

    for point in points:
        if point[3] == nearest_point[3]:
            same_intensity_count += 1
            
    return nearest_point, same_intensity_count

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python getIntensity.py <pcd_file>")
        sys.exit(1)

    pcd_file = sys.argv[1]
    try:
        points = load_pcd(pcd_file)
    except FileNotFoundError:
        print(f"Error: File '{pcd_file}' not found.")
        sys.exit(1)

    print("PCD file loaded successfully.")

    while True:
        try:
            x = float(input("Enter x coordinate: "))
            y = float(input("Enter y coordinate: "))
            z = float(input("Enter z coordinate: "))
        except ValueError:
            print("Invalid input. Please enter numerical values.")
            continue

        nearest_point, same_intensity_count = find_nearest_point(points, x, y, z)
        total_points = len(points)
        if total_points > 0:
            ratio = same_intensity_count / total_points
        else:
            ratio = 0.0
        print(f"Nearest point: x={nearest_point[0]}, y={nearest_point[1]}, z={nearest_point[2]}, intensity={nearest_point[3]}")
        print(f"Number of points with same intensity: {same_intensity_count}")
        print(f"Total number of points: {total_points}")
        print(f"Ratio of points with same intensity to total points: {ratio}")

