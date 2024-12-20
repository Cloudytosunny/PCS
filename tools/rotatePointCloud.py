import sys
import os
import numpy as np

def rotate_point_cloud_x(point_cloud, angle):
  
    rotation_matrix = np.array([[1, 0, 0, 0],
                                 [0, np.cos(angle), -np.sin(angle), 0],
                                 [0, np.sin(angle), np.cos(angle), 0],
                                 [0, 0, 0, 1]])
 
    rotated_point_cloud = np.dot(point_cloud, rotation_matrix.T)
    return rotated_point_cloud

def rotate_point_cloud_y(point_cloud, angle):
  
    rotation_matrix = np.array([[np.cos(angle), 0, np.sin(angle), 0],
                                 [0, 1, 0, 0],
                                 [-np.sin(angle), 0, np.cos(angle), 0],
                                 [0, 0, 0, 1]])
 
    rotated_point_cloud = np.dot(point_cloud, rotation_matrix.T)
    return rotated_point_cloud

def rotate_point_cloud_z(point_cloud, angle):
 
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                                 [np.sin(angle), np.cos(angle), 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
  
    rotated_point_cloud = np.dot(point_cloud, rotation_matrix.T)
    return rotated_point_cloud

def load_pcd_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    data_start_index = lines.index('DATA ascii\n') + 1
  
    points = []
    for line in lines[data_start_index:]:
        if line.strip():  
            data = line.split()
            points.append([float(coord) for coord in data])
    return np.array(points)

def save_pcd_file(file_path, points):
    with open(file_path, 'w') as file:
        file.write('VERSION .7\n')
        file.write('FIELDS x y z intensity\n')
        file.write('SIZE 4 4 4 4\n')
        file.write('TYPE F F F F\n')
        file.write('COUNT 1 1 1 1\n')
        file.write('WIDTH {}\n'.format(len(points)))
        file.write('HEIGHT 1\n')
        file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        file.write('POINTS {}\n'.format(len(points)))
        file.write('DATA ascii\n')
    
        for point in points:
            file.write(' '.join(map(str, point)) + '\n')

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python rotate.py input_cloud.pcd")
        sys.exit(1)

    input_file = sys.argv[1]
    if not os.path.isfile(input_file):
        print(f"Error: File '{input_file}' not found.")
        sys.exit(1)


    point_cloud = load_pcd_file(input_file)


    # rotated_point_cloud = rotate_point_cloud_x(point_cloud, np.radians(30))

   
    rotated_point_cloud = rotate_point_cloud_y(point_cloud, np.radians(-15))

 
    output_file = os.path.splitext(input_file)[0] + '_rotated_y-15.pcd'

  
    save_pcd_file(output_file, rotated_point_cloud)
    
    print(f"Point cloud rotated and saved as '{output_file}'.")

