import numpy as np
import sys

def read_pcd(file_path):
    intensity_count = {}
    with open(file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith('DATA'):
                break
            values = line.split()
            if len(values) >= 4:
                intensity = int(float(values[3]) * 255)  # Assuming intensity is stored in the fourth column
                if intensity in intensity_count:
                    intensity_count[intensity] += 1
                else:
                    intensity_count[intensity] = 1
    return intensity_count

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python readPCDlabel.py <pcd_file>")
        sys.exit(1)

    pcd_file = sys.argv[1]
    try:
        intensity_count = read_pcd(pcd_file)
        print("Intensity count:")
        for intensity, count in intensity_count.items():
            print(f"Intensity {intensity}: {count} points")
    except Exception as e:
        print(f"Error: {e}")

