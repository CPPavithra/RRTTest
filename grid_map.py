import numpy as np
from pyntcloud import PyntCloud

def create_grid_map(ply_file, grid_resolution, height=0.1):
    # Load point cloud from PLY file
    cloud = PyntCloud.from_file(ply_file)
    points = cloud.xyz  # Extracting x, y, z coordinates

    # Define the boundaries of the grid
    min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
    min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
    x_bins = int((max_x - min_x) / grid_resolution) + 1  # Include the last bin
    y_bins = int((max_y - min_y) / grid_resolution) + 1  # Include the last bin
    occupancy_grid = np.zeros((x_bins, y_bins), dtype=bool)

    # Fill the occupancy grid
    for point in points:
        x_idx = int((point[0] - min_x) / grid_resolution)
        y_idx = int((point[1] - min_y) / grid_resolution)

        # Ensure indices are within the valid range
        if 0 <= x_idx < x_bins and 0 <= y_idx < y_bins:
            if point[2] > height:  # Check height condition
                occupancy_grid[x_idx, y_idx] = True
        else:
            # Debug information
            print(f"Warning: Point {point} maps to out-of-bounds index (x_idx: {x_idx}, y_idx: {y_idx})")

    # Mark grid cells outside the defined boundaries as occupied
    for i in range(x_bins):
        for j in range(y_bins):
            if (min_x + i * grid_resolution < min_x or min_x + i * grid_resolution > max_x or
                min_y + j * grid_resolution < min_y or min_y + j * grid_resolution > max_y):
                occupancy_grid[i, j] = True

    return occupancy_grid, (min_x, max_x, min_y, max_y)

if __name__ == "__main__":
    ply_file = '3dmap.ply'  # Path to your PLY file
    grid_resolution = 0.05   # Grid resolution in meters
    occupancy_grid, boundaries = create_grid_map(ply_file, grid_resolution)

    print("Occupancy Grid:")
    print(occupancy_grid)
    print("Boundaries:", boundaries)

