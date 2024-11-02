import numpy as np

def print_free_spaces(occupancy_grid, grid_resolution, boundaries):
    min_x, max_x, min_y, max_y = boundaries
    
    # Iterate through each cell in the occupancy grid
    print("Free spaces (coordinates):")
    for x_idx in range(occupancy_grid.shape[0]):
        for y_idx in range(occupancy_grid.shape[1]):
            if not occupancy_grid[x_idx, y_idx]:  # Check if the cell is free
                # Calculate real-world coordinates from grid indices
                x_coord = min_x + x_idx * grid_resolution
                y_coord = min_y + y_idx * grid_resolution
                print(f"({x_coord}, {y_coord})")

# Load the occupancy grid and boundaries
occupancy_grid = np.load("occupancy_grid.npy")
boundaries = np.load("boundaries.npy")
grid_resolution = 0.05  # Set the same grid resolution you used in the main code

# Print free spaces
print_free_spaces(occupancy_grid, grid_resolution, boundaries)

