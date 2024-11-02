import numpy as np
from rrt_star import RRTStar
from grid_map import create_grid_map

def print_final_path(nodes):
    print("Final Path Coordinates:")
    for node in nodes:
        print(f"Coordinate: ({node[0]}, {node[1]})")
    print("Goal reached!")

def main():
    start = (40, -140)
    goal = (20, -90)
    plyfile = '3dmap.ply'
    grid_resolution = 0.05

    # Create grid map from the PLY file
    occupancy_grid, boundaries = create_grid_map(plyfile, grid_resolution)

    # Plan the RRT* path
    rrt_star = RRTStar(start, goal, occupancy_grid, boundaries)
    nodes, edges = rrt_star.plan()

    # Print the coordinates of the final path
    print_final_path(nodes)

    # Save the occupancy grid and boundaries for further use
    np.save("occupancy_grid.npy", occupancy_grid)
    np.save("boundaries.npy", np.array(boundaries))  # Saving boundaries as an array for easy loading

if __name__ == "__main__":
    main()

