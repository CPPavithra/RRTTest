import numpy as np
from rrt_star import RRTStar
from grid_map import create_grid_map
import matplotlib.pyplot as plt

def visualize_path(nodes, edges, occupancy_grid, boundaries, filename='pathfar.png'):
    plt.figure(figsize=(10, 10))

    min_x, max_x, min_y, max_y = boundaries
    x_range = np.linspace(min_x, max_x, occupancy_grid.shape[0])
    y_range = np.linspace(min_y, max_y, occupancy_grid.shape[1])

    #plot grid
    plt.imshow(occupancy_grid.T, extent=(min_x, max_x, min_y, max_y), origin='lower', cmap='Greys', alpha=0.3)
    #plot edges
    for edge in edges:
        plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color='blue')

    # Plot the nodes
    plt.scatter(*zip(*nodes), color='red', s=10)

    # Mark the start and goal
    plt.scatter(nodes[0][0], nodes[0][1], color='green', label='Start', s=100)
    plt.scatter(nodes[-1][0], nodes[-1][1], color='purple', label='Goal', s=100)

    plt.title("RRT* Path on Occupancy Grid")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid(True)

    # Save the figure as a PNG file
    plt.savefig(filename)
    plt.close()

def main():
    #start = (-54.4546012878418,-90.22299194335938)
    #goal = (-54.404598336083984, -79.17298889160156)
    #goal = (-8.80459976196289, -166.62298583984375)
    #start = (40,-140)
    #goal = (60,-140) #INSIDE THE BOUND (ROOM)
    #start = (-20,-150)
    #goal = (-20,-100)  #TO CHECK OUT OF BOUNDS
    start = (40,-140)
    goal = (20,-90)
    plyfile = '3dmap.ply'
    grid_resolution = 0.05

    # Create grid map from the PLY file
    occupancy_grid, boundaries = create_grid_map(plyfile, grid_resolution)

    # Plan the RRT* path
    rrt_star = RRTStar(start, goal, occupancy_grid, boundaries)
    final_nodes, final_edges = rrt_star.plan()

    # Visualize and save the path on the occupancy grid
    visualize_path(final_nodes, final_edges, occupancy_grid, boundaries, filename='pathfar.png')

    import numpy as np

    # Assuming `occupancy_grid` and `boundaries` are the outputs from your `create_grid_map` function
    np.save("occupancy_grid.npy", occupancy_grid)
    np.save("boundaries.npy", np.array(boundaries))  # Saving boundaries as an array for easy loading


if __name__ == "__main__":
    main()

