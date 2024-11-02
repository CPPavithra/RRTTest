import numpy as np

class RRTStar:
    def __init__(self, start, goal, occupancy_grid, boundaries, max_iter=20000, step_size=0.5, neighbor_radius=0.1):
        self.start = start
        self.goal = goal
        self.occupancy_grid = occupancy_grid
        self.boundaries = boundaries
        self.max_iter = max_iter
        self.step_size = step_size
        self.neighbor_radius = neighbor_radius
        self.nodes = [start]
        self.edges = []
        self.parent = {start: None}  # Store the parent of each node

    def plan(self):
        for _ in range(self.max_iter):
            rand_point = self.get_random_point()
            nearest_node = self.nearest_neighbor(rand_point)
            new_node = self.steer(nearest_node, rand_point)

            if self.is_free(new_node):
                self.nodes.append(new_node)
                self.edges.append((nearest_node, new_node))
                self.parent[new_node] = nearest_node  # Set the parent for the new node
                self.rewire(new_node)

                # Check if new_node is close to the goal
                if np.linalg.norm(np.array(new_node) - np.array(self.goal)) < self.step_size:
                    print(f"Goal reached at: {new_node}")
                    self.nodes.append(self.goal)  
                    self.parent[self.goal] = new_node  # Link the goal to its parent
                    return self.construct_path()  # Return the optimal path after reaching the goal

        # After the loop, check if the goal is reachable
        if not any(np.linalg.norm(np.array(node) - np.array(self.goal)) < self.step_size for node in self.nodes):
            print(f"Goal {self.goal} not reached. Closest node: {self.nearest_neighbor(self.goal)}")

        return [], []  # Return empty path if goal is unreachable

    def get_random_point(self):
        min_x, max_x, min_y, max_y = self.boundaries
        rand_x = np.random.uniform(min_x, max_x)
        rand_y = np.random.uniform(min_y, max_y)
        return (rand_x, rand_y)

    def nearest_neighbor(self, point):
        return min(self.nodes, key=lambda node: np.linalg.norm(np.array(node) - np.array(point)))

    def steer(self, from_node, to_point):
        direction = np.array(to_point) - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = direction / distance * self.step_size
        new_node = np.array(from_node) + direction
        return (new_node[0], new_node[1])

    def is_free(self, node):
        x_index, y_index = self.to_grid_index(node)
        if not (0 <= x_index < self.occupancy_grid.shape[0] and 0 <= y_index < self.occupancy_grid.shape[1]):
            return False

        if self.occupancy_grid[x_index, y_index]:
            return False

        return True

    def to_grid_index(self, node):
        min_x, min_y = self.boundaries[0], self.boundaries[2]
        x_idx = int((node[0] - min_x) / self.step_size)
        y_idx = int((node[1] - min_y) / self.step_size)
        return (x_idx, y_idx)

    def rewire(self, new_node):
        for node in self.nodes:
            if node != new_node:
                if np.linalg.norm(np.array(node) - np.array(new_node)) < self.neighbor_radius:
                    if self.is_free(node):
                        self.edges.append((node, new_node))
                        self.parent[node] = new_node

    def construct_path(self):
        # Trace back from the goal to the start using the parent dictionary
        path = [self.goal]
        current_node = self.goal
        while current_node != self.start:
            current_node = self.parent[current_node]
            path.append(current_node)
        path.reverse()  # Reverse the path to go from start to goal

        # Create edges for visualization
        optimal_edges = [(path[i], path[i+1]) for i in range(len(path) - 1)]
        return path, optimal_edges

