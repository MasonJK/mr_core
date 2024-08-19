import numpy as np
import heapq
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time

class GPP:
    def __init__(self, map_data, map_width, map_height, map_resolution, origin, log_func):
        self.map_data = np.array(map_data).reshape((map_height, map_width))
        self.map_resolution = map_resolution
        self.origin = origin  # Origin from the YAML file
        self.log_func = log_func

    def position_to_grid(self, position):
        """Convert a position (x, y) to a grid cell (i, j)"""
        i = round((position[0] - self.origin[0]) / self.map_resolution)
        j = round((position[1] - self.origin[1]) / self.map_resolution)
        self.log_func(f"Position {position} converted to grid {(j, i)}")
        return (j, i)  # Swap i and j to match the (row, col) order

    def grid_to_position(self, grid):
        """Convert a grid cell (i, j) back to a position (x, y)"""
        x = grid[1] * self.map_resolution + self.origin[0]
        y = grid[0] * self.map_resolution + self.origin[1]
        self.log_func(f"Grid {grid} converted to position {(x, y)}")
        return (x, y)

    def heuristic(self, a, b):
        # Manhattan distance as heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan_gpp(self, start_position, goal_position):
        self.log_func(f"Planning path from {start_position} to {goal_position}")

        start_grid = self.position_to_grid(start_position)
        goal_grid = self.position_to_grid(goal_position)

        self.log_func(f"start grid : {start_grid}")
        self.log_func(f"goal grid : {goal_grid}")

        grid_path = self.a_star_algorithm(start_grid, goal_grid)

        if not grid_path:
            self.log_func("No path found")
            return []

        waypoints = [self.grid_to_position(cell) for cell in grid_path]
        return waypoints

    def a_star_algorithm(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1  # Assuming uniform cost for simplicity
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # Return empty if no path found

    def get_neighbors(self, node):
        # Get neighbors considering occupancy grid bounds and checking for obstacles
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])

            # Check if the neighbor is within the bounds of the map and not an obstacle
            if 0 <= neighbor[0] < self.map_data.shape[0] and 0 <= neighbor[1] < self.map_data.shape[1]:
                if self.map_data[neighbor[0], neighbor[1]] == 0:  # Assuming 0 represents free space
                    neighbors.append(neighbor)

        return neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # Return reversed path

    def convert_waypoints_to_pose_stamped(self, waypoints, frame_id="map"):
        """Convert a list of (x, y) waypoints to PoseStamped messages"""
        pose_stamped_list = []
        for point in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = Time().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation, facing forward
            pose_stamped_list.append(pose)
        return pose_stamped_list
