#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import matplotlib.pyplot as plt

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.map_data = None

    def map_callback(self, msg: OccupancyGrid):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        # ✅ Set map_data FIRST
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.get_logger().info(
            f"Map loaded: {self.map_width}x{self.map_height} at resolution {self.map_resolution}"
        )

    # ✅ THEN call obstacle inflation and save image
        self.inflate_obstacles(inflation_radius=1)
        self.debug_save_map_image()

        # Initial start and goal guesses
        raw_start = (17, 60)   # ← from your click: pixel (167, 243)
        raw_goal = (61, 68)    # ← from your click: pixel (579, 162)

        start = self.find_nearest_free(raw_start)
        goal = self.find_nearest_free(raw_goal)

        self.get_logger().info(f"Adjusted start: {start}, occupancy: {self.map_data[start[1], start[0]]}")
        self.get_logger().info(f"Adjusted goal: {goal}, occupancy: {self.map_data[goal[1], goal[0]]}")

        path = self.astar(start, goal)
        if path:
            self.get_logger().info("Path found!")
            self.publish_path(path)
        else:
            self.get_logger().warn("No path found!")
    
    def debug_save_map_image(self, filename="/tmp/inflated_map_debug.png"):
        plt.figure(figsize=(8, 10))
        plt.imshow(self.map_data, cmap='gray_r', origin='lower')
        plt.title('Inflated Occupancy Grid')
        plt.axis('off')
        plt.savefig(filename, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f"Saved inflated map image to {filename}")


    def inflate_obstacles(self, inflation_radius=1):
        inflated_map = np.copy(self.map_data)
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.map_data[y, x] > 50:
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                                inflated_map[ny, nx] = 100
        self.map_data = inflated_map

    def find_nearest_free(self, cell, max_radius=10):
        x, y = cell
        if self.map_data[y, x] <= 50:
            return cell  # Already free

        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        if self.map_data[ny, nx] == 00:
                            return (nx, ny)

        self.get_logger().warn(f"No free cell found near {cell}. Using original.")
        return cell

    def astar(self, start, goal):
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_set = []
        heapq.heappush(open_set, (heuristic(start, goal), 0, start, [start]))
        visited = set()

        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            if current in visited:
                continue

            if current == goal:
                self.save_path_overlay(visited, path, start, goal)
                return path

            visited.add(current)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not (0 <= neighbor[0] < self.map_width and 0 <= neighbor[1] < self.map_height):
                    continue
                if neighbor in visited:
                    continue
                if self.map_data[neighbor[1], neighbor[0]] != 0:
                    continue

                step_cost = 1.414 if dx != 0 and dy != 0 else 1
                new_cost = cost + 1
                new_total = new_cost + heuristic(neighbor, goal)
                if len(path) == 1:
                    self.get_logger().info(f"pushing neighbor: {neighbor}, cost: {new_cost}")
                heapq.heappush(open_set, (new_total, new_cost, neighbor, path + [neighbor]))
        
        self.save_debug_search_image(visited, start, goal)
        return None
    
    def save_debug_search_image(self, visited, start, goal, filename="/tmp/astar_search_debug.png"):
        fig, ax = plt.subplots(figsize=(8, 10))
        ax.imshow(self.map_data, cmap='gray_r', origin='lower')

        # Plot visited nodes in green
        for x, y in visited:
            ax.plot(x, y, 'g.', markersize=2)

        # Plot start in red
        ax.plot(start[0], start[1], 'ro', markersize=6, label='Start')

        # Plot goal in blue
        ax.plot(goal[0], goal[1], 'bo', markersize=6, label='Goal')

        ax.set_title("A* Visited Cells Debug View")
        ax.legend()
        plt.axis('off')
        plt.savefig(filename, bbox_inches='tight')
        plt.close()

        self.get_logger().info(f"Saved A* search debug image to {filename}")

    def save_path_overlay(self, visited, path, start, goal, filename="/tmp/astar_path_debug.png"):
        fig, ax = plt.subplots(figsize=(8, 10))
        ax.imshow(self.map_data, cmap='gray_r', origin='lower')

        # Visited
        for x, y in visited:
            ax.plot(x, y, 'g.', markersize=2)

        # Path
        if path:
            px, py = zip(*path)
            ax.plot(px, py, 'y-', linewidth=2, label='Path')

        # Start & Goal
        ax.plot(start[0], start[1], 'ro', markersize=6, label='Start')
        ax.plot(goal[0], goal[1], 'bo', markersize=6, label='Goal')

        ax.set_title("A* Path Overlay")
        ax.legend()
        plt.axis('off')
        plt.savefig(filename, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f"Saved A* path overlay to {filename}")



    def publish_path(self, grid_path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for cell in grid_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.origin[0] + (cell[0] + 0.5) * self.map_resolution
            pose.pose.position.y = self.origin[1] + (cell[1] + 0.5) * self.map_resolution
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Published path to 'planned_path' topic.")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
