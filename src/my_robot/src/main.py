import rclpy
from rclpy.node import Node
from path_planner import AStarPlanner
from robot_controller import RobotController
from obstacle_detector import ObstacleDetector

class MainNode(Node):
    def __init__(self):
    super().__init__('a_star_robot')

    # Example grid (0 = free, 1 = obstacle)
    grid = np.zeros((10, 10))
    grid[3:7, 4] = 1  # Obstacle

    planner = AStarPlanner(grid)
    controller = RobotController()
    detector = ObstacleDetector()

    start = (0, 0)
    goal = (9, 9)

    path = planner.plan(start, goal)

    if path:
        rospy.loginfo("Path found. Following path.")
        controller.follow_path(path)
    else:
        rospy.loginfo("No path found.")

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()