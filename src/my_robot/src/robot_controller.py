import rospy
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def follow_path(self, path):
        for waypoint in path:
            move_cmd = Twist()
            move_cmd.linear.x = 0.5  # Adjust velocity
            move_cmd.angular.z = 0.0  # Adjust angular velocity if needed
            self.pub.publish(move_cmd)
            rospy.sleep(1)  # Adjust timing based on distances
