import rospy
from sensor_msgs.msg import LaserScan

class ObstacleDetector:
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.obstacle_detected = False

    def scan_callback(self, scan_data):
        for distance in scan_data.ranges:
            if distance < 0.5:  # Threshold for obstacles
                self.obstacle_detected = True
                return
        self.obstacle_detected = False
