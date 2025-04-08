#!/usr/bin/env python3

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2


class ObjectAvoidance:

    def __init__(self):
        rospy.init_node('object_avoidance', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz control loop

        # Publishers
        self.stop_pub = rospy.Publisher('/OBJECT_DETECTION/stop', Bool, queue_size=1)
        self.restart_pub = rospy.Publisher('/OBJECT_DETECTION/restart', Bool, queue_size=1)

        # Subscriber
        self.lidar_points_sub = rospy.Subscriber('/ouster/points', PointCloud2, self.lidar_callback)

    def lidar_callback(self, msg):
        rospy.loginfo("Received LiDAR point cloud data")

        # Placeholder: Add logic to detect obstacles and publish stop/restart
        # Example:
        # if obstacle_detected:
        #     self.stop_pub.publish(Bool(data=True))
        # else:
        #     self.restart_pub.publish(Bool(data=True))

    def run(self):
        rospy.loginfo("ObjectAvoidance node is running...")
        while not rospy.is_shutdown():
            self.rate.sleep()


# ============================
# Main Entry Point for Node
# ============================

if __name__ == '__main__':
    try:
        node = ObjectAvoidance()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ObjectAvoidance node shut down.")
