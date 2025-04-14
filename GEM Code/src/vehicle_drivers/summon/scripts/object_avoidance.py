#!/usr/bin/env python3

# ROS Headers
import rospy
import ros_numpy
# GEM Sensor Headers
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
import numpy as np

class ObjectAvoidance:

    def __init__(self):
        rospy.init_node('object_avoidance', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz control loop
        self.obstacle_detected = False
        
        # Publishers
        self.stop_pub = rospy.Publisher('/OBJECT_DETECTION/stop', Bool, queue_size=1)
        self.restart_pub = rospy.Publisher('/OBJECT_DETECTION/restart', Bool, queue_size=1)

        # Subscriber
        self.lidar_points_sub = rospy.Subscriber('/ouster/points', PointCloud2, self.lidar_callback)

    def lidar_callback(self, msg):
        rospy.loginfo("Received LiDAR point cloud data")
        try:
            pc = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

            x = pc['x']
            y = pc['y']
            z = pc['z']

            # Filter valid points
            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            x = x[valid]
            y = y[valid]
            z = z[valid]
            ring = pc['ring'][valid]

            # Region of interest (in meters) #Note for me: check this based on what we are seeing
            # defining x as Front: 0 < x < 5, Side: -1 < y < 1, Height: -1.0 < z < 2
            roi = (x > 0.5) & (x < 5.0) & (np.abs(y) < 1.0) & (z > -1.0) & (z < 2.0) & (ring > 4) & (ring < 12)
            #check the ring tomorrow - depending on which ones gives what generally using 16 ring config. 

            if np.count_nonzero(roi) > 50:  # Threshold for obstacle - saying 50 points - change this how it goes tomorrow 
                if not self.obstacle_detected:
                    rospy.logwarn("Obstacle detected. STOPPING.")
                    self.stop_pub.publish(Bool(data=True))
                    self.obstacle_detected = True
            else:
                if self.obstacle_detected:
                    rospy.loginfo("Obstacle cleared. RESUMING.")
                    self.restart_pub.publish(Bool(data=True))
                    self.obstacle_detected = False

        except Exception as e:
            rospy.logerr(f"Error in lidar_callback: {e}")

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
