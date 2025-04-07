#!/usr/bin/env python3

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import Float64, Int32


class Networking:

    def __init__(self):
        rospy.init_node('networking', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz control loop

        # Publishers
        self.goal_long_pub = rospy.Publisher('/WEBAPP/goal_long', Float64, queue_size=1)
        self.goal_lat_pub = rospy.Publisher('/WEBAPP/goal_lat', Float64, queue_size=1)

        # Subscriber
        self.status_sub = rospy.Subscriber('/WEBAPP/status', Int32, self.status_callback)

    def status_callback(self, msg):
        rospy.loginfo(f"Received status: {msg.data}")

    def run(self):
        rospy.loginfo("Networking node is running...")
        while not rospy.is_shutdown():
            # Placeholder loop logic (optional publishing)
            self.rate.sleep()


# ============================
# Main Entry Point for Node
# ============================

if __name__ == '__main__':
    try:
        node = Networking()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Networking node shut down.")
