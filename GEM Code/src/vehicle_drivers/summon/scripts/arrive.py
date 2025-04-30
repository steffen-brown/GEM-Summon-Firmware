#!/usr/bin/env python3

import rospy
import ros_numpy
import numpy as np
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import NavSatFix

class Arrive:

    def __init__(self):
        rospy.init_node('arrive', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz control loop
        
        # Publishers
        self.arrived_pub = rospy.Publisher('/ARRIVAL/arrived', Bool, queue_size=1, latch=True)

        # Subscribers
        self.gps_sub = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gps_callback)
        self.goal_long_sub = rospy.Subscriber('/ARRIVAL/goal_long', Float64, self.goal_long_callback)
        self.goal_lat_sub  = rospy.Subscriber('/ARRIVAL/goal_lat', Float64, self.goal_lat_callback)

        self.arrived_pub.publish(Bool(data=False))


    def gps_callback(self, msg):
        # TODO: Add GPS handling logic here
        pass

    def goal_lat_callback(self, msg):
        # TODO: Store goal latitude from msg.data
        pass

    def goal_long_callback(self, msg):
        # TODO: Store goal longitude from msg.data
        pass

    def run(self):
        rospy.loginfo("Arrive node is running...")
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = Arrive()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arrive node shut down.")
