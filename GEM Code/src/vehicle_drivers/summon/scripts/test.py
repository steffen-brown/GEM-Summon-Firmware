#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64, Int32
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from sensor_msgs.msg import NavSatFix

class TopicMonitor:
    def __init__(self):
        rospy.init_node('topic_monitor', anonymous=True)

        # List of (topic, message type)
        topics = [
            ('/pacmod/as_rx/enable', Bool),
            ('/pacmod/as_rx/shift_cmd', PacmodCmd),
            ('/pacmod/as_rx/brake_cmd', PacmodCmd),
            ('/pacmod/as_rx/accel_cmd', PacmodCmd),
            ('/pacmod/as_rx/turn_cmd', PacmodCmd),
            ('/pacmod/as_rx/steer_cmd', PositionWithSpeed),

            ('/pacmod/as_tx/enable', Bool),

            ('/WEBAPP/status', Int32),
            #('/WEBAPP/goal_long', Float64),
            #('/WEBAPP/goal_lat', Float64),

            ('/septentrio_gnss/navsatfix', NavSatFix),

            ('/LANE_DETECTION/active', Bool),
            ('/EXIT_PARK/active', Bool),

            ('/OBJECT_DETECTION/stop', Bool),
            ('/OBJECT_DETECTION/restart', Bool),

            ('/LF_OUTPUT/enable', Bool),
            ('/LF_OUTPUT/shift_cmd', PacmodCmd),
            ('/LF_OUTPUT/brake_cmd', PacmodCmd),
            ('/LF_OUTPUT/accel_cmd', PacmodCmd),
            ('/LF_OUTPUT/turn_cmd', PacmodCmd),
            ('/LF_OUTPUT/steer_cmd', PositionWithSpeed),

            ('/EP_OUTPUT/enable', Bool),
            ('/EP_OUTPUT/shift_cmd', PacmodCmd),
            ('/EP_OUTPUT/brake_cmd', PacmodCmd),
            ('/EP_OUTPUT/accel_cmd', PacmodCmd),
            ('/EP_OUTPUT/turn_cmd', PacmodCmd),
            ('/EP_OUTPUT/steer_cmd', PositionWithSpeed),
        ]

        # Create all subscribers dynamically
        for topic, msg_type in topics:
            rospy.Subscriber(topic, msg_type, self.generic_callback, callback_args=topic)

        rospy.loginfo("Topic monitor is running...")
        rospy.spin()

    def generic_callback(self, msg, topic_name):
        rospy.loginfo(f"[{topic_name}] → {msg}")

if __name__ == '__main__':
    try:
        TopicMonitor()
    except rospy.ROSInterruptException:
        pass
