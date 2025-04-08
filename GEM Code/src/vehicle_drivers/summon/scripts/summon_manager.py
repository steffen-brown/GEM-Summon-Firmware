#!/usr/bin/env python3

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import Bool, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from sensor_msgs.msg import NavSatFix


class SummonManager:

    def __init__(self):
        rospy.init_node('summon_manager', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz control loop

        # PACMod command publishers
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)

        # Webapp interaction
        self.status_pub = rospy.Publisher('/WEBAPP/status', Bool, queue_size=1)
        self.goal_long_sub = rospy.Subscriber('/WEBAPP/goal_long', Float64, self.goal_long_callback)
        self.goal_lat_sub = rospy.Subscriber('/WEBAPP/goal_lat', Float64, self.goal_lat_callback)

        # GPS
        self.gps_sub = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gps_callback)

        # Lane detection and exit parking management
        self.lane_detection_active_pub = rospy.Publisher('/LANE_DETECTION/active', Bool, queue_size=1)
        self.exit_parking_active_pub = rospy.Publisher('/EXIT_PARK/active', Bool, queue_size=1)
        self.exit_parking_active_sub = rospy.Subscriber('/EXIT_PARK/active', Bool, self.exit_parking_active_callback)
        self.exit_direction_pub = rospy.Publisher('/EXIT_PARK/direction', Bool, queue_size=1)

        # Object detection interface
        self.stop_sub = rospy.Subscriber('/OBJECT_DETECTION/stop', Bool, self.stop_callback)
        self.restart_sub = rospy.Subscriber('/OBJECT_DETECTION/restart', Bool, self.restart_callback)

        # Lane following mirror
        self.lf_enable_pub = rospy.Publisher('/LF_OUTPUT/enable', Bool, queue_size=1)
        self.lf_enable_sub = rospy.Subscriber('/LF_OUTPUT/enable', Bool, self.lf_enable_callback)
        self.lf_gear_sub = rospy.Subscriber('/LF_OUTPUT/shift_cmd', PacmodCmd, self.lf_gear_callback)
        self.lf_brake_sub = rospy.Subscriber('/LF_OUTPUT/brake_cmd', PacmodCmd, self.lf_brake_callback)
        self.lf_accel_sub = rospy.Subscriber('/LF_OUTPUT/accel_cmd', PacmodCmd, self.lf_accel_callback)
        self.lf_turn_sub = rospy.Subscriber('/LF_OUTPUT/turn_cmd', PacmodCmd, self.lf_turn_callback)
        self.lf_steer_sub = rospy.Subscriber('/LF_OUTPUT/steer_cmd', PositionWithSpeed, self.lf_steer_callback)

        # Exit parking mirror
        self.ep_enable_pub = rospy.Publisher('/EP_OUTPUT/enable', Bool, queue_size=1)
        self.ep_enable_sub = rospy.Subscriber('/EP_OUTPUT/enable', Bool, self.ep_enable_callback)
        self.ep_gear_sub = rospy.Subscriber('/EP_OUTPUT/shift_cmd', PacmodCmd, self.ep_gear_callback)
        self.ep_brake_sub = rospy.Subscriber('/EP_OUTPUT/brake_cmd', PacmodCmd, self.ep_brake_callback)
        self.ep_accel_sub = rospy.Subscriber('/EP_OUTPUT/accel_cmd', PacmodCmd, self.ep_accel_callback)
        self.ep_turn_sub = rospy.Subscriber('/EP_OUTPUT/turn_cmd', PacmodCmd, self.ep_turn_callback)
        self.ep_steer_sub = rospy.Subscriber('/EP_OUTPUT/steer_cmd', PositionWithSpeed, self.ep_steer_callback)

    # ============ Placeholder Callbacks ============

    def goal_long_callback(self, msg):
        rospy.loginfo(f"Received goal longitude: {msg.data}")

    def goal_lat_callback(self, msg):
        rospy.loginfo(f"Received goal latitude: {msg.data}")

    def gps_callback(self, msg):
        rospy.loginfo("Received GPS position")

    def exit_parking_active_callback(self, msg):
        rospy.loginfo(f"Exit parking active: {msg.data}")

    def stop_callback(self, msg):
        rospy.loginfo(f"Object detection stop: {msg.data}")

    def restart_callback(self, msg):
        rospy.loginfo(f"Object detection restart: {msg.data}")

    def lf_enable_callback(self, msg):
        rospy.loginfo(f"LF enable: {msg.data}")

    def lf_gear_callback(self, msg):
        rospy.loginfo("LF gear command received")

    def lf_brake_callback(self, msg):
        rospy.loginfo("LF brake command received")

    def lf_accel_callback(self, msg):
        rospy.loginfo("LF accel command received")

    def lf_turn_callback(self, msg):
        rospy.loginfo("LF turn command received")

    def lf_steer_callback(self, msg):
        rospy.loginfo("LF steer command received")

    def ep_enable_callback(self, msg):
        rospy.loginfo(f"EP enable: {msg.data}")

    def ep_gear_callback(self, msg):
        rospy.loginfo("EP gear command received")

    def ep_brake_callback(self, msg):
        rospy.loginfo("EP brake command received")

    def ep_accel_callback(self, msg):
        rospy.loginfo("EP accel command received")

    def ep_turn_callback(self, msg):
        rospy.loginfo("EP turn command received")

    def ep_steer_callback(self, msg):
        rospy.loginfo("EP steer command received")

    # ============ Run Loop ============

    def run(self):
        rospy.loginfo("SummonManager node is running...")
        while not rospy.is_shutdown():
            # Add logic here if needed for coordination between modules
            self.rate.sleep()


# ============ Main Entry ============

if __name__ == '__main__':
    try:
        node = SummonManager()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("SummonManager node shut down.")
