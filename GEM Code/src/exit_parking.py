#!/usr/bin/env python3

# ROS Headers
import rospy

# GEM Sensor Headers
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt


class ExitParking:

    def __init__(self):
        rospy.init_node('exit_parking', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz control loop

        # Subscribers
        self.enable_sub = rospy.Subscriber("/EP_OUTPUT/enable", Bool, self.enable_callback)
        self.speed_sub = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.active_sub = rospy.Subscriber("/EXIT_PARK/active", Bool, self.active_callback)
        self.imu_sub = rospy.Subscriber('/novatel/imu', Imu, self.imu_callback)
        self.contrasted_image_sub = rospy.Subscriber('/LANE_DETECTION/contrasted_image', Image, self.contrasted_image_callback)
        self.image_sub = rospy.Subscriber('zed2/zed_node/rgb/image_rect_color', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.depth_callback)

        # Publishers
        self.enable_pub = rospy.Publisher('/EP_OUTPUT/enable', Bool, queue_size=1)
        self.gear_pub = rospy.Publisher('/EP_OUTPUT/shift_cmd', PacmodCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/EP_OUTPUT/brake_cmd', PacmodCmd, queue_size=1)
        self.accel_pub = rospy.Publisher('/EP_OUTPUT/accel_cmd', PacmodCmd, queue_size=1)
        self.turn_pub = rospy.Publisher('/EP_OUTPUT/turn_cmd', PacmodCmd, queue_size=1)
        self.steer_pub = rospy.Publisher('/EP_OUTPUT/steer_cmd', PositionWithSpeed, queue_size=1)

        self.active_pub = rospy.Publisher('/EXIT_PARK/active', Bool, queue_size=1)
        self.direction_pub = rospy.Publisher('/EXIT_PARK/direction', Bool, queue_size=1)

    # =====================
    # Callback Placeholders
    # =====================

    def enable_callback(self, msg):
        rospy.loginfo("Received enable message")

    def speed_callback(self, msg):
        rospy.loginfo(f"Vehicle speed: {msg.vehicle_speed:.2f} m/s")

    def active_callback(self, msg):
        rospy.loginfo(f"Exit park active: {msg.data}")

    def imu_callback(self, msg):
        rospy.loginfo("Received IMU data")

    def contrasted_image_callback(self, msg):
        rospy.loginfo("Received contrasted image")

    def image_callback(self, msg):
        rospy.loginfo("Received RGB image")

    def depth_callback(self, msg):
        rospy.loginfo("Received depth image")

    def run(self):
        rospy.loginfo("ExitParking node is running...")
        while not rospy.is_shutdown():
            # Placeholder control loop logic
            self.rate.sleep()


# ============================
# Main Entry Point for Node
# ============================

if __name__ == '__main__':
    try:
        node = ExitParking()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ExitParking node shut down.")
