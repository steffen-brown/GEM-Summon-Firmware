#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import NavSatFix, INSNavGeod

ANGLE_TOL    = math.radians(10)  # how close to 90 ° is acceptable
DIST_THRESH  = 1.5               # metres

class Arrive:

    def __init__(self):
        rospy.init_node('arrive', anonymous=True)
        rospy.loginfo("Arrive node initializing...")  # init log :contentReference[oaicite:0]{index=0}

        self.rate = rospy.Rate(10)  # 10 Hz loop

        # State variables
        self.goal_lat     = None
        self.goal_long    = None
        self.curr_lat     = None
        self.curr_long    = None
        self.curr_heading = None

        # Publisher
        self.arrived_pub = rospy.Publisher('/ARRIVAL/arrived', Bool, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/ARRIVAL/goal_lat", Float64, self.goal_lat_callback)
        rospy.Subscriber("/ARRIVAL/goal_long", Float64, self.goal_long_callback)
        rospy.Subscriber( "/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)

        # Start with arrived = False
        self.arrived_pub.publish(Bool(data=False))
        rospy.loginfo("Arrive node initialized and waiting for data.")

    def gps_callback(self, msg):
        self.curr_lat  = round(msg.latitude,  6)
        self.curr_long = round(msg.longitude, 6)
        rospy.loginfo("GPS update → lat: %.6f, lon: %.6f",self.curr_lat, self.curr_long)

    def goal_lat_callback(self, msg):
        self.goal_lat = msg.data
        rospy.loginfo("Goal latitude set to %.6f", self.goal_lat) 

    def goal_long_callback(self, msg):
        self.goal_long = msg.data
        rospy.loginfo("Goal longitude set to %.6f", self.goal_long)

    def ins_callback(self, msg):
        self.curr_heading = round(msg.heading, 6)
        rospy.loginfo("IMU heading update → %.6f°", self.curr_heading) 

    def run(self):
        rospy.loginfo("Arrive node is running...")
        arrived = False 

        while not rospy.is_shutdown():
            # Wait for all inputs
            if None in (self.goal_lat, self.goal_long,
                        self.curr_lat, self.curr_long, self.curr_heading):
                rospy.loginfo_throttle(5,  # seconds
                    "Waiting for data → goal(lat,lon)=%s,%s  GPS(lat,lon)=%s,%s  heading=%s",
                    self.goal_lat, self.goal_long,
                    self.curr_lat, self.curr_long,
                    self.curr_heading
                )
                self.rate.sleep()
                continue

            # 2‑flat ENU conversion
            m_per_deg_lat = 111_132.92
            m_per_deg_lon = (
                111_132.92 * math.cos(math.radians(self.curr_lat)))
            d_lat = self.goal_lat  - self.curr_lat
            d_lon = self.goal_long - self.curr_long
            north = d_lat * m_per_deg_lat
            east  = d_lon * m_per_deg_lon
            dist  = math.hypot(east, north)

            # 3‑flat bearing (0 = North, CW+)
            bearing = math.atan2(east, north)

            # Heading error wrapped to [−π, π]
            head     = math.radians(self.curr_heading)
            head     = (head + math.pi) % (2*math.pi) - math.pi
            ang_diff = (bearing - head + math.pi) % (2*math.pi) - math.pi

            perpendicular = abs(math.pi/2 - abs(ang_diff)) <= ANGLE_TOL
            close_enough  = dist <= DIST_THRESH

            # Debug log for each loop
            rospy.loginfo(
                "dist=%.2fm  bearing=%.1f°  head=%.1f°  err=%.1f°  perp=%s  close=%s",
                dist,
                math.degrees(bearing),
                self.curr_heading,
                math.degrees(ang_diff),
                perpendicular,
                close_enough
            ) 

            # Publish arrival once
            if perpendicular and close_enough and not arrived:
                self.arrived_pub.publish(Bool(data=True))
                rospy.loginfo("** ARRIVED **")
                arrived = True

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = Arrive()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arrive node shut down.")
