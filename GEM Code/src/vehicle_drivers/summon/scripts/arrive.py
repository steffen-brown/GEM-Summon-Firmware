#!/usr/bin/env python3
# ==============================================================
# arrive.py  –  Publishes /ARRIVAL/arrived once the vehicle is
#               both close to the goal and roughly perpendicular
#               to it.  Uses great‑circle distance and bearing.
# ==============================================================

import rospy
import math
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod
import csv, datetime as dt, os 

# ─── TUNABLE CONSTANTS ─────────────────────────────────────────
ANGLE_TOL   = math.radians(10)     # “perpendicular” band (deg → rad)
DIST_THRESH = 9.0                  # arrival radius in metres
R_EARTH     = 6_371_000.0          # mean Earth radius (m)
# ───────────────────────────────────────────────────────────────


class Arrive:
    def __init__(self):
        rospy.init_node('arrive', anonymous=True)

        # 2 a.  Make sure the folder exists
        log_dir = os.path.join('.', 'data', 'car_location')
        os.makedirs(log_dir, exist_ok=True)

        # 2 b.  Build a timestamped filename
        fname = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
        self._csv_path = os.path.join(log_dir, fname)

        # 2 c.  Open the file & keep the handle
        self._csv_file   = open(self._csv_path, mode='w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['timestamp', 'longitude', 'latitude'])  # header

        # right after you open the file
        rospy.on_shutdown(self._close_log)

        rospy.loginfo("Arrive node initializing…")

        self.rate = rospy.Rate(10)        # 10 Hz main loop

        # Goal & state
        self.goal_lat  = None
        self.goal_long = None
        self.curr_lat  = None
        self.curr_long = None
        self.curr_head = None   # degrees (0 = north, CW+)

        # Pub / sub
        self.arrived_pub = rospy.Publisher('/ARRIVAL/arrived',
                                           Bool, queue_size=1, latch=True)
        rospy.Subscriber("/septentrio_gnss/navsatfix",
                         NavSatFix, self.gps_cb)
        rospy.Subscriber("/ARRIVAL/goal_lat",
                         Float64, self.goal_lat_cb)
        rospy.Subscriber("/ARRIVAL/goal_long",
                         Float64, self.goal_long_cb)
        rospy.Subscriber("/septentrio_gnss/insnavgeod",
                         INSNavGeod, self.ins_cb)

        # Initialise “not arrived”
        self.arrived_pub.publish(False)
        rospy.loginfo("Arrive node ready – waiting for data.")

    # ─── CALLBACKS ────────────────────────────────────────────
    def gps_cb(self, msg):
        self.curr_lat  = msg.latitude
        self.curr_long = msg.longitude
        rospy.loginfo("GPS → lat %.6f  lon %.6f", self.curr_lat, self.curr_long)

        # append one line to the CSV and flush so nothing sits in a buffer
        self._csv_writer.writerow([
            dt.datetime.now().isoformat(timespec="seconds"),
            self.curr_long,
            self.curr_lat
        ])
        self._csv_file.flush()

    def goal_lat_cb(self, msg):
        self.goal_lat = msg.data
        rospy.loginfo("Goal lat set → %.6f", self.goal_lat)

    def goal_long_cb(self, msg):
        self.goal_long = msg.data
        rospy.loginfo("Goal lon set → %.6f", self.goal_long)

    def ins_cb(self, msg):
        self.curr_head = msg.heading       # degrees
        rospy.loginfo("Heading → %.2f°", self.curr_head)
    # ──────────────────────────────────────────────────────────

    def _close_log(self):
        if not self._csv_file.closed:
            self._csv_file.close()
            rospy.loginfo("Location CSV closed.")

    # ─── MAIN LOOP ────────────────────────────────────────────
    def run(self):
        rospy.loginfo("Arrive node running…")
        arrived = False

        # helper to format values that may still be None
        def fmt(val, prec=6):
            return f"{val:.{prec}f}" if val is not None else "—"

        while not rospy.is_shutdown():
            # Wait until we have everything
            if None in (self.curr_lat, self.curr_long, self.curr_head):
                rospy.loginfo_throttle(
                    5,
                    "Waiting… goal(%s, %s)  gps(%s, %s)  head=%s",
                    fmt(self.goal_lat), fmt(self.goal_long),
                    fmt(self.curr_lat), fmt(self.curr_long),
                    fmt(self.curr_head, 2))
                self.rate.sleep()
                continue

            # ── GREAT‑CIRCLE RANGE & BEARING ─────────────────
            lat1 = math.radians(self.curr_lat)
            lon1 = math.radians(self.curr_long)
            lat2 = math.radians(self.goal_lat)
            lon2 = math.radians(self.goal_long)

            print(self.goal_lat)
            print(self.goal_long)

            dlat = lat2 - lat1
            dlon = lon2 - lon1

            # Haversine distance
            a = (math.sin(dlat / 2) ** 2 +
                 math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            dist = R_EARTH * c                         # metres

            # Initial bearing (CW from north)
            y = math.sin(dlon) * math.cos(lat2)
            x = (math.cos(lat1) * math.sin(lat2) -
                 math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
            bearing = math.atan2(y, x)                # rad
            bearing = (bearing + 2 * math.pi) % (2 * math.pi)

            # ── ANGLE ERROR & THRESHOLDS ─────────────
            head_rad = math.radians(self.curr_head)
            head_rad = (head_rad + math.pi) % (2 * math.pi) - math.pi
            ang_err  = (bearing - head_rad + math.pi) % (2 * math.pi) - math.pi

            perpendicular = abs(math.pi / 2 - abs(ang_err)) <= ANGLE_TOL
            close_enough  = dist <= DIST_THRESH

            rospy.loginfo(
                "dist %.2fm  bear %.1f°  head %.1f°  err %.1f°  ⟂ %s  close %s",
                dist,
                math.degrees(bearing),
                self.curr_head,
                math.degrees(ang_err),
                perpendicular, close_enough)

            # ── ARRIVAL LATCH ────────────────────────
            if perpendicular and close_enough and not arrived:
                self.arrived_pub.publish(True)
                rospy.loginfo("** ARRIVED **")
                arrived = True

            self.rate.sleep()
    # ──────────────────────────────────────────────────────────


if __name__ == '__main__':
    try:
        Arrive().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arrive node shut down.")
