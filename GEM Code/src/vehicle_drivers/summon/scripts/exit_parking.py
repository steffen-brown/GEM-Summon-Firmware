#!/usr/bin/env python3

"""
ExitParking node (original skeleton + lane‑distance extension)
=============================================================
• Keeps all original subscribers/publishers so nothing breaks downstream.
• **Adds** a synchronised RGB+Depth callback that:
    1. Detects the perpendicular lane marking in front of the car (Canny+Hough, tolerant to ±22 deg).
    2. Queries depth for those lane pixels and publishes the median distance on
       `/EXIT_PARK/lane_distance` (Float32).
    3. Publishes a debug image on `/EXIT_PARK/lane_debug`.
• All new topics/parameters are namespaced so they can be ignored if unused.

If you run on GEM E4 (OAK‑D) set:
    rosparam set /exit_parking/rgb_topic  /oak/rgb/image_raw
    rosparam set /exit_parking/depth_topic /zdepth_raw

For GEM E2 (ZED2) defaults already match.
"""

# ───────── Original imports ───────────────────────────────────────────────────
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, CameraInfo
from std_msgs.msg import Bool, Float32
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ExitParking:
    def __init__(self):
        rospy.init_node('exit_parking', anonymous=True)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        # ───────── Parameters (new) ─────────────────────────────────────────
        self.rgb_topic   = rospy.get_param('~rgb_topic',   '/zed2/zed_node/rgb/image_rect_color')
        self.depth_topic = rospy.get_param('~depth_topic', '/zed2/zed_node/depth/depth_registered')
        self.slope_thresh = rospy.get_param('~slope_thresh', 0.4)  # |slope| <= 0.4 → ~22°
        self.roi_top_frac = rospy.get_param('~roi_top_frac', 0.40)
        self.roi_bot_frac = rospy.get_param('~roi_bot_frac', 0.70)

        # ───────── Existing subscribers ────────────────────────────────────
        self.enable_sub = rospy.Subscriber('/EP_OUTPUT/enable', Bool, self.enable_callback)
        self.speed_sub  = rospy.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt', VehicleSpeedRpt, self.speed_callback)
        self.active_sub = rospy.Subscriber('/EXIT_PARK/active', Bool, self.active_callback)
        self.imu_sub    = rospy.Subscriber('/septentrio_gnss/imu', Imu, self.imu_callback)
        self.contrasted_image_sub = rospy.Subscriber('/LANE_DETECTION/contrasted_image', Image, self.contrasted_image_callback)
        self.image_sub  = rospy.Subscriber('zed2/zed_node/rgb/image_rect_color', Image, self.image_callback)
        self.depth_sub  = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.depth_callback)

        # ───────── New: synchronised RGB + depth subscriber ───────────────
        rgb_sub   = Subscriber(self.rgb_topic,   Image)
        depth_sub = Subscriber(self.depth_topic, Image)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.03)
        self.sync.registerCallback(self.synced_cb)

        # ───────── Publishers (original + new) ─────────────────────────────
        self.enable_pub = rospy.Publisher('/EP_OUTPUT/enable', Bool, queue_size=1)
        self.gear_pub   = rospy.Publisher('/EP_OUTPUT/shift_cmd', PacmodCmd, queue_size=1)
        self.brake_pub  = rospy.Publisher('/EP_OUTPUT/brake_cmd', PacmodCmd, queue_size=1)
        self.accel_pub  = rospy.Publisher('/EP_OUTPUT/accel_cmd', PacmodCmd, queue_size=1)
        self.turn_pub   = rospy.Publisher('/EP_OUTPUT/turn_cmd', PacmodCmd, queue_size=1)
        self.steer_pub  = rospy.Publisher('/EP_OUTPUT/steer_cmd', PositionWithSpeed, queue_size=1)

        self.active_pub    = rospy.Publisher('/EXIT_PARK/active', Bool, queue_size=1)
        self.direction_pub = rospy.Publisher('/EXIT_PARK/direction', Bool, queue_size=1)
        # New outputs
        self.lane_dist_pub = rospy.Publisher('/EXIT_PARK/lane_distance', Float32, queue_size=1)
        self.debug_pub     = rospy.Publisher('/EXIT_PARK/lane_debug', Image, queue_size=1)

    # ───────── Original callbacks (unchanged) ──────────────────────────────
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
        pass  # redundant with sync callback now
    def depth_callback(self, msg):
        pass  # redundant with sync callback now

    # ───────── New synchronised RGB+Depth callback ────────────────────────
    def synced_cb(self, rgb_msg: Image, depth_msg: Image):
        rgb   = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth = depth.astype(np.float32) / 1000.0

        h, w = rgb.shape[:2]
        y0 = int(h * self.roi_top_frac)
        y1 = int(h * self.roi_bot_frac)
        rgb_roi   = rgb[y0:y1]
        depth_roi = depth[y0:y1]

        # Lane mask extraction tolerant to slight tilt
        lane_mask = self.get_lane_mask(rgb_roi)
        ys, xs = np.where(lane_mask > 0)
        if xs.size == 0:
            rospy.logwarn_throttle(2.0, 'No lane detected in ROI')
            return
                # ── Split mask into separate blobs and keep the farthest stripe ──
        lane_depths_farthest = None
        num_labels, labels = cv2.connectedComponents(lane_mask)
        for lbl in range(1, num_labels):  # label 0 = background
            yb, xb = np.where(labels == lbl)
            blob_depths = depth_roi[yb, xb]
            blob_depths = blob_depths[np.isfinite(blob_depths) & (blob_depths > 0)]
            if blob_depths.size == 0:
                continue
            median_d = np.median(blob_depths)
            if (lane_depths_farthest is None) or (median_d > lane_depths_farthest):
                lane_depths_farthest = median_d

        if lane_depths_farthest is None:
            rospy.logwarn_throttle(2.0, 'All lane blobs had invalid depth')
            return

        dist = float(lane_depths_farthest)   # farthest stripe distance
        self.lane_dist_pub.publish(Float32(dist))
        rospy.loginfo_throttle(1.0, f"Farthest lane ≈ {dist:.2f} m")

        # Debug overlay
        debug = rgb.copy()
        for x, y in zip(xs, ys):
            cv2.circle(debug, (x, y + y0), 1, (0,255,255), -1)
        cv2.putText(debug, f"{dist:.2f} m", (30,50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

        return  # end synced_cb

    # ───────── Simple lane mask helper ─────────────────────────────────────
    def get_lane_mask(self, bgr_img):.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # ───────── Simple lane mask helper ─────────────────────────────────────
    def get_lane_mask(self, bgr_img):
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=60, maxLineGap=100)
        mask = np.zeros_like(gray)
        if lines is not None:
            for x1,y1,x2,y2 in lines[:,0]:
                slope = (y2 - y1) / float(x2 - x1 + 1e-6)
                if abs(slope) < self.slope_thresh:
                    cv2.line(mask, (x1,y1), (x2,y2), 255, 5)
        return mask

    # ───────── Run loop (unchanged) ────────────────────────────────────────
    def run(self):
        rospy.loginfo('ExitParking node with lane distance extension running …')
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = ExitParking()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('ExitParking node shut down.')
