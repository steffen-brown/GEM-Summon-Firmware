#!/usr/bin/env python3

"""
ExitParking node (original skeleton + lane‑distance extension)
=============================================================
• Keeps all original subscribers/publishers so nothing breaks downstream.
• Adds a synchronised RGB+Depth callback that:
    1. Detects the perpendicular lane marking in front of the car (using Canny+Hough).
    2. Queries depth for those lane pixels and computes the horizontal (ground) distance
       from the camera to the lane. This value is published on `/EXIT_PARK/lane_distance` (Float32).
    3. Publishes a debug image on `/EXIT_PARK/lane_debug` with only the nearest lane highlighted.
• All new topics/parameters are namespaced so they can be ignored if unused.

If you run on GEM E4 (OAK‑D) set:
    rosparam set /exit_parking/rgb_topic  /oak/rgb/image_raw
    rosparam set /exit_parking/depth_topic /zdepth_raw

For GEM E2 (ZED2) defaults already match.
"""

# ───────── Original imports ───────────────────────────────────────────────────
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, CameraInfo, NavSatFix
from std_msgs.msg import Bool, Float32
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt
from message_filters import Subscriber, ApproximateTimeSynchronizer
from septentrio_gnss_driver.msg import INSNavGeod
from pid import PID

import math
import tf


# ───────── Helper function to compute minimal angular difference ─────────
def angle_diff(a, b):
    """Returns the absolute minimal difference between angles a and b (in radians)."""
    diff = a - b
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return abs(diff)


# ───────── Helper function to normalize a signed angle error ─────────
def normalize_angle_error(err):
    """Normalize angle error to [-pi, pi]."""
    return math.atan2(math.sin(err), math.cos(err))


class ExitParking:
    def __init__(self):
        rospy.init_node("exit_parking", anonymous=True)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        # ───────── Parameters (new) ─────────────────────────────────────────
        self.rgb_topic = rospy.get_param("~rgb_topic", "/zed2/zed_node/rgb/image_rect_color")
        self.depth_topic = rospy.get_param("~depth_topic", "/zed2/zed_node/depth/depth_registered")
        self.slope_thresh = rospy.get_param("~slope_thresh", 0.4)  # |slope| <= 0.4 → ~22°
        # Set ROI to the lower part of the image where the nearest lane appears.
        self.roi_top_frac = rospy.get_param("~roi_top_frac", 0.50)
        self.roi_bot_frac = rospy.get_param("~roi_bot_frac", 0.90)

        ## Additional variables for parking logic
        self.gem_enable = False
        self.pacmod_enable = False
        self.lane_distance = None           # This will be the horizontal ground distance (meters)
        self.exit_direction = True
        # We no longer update this by integration; it's unused now.
        self.init_lane_distance = None

        ## Variables for heading-based steering/turn control
        self.current_heading = 0.0
        self.desired_heading = None
        self.turn_start_heading = None      # Heading when turn begins
        self.turning = False
        self.heading = 0.0                  # (Unused now, replaced by current_heading)

        self.start_time = None
        self.curr_time = None
        self.prev_time = None

        self.speed = 0.0
        self.desired_speed = 1
        self.max_accel = 2.5  
        self.pid_speed = PID(kp=0.5, ki=0.0, kd=0.1, wg=20)
        self.kp = 0.5  # heading correction gain (if used)

        # ───────── Initialize vehicle command message placeholders ─────────
        self.gear_cmd = PacmodCmd()
        self.brake_cmd = PacmodCmd()
        self.accel_cmd = PacmodCmd()
        self.turn_cmd = PacmodCmd()
        self.steer_cmd = PositionWithSpeed()

        # ───────── Existing subscribers ────────────────────────────────────
        self.enable_sub = rospy.Subscriber("/EP_OUTPUT/enable", Bool, self.enable_callback)
        self.speed_sub = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.active_sub = rospy.Subscriber("/EXIT_PARK/active", Bool, self.active_callback)
        # self.imu_sub = rospy.Subscriber("/septentrio_gnss/imu", Imu, self.imu_callback) # commented out
        self.gnss_sub = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gnss_callback)
        self.ins_sub = rospy.Subscriber("/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)
        self.contrasted_image_sub = rospy.Subscriber("/LANE_DETECTION/contrasted_image", Image, self.contrasted_image_callback)
        self.image_sub = rospy.Subscriber("zed2/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depth_callback)

        # ───────── New: synchronized RGB + depth subscriber ───────────────
        rgb_sub = Subscriber(self.rgb_topic, Image)
        depth_sub = Subscriber(self.depth_topic, Image)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.03)
        self.sync.registerCallback(self.synced_cb)

        # ───────── New: Exit direction subscriber ───────────────
        self.direction_sub = rospy.Subscriber("/EXIT_PARK/direction", Bool, self.exit_direction_callback)

        # ───────── Publishers (original + new) ─────────────────────────────
        self.active_pub = rospy.Publisher("/EXIT_PARK/active", Bool, queue_size=1)
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.gear_pub   = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.brake_pub  = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.accel_pub  = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.turn_pub   = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.steer_pub  = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)

        # New outputs for lane distance and debug image
        self.lane_dist_pub = rospy.Publisher("/EXIT_PARK/lane_distance", Float32, queue_size=1)
        self.debug_pub = rospy.Publisher("/EXIT_PARK/lane_debug", Image, queue_size=1)

    # ───────── Original callbacks ──────────────────────────────────────────────
    def enable_callback(self, msg):
        rospy.loginfo("Received enable message")

    def speed_callback(self, msg):
        self.speed = round(msg.vehicle_speed, 3)

    def active_callback(self, msg):
        rospy.loginfo(f"Exit park active: {msg.data}")

    # def imu_callback(self, msg):  # commented out
    #     ...
    
    def contrasted_image_callback(self, msg):
        rospy.loginfo("Received contrasted image")

    def image_callback(self, msg):
        pass  # Redundant with sync callback

    def depth_callback(self, msg):
        pass  # Redundant with sync callback

    # ------ New Callbacks ----------------
    def inspva_callback(self, inspva_msg):
        self.lat = inspva_msg.latitude
        self.lon = inspva_msg.longitude
        self.heading = inspva_msg.azimuth  # heading in degrees

    def ins_callback(self, msg):
        self.current_heading = round(msg.heading, 6)

    def gnss_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)

    def heading_to_yaw(self, heading_curr):
        if heading_curr >= 270 and heading_curr < 360:
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr

    def front2steer(self, f_angle):
        """
        Convert desired front wheel angle to steering wheel angle using a non-linear mapping.
        
        Args:
            f_angle (float): Desired front wheel angle in degrees.
            
        Returns:
            float: Steering wheel angle in degrees after calibration.
        """
        max_front_angle = 35  # degrees safety limit
        if f_angle > max_front_angle:
            f_angle = max_front_angle
        elif f_angle < -max_front_angle:
            f_angle = -max_front_angle

        # Example quadratic mapping (adjust coefficients based on calibration)
        if f_angle > 0:
            steer_angle = -0.1084 * (f_angle ** 2) + 21.775 * f_angle
        elif f_angle < 0:
            f_angle_pos = -f_angle
            steer_angle = -(-0.1084 * (f_angle_pos ** 2) + 21.775 * f_angle_pos)
        else:
            steer_angle = 0.0

        return steer_angle

    # ───────── New synchronized RGB+Depth callback ────────────────────────
    def synced_cb(self, rgb_msg: Image, depth_msg: Image):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if depth_msg.encoding == "16UC1":
            depth = depth.astype(np.float32) / 1000.0

        h, w = rgb.shape[:2]
        y0 = int(h * self.roi_top_frac)
        y1 = int(h * self.roi_bot_frac)
        rgb_roi = rgb[y0:y1]
        depth_roi = depth[y0:y1]

        # Get lane mask for debugging purposes
        lane_mask, avg_lane_angle = self.get_lane_mask(rgb_roi)
        ys, xs = np.where(lane_mask > 0)
        if xs.size == 0:
            rospy.logwarn_throttle(2.0, "No lane detected in ROI. Continuing with current heading.")
            if self.current_heading is not None:
                self.desired_heading = self.current_heading
            else:
                return
        else:
            if self.current_heading is not None:
                self.desired_heading = self.current_heading

        # Select the nearest lane blob (smallest median depth) from the ROI.
        lane_depths_nearest = None
        best_label = None
        num_labels, labels = cv2.connectedComponents(lane_mask)
        for lbl in range(1, num_labels):  # Skip background (label 0)
            yb, xb = np.where(labels == lbl)
            blob_depths = depth_roi[yb, xb]
            blob_depths = blob_depths[np.isfinite(blob_depths) & (blob_depths > 0)]
            if blob_depths.size == 0:
                continue
            median_d = np.median(blob_depths)
            if (lane_depths_nearest is None) or (median_d < lane_depths_nearest):
                lane_depths_nearest = median_d
                best_label = lbl

        if lane_depths_nearest is None:
            rospy.logwarn_throttle(2.0, "All lane blobs had invalid depth")
            return

        # 'dist' is the slant (Euclidean) distance from camera to lane blob.
        dist = float(lane_depths_nearest)

        # Convert to horizontal (ground) distance:
        CAMERA_HEIGHT = 1.80  # meters (camera mounting height)
        if dist > CAMERA_HEIGHT:
            horizontal_distance = math.sqrt(dist**2 - CAMERA_HEIGHT**2)
            rospy.loginfo_throttle(1.0, f"Lane horizontal distance ≈ {horizontal_distance:.2f} m")
        else:
            rospy.logwarn_throttle(1.0, "Invalid depth: lane is below camera or distorted")
            return

        # Optional low-pass filtering for stability (smoothing factor ALPHA)
        ALPHA = 0.2
        if self.lane_distance is None:
            self.lane_distance = horizontal_distance
        else:
            self.lane_distance = ALPHA * horizontal_distance + (1 - ALPHA) * self.lane_distance

        # Publish the horizontal lane distance
        self.lane_dist_pub.publish(Float32(self.lane_distance))

        # Debug overlay: draw only the nearest lane blob.
        debug = rgb.copy()
        if best_label is not None:
            yb, xb = np.where(labels == best_label)
            for (px, py) in zip(xb, yb):
                cv2.circle(debug, (px, py + y0), 1, (0, 255, 255), -1)
        cv2.putText(debug, f"Nearest: {self.lane_distance:.2f} m", (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        cv2.imshow("Lane Debug View", debug)
        cv2.waitKey(1)

    def get_lane_mask(self, bgr_img):
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=60, maxLineGap=100)
        mask = np.zeros_like(gray)
        angles = []
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                slope = (y2 - y1) / float(x2 - x1 + 1e-6)
                if abs(slope) < self.slope_thresh:
                    cv2.line(mask, (x1, y1), (x2, y2), 255, 5)
                    angle = math.atan2(y2 - y1, x2 - x1)
                    angles.append(angle)
        avg_angle = None
        if angles:
            avg_angle = sum(angles) / len(angles)
        return mask, avg_angle

    def endgoal_callback(self, msg):
        self.endgoal_x = msg.pose.position.x
        self.endgoal_y = msg.pose.position.y

    def exit_direction_callback(self, msg):
        self.exit_direction = msg.data
        direction_str = "Right" if msg.data else "Left"
        rospy.loginfo(f"Exit direction received: {direction_str}")

    def run(self):
        rospy.loginfo("ExitParking node with lane distance extension running …")
        if not self.pacmod_enable:
            self.pacmod_enable = True
            self.enable_pub.publish(Bool(data=True))
            rospy.loginfo("PACMOD interface ASSUMED ready for simulation.")

        # Wait until a valid lane distance measurement is available.
        while self.lane_distance is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(2.0, "calculating lane distance from image data...")
            self.rate.sleep()

        # No longer integrating speed; use direct measurements.
        while not rospy.is_shutdown():
            # ───────── Vehicle Initialization ─────────
            if not self.gem_enable:
                if self.pacmod_enable:
                    self.gear_cmd.ui16_cmd = 3  # FORWARD gear
                    self.brake_cmd.enable = True
                    self.brake_cmd.clear = False
                    self.brake_cmd.ignore = False
                    self.brake_cmd.f64_cmd = 0.0
                    self.accel_cmd.enable = True
                    self.accel_cmd.clear = False
                    self.accel_cmd.ignore = False
                    self.accel_cmd.f64_cmd = 1.5

                    self.gear_pub.publish(self.gear_cmd)
                    self.turn_pub.publish(self.turn_cmd)
                    self.brake_pub.publish(self.brake_cmd)
                    self.accel_pub.publish(self.accel_cmd)
                    self.gem_enable = True
                    rospy.loginfo("GEM Enabled with Forward Gear!")

            if self.lane_distance is None:
                rospy.logwarn_throttle(2.0, "Lane distance not available yet")
            else:
                # Use the measured horizontal lane distance for threshold-based logic.
                threshold = 4.2  # horizontal distance threshold in meters

                # Phase 1: Drive straight until the lane (horizontal) distance becomes less than threshold.
                if self.lane_distance > threshold:
                    self.steer_cmd.angular_position = 0.0

                    # Here, we are not updating distance by integration. We rely solely on the depth reading.
                    speed_time = rospy.get_time()
                    speed_error = self.desired_speed - self.speed
                    if abs(speed_error) > 0.1:
                        speed_output_accel = self.pid_speed.get_control(speed_time, speed_error)
                        speed_output_accel = min(max(speed_output_accel, 0.2), self.max_accel)
                    else:
                        speed_output_accel = 0.0
                    
                    self.accel_cmd.f64_cmd = speed_output_accel
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    rospy.loginfo_throttle(1.0, f"Driving forward: lane horizontal distance = {self.lane_distance:.2f} m")
                else:
                    # Phase 2: Threshold reached; execute turn using a fixed steering command.
                    if self.exit_direction is not None:
                        if self.exit_direction:
                            front_wheel_angle = -30  # Right turn (desired front wheel angle)
                        else:
                            front_wheel_angle = 30   # Left turn

                        # Convert front wheel angle to steering wheel angle.
                        steering_wheel_angle = self.front2steer(front_wheel_angle)

                        # Record turn start heading if not already done.
                        if self.turn_start_heading is None:
                            if self.current_heading is None:
                                rospy.logwarn("IMU heading not available; cannot start turn")
                            else:
                                self.turn_start_heading = self.current_heading
                                rospy.loginfo(f"Starting turn. Recorded heading: {math.degrees(self.turn_start_heading):.2f} deg")

                        heading_change = 0.0
                        if self.current_heading is not None and self.turn_start_heading is not None:
                            heading_change = angle_diff(self.current_heading, self.turn_start_heading)
                            rospy.loginfo_throttle(1.0, f"Turning... Heading change: {math.degrees(heading_change):.2f} deg")

                        if heading_change < (math.pi / 2):
                            self.steer_cmd.angular_position = math.radians(steering_wheel_angle)
                            self.steer_pub.publish(self.steer_cmd)
                            rospy.loginfo_throttle(1.0, f"Issuing steering command: {steering_wheel_angle:.2f} deg")
                        else:
                            self.steer_cmd.angular_position = 0.0
                            self.steer_pub.publish(self.steer_cmd)
                            rospy.loginfo("90 degree turn accomplished. Steering set to zero.")
                            self.active_pub.publish(Bool(False))
                            rospy.loginfo("Exit parking complete. PID control activated.")
                            break

                        speed_time = rospy.get_time()
                        speed_error = self.desired_speed - self.speed
                        if abs(speed_error) > 0.1:
                            speed_output_accel = self.pid_speed.get_control(speed_time, speed_error)
                            speed_output_accel = min(max(speed_output_accel, 0.2), self.max_accel)
                        else:
                            speed_output_accel = 0.0
                        self.accel_cmd.f64_cmd = speed_output_accel
                        self.steer_pub.publish(self.steer_cmd)
                        self.accel_pub.publish(self.accel_cmd)
                    else:
                        rospy.logwarn_throttle(2.0, "Exit direction not received yet")

if __name__ == "__main__":
    try:
        node = ExitParking()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ExitParking node shut down.")
    finally:
        cv2.destroyAllWindows()
