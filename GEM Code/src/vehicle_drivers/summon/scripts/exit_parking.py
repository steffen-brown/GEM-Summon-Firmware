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


Num of files to be created:

4 Files:

1. No Heading Control, No integration, only using lane distance
2. No Heading Control, with Integration, not using lane distance
3. Heading Control, no integration, only using lane distance
4. Heading control, with integration, not using lane distance
"""

# ───────── Original imports ───────────────────────────────────────────────────
import rospy

## GEM E2 imports
from sensor_msgs.msg import Image, Imu, CameraInfo
from std_msgs.msg import Bool, Float32
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt
from message_filters import Subscriber, ApproximateTimeSynchronizer

# from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt

# ───────── Additional imports ─────────────────────────────────────────────
from pid import PID
import math
import numpy as np
from filters import OnlineFilter

## Open CV imports
import cv2
from cv_bridge import CvBridge


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


# ───────── Exit Parking Class ────────────────────────────────────────
class ExitParking:
    def __init__(self):
        self.test_mode = False

        rospy.init_node("exit_parking", anonymous=True)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        # ───────── Parameters retrieved from ROS parameter server ─────────
        self.rgb_topic = rospy.get_param(
            "~rgb_topic", "/zed2/zed_node/rgb/image_rect_color"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/zed2/zed_node/depth/depth_registered"
        )
        self.slope_thresh = rospy.get_param(
            "~slope_thresh", 0.4
        )  # |slope| <= 0.4 → ~22°
        self.roi_top_frac = rospy.get_param("~roi_top_frac", 0.570)
        self.roi_bot_frac = rospy.get_param("~roi_bot_frac", 0.90)
        self.roi_left_frac  = rospy.get_param("~roi_left_frac", 0.40)   # keep center third by default
        self.roi_right_frac = rospy.get_param("~roi_right_frac", 0.60)

        # ───────── Additional variables for parking logic ─────────
        self.gem_enable = False  # Indicates vehicle initialization complete
        self.pacmod_enable = False  # Indicates PACMOD (vehicle interface) is ready
        self.lane_distance = None  # Latest computed lane distance (meters)

        ## (Neel :) Set this to true for right turn, false for left turn *****
        if self.test_mode:
            self.exit_direction = False
        else:
            self.exit_direction = None

        self.init_lane_distance = None
        self.updated_lane_distance = None

        # ───────── Variables for heading-based realignment and turning ─────────
        self.current_heading = (
            None  # Current vehicle heading (from INS/IMU), in degrees from 0-360
        )
        self.desired_heading = None  # Desired heading (perpendicular to lane)
        self.turn_start_heading = None  # Recorded heading when beginning the turn
        self.turning = False  # Flag indicating if turn maneuver has started
        self.init_pos = None
        self.wheelbase = 1.75  # meters

        self.prev_time = None

        # ───────── Variables for Speed Control ─────────
        self.speed = 0.0
        self.desired_speed = 0.75  # m/s, reference speed
        self.max_accel = 0.48  # % of acceleration
        self.pid_speed = PID(kp=0.5, ki=0.0, kd=0.1, wg=20)
        self.speed_filter = OnlineFilter(1.2, 30, 4)

        ## This is the heading that is used by the actual serpent IMU/GPS node!! This heading val should be used
        self.lat = 0.0
        self.long = 0.0
        self.heading = 0.0  ## nvm dont use this anymore, i overwrote the function to write to current heading instead

        self.olat = 40.0928563
        self.olon = -88.2359994

        # (Neel :) Control gain for heading correction in Phase 1
        self.kp = 0.5

        # ───────── (Neel :) Initialize vehicle command message placeholders ─────────
        self.gear_cmd = PacmodCmd()
        self.brake_cmd = PacmodCmd()
        self.accel_cmd = PacmodCmd()
        self.turn_cmd = PacmodCmd()
        self.steer_cmd = PositionWithSpeed()

        # ───────── subscribers ──────────────────────────────────────────────────────
        self.enable_sub = rospy.Subscriber(
            "/EP_OUTPUT/enable", Bool, self.enable_callback
        )
        self.speed_sub = rospy.Subscriber(
            "/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback
        )
        self.active_sub = rospy.Subscriber(
            "/EXIT_PARK/active", Bool, self.active_callback
        )

        # self.imu_sub = rospy.Subscriber("/septentrio_gnss/imu", Imu, self.imu_callback)

        self.gnss_sub = rospy.Subscriber(
            "/septentrio_gnss/navsatfix", NavSatFix, self.gnss_callback
        )
        self.ins_sub = rospy.Subscriber(
            "/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback
        )
        self.contrasted_image_sub = rospy.Subscriber(
            "/LANE_DETECTION/contrasted_image", Image, self.contrasted_image_callback
        )
        self.image_sub = rospy.Subscriber(
            "zed2/zed_node/rgb/image_rect_color", Image, self.image_callback
        )
        self.depth_sub = rospy.Subscriber(
            "/zed2/zed_node/depth/depth_registered", Image, self.depth_callback
        )
        self.direction_sub = rospy.Subscriber(
            "/EXIT_PARK/direction", Bool, self.exit_direction_callback
        )

        # ───────── New: synchronised RGB + depth subscriber ───────────────
        rgb_sub = Subscriber(self.rgb_topic, Image)
        depth_sub = Subscriber(self.depth_topic, Image)
        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.03
        )
        self.sync.registerCallback(self.synced_cb)

        # ───────── Publishers (original + new) ─────────────────────────────
        if self.test_mode:
            self.enable_pub = rospy.Publisher(
                "/pacmod/as_rx/enable", Bool, queue_size=1, latch=True
            )
            self.gear_pub = rospy.Publisher(
                "/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1, latch=True
            )
            self.brake_pub = rospy.Publisher(
                "/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1, latch=True
            )
            self.accel_pub = rospy.Publisher(
                "/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1, latch=True
            )
            self.turn_pub = rospy.Publisher(
                "/pacmod/as_rx/turn_cmd", PacmodCmd, queue_size=1, latch=True
            )
            self.steer_pub = rospy.Publisher(
                "/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1, latch=True
            )
        else:
            self.enable_pub = rospy.Publisher(
                "/EP_OUTPUT/enable", Bool, queue_size=1, latch=True
            )
            self.gear_pub = rospy.Publisher(
                "/EP_OUTPUT/shift_cmd", PacmodCmd, queue_size=1, latch=True
            )
            self.brake_pub = rospy.Publisher(
                "/EP_OUTPUT/brake_cmd", PacmodCmd, queue_size=1, latch=True
            )
            self.accel_pub = rospy.Publisher(
                "/EP_OUTPUT/accel_cmd", PacmodCmd, queue_size=1, latch=True
            )
            self.turn_pub = rospy.Publisher(
                "/EP_OUTPUT/turn_cmd", PacmodCmd, queue_size=1, latch=True
            )
            self.steer_pub = rospy.Publisher(
                "/EP_OUTPUT/steer_cmd", PositionWithSpeed, queue_size=1, latch=True
            )

        self.active_pub = rospy.Publisher(
            "/EXIT_PARK/active", Bool, queue_size=1, latch=True
        )

        # New outputs for lane distance and debug image
        self.lane_dist_pub = rospy.Publisher(
            "/EXIT_PARK/lane_distance", Float32, queue_size=1
        )
        self.debug_pub = rospy.Publisher("/EXIT_PARK/lane_debug", Image, queue_size=1)

        self.steer_cmd.angular_velocity_limit = 3.5

        self.module_active = False

    # ───────── Original callbacks (unchanged) ──────────────────────────────
    def enable_callback(self, msg):
        rospy.loginfo("Received enable message")

    def speed_callback(self, msg):
        # rospy.loginfo(f"Vehicle speed: {msg.vehicle_speed:.2f} m/s")
        self.speed = round(msg.vehicle_speed, 3)

    def active_callback(self, msg):
        self.module_active = msg.data
        rospy.loginfo(f"Exit park active: {msg.data}")

    def contrasted_image_callback(self, msg):
        rospy.loginfo("Received contrasted image")

    def image_callback(self, msg):
        pass  # redundant with sync callback now

    def depth_callback(self, msg):
        pass  # redundant with sync callback now

    # ------ New Callbacks ----------------
    # def inspva_callback(self, inspva_msg):
    #     self.lat = inspva_msg.latitude  # latitude
    #     self.lon = inspva_msg.longitude  # longitude
    #     self.heading = inspva_msg.azimuth  # heading in degrees

    # rounds heading!!
    def ins_callback(self, msg):
        self.current_heading = round(msg.heading, 6)

    # we dont use this but its here anyways :)
    def gnss_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)

    # this function basiaclly converts from 0-360 to -180 to +180 (OUTPUT IS RADIANS)
    def heading_to_yaw(self, heading_curr):
        if heading_curr >= 270 and heading_curr < 360:
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr

    # ───────── New synchronised RGB+Depth callback ────────────────────────
    def synced_cb(self, rgb_msg: Image, depth_msg: Image):
        #rospy.loginfo("Entered synced_cb() callback — RGB + Depth received.")
        rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   "bgr8")

        # depth still arrives but is no longer used
        # depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

        # ───── crop (top/bot & left/right) ─────
        h, w = rgb.shape[:2]
        y0 = int(h * self.roi_top_frac)
        y1 = int(h * self.roi_bot_frac)
        x0 = int(w * self.roi_left_frac)
        x1 = int(w * self.roi_right_frac)
        rgb_roi = rgb[y0:y1, x0:x1]

        # ───── lane mask & angle ─────
        lane_mask, avg_lane_angle = self.get_lane_mask(rgb_roi)
        ys, xs = np.where(lane_mask > 0)
        if xs.size == 0:
            rospy.logwarn_throttle(2.0, "No lane detected in ROI")
            return

        # desired-heading logic (unchanged)
        if avg_lane_angle is not None and self.current_heading is not None:
            opt1 = avg_lane_angle + math.pi / 2
            opt2 = avg_lane_angle - math.pi / 2
            cur  = math.radians(self.current_heading)
            self.desired_heading = math.degrees(
                opt1 if abs(normalize_angle_error(opt1-cur)) <
                       abs(normalize_angle_error(opt2-cur)) else opt2)
        elif self.current_heading is not None:
            self.desired_heading = self.current_heading

        # ───── gather two closest stripes (ranked by lowest row) ─────
        lane_blobs = []
        num_labels, labels = cv2.connectedComponents(lane_mask)
        for lbl in range(1, num_labels):
            yb, xb = np.where(labels == lbl)
            if xb.size == 0:
                continue
            lane_blobs.append((yb.max(), xb, yb))        # lower y ⇒ nearer
        if len(lane_blobs) < 2:
            rospy.logwarn("Need at least two stripes — skipping")
            return
        lane_blobs.sort(key=lambda t: t[0], reverse=True)
        _, xb1, yb1 = lane_blobs[0]                      # nearest stripe
        _, xb2, yb2 = lane_blobs[1]                      # second-nearest stripe

        # representative pixels (closest to centre X)
        centre_x = (x0 + x1) // 2
        rep1 = (int(xb1[np.argmin(np.abs((xb1 + x0) - centre_x))] + x0),
                int(yb1[np.argmin(np.abs((xb1 + x0) - centre_x))] + y0))
        rep2 = (int(xb2[np.argmin(np.abs((xb2 + x0) - centre_x))] + x0),
                int(yb2[np.argmin(np.abs((xb2 + x0) - centre_x))] + y0))

        # ───── initial-only distance calculation ─────
        if self.init_lane_distance is None:
            self.init_lane_distance = self.pixel_pair_to_distance(rep1, rep2)
            self.lane_distance      = self.init_lane_distance
            self.updated_lane_distance = self.init_lane_distance
            self.lane_dist_pub.publish(Float32(self.lane_distance))
            rospy.loginfo(f"Initial lane distance set: {self.lane_distance:.2f} m")

        # ───── debug overlay ─────
        debug = rgb.copy()
        for px, py in zip(xb1, yb1):
            cv2.circle(debug, (px + x0, py + y0), 1, (0,255,255), -1)
        for px, py in zip(xb2, yb2):
            cv2.circle(debug, (px + x0, py + y0), 1, (0,165,255), -1)
        cv2.circle(debug, rep1, 6, (255,0,0), 2)      # first stripe sample
        cv2.circle(debug, rep2, 6, (255,0,255), 2)    # second stripe sample

        cv2.putText(debug, f"Fixed Dist: {self.lane_distance:.2f} m", (30,50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 2)
        cv2.putText(debug, f"Init Dist:  {self.init_lane_distance:.2f} m", (30,80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 2)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        cv2.imshow("Lane Debug View", debug)
        cv2.waitKey(1)


    def pixel_pair_to_distance(self, pt1, pt2):
            """
            Estimate horizontal distance from the car to the first stripe
            using ONLY the vertical image coordinates of the two stripes.

            Args:
                pt1, pt2 : (x, y) tuples in full-frame pixel coords.
                        pt1 is the stripe nearest the bumper,
                        pt2 is the next stripe farther away.

            Returns:
                float : estimated distance in metres
            """
            y1 = pt1[1]          # y-pixel of first stripe (lower in image → larger y)
            y2 = pt2[1]          # y-pixel of second stripe


            print(y1)
            print(y2)
            stripe_spacing = -1.243
            y_horizon = 694.9

            return stripe_spacing * (y2 - y_horizon) / (y1 - y2)


    # ───────── Simple lane mask helper ─────────────────────────────────────
    def get_lane_mask(self, bgr_img):
        # Convert image to grayscale and blur to reduce noise
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Edge detection using Canny
        edges = cv2.Canny(blur, 50, 150)
        # Find line segments with Hough Transform
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, 50, minLineLength=60, maxLineGap=100
        )
        mask = np.zeros_like(gray)
        angles = []
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                slope = (y2 - y1) / float(x2 - x1 + 1e-6)
                if abs(slope) < self.slope_thresh:
                    cv2.line(mask, (x1, y1), (x2, y2), 255, 5)
                    # (Neel :) added code to alculate angle of the line segment (atan2 returns value in radians)
                    angle = math.atan2(y2 - y1, x2 - x1)
                    angles.append(angle)
        avg_angle = None
        if angles:
            avg_angle = sum(angles) / len(angles)
        return mask, avg_angle

    # ───────── Additional Helpers ─────────────────────────────────────
    # (Neel :) Added this from If_pid.py in order to get endgoal_x
    def endgoal_callback(self, msg):
        """
        Callback for lane detection endgoal messages.

        Updates target position for steering control based on lane detection.

        Args:
            msg: PoseStamped message containing target position in image
        """
        self.endgoal_x = msg.pose.position.x
        self.endgoal_y = msg.pose.position.y

    # (Neel :)
    # ───────── Exit Direction Callback ─────────────────────────────────────
    def exit_direction_callback(self, msg):
        self.exit_direction = msg.data  # True for right, False for left
        direction_str = "Right" if msg.data else "Left"
        rospy.loginfo(f"Exit direction received: {direction_str}")

    def front2steer(self, f_angle):
        """
        Convert front wheel angle to steering wheel angle.

        This function implements the non-linear mapping between desired front
        wheel angle and required steering wheel angle based on vehicle geometry.

        Args:
            f_angle: Desired front wheel angle in degrees

        Returns:
            Required steering wheel angle in degrees
        """
        # Safety limits for front wheel angle
        if f_angle > 35:
            f_angle = 35  # Maximum right turn
        if f_angle < -35:
            f_angle = -35  # Maximum left turn

        # Non-linear mapping based on vehicle-specific calibration
        if f_angle > 0:
            # Right turn mapping
            steer_angle = round(-0.1084 * f_angle**2 + 21.775 * f_angle, 2)
        elif f_angle < 0:
            # Left turn mapping (use same curve but negate result)
            f_angle_p = -f_angle
            steer_angle = -round(-0.1084 * f_angle_p**2 + 21.775 * f_angle_p, 2)
        else:
            # No steering
            steer_angle = 0.0

        return steer_angle

    def speed_control(self):
        speed_time = rospy.get_time()
        filt_vel = self.speed_filter.get_data(self.speed)
        # Calculate speed error
        speed_error = self.desired_speed - filt_vel

        # Only adjust acceleration if speed error is significant
        if abs(speed_error) > 0.1:
            # Calculate acceleration using PID controller
            output_accel = self.pid_speed.get_control(speed_time, speed_error)

            if output_accel > self.max_accel:
                output_accel = self.max_accel

            if output_accel < 0.3:
                output_accel = 0.3
        else:
            output_accel = 0.0
        return output_accel

    def heading_control(self):
        # If we have a desired_heading from lane detection and current IMU heading,
        # compute heading error and adjust steering.
        if self.current_heading is not None and self.desired_heading is not None:
            ## 'L' Represents the distance from the goal point to the current point
            L = self.updated_lane_distance
            curr_yaw = self.heading_to_yaw(self.heading)  ## output in radians
            # find the curvature and the angle
            alpha = (
                self.heading_to_yaw(self.desired_heading) - curr_yaw
            )  ## output in radians

            # ----------------- tuning this part as needed -----------------
            k = 0.41
            angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L)
            angle = angle_i * 2
            # ----------------- tuning this part as needed -----------------

            f_delta = round(np.clip(angle, -0.61, 0.61), 3)

            f_delta_deg = np.degrees(f_delta)

            # steering_angle in degrees
            steering_angle = self.front2steer(f_delta_deg)
            #self.turn_signal(f_delta_deg)
        else:
            # If headings are unavailable, default to straight (0) steering.
            steering_angle = 0.0
        return steering_angle

    def turn_signal(self, f_delta_deg):
        if f_delta_deg <= 30 and f_delta_deg >= -30:
            self.turn_cmd.ui16_cmd = 0
        elif f_delta_deg > 30:
            self.turn_cmd.ui16_cmd = 1  # turn left
        else:
            self.turn_cmd.ui16_cmd = 2 # turn right

    def log_heading(self):
        # If we haven't started turning yet, record the initial heading (will only be done once)
        if self.turn_start_heading is None:
            if self.current_heading is None:
                rospy.logwarn("IMU heading not available; cannot start turn")
            else:
                self.turn_start_heading = self.current_heading
                rospy.loginfo(
                    f"Starting turn. Recorded heading: {(self.turn_start_heading):.2f} deg"
                )

        # Compute heading change if current heading is available
        heading_change = 0.0
        if self.current_heading is not None and self.turn_start_heading is not None:
            heading_change = angle_diff(
                math.radians(self.current_heading),
                math.radians(self.turn_start_heading),
            )  ## output in radians
            rospy.loginfo_throttle(
                1.0,
                f"Turning... Heading change: {math.degrees(heading_change):.2f} deg",
            )
        return math.degrees(heading_change)  ## output in degrees

    def PACMON_setup(self):
        # Configure vehicle for autonomous mode
        self.gear_cmd.ui16_cmd = 3  # FORWARD gear

        # Enable brake control with zero brake pressure
        self.brake_cmd.enable = True
        self.brake_cmd.clear = False
        self.brake_cmd.ignore = False
        self.brake_cmd.f64_cmd = 0.0

        # Enable acceleration control with initial acceleration
        self.accel_cmd.enable = True
        self.accel_cmd.clear = False
        self.accel_cmd.ignore = False
        self.accel_cmd.f64_cmd = 1.5

        # Send initialization commands
        self.gear_pub.publish(self.gear_cmd)
        self.turn_pub.publish(self.turn_cmd)
        self.brake_pub.publish(self.brake_cmd)
        self.accel_pub.publish(self.accel_cmd)

        # Set flag indicating vehicle is now enabled
        self.gem_enable = True
        rospy.loginfo("GEM Enabled with Forward Gear!")

        return

    # ───────── Run loop ────────────────────────────────────────────────────
    def run(self):
        if not self.test_mode:
            while (self.module_active == False):
                pass

        rospy.loginfo("ExitParking node with lane distance extension running …")

        ##  ******* Initialize PACMOD if not alr inited ******
        if not self.pacmod_enable:
            self.pacmod_enable = True
            self.gear_cmd.ui16_cmd = 3  # Forward gear
            self.gear_pub.publish(self.gear_cmd)
            self.enable_pub.publish(Bool(data=True))
            #rospy.loginfo("PACMOD interface ready for simulation5## (Neel :) lets first wait for the first i56mage to calcualte distance from lane b4 proceeding
        while self.lane_distance is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(
                2.0, "calculating nearest lane distance from image data..."
            )
            self.rate.sleep()

        ## initialize prev_time before first using it
        if self.prev_time is None:
            self.prev_time = rospy.get_time()

        ## *************************************************************************
        ## ************************** Main Code ************************************
        ## *************************************************************************
        while not rospy.is_shutdown():
            # ───────── Vehicle Initialization ─────────
            ## (Neel :) now that we have lane distance, init the vehicle.
            if not self.gem_enable:
                # Enable vehicle if PACMOD is ready but vehicle not yet enabled
                if self.pacmod_enable:
                    self.PACMON_setup()
            # curr_time = rospy.get_time()

            # self.updated_lane_distance -= (curr_time - self.prev_time) * self.speed

            # self.prev_time = curr_time

            ## ****************** Tunable Parameter ***************************
            # (Neel :) Define the threshold distance from which the car needs
            #          to stop before making the full turn.
            threshold = 0.8  # meters = around 10.5 feet

            if self.updated_lane_distance is None:
                self.updated_lane_distance = self.lane_distance

            # ───────── Phase 1: Straight Line Follow Until Threshold ─────────

            if self.updated_lane_distance > threshold:
                ## *************** Heading Control ****************
                # output_angle = self.heading_control()
                output_angle = 0.0

                ## *************** Speed Control ****************
                output_accel = self.speed_control()

                ## *************** Integrate for Δ Distance ****************
                curr_time = rospy.get_time()

                self.updated_lane_distance -= (curr_time - self.prev_time) * self.speed

                self.prev_time = curr_time
                ## *************** Update Commands ****************
                # Update acceleration command
                self.accel_cmd.f64_cmd = output_accel
                self.steer_cmd.angular_position = output_angle

                ## *************** Publish Commands ****************
                # Continue driving forward at set acceleration.
                # self.accel_cmd.f64_cmd = 0.2
                self.accel_pub.publish(self.accel_cmd)
                self.steer_pub.publish(self.steer_cmd)
                rospy.loginfo_throttle(
                    1.0,
                    f"Driving forward: lane distance remaining = {self.updated_lane_distance:.2f} m",
                )

            ## ───────── Phase 2: threshold reached, apply max steering ─────────
            else:
                if self.exit_direction is not None:
                    ## *************** Determine maximum steering angle ****************
                    # For example, use 30° for left turn or -30° for right turn.
                    if self.exit_direction:
                        max_steering_deg = -35  # Right turn
                    else:
                        max_steering_deg = 35  # Left turn

                    max_steering_deg = self.front2steer(max_steering_deg)

                    ## *************** Update the heading change ****************
                    heading_change = self.log_heading()  ## output in degrees

                    # If the cumulative heading change is less than 90°,
                    # maintain maximum steering; otherwise, set steering to zero.
                    ## *************** Apply Max Turning ****************
                    if heading_change < (70):
                        self.steer_cmd.angular_position = math.radians(max_steering_deg)
                        rospy.loginfo_throttle(
                            1.0,
                            f"Issuing max steering command: {max_steering_deg} deg",
                        )
                        self.turn_signal(max_steering_deg)

                    # we have reached the end of the 90 deg turn -- successfully exited lane!
                    # ******** Finished Turning; Apply Zero Steering & Exit Loop *********
                    else:
                        self.steer_cmd.angular_position = 0.0
                        rospy.loginfo("90 degree turn accomplished. Steering set to zero.")

                        # Apply brake at 0.8
                        self.brake_cmd.enable = True
                        self.brake_cmd.clear = False
                        self.brake_cmd.ignore = False
                        self.brake_cmd.f64_cmd = 0.5
                        self.brake_pub.publish(self.brake_cmd)
                        rospy.loginfo("Brake applied at 0.8")


                        # Publish zero steering to confirm command
                        self.steer_pub.publish(self.steer_cmd)

                                                # Wait for 2 seconds
                        rospy.sleep(5.0)

                        # Disable exit parking mode
                        self.active_pub.publish(Bool(data=False))
                        rospy.loginfo("Exit parking complete. PID control activated.")
                        break  # Exit run loop

                    ## *************** Speed Control ***1*************
                    output_accel = self.speed_control()

                    ## *************** Update Commands ****************
                    self.accel_cmd.f64_cmd = output_accel

                    ## *************** Publish Commands ****************
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
