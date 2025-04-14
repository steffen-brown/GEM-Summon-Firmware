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

from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod

from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, VehicleSpeedRpt

from pid import PID

## (Neel :) added imports -> do we have to install these libraries??
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
        self.rgb_topic = rospy.get_param(
            "~rgb_topic", "/zed2/zed_node/rgb/image_rect_color"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/zed2/zed_node/depth/depth_registered"
        )
        self.slope_thresh = rospy.get_param(
            "~slope_thresh", 0.4
        )  # |slope| <= 0.4 → ~22°
        self.roi_top_frac = rospy.get_param("~roi_top_frac", 0.40)
        self.roi_bot_frac = rospy.get_param("~roi_bot_frac", 0.70)

        ## (Neel :) added additional vars for parking logic
        self.gem_enable = False
        self.pacmod_enable = False
        self.lane_distance = None
        self.exit_direction = True
        self.init_lane_distance = None

        ## (Neel :) added vars for heading based straighten lane
        self.current_heading = 0.0
        self.desired_heading = None
        self.turn_start_heading = None  # Heading when starting the turn
        self.turning = False
        ## This is the heading that is used by the actual serpent IMU/GPS node!! This heading val should be used
        self.heading = 0.0  ## nvm dont use this anymore, i overwrote the function to write to current heading instead

        self.start_time = None
        self.curr_time = None
        self.prev_time = None

        self.speed = 0.0
        self.desired_speed = 1
        self.max_accel = 2.5  
        self.pid_speed = PID(kp=0.5, ki=0.0, kd=0.1, wg=20) 
        # (Neel :) Control gain for heading correction in Phase 1
        self.kp = 0.5

        # ───────── (Neel :) Initialize vehicle command message placeholders ─────────
        self.gear_cmd = PacmodCmd()
        self.brake_cmd = PacmodCmd()
        self.accel_cmd = PacmodCmd()
        self.turn_cmd = PacmodCmd()
        self.steer_cmd = PositionWithSpeed()

        # ───────── Existing subscribers ────────────────────────────────────
        self.enable_sub = rospy.Subscriber(
            "/EP_OUTPUT/enable", Bool, self.enable_callback
        )
        self.speed_sub = rospy.Subscriber(
            "/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback
        )
        self.active_sub = rospy.Subscriber(
            "/EXIT_PARK/active", Bool, self.active_callback
        )

        ## (Neel :) are we really using the septentrio_gnss imu?? I thought it was novatel??...
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

        # ───────── New: synchronised RGB + depth subscriber ───────────────
        rgb_sub = Subscriber(self.rgb_topic, Image)
        depth_sub = Subscriber(self.depth_topic, Image)
        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.03
        )
        self.sync.registerCallback(self.synced_cb)

        # ───────── New: Exit direction subscriber ───────────────
        self.direction_sub = rospy.Subscriber(
            "/EXIT_PARK/direction", Bool, self.exit_direction_callback
        )

        # ───────── Publishers (original + new) ─────────────────────────────
        # self.enable_pub = rospy.Publisher("/EP_OUTPUT/enable", Bool, queue_size=1)
        # self.gear_pub = rospy.Publisher("/EP_OUTPUT/shift_cmd", PacmodCmd, queue_size=1)
        # self.brake_pub = rospy.Publisher(
        #     "/EP_OUTPUT/brake_cmd", PacmodCmd, queue_size=1
        # )
        # self.accel_pub = rospy.Publisher(
        #     "/EP_OUTPUT/accel_cmd", PacmodCmd, queue_size=1
        # )
        # self.turn_pub = rospy.Publisher("/EP_OUTPUT/turn_cmd", PacmodCmd, queue_size=1)
        # self.steer_pub = rospy.Publisher(
        #     "/EP_OUTPUT/steer_cmd", PositionWithSpeed, queue_size=1
        # )

        self.active_pub = rospy.Publisher("/EXIT_PARK/active", Bool, queue_size=1)

        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.gear_pub   = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.brake_pub  = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.accel_pub  = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.turn_pub   = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.steer_pub  = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)

        ## (Neel :) AFIK we dont need to publish the direction right... it doesnt matter
        # self.direction_pub = rospy.Publisher('/EXIT_PARK/direction', Bool, queue_size=1)

        # New outputs for lane distance and debug image
        self.lane_dist_pub = rospy.Publisher(
            "/EXIT_PARK/lane_distance", Float32, queue_size=1
        )
        self.debug_pub = rospy.Publisher("/EXIT_PARK/lane_debug", Image, queue_size=1)

    # ───────── Original callbacks (unchanged) ──────────────────────────────
    def enable_callback(self, msg):
        rospy.loginfo("Received enable message")

    def speed_callback(self, msg):
        #rospy.loginfo(f"Vehicle speed: {msg.vehicle_speed:.2f} m/s")
        self.speed = round(msg.vehicle_speed, 3)
        pass

    def active_callback(self, msg):
        rospy.loginfo(f"Exit park active: {msg.data}")

    ## (Neel :) changed callback function to find current heading every time
    # def imu_callback(self, msg):
    #     rospy.loginfo("Received IMU data")
    #     # Convert quaternion to Euler angles and store the yaw (heading)
    #     orientation_q = msg.orientation
    #     quaternion = (
    #         orientation_q.x,
    #         orientation_q.y,
    #         orientation_q.z,
    #         orientation_q.w,
    #     )
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     self.current_heading = euler[2]
    #     rospy.logdebug(f"Current heading: {math.degrees(self.current_heading):.2f} deg")

    def contrasted_image_callback(self, msg):
        rospy.loginfo("Received contrasted image")

    def image_callback(self, msg):
        pass  # redundant with sync callback now

    def depth_callback(self, msg):
        pass  # redundant with sync callback now

    # ------ New Callbacks ----------------
    def inspva_callback(self, inspva_msg):
        self.lat = inspva_msg.latitude  # latitude
        self.lon = inspva_msg.longitude  # longitude
        self.heading = inspva_msg.azimuth  # heading in degrees

    # rounds heading!!
    def ins_callback(self, msg):
        self.current_heading = round(msg.heading, 6)

    # we dont use this but its here anyways :)
    def gnss_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)

    # this function basiaclly converts from 0-360 to -180 to +180
    def heading_to_yaw(self, heading_curr):
        if heading_curr >= 270 and heading_curr < 360:
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr

    # ───────── New synchronised RGB+Depth callback ────────────────────────
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

        lane_mask, avg_lane_angle = self.get_lane_mask(rgb_roi)
        ys, xs = np.where(lane_mask > 0)
        if xs.size == 0:
            rospy.logwarn_throttle(2.0, "No lane detected in ROI")
            return

        if avg_lane_angle is not None and self.current_heading is not None:
            option1 = avg_lane_angle + math.pi / 2
            option2 = avg_lane_angle - math.pi / 2
            error1 = abs(normalize_angle_error(option1 - self.current_heading))
            error2 = abs(normalize_angle_error(option2 - self.current_heading))
            self.desired_heading = option1 if error1 < error2 else option2
            rospy.loginfo_throttle(
                2.0,
                f"Desired heading set to: {math.degrees(self.desired_heading):.2f} deg",
            )
        else:
            if self.current_heading is not None:
                self.desired_heading = self.current_heading

        # Updated: Select the **nearest** stripe (smallest depth)
        lane_depths_nearest = None
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

        if lane_depths_nearest is None:
            rospy.logwarn_throttle(2.0, "All lane blobs had invalid depth")
            return

        dist = float(lane_depths_nearest)
        self.lane_dist_pub.publish(Float32(dist))
        if self.lane_distance is None:
            self.init_lane_distance = dist
        self.lane_distance = dist
        rospy.loginfo_throttle(1.0, f"Nearest lane ≈ {dist:.2f} m")

        # Debug overlay
        debug = rgb.copy()
        for x, y in zip(xs, ys):
            cv2.circle(debug, (x, y + y0), 1, (0, 255, 255), -1)
        cv2.putText(
            debug,
            f"Nearest: {self.init_lane_distance:.2f} m",
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 0, 255),
            2,
        )
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

        # Show debug image in OpenCV window
        cv2.imshow("Lane Debug View", debug)
        cv2.waitKey(1)

  # end synced_cb

    # ───────── Simple lane mask helper ─────────────────────────────────────
    ## Neel comment : what is this??.......... seems like it's giving a bug and is
    #                 alr declared somewhere else. I'm commenting out for now.
    # def get_lane_mask(self, bgr_img):.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # ───────── Simple lane mask helper ─────────────────────────────────────
    def get_lane_mask(self, bgr_img):
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
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

    # ───────── Run loop (unchanged) ────────────────────────────────────────
    def run(self):
        rospy.loginfo("ExitParking node with lane distance extension running …")
        # Assume that pacmod_enable will be set true externally (e.g., via the enable callback)
        # For simulation, you can manually set:
        if not self.pacmod_enable:
            self.pacmod_enable = True
            self.enable_pub.publish(Bool(data=True))
            rospy.loginfo("PACMOD interface ASSUMED ready for simulation.")

        ## (Neel :) lets first wait for the first image to calcualte distance from lane b4 proceeding
        while self.lane_distance is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(2.0, "calculating lane distance from image data...")
            self.rate.sleep()

        self.prev_time = rospy.get_time()

        while not rospy.is_shutdown():
            # ───────── Vehicle Initialization ─────────
            ## (Neel :) now that we have lane distance, init the vehicle.
            if not self.gem_enable:
                # Enable vehicle if PACMOD is ready but vehicle not yet enabled
                if self.pacmod_enable:
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

            if self.lane_distance is None:
                rospy.logwarn_throttle(2.0, "Lane distance not available yet")
            else:
                # (Neel :) Define the threshold distance from which the car needs
                #          to stop before making the full turn.
                threshold = 8.5  # meters = around 10.5 feet

                # ───────── Phase 1: drive forward until lane distance met ─────────
                if self.init_lane_distance > threshold:
                    # If we have a desired_heading from lane detection and current IMU heading,
                    # compute heading error and adjust steering.
                    # if (
                    #     self.current_heading is not None
                    #     and self.desired_heading is not None
                    # ):
                    #     error = normalize_angle_error(
                    #         self.desired_heading - self.current_heading
                    #     )
                    #     adjustment = self.kp * error
                    #     # Optionally, saturate the adjustment to a maximum steering command.
                    #     max_adjustment = math.radians(5)  # e.g., limit to ±5°
                    #     if adjustment > max_adjustment:
                    #         adjustment = max_adjustment
                    #     elif adjustment < -max_adjustment:
                    #         adjustment = -max_adjustment
                    #     self.steer_cmd.angular_position = adjustment
                    #     rospy.loginfo_throttle(
                    #         1.0,
                    #         f"Realigning: error={math.degrees(error):.2f}°, adjustment={math.degrees(adjustment):.2f}°",
                    #     )
                    # else:
                    # If headings are unavailable, default to straight (0) steering.
                    self.steer_cmd.angular_position = 0.0

                    self.curr_time = rospy.get_time()
                    
                    self.init_lane_distance -= (self.curr_time - self.prev_time)*self.speed
                    self.prev_time = self.curr_time

                    # Get current time for speed PID controller
                    speed_time = rospy.get_time()
                    
                    # Calculate speed error
                    speed_error = self.desired_speed - self.speed
                    
                    # Only adjust acceleration if speed error is significant
                    if abs(speed_error) > 0.1:
                        # Calculate acceleration using PID controller
                        speed_output_accel = self.pid_speed.get_control(speed_time, speed_error)
                        
                        # Apply limits to acceleration command
                        if speed_output_accel > self.max_accel:
                            speed_output_accel = self.max_accel  # Cap maximum acceleration
                        if speed_output_accel < 0.2:
                            speed_output_accel = 0.2  # Minimum acceleration to prevent stalling
                    else:
                        # Maintain current speed
                        speed_output_accel = 0.0
                    
                    # Update acceleration command
                    self.accel_cmd.f64_cmd = speed_output_accel

                    # Continue driving forward at set acceleration.
                    # self.accel_cmd.f64_cmd = 0.2
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    rospy.loginfo_throttle(
                        1.0,
                        f"Driving forward: lane distance = {self.lane_distance:.2f} m",
                    )
                else:
                    # ───────── Phase 2: threshold reached, apply max steering ─────────
                    if self.exit_direction is not None:
                        # Determine maximum steering angle based on exit direction
                        # For example, use 30° for left turn or -30° for right turn.
                        if self.exit_direction:
                            max_steering_deg = -30  # Right turn
                        else:
                            max_steering_deg = 30  # Left turn

                        # If we haven't started turning yet, record the initial heading
                        if self.turn_start_heading is None:
                            if self.current_heading is None:
                                rospy.logwarn(
                                    "IMU heading not available; cannot start turn"
                                )
                            else:
                                self.turn_start_heading = self.current_heading
                                rospy.loginfo(
                                    f"Starting turn. Recorded heading: {math.degrees(self.turn_start_heading):.2f} deg"
                                )

                        # Compute heading change if current heading is available
                        heading_change = 0.0
                        if (
                            self.current_heading is not None
                            and self.turn_start_heading is not None
                        ):
                            heading_change = angle_diff(
                                self.current_heading, self.turn_start_heading
                            )
                            rospy.loginfo_throttle(
                                1.0,
                                f"Turning... Heading change: {math.degrees(heading_change):.2f} deg",
                            )

                        # If the cumulative heading change is less than 90° (pi/2 radians),
                        # maintain maximum steering; otherwise, set steering to zero.
                        if heading_change < (math.pi / 2):
                            self.steer_cmd.angular_position = math.radians(
                                max_steering_deg
                            )
                            self.pub.publish(self.steer_cmd)
                            rospy.loginfo_throttle(
                                1.0,
                                f"Issuing max steering command: {max_steering_deg} deg",
                            )
                        else:
                            self.steer_cmd.angular_position = 0.0
                            rospy.loginfo(
                                "90 degree turn accomplished. Steering set to zero."
                            )
                            # Publish zero steering to confirm command
                            self.steer_pub.publish(self.steer_cmd)
                            # Phase 3: Disable exit parking mode and hand over control
                            self.active_pub.publish(Bool(False))
                            rospy.loginfo(
                                "Exit parking complete. PID control activated."
                            )
                            break  # Exit run loop

                        # Get current time for speed PID controller
                        speed_time = rospy.get_time()
                        
                        # Calculate speed error
                        speed_error = self.desired_speed - self.speed
                        
                        # Only adjust acceleration if speed error is significant
                        if abs(speed_error) > 0.1:
                            # Calculate acceleration using PID controller
                            speed_output_accel = self.pid_speed.get_control(speed_time, speed_error)
                            
                            # Apply limits to acceleration command
                            if speed_output_accel > self.max_accel:
                                speed_output_accel = self.max_accel  # Cap maximum acceleration
                            if speed_output_accel < 0.2:
                                speed_output_accel = 0.2  # Minimum acceleration to prevent stalling
                        else:
                            # Maintain current speed
                            speed_output_accel = 0.0
                        
                        # Update acceleration command
                        self.accel_cmd.f64_cmd = speed_output_accel

                        # Publish the steering command and keep driving forward
                        self.steer_pub.publish(self.steer_cmd)
                        # self.accel_cmd.f64_cmd = 0.2
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