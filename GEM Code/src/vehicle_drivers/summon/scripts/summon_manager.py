#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64, Int32
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
from sensor_msgs.msg import NavSatFix
#from septentrio_gnss_msgs.msg import InsNavGeod
import math

class SummonManager:

    def __init__(self):
        rospy.init_node('summon_manager', anonymous=True)

        # FSM States: 0 = IDLE, 1 = EXIT, 2 = LINE FOLLOW
        self.fsm_state = 0

        # PACMod command publishers
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1, latch=True)
        self.gear_pub   = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1, latch=True)
        self.brake_pub  = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1, latch=True)
        self.accel_pub  = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1, latch=True)
        self.turn_pub   = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1, latch=True)
        self.steer_pub  = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1, latch=True)
        
        # New subscriber for the pacmod enable signal
        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.pacmod_enable_callback)

        # Webapp interaction
        self.status_pub    = rospy.Publisher('/WEBAPP/status', Int32, queue_size=1, latch=True)
        self.goal_long_sub = rospy.Subscriber('/WEBAPP/goal_long', Float64, self.goal_long_callback)
        self.goal_lat_sub  = rospy.Subscriber('/WEBAPP/goal_lat', Float64, self.goal_lat_callback)

        # GPS
        self.gps_sub = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gps_callback)
        #self.insnavgeod_sub = rospy.Subscriber("/septentrio_gnss/insnavgeod", InsNavGeod, self.insnavgeod_callback) # CHECK THIS

        # Lane detection and exit parking management
        self.lane_detection_active_pub = rospy.Publisher('/LANE_DETECTION/active', Bool, queue_size=1, latch=True)
        self.exit_parking_active_pub   = rospy.Publisher('/EXIT_PARK/active', Bool, queue_size=1, latch=True)
        self.exit_parking_active_sub   = rospy.Subscriber('/EXIT_PARK/active', Bool, self.exit_parking_active_callback)
        self.exit_direction_pub        = rospy.Publisher('/EXIT_PARK/direction', Bool, queue_size=1, latch=True)

        # Object detection
        self.stop_sub    = rospy.Subscriber('/OBJECT_DETECTION/stop', Bool, self.stop_callback)
        self.restart_sub = rospy.Subscriber('/OBJECT_DETECTION/restart', Bool, self.restart_callback)
        self.stop_pub    = rospy.Publisher('/OBJECT_DETECTION/stop', Bool, queue_size=1, latch=True)
        self.restart_pub = rospy.Publisher('/OBJECT_DETECTION/restart', Bool, queue_size=1, latch=True)

        # Lane following mirror (LF)
        self.lf_enable_pub = rospy.Publisher('/LF_OUTPUT/enable', Bool, queue_size=1, latch=True)
        self.lf_enable_sub = rospy.Subscriber('/LF_OUTPUT/enable', Bool, self.lf_enable_callback)
        self.lf_gear_sub   = rospy.Subscriber('/LF_OUTPUT/shift_cmd', PacmodCmd, self.lf_gear_callback)
        self.lf_brake_sub  = rospy.Subscriber('/LF_OUTPUT/brake_cmd', PacmodCmd, self.lf_brake_callback)
        self.lf_accel_sub  = rospy.Subscriber('/LF_OUTPUT/accel_cmd', PacmodCmd, self.lf_accel_callback)
        self.lf_turn_sub   = rospy.Subscriber('/LF_OUTPUT/turn_cmd', PacmodCmd, self.lf_turn_callback)
        self.lf_steer_sub  = rospy.Subscriber('/LF_OUTPUT/steer_cmd', PositionWithSpeed, self.lf_steer_callback)

        # Exit parking mirror (EP)
        self.ep_enable_pub = rospy.Publisher('/EP_OUTPUT/enable', Bool, queue_size=1, latch=True)
        self.ep_enable_sub = rospy.Subscriber('/EP_OUTPUT/enable', Bool, self.ep_enable_callback)
        self.ep_gear_sub   = rospy.Subscriber('/EP_OUTPUT/shift_cmd', PacmodCmd, self.ep_gear_callback)
        self.ep_brake_sub  = rospy.Subscriber('/EP_OUTPUT/brake_cmd', PacmodCmd, self.ep_brake_callback)
        self.ep_accel_sub  = rospy.Subscriber('/EP_OUTPUT/accel_cmd', PacmodCmd, self.ep_accel_callback)
        self.ep_turn_sub   = rospy.Subscriber('/EP_OUTPUT/turn_cmd', PacmodCmd, self.ep_turn_callback)
        self.ep_steer_sub  = rospy.Subscriber('/EP_OUTPUT/steer_cmd', PositionWithSpeed, self.ep_steer_callback)

        # ======== Store latest mirror commands ========
        # Lane Following commands
        self.lf_enable_msg = Bool()
        self.lf_gear_msg   = PacmodCmd()
        self.lf_brake_msg  = PacmodCmd()
        self.lf_accel_msg  = PacmodCmd()
        self.lf_turn_msg   = PacmodCmd()
        self.lf_steer_msg  = PositionWithSpeed()

        # Exit Parking commands
        self.ep_enable_msg = Bool()
        self.ep_gear_msg   = PacmodCmd()
        self.ep_brake_msg  = PacmodCmd()
        self.ep_accel_msg  = PacmodCmd()
        self.ep_turn_msg   = PacmodCmd()
        self.ep_steer_msg  = PositionWithSpeed()

        # heading (yaw) from /insnavgeod and current lat/lon from /navsatfix:
        self.current_lat = None
        self.current_lon = None
        self.current_heading = 180 # TEMPOARY
        self.goal_long = None
        self.goal_lat = None

        self.first_goal_lat = None
        self.first_goal_long = None
        self.init_goal_long = False
        self.init_goal_lat = False

        # Object detection break
        self.obsticle_stop = False

        # Set control scripts to unactive by default
        self.exit_parking_active_pub.publish(Bool(data=False))
        self.lane_detection_active_pub.publish(Bool(data=False))

        # Immediately publish IDLE output to keep the car stationary
        self.publish_muxed_commands()
        self.enable_pub.publish(Bool(data=True))
        

        # Instead of a loop, we'll just spin:
        rospy.loginfo("SummonManager node is ready.")
        rospy.spin()


        


    # ---------------------------------------------------------------------
    # Callback to mux the enable_sub signal based on fsm_state.
    # ---------------------------------------------------------------------
    def pacmod_enable_callback(self, msg):
        rospy.loginfo(f"Received pacmod/as_tx/enable: {msg.data}")
        if self.fsm_state == 2:
            self.lf_enable_pub.publish(msg)
            rospy.loginfo("Forwarded enable msg to LF_OUTPUT/enable")
        elif self.fsm_state == 1:
            self.ep_enable_pub.publish(msg)
            rospy.loginfo("Forwarded enable msg to EP_OUTPUT/enable")
        else:
            rospy.loginfo("FSM state is IDLE; not forwarding enable msg to mirror publishers")

    # ---------------------------------------------------------------------
    # Common MUX logic: pick correct PACMod commands based on self.fsm_state
    # ---------------------------------------------------------------------
    def publish_muxed_commands(self):
        if self.fsm_state == 0 or self.obsticle_stop:
            # IDLE: keep the car stationary
            rospy.loginfo("Put car in IDLE")
            enable_msg = Bool(data=True)
            gear_msg   = PacmodCmd(ui16_cmd=3)    # PARK
            brake_msg  = PacmodCmd(f64_cmd=1.0)     # Full brake
            accel_msg  = PacmodCmd(f64_cmd=0.0)
            turn_msg   = PacmodCmd(ui16_cmd=1)      # No turn command
            steer_msg  = PositionWithSpeed(angular_position=0.0)

        elif self.fsm_state == 2:
            # EXIT: use Lane Following mirror commands
            enable_msg = self.lf_enable_msg
            gear_msg   = self.lf_gear_msg
            brake_msg  = self.lf_brake_msg
            accel_msg  = self.lf_accel_msg
            turn_msg   = self.lf_turn_msg
            steer_msg  = self.lf_steer_msg

        elif self.fsm_state == 1:
            # LINE FOLLOW: use Exit Parking mirror commands
            enable_msg = self.ep_enable_msg
            gear_msg   = self.ep_gear_msg
            brake_msg  = self.ep_brake_msg
            accel_msg  = self.ep_accel_msg
            turn_msg   = self.ep_turn_msg
            steer_msg  = self.ep_steer_msg

        # Publish the muxed commands:
        self.enable_pub.publish(enable_msg)
        self.gear_pub.publish(gear_msg)
        self.brake_pub.publish(brake_msg)
        self.accel_pub.publish(accel_msg)
        self.turn_pub.publish(turn_msg)
        self.steer_pub.publish(steer_msg)

    # ---------------------------------------------------------------------
    # Example method to change FSM state on demand:
    # (You might use a service, or a dedicated topic, etc.)
    # ---------------------------------------------------------------------
    def set_fsm_state(self, new_state):
        rospy.loginfo(f"FSM state changed from {self.fsm_state} to {new_state}")
        self.fsm_state = new_state
        # Optionally republish based on the updated state
        self.publish_muxed_commands()

    def compute_bearing(self, lat1, lon1, lat2, lon2):
        # Convert degrees to radians
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        diff_long = math.radians(lon2 - lon1)

        x = math.sin(diff_long) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(diff_long)

        initial_bearing = math.atan2(x, y)
        initial_bearing_deg = math.degrees(initial_bearing)
        # Normalize to 0–360
        return (initial_bearing_deg + 360) % 360
    
    def update_exit_direction(self):
        if self.current_lat is None or self.current_lon is None or self.current_heading is None or self.goal_long is None or self.goal_lat is None:
            rospy.logwarn("Can't compute direction yet; missing lat/lon or heading.")
            return

        bearing_to_goal = self.compute_bearing(
            self.current_lat,
            self.current_lon,
            self.goal_lat,
            self.goal_long
        )
        # Compute relative angle from current heading to bearing
        relative_angle = (bearing_to_goal - self.current_heading + 360) % 360

        # Decide left (0) or right (1) if relative_angle > 180, it's to the left
        if relative_angle > 180:
            direction_msg = Bool(data=False)  # 0 = left
        else:
            direction_msg = Bool(data=True)   # 1 = right

        rospy.loginfo(f"Bearing: {bearing_to_goal}, heading: {self.current_heading}, relative: {relative_angle}, direction: {direction_msg.data}")
        self.exit_direction_pub.publish(direction_msg)

    def pull_out_activator(self):
        if self.first_goal_long is not None and self.first_goal_lat is not None:
            if self.first_goal_lat != self.goal_lat or self.first_goal_long != self.goal_long:
                self.exit_parking_active_pub.publish(Bool(data=True))
                self.set_fsm_state(1)
                self.status_pub.publish(Int32(data=1))
                rospy.loginfo("Transitioned to parking exit mode")
                self.init_goal_long = False
                self.init_goal_lat = False

    # ---------------------------------------------------------------------
    # Other Callbacks (Webapp, GPS, etc.)
    # ---------------------------------------------------------------------
    def goal_long_callback(self, msg):
        if not self.init_goal_long:
            self.first_goal_long = msg.data
            self.init_goal_long = True
        self.goal_long = msg.data
        self.update_exit_direction()
        if self.fsm_state == 0:
            self.pull_out_activator()
        #rospy.loginfo(f"Received goal longitude: {msg.data}")

    def goal_lat_callback(self, msg):
        if not self.init_goal_lat:
            self.first_goal_lat = msg.data
            self.init_goal_lat = True
        self.goal_lat = msg.data
        self.update_exit_direction()
        if self.fsm_state == 0:
            self.pull_out_activator()
        #rospy.loginfo(f"Received goal latitude: {msg.data}")

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        rospy.loginfo(f"Received GPS pos lat={self.current_lat}, lon={self.current_lon}")

    def insnavgeod_callback(self, msg):
        #self.current_heading = msg.heading
        rospy.loginfo(f"INS Heading: {self.current_heading} degrees")

    def exit_parking_active_callback(self, msg):
        if not msg.data and self.fsm_state == 1:
            rospy.loginfo("Exit complete. Transitioning to FSM state 2 (LINE FOLLOW).")
            self.set_fsm_state(2)
            self.lane_detection_active_pub.publish(Bool(data=True))

    def stop_callback(self, msg):
        if msg.data:
            self.obsticle_stop = True
            self.stop_pub.publish(Bool(data=False))
            self.publish_muxed_commands()
        rospy.loginfo(f"Object detection stop: {self.obsticle_stop}")

    def restart_callback(self, msg):
        if msg.data:
            self.obsticle_stop = False
            self.restart_pub.publish(Bool(data=False))
            self.publish_muxed_commands()
        rospy.loginfo(f"Object detection restart: {self.obsticle_stop}")

    # ---------------------------------------------------------------------
    # Lane Following mirror callbacks
    # Store new commands, then publish if in the proper state.
    # ---------------------------------------------------------------------
    def lf_enable_callback(self, msg):
        self.lf_enable_msg = msg
        self.publish_muxed_commands()

    def lf_gear_callback(self, msg):
        self.lf_gear_msg = msg
        self.publish_muxed_commands()

    def lf_brake_callback(self, msg):
        self.lf_brake_msg = msg
        self.publish_muxed_commands()

    def lf_accel_callback(self, msg):
        self.lf_accel_msg = msg
        self.publish_muxed_commands()

    def lf_turn_callback(self, msg):
        self.lf_turn_msg = msg
        self.publish_muxed_commands()

    def lf_steer_callback(self, msg):
        self.lf_steer_msg = msg
        self.publish_muxed_commands()

    # ---------------------------------------------------------------------
    # Exit Parking mirror callbacks
    # Store new commands, then publish if in the proper state.
    # ---------------------------------------------------------------------
    def ep_enable_callback(self, msg):
        self.ep_enable_msg = msg
        self.publish_muxed_commands()

    def ep_gear_callback(self, msg):
        self.ep_gear_msg = msg
        self.publish_muxed_commands()

    def ep_brake_callback(self, msg):
        self.ep_brake_msg = msg
        self.publish_muxed_commands()

    def ep_accel_callback(self, msg):
        self.ep_accel_msg = msg
        self.publish_muxed_commands()

    def ep_turn_callback(self, msg):
        self.ep_turn_msg = msg
        self.publish_muxed_commands()

    def ep_steer_callback(self, msg):
        self.ep_steer_msg = msg
        self.publish_muxed_commands()

if __name__ == '__main__':

    try:
        node = SummonManager()
    except rospy.ROSInterruptException:
        rospy.loginfo("SummonManager node shut down.")
