#!/usr/bin/env python3

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import NavSatFix

# For networking calls
import requests

class Networking:

    def __init__(self):
        rospy.init_node('networking', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz control loop

        # Publishers
        self.goal_long_pub = rospy.Publisher('/WEBAPP/goal_long', Float64, queue_size=1, latch = True)
        self.goal_lat_pub = rospy.Publisher('/WEBAPP/goal_lat', Float64, queue_size=1, latch = True)

        # GPS
        self.gps_sub = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gps_callback)

        # Subscriber
        self.status_sub = rospy.Subscriber('/WEBAPP/status', Int32, self.status_callback)

        # -- Additional lines after init lines (don't remove the lines above) --
        # Summon API configuration
        self.API_BASE = "https://gem-summon-backend.onrender.com"
        self.TOKEN = "sfdvghtrgta4y5tes4srt4awesrfg"
        self.HEADERS = {"Content-Type": "application/json", "Authorization": self.TOKEN}

        # In case we want to store the previous phone location
        self.last_phone_location = None
        self.latest_gps = None  
        rospy.Timer(rospy.Duration(1), self.process_latest_gps)

        self.update_car_status(0)

    def gps_callback(self, msg):
        self.latest_gps = msg  # don’t do any logging or requests here
    
    def process_latest_gps(self, event):
        if self.latest_gps is not None:
            lat = self.latest_gps.latitude
            lng = self.latest_gps.longitude
            rospy.loginfo(f"Fresh GPS → lat: {lat}, lng: {lng}")
            self.update_car_location(lat, lng)  # blocking call happens here



    def status_callback(self, msg):
        """
        Callback for receiving the vehicle's status.
        We send that status to the Summon API.
        """
        status_int = msg.data
        rospy.loginfo(f"Received status: {status_int}")
        self.update_car_status(status_int)

    def update_car_location(self, lat, lng):
        """
        Send the car's location to the Summon API.
        """
        try:
            response = requests.post(
                f"{self.API_BASE}/car-location",
                headers=self.HEADERS,
                json={"location": {"lat": lat, "lng": lng}}
            )
            if response.status_code != 200:
                rospy.logwarn(f"Error updating car location: {response.text}")
        except Exception as e:
            rospy.logerr(f"Exception updating car location: {e}")

    def update_car_status(self, status):
        """
        Send the car's status to the Summon API.
        """
        if status == 0:
            status_text = "IDLE"
        if status == 1:
            status_text = "SUMMONING"
        if status == 2:
            status_text = "ARRIVED"

        try:
            response = requests.post(
                f"{self.API_BASE}/car-status",
                headers=self.HEADERS,
                json={"status": status_text}
            )
            if response.status_code != 200:
                rospy.logwarn(f"Error updating car status: {response.text}")
        except Exception as e:
            rospy.logerr(f"Exception updating car status: {e}")

    def get_phone_location(self):
        """
        Fetch the current 'summon' (goal) location from the API.
        Returns a dict with {'lat': ..., 'lng': ...} or None.
        """
        try:
            response = requests.get(f"{self.API_BASE}/latest-location", headers=self.HEADERS)
            if response.status_code == 200:
                data = response.json()
                return data.get("location")
            else:
                rospy.logwarn(f"Error fetching phone location: {response.text}")
        except Exception as e:
            rospy.logerr(f"Error fetching phone location: {e}")
        return None

    def run(self):
        rospy.loginfo("Networking node is running...")
        while not rospy.is_shutdown():
            # Periodically fetch the phone (goal) location and publish it
            phone_location = self.get_phone_location()
            if phone_location:
                # phone_location should be a dict like {"lat": x, "lng": y}
                goal_lat = phone_location.get("lat", 0.0)
                goal_lng = phone_location.get("lng", 0.0)
            else:
                goal_lat = 0.0
                goal_lng = 0.0

            self.goal_lat_pub.publish(goal_lat)
            self.goal_long_pub.publish(goal_lng)
            self.rate.sleep()

# ============================
# Main Entry Point for Node
# ============================

if __name__ == '__main__':
    try:
        node = Networking()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Networking node shut down.")
