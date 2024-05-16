#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix

class GPSPublisher():
    def __init__(self):
        rospy.init_node('gps_node')
        rospy.loginfo('GPS publisher node started')
        self.publish_gps_data()
    
    def generate_gps_data(self):
        latitude = 51.5074  # Example latitude (London)
        longitude = -0.1278  # Example longitude (London)
        altitude = 100  # Example altitude
        return latitude, longitude, altitude
   
    def publish_gps_data(self):
        gps_pub = rospy.Publisher('gps_data', NavSatFix, queue_size=10)
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown():
            latitude, longitude, altitude = self.generate_gps_data()
            gps_msg = NavSatFix()
            gps_msg.latitude = latitude
            gps_msg.longitude = longitude
            gps_msg.altitude = altitude
            gps_pub.publish(gps_msg)
            rate.sleep()

if __name__ == "__main__":
    GPSPublisher()
