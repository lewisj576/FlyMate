#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
import random


class Lidar():
    def __init__(self):
        rospy.init_node('lidar_node')
        rospy.loginfo('lidar publisher node started')
        self.generate_lidar_data()
        self.publish_lidar_data()
    
    def generate_lidar_data(self):
        x = random.randint(0, 100) #generate lidar data
        y = random.randint(0, 100)
        z = random.randint(0, 100)
        lidar_data = [x, y, z]
        return lidar_data
   
    def publish_lidar_data(self):
        lidar_pub = rospy.Publisher('lidar_data', Int32MultiArray, queue_size=10)
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown():
            lidar_data = self.generate_lidar_data()
            lidar_data_msg = Int32MultiArray()
            lidar_data_msg.data = lidar_data
            lidar_pub.publish(lidar_data_msg)
            rate.sleep()

if __name__ == "__main__":
    Lidar()
