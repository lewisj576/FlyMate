#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import random

class IMU():
    def __init__(self):
        rospy.init_node('imu_node')
        rospy.loginfo('IMU publisher node started')
        self.imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        self.publish_imu_data()

    def generate_imu_data(self):
        magnetometer_data = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)] #generate magnetometer data
        gyroscope_data = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)] #generate gyroscope data
        accelerometer_data = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)] #generate accelerometer data
        return magnetometer_data, gyroscope_data, accelerometer_data
   
    def publish_imu_data(self):
        rate = rospy.Rate(0.1)  #publish at 1hz

        while not rospy.is_shutdown():
            magnetometer_data, gyroscope_data, accelerometer_data = self.generate_imu_data()
            
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "base_link"

            imu_msg.orientation.x = 0
            imu_msg.orientation.y = 0
            imu_msg.orientation.z = 0
            imu_msg.orientation.w = 1

            imu_msg.angular_velocity.x = gyroscope_data[0]
            imu_msg.angular_velocity.y = gyroscope_data[1]
            imu_msg.angular_velocity.z = gyroscope_data[2]

            imu_msg.linear_acceleration.x = accelerometer_data[0]
            imu_msg.linear_acceleration.y = accelerometer_data[1]
            imu_msg.linear_acceleration.z = accelerometer_data[2]

            imu_msg.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
            imu_msg.angular_velocity_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
            imu_msg.linear_acceleration_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

            self.imu_pub.publish(imu_msg)
            rate.sleep()

if __name__ == "__main__":
    IMU()
