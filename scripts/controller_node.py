#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu, NavSatFix
from flymate.srv import MotorControl
from flymate.msg import FlightPathAction, FlightPathGoal


class ControllerNode():
    def __init__(self):
        rospy.init_node("controller_node")
        rospy.loginfo("Hi, I'm launched!")
        self.lidar_subscriber()
        self.imu_subscriber()
        self.gps_subscriber()
        self.send_motor_command(0.5) 
        self.publish_target_position()  
        self.nav_client()
        rospy.spin()

    def imu_callback(self, msg):
        rospy.loginfo("Received IMU data:")
        rospy.loginfo("Orientation (x, y, z, w): %f, %f, %f, %f", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        rospy.loginfo("Angular Velocity (x, y, z): %f, %f, %f", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        rospy.loginfo("Linear Acceleration (x, y, z): %f, %f, %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

    def imu_subscriber(self):
        rospy.Subscriber('imu_data', Imu, self.imu_callback)

    def lidar_callback(self, msg):
        rospy.loginfo("Received lidar data: %s", msg.data)

    def lidar_subscriber(self):
        rospy.Subscriber('lidar_data', Int32MultiArray, self.lidar_callback)

    def gps_callback(self, msg):
        rospy.loginfo("Received GPS data:")
        rospy.loginfo("Latitude: %f, Longitude: %f", msg.latitude, msg.longitude)

    def gps_subscriber(self):
        rospy.Subscriber('gps_data', NavSatFix, self.gps_callback)

    def send_motor_command(self, speed):
        rospy.wait_for_service('MotorControl')
        try:
            motor_control = rospy.ServiceProxy('MotorControl', MotorControl)
            resp = motor_control(speed)
            if resp.success:
                rospy.loginfo("Motor control command succeeded")
            else:
                rospy.logwarn("Motor control command failed")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def feedback_callback(self, feedback):
        rospy.loginfo("Distance to Goal: " + str(feedback.distance_to_point))

    def nav_client(self):
        client = actionlib.SimpleActionClient("navigate_2D_action", FlightPathAction)
        client.wait_for_server()
        point_msg = Point(x=0, y=0, z=0)
        goal = FlightPathGoal(point_msg)
        client.send_goal(goal, feedback_cb=self.feedback_callback)
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo("Navigation result: %s", result)

    def publish_target_position(self):
        pub = rospy.Publisher("robot_point", Point, queue_size=10)
        rospy.loginfo("Publishing target position (0, 0, 0)")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pub.publish(Point(x=0, y=0, z=0))
            rate.sleep()


if __name__ == "__main__":
    ControllerNode()
