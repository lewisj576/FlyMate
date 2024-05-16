#!/usr/bin/env python3

import rospy
from flymate.srv import MotorControl


class Motor():
    def __init__(self):
        rospy.init_node('motor_node')
        rospy.loginfo('Motor node started')
        self.motor_node()

    def handle_motor_control(self, req):
        speed = req.speed
        # Execute motor control command here (e.g., adjust motor speed)
        rospy.loginfo("Received motor control command. Speed: %.2f", speed)
        return True  # Indicate success

    def motor_node(self):
        rospy.Service('motor_control', MotorControl, self.handle_motor_control)
        rospy.loginfo("Motor control service is ready")
        rospy.spin()


if __name__ == "__main__":
    Motor()