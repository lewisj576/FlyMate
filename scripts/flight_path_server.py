#!/usr/bin/env python3
import rospy
import actionlib
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from flymate.msg import FlightPathAction, FlightPathFeedback, FlightPathResult

class NavigateDroneClass:
    
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer("flight_path_action", FlightPathAction, self.flight_path)
        self.drone_gps_sub = rospy.Subscriber("gps_data", NavSatFix, self.update_drone_position)
        self.drone_current_pos = None
        self.drone_target_pos = None
        self.distance_threshold = 2

    def flight_path(self, goal):
        self.start_time = rospy.get_time()
        self.drone_target_pos = [goal.point.latitude, goal.point.longitude, goal.point.altitude]

        while self.drone_current_pos is None:
            rospy.loginfo("Waiting for drone position")
            rospy.sleep(1)

        distance_to_target = math.dist(self.drone_current_pos, self.drone_target_pos)

        while distance_to_target > self.distance_threshold:
            self.action_server.publish_feedback(FlightPathFeedback(distance_to_target=distance_to_target))
            distance_to_target = math.dist(self.drone_current_pos, self.drone_target_pos)
            rospy.sleep(1)

        self.end_time = rospy.get_time()
        elapsed_time = self.end_time - self.start_time
        rospy.loginfo("Drone mission successful, time elapsed: " + str(elapsed_time) + "secs" )
        self.action_server.set_succeeded(FlightPathResult(elapsed_time))

    def update_drone_position(self, gps_msg):
        self.drone_current_pos = [gps_msg.latitude, gps_msg.longitude, gps_msg.altitude]

if __name__ == '__main__':
    rospy.init_node("flight_path_drone")
    drone_server = NavigateDroneClass()
    rospy.spin() 



