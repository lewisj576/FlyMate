#!/usr/bin/python3
import rospy

if __name__ == '__main__':
    rospy.init_node('param_node')
    rospy.loginfo('param server started')

    # Set the parameter with a default value of 40
    rospy.set_param('battery_required',0)

    rospy.set_param('current_lat',0) #once order is made this is set to nearest drone charging station where done is ready
    rospy.set_param('current_long',0)
    rospy.set_param('sending_lat',0)    
    rospy.set_param('sending_long',0)
    rospy.set_param('delivery_lat',0)
    rospy.set_param('delivery_long',0)
    
    rospy.set_param('lidar', [0,0,0]) #x, y, z Cartesian coordinates point cloud
    rospy.set_param('linear_acceleration', [0,0, 0]) #stationary
    rospy.set_param('angular_velocity', [0,0,0]) #stationary
    rospy.set_param('orientation', [0,0,0]) #roll pitch yaw
    rospy.set_param('pressure_mb', 1013.25) #standard sea level atmospheric pressure

