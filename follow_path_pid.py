#!/usr/bin/python

import sys
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
from math import cos, sin, atan2, sqrt
import math

def follow_path(target_x, target_y):
    rospy.init_node('follow_path', anonymous=True)
    rate = rospy.Rate(10)

    forward = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    model_coords = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    prev_error = 0
    integral_error = 0
 
    Kl = 1
    Ka = 4
    
    twist = Twist()

    count = 0    
    while not rospy.is_shutdown():
        drone_coords = model_coords('mobile_base', '')
        
        current_pos = drone_coords.pose.position
        current_orientation = drone_coords.pose.orientation
        
        _, _, current_yaw = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        
        #print(current_pos.x, " ",current_pos.y," ",current_pos.z)
        error_x = target_x - current_pos.x
        error_y = target_y - current_pos.y
        
        # Rotate the TurtleBot towards the target
        desired_yaw = atan2(error_y, error_x)
        rotate_error = desired_yaw - current_yaw
	
        if abs(rotate_error) > 0.01:
            twist.linear.x = 0.0
            twist.angular.z = rotate_error * Ka  # Apply proportional control
            forward.publish(twist)
            rate.sleep()
            #rospy.sleep(1.0)
            continue
	
	#calculate distance to target and adjust velocity accordingly
        distance = sqrt(error_x ** 2 + error_y ** 2)
        linear_velocity=0.0
	if (distance>0.01):
            max_linear_velocity = 1.0
            linear_velocity = min(distance * Kl, max_linear_velocity)
	else:
            twist.linear.x = linear_velocity
	    twist.angular.z = 0.0
            forward.publish(twist)
	    rospy.sleep(1.0)
	    return #break

        #move forward using the calculated linear velocity
        twist.linear.x = linear_velocity 
	twist.angular.z = 0.0
        forward.publish(twist)
	rospy.sleep(1.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        #path = path_from_prm() - compute path using prm
	#example
	path=[[-2.0,-1.0],[-3.0,0],[-5.0,1.0],[-5.0,3.0],[-3.0,1.0],[0.0,1.0],[0.0,0.0]]
        for node in path:
           follow_path(node[0], node[1])
    except rospy.ROSInterruptException:
        pass
