#! /usr/bin/env python

#Basic imports
import sys
import rospy
import time
import tf
import numpy as np


from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from math import radians, pow
import tf.transformations
# from std_msgs.msg import Empty
from std_msgs.msg import String
import math


#Publish directly what we need for the motor
# global motor_pub
# motorpub = rospy.Publisher('/motor_state', String, queue_size=100)
#
# global mot_pub
# mot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
# global mot_msg
# # Global variable..
# global ready_to_go
# ready_to_go = rospy.set_param('/camera_dragonfly/ready_to_go', False)#
# global success
# success = rospy.set_param('/camera_dragonfly/success', False)#
# global target_x
# target_x = rospy.set_param('/camera_dragonfly/target_x', 1.0)
# global target_y
# target_y = rospy.set_param('/camera_dragonfly/target_y', -1.5)


# def turning_callback(msg):
#
#     print("Successful mission !")




def run():


    start_time = time.time()
    rospy.loginfo("Camera_dragonfly_node starting up")
    # pos_gps_pub = rospy.Publisher('/odom_dragonfly', Twist, queue_size=100)
    #basic program code

    rospy.init_node('Camera_dragonfly_node')
    print('Yaaay')
    # rospy.Subscriber("/base_link_odom_camera_is1500", Odometry, turning_callback)

    # timer = rospy.Timer(rospy.Duration(0.1), main_timer_callback)

    # blinkertimer = rospy.Timer(rospy.Duration(0.5), blinker_state_callback) #set the duration of this callback to set the speed of the blink

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # timer.shutdown()

if __name__ == '__main__':
    print("start")
    try:
        run()
    except rospy.ROSInterruptException:
        pass

    print("end")
    mot_pub.publish(mot_msg)
