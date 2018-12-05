#! /usr/bin/env python

# Code not nice but have to do a fast test!

#Basic imports
import sys
import rospy
import time
import tf
import numpy as np
import utm
import json
import requests

from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from math import radians, pow
import tf.transformations
from std_msgs.msg import String
import math

##########################################################
#   Main
##########################################################
if __name__ == '__main__':

    start_time = time.time()
    rospy.loginfo("Camera_dragonfly_node starting up")
    rospy.init_node('Camera_dragonfly_node')

##########################################################
#   Define variable (also for ROS)
##########################################################
    # deviceNumber # smartphone we want to follow
    deviceNumber = 1
    user = rospy.get_param('/camera_dragonfly_node/usr')
    mpd = rospy.get_param('/camera_dragonfly_node/mpd')
    last_x = 0
    last_y = 0
    last_yaw = 0
    last_time = 0.0
    rot = 0
    pub_odom = rospy.Publisher('/GPS_Dragonfly', Odometry, queue_size=1)
    last_time = rospy.Time.now()
    init = False
    url = 'https://cvnav.accuware.com/api/v1/sites/100308/dragonfly/devices/'

    # Set origin if needed, but don't forget to init = True!
    # utm0 = [468589.12, 5264024.62]# Joao origin #utm.from_latlon(47.52889209247521, 8.582779991059633)
    # utm0 = [469665.57, 5262709.04]# Embrach origin #utm.from_latlon(47.517102, 8.597093)
    utm0 = [469665.41, 5262707.82]# Embrach 47.517074, 8.597091


##########################################################
#   Main loop to request data from accuware and publish it on server
##########################################################
    print('Getting data from camera_dragonfly')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
            # MSG FROM website of accuware
            #     [
            #   {
            #     "mac": "DCNVC458VMMD",
            #     "device_status": {
            #       "battery": 36,
            #       "fw_version": "2.1.0-a25",
            #       "received_at": 1539611233413,
            #       "type_of_device": "Dragonfly"
            #     },
            #     "position": {
            #       "siteId": "100308",
            #       "levelId": 0,
            #       "source": "Dragonfly",
            #       "device": "DCNVC458VMMD",
            #       "fixed_at": 1539611233413,
            #       "lat": 47.52889209247521,
            #       "lng": 8.582779991059633,
            #       "alt": -4.470291959490914,
            #       "precision": 0.0
            #     },
            #     "device_type": "D",
            #     "udo": {
            #       "name": "SM-G955F",
            #       "desc": "SM-G955F"
            #     },
            #     "current_server_time": 1539611327573
            #   }
            # ]
        # Request JSON from website
        r = requests.get(url, auth = (user, mpd))#,auth=('USR', 'MPD')
        # Transform request to JSON
        json_data = json.loads(r.text)
        # print(json_data[1]['udo']['name'])

        # Take the first GPS from Dragonfly and init to have a new zero..
        # Get only lat and long
        if(init == False):
            print('init')
            utm0 = utm.from_latlon(json_data[deviceNumber]['position']['lat'], json_data[deviceNumber]['position']['lng'])
            init = True
        utm1 = utm.from_latlon(json_data[deviceNumber]['position']['lat'], json_data[deviceNumber]['position']['lng'])
        # print('lat, lon : ', utm1[0], utm1[1])
        # print utm
        # utm1 = [2.0,1.0]
        # Create message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = (utm1[0] - utm0[0]) #utm_pos.easting - self.origin.x
        odom.pose.pose.position.y = (utm1[1]- utm0[1]) #utm_pos.northing - self.origin.y
        odom.pose.pose.position.z = 0

        # Velocities are not useful yet
        dt = (rospy.Time.now() - last_time).to_sec()
        dx = (odom.pose.pose.position.x - last_x)
        dy = (odom.pose.pose.position.y - last_y)
        dyaw = 0 # TODO radians(self.rot - self.last_yaw)
        # print('Time dt: ', dt, 's, x, y: ', odom.pose.pose.position.x, odom.pose.pose.position.y)
        vx = dx/dt
        vy = dy/dt
        vth = 0 #TODO dyaw/dt
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # Orientation
        orientation = tf.transformations.quaternion_from_euler( radians(0),
            radians(0), radians(0), 'syxz')
        # Not possible to have an orientation
        odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)

        # Publish odom
        pub_odom.publish(odom)
        last_x = odom.pose.pose.position.x
        last_y = odom.pose.pose.position.y
        last_yaw = rot
        last_time = rospy.Time.now()
        rate.sleep()

    print("End of the main!")
