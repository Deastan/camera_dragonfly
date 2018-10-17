#! /usr/bin/env python

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
# from std_msgs.msg import Empty
from std_msgs.msg import String
import math


#define variable
global last_x
global last_y
global last_yaw
global last_time
global rot
global pub_odom
global user
global mpd
user = rospy.get_param('/camera_dragonfly_node/usr')
mpd = rospy.get_param('/camera_dragonfly_node/mpd')
last_x = 0
last_y = 0
last_yaw = 0
last_time = 0.0
rot = 0
pub_odom = rospy.Publisher('/GPS_Dragonfly', Odometry, queue_size=1)


def run():

    global last_x
    global last_y
    global last_yaw
    global last_time
    global rot
    global pub_odom
    global user

    global mpd
    init = False
    url = 'https://cvnav.accuware.com/api/v1/sites/100308/dragonfly/devices/'

    # Set origin
    utm0 = [468589.12, 5264024.62]# Joao origin #utm.from_latlon(47.52889209247521, 8.582779991059633)

    print('Getting data from camera_dragonfly')
    while True:
            # MSG FROM accuware
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
        # Transform requestion to JSON
        json_data = json.loads(r.text)
        # print(json_data[1]['udo']['name'])
        # Get only lat and long
        if(init == False):
            print('init')
            utm0 = utm.from_latlon(json_data[0]['position']['lat'], json_data[0]['position']['lng'])
            init = True
        utm1 = utm.from_latlon(json_data[0]['position']['lat'], json_data[0]['position']['lng'])
        # print('lat, lon : ', utm1[0], utm1[1])
        # print utm
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom_cam_drag'
        odom.child_frame_id = 'base_link_cam_drag'
        odom.pose.pose.position.x = (utm1[0] - utm0[0]) #utm_pos.easting - self.origin.x
        odom.pose.pose.position.y = (utm1[1]- utm0[1]) #utm_pos.northing - self.origin.y
        odom.pose.pose.position.z = 0

        dt = (rospy.Time.now() - last_time).to_sec()
        dx = (odom.pose.pose.position.x - last_x)
        dy = (odom.pose.pose.position.y - last_y)
        dyaw = 0 # TODO radians(self.rot - self.last_yaw)
        print('Time dt: ', dt, 's, x, y: ', odom.pose.pose.position.x, odom.pose.pose.position.y)
        vx = dx/dt
        vy = dy/dt
        vth = 0 #TODO dyaw/dt
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    	# Orientation
    	# Save this on an instance variable, so that it can be published
    	# with the IMU message as well.
        orientation = tf.transformations.quaternion_from_euler( radians(0), radians(0), radians(0), 'syxz')#-radians(rot-90), 'syxz')
        odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)#orientation)
    	# odom.pose.covariance[21] = self.orientation_covariance[0] = np.var(self.x_vec)
    	# odom.pose.covariance[28] = self.orientation_covariance[4] = np.var(self.y_vec)
    	# odom.pose.covariance[35] = self.orientation_covariance[8] = pow(2, 0.001)
        #
    	# #~ # Twist is relative to vehicle frame
        #
    	# TWIST_COVAR[0] = pow(2, 0.0001)
    	# TWIST_COVAR[7] = pow(2, 0.0001)
    	# TWIST_COVAR[14] = pow(2, 0.0001)
        #
    	# odom.twist.covariance = TWIST_COVAR

        pub_odom.publish(odom)
        last_x = odom.pose.pose.position.x
        last_y = odom.pose.pose.position.y
        last_yaw = rot
        last_time = rospy.Time.now()


if __name__ == '__main__':

    start_time = time.time()
    rospy.loginfo("Camera_dragonfly_node starting up")
    # pos_gps_pub = rospy.Publisher('/odom_dragonfly', Twist, queue_size=100)
    #basic program code
    rospy.init_node('Camera_dragonfly_node')
    last_time = rospy.Time.now()
    # while True:
    try:
        run()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
