#! /usr/bin/env python

#Basic imports
import sys
import rospy
import time
import tf
import numpy as np
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
    url = 'https://cvnav.accuware.com/api/v1/sites/100308/dragonfly/devices/'
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
    r = requests.get(url, auth=)#,auth=('USR', 'MPD')
    # print r.headers.get("mac")#['mac']
    json_data = json.loads(r.text)
    print json_data[0]['position']['lat']
    print json_data[0]['position']['lng']
    # print(payload)
    # print r.text #allow to screen the JSON file
    # data = json.loads(r.json)
    # data['position']
    # r.status_code
    # r.headers['content-type']
    # r.encoding
    # r.text
    # j = r.json()
    # msg = j.loads('{"position" : "lat", }')
    # print msg['lat']




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
