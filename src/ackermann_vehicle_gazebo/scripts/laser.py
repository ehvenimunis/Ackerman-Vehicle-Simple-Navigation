#!/usr/bin/env python
# -*-coding: utf-8 -*-

"""
# Bilgi : Laser scan verisi anlamlandırır. 
# Hazırlayan : Muhammed Salih Aydoğan
"""

import rospy, tf, math, sys, random
from sensor_msgs.msg import NavSatFix, LaserScan
from math import *
    
def clbk_laser(msg):
    global regions_, distance
    i = 0
    distance = []
    for i in range(0, 720, 15): # üretilen 720 veri 15 birimlik adımlar ile incelenir. 
        distance.append(min(min(msg.ranges[i:i + 15]), 'nan'))
        #print i

# düğüm sonlanınca robotu durdurur.
def shutdown():
    rospy.loginfo("Stopping the robot...")
    rospy.sleep()

if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=False) # Node name 
    scan_sub = rospy.Subscriber('laser/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rospy.spin()





