#!/usr/bin/env python
# -*-coding: utf-8 -*-

"""
# Bilgi : IMU verisi üretir.
# Hazırlayan : Muhammed Salih Aydoğan

"""

import rospy, tf, math, sys, random
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import *

def callbackimu(msg):
    global roll, pitch, yaw, derece
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    derece = yaw * (180 / math.pi)
    print ("angle value -->", derece)
    rospy.sleep(.5)  # zaman veriliyor

# düğüm sonlanınca robotu durdurur.
def shutdown():
    rospy.loginfo("Stopping the robot...")
    rospy.sleep()


if __name__ == '__main__':
    rospy.init_node('IMU', anonymous=False)
    imu_sub = rospy.Subscriber('/imu', Imu, callbackimu)
    rospy.on_shutdown(shutdown)
    rospy.spin()