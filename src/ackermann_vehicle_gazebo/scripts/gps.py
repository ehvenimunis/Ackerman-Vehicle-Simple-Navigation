#!/usr/bin/env python
# -*-coding: utf-8 -*-

"""

# Bilgi : GPS verisi yakalar. 
# Hazırlayan : Muhammed Salih Aydoğan

"""

import rospy, tf, math, sys, random
from std_msgs.msg import Float64, String, Int16
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import *


def gps_callback(data):
    global x
    global y
    pub_x = rospy.Publisher('/x', Float64, queue_size=1)
    pub_y = rospy.Publisher('/y', Float64, queue_size=1)

    get_local_coord(data.latitude, data.longitude)
    
    pub_x.publish(x)
    pub_y.publish(y)

def get_local_coord(lat, lon):
    global x
    global y
    WORLD_POLAR_M = 6356752.3142
    WORLD_EQUATORIAL_M = 6378137.0

    eccentricity = math.acos(WORLD_POLAR_M / WORLD_EQUATORIAL_M)
    n_prime = 1 / (math.sqrt(1 - math.pow(math.sin(math.radians(float(lat))), 2.0) * math.pow(math.sin(eccentricity), 2.0)))
    m = WORLD_EQUATORIAL_M * math.pow(math.cos(eccentricity), 2.0) * math.pow(n_prime, 3.0)
    n = WORLD_EQUATORIAL_M * n_prime

    difflon = float(lon) - float(-51.173913575780311191)
    difflat = float(lat) - float(-30.06022459407145675)

    surfdistLon = math.pi / 180.0 * math.cos(math.radians(float(lat))) * n
    surfdistLat = math.pi / 180.00 * m

    x = difflon * surfdistLon
    y = difflat * surfdistLat      
    rospy.sleep(2)  # tf in buffer ı doldurması için zaman veriliyor

    # düğüm sonlanınca robotu durdurur.
def shutdown():
    rospy.loginfo("Stopping the robot...")
    rospy.sleep()

if __name__ == '__main__':
    rospy.init_node('gps_read', anonymous=False)
    gps_sub = rospy.Subscriber("gps/fix", NavSatFix, gps_callback, queue_size=1)
    rospy.on_shutdown(shutdown)
    rospy.spin()