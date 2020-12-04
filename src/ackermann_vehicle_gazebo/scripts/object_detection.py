#!/usr/bin/env python
# -*-coding: utf-8 -*-

"""
# Bilgi : Lidar kullanarak etrafondaki engelleri algılar. Hesaplama yaparak engel konumu bulmaktadır. 
          Ardından location konusuna bunu dizi olarak paylaşır. Bu dizinin ilk elemanı aracın konumudur. 
# Hazırlayan : Muhammed Salih Aydoğan

"""

import rospy, tf, math, sys, random
from std_msgs.msg import Float32MultiArray, Float64, String, Int16, Bool
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDrive
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import *

global x
global y
global rotation
global goalx
global goaly
global paylas
paylas = True
x_cordinate = []
y_cordinate = []


class Control():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.scan_sub = rospy.Subscriber('laser/scan', LaserScan, clbk_laser)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.callbackimu)
        self.gps_sub = rospy.Subscriber("gps/fix", NavSatFix, gps_callback, queue_size=1)
        rospy.sleep(2)  # tf in bufferı doldurması için zaman veriliyor

        global derece # bu değişken
        global distance
        global paylas

        while(1):
            if paylas == True:
                print("engel verisi paylaşılıyor")
                if derece <= 0:
                    i = 0
                    i = int(derece / 7.5)
                    #print "negatif derece"
                    # printing the original list
                    # print ("The original list is : " + str(distance))

                    # using list slicing and "+" operator
                    # shift last element to first
                    distance = distance[-i:] + distance[:-i]  # shift

                    hesapla(distance)
                    # printing result
                    # print ("The list after shift is : " + str(distance))
                else:
                    i = 0
                    i = int(derece / 7.5)
                    #print "pozitif derece"
                    # printing the original list
                    # print ("The original list is : " + str(distance))
                    distance = distance[-i:] + distance[:-i]  # shift
                    # using list slicing and "+" operator
                    # shift last element to first

                    hesapla(distance)
                    # printing result
                    # print ("The list after shift is : " + str(distance))

                #print (len(distance))
                #print (distance[0:11])
                rospy.sleep(1)
            else:
                print("onay bekleniyor..")
                rospy.sleep(1)  # bu emir döngü içinde 2 saniye bekletir.

    def callbackimu(self, msg):
        global roll, pitch, yaw, derece
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.rotation = yaw
        derece = yaw * (180 / math.pi)

    # düğüm sonlanınca robotu durdurur.
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep()

def bool_callback(data):
    global paylas
    if data.data == True:
        print("data paylas")
        paylas = True
    else:
        paylas = False

if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=False)
    cmd = rospy.Publisher('location', Float32MultiArray, queue_size=10)
    bol = rospy.Subscriber('start', Bool, bool_callback)

    def gps_callback(data):
        global x,y
        pub = rospy.Publisher('/x', Float64, queue_size=1)
        longitude = data.longitude
        latitude = data.latitude
        get_local_coord(latitude, longitude)
        pub.publish(x)

    def get_local_coord(lat, lon):
        global x,y
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

    def clbk_laser(msg):
        global regions_
        i = 0
        global distance
        distance = []
        for i in range(0, 720, 15):
            distance.append(min(min(msg.ranges[i:i + 15]), 'nan'))

    def array_publish(data):   # fonksiyon içinde fonksiyon tanımlandı
        array = Float32MultiArray(data = data)
        cmd.publish(array)  # dizi elemanları tek tek paylaşılır

    def hesapla(data):
        global x, y, derece, a
        i = 0
        a = 0
        b = 0

        tuple = (x, y)
        liste = [] # içi boşalır her seferinde 
        liste.append(tuple)
        array_publish(liste[-1])
        print ("\nengel konumları paylaşılıyor ...")
        for i in range(48):
            if i <= 11:
                a = x - (math.cos(i * (math.pi / 24)) * (float(data[i])))
                b = y - (math.sin(i * (math.pi / 24)) * (float(data[i])))
                if (not math.isinf(-a)):
                    if (not math.isinf(-b)):
                        tuple = (a, b)
                        liste.append(tuple)
                        array_publish(liste[-1]) 
            elif 12 <= i <= 23:
                a = x - (math.cos(i * (math.pi / 24)) * (float(data[i])))
                b = y - (math.sin(i * (math.pi / 24)) * (float(data[i])))
                if (not math.isinf(a)):
                    if (not math.isinf(-b)):
                        tuple = (a, b)
                        liste.append(tuple)
                        array_publish(liste[-1])
            elif 24 <= i <= 35:
                a = x - (math.cos(i * (math.pi / 24)) * (float(data[i])))
                b = y - (math.sin(i * (math.pi / 24)) * (float(data[i])))
                if (not math.isinf(a)):
                    if (not math.isinf(b)):
                        tuple = (a, b)
                        liste.append(tuple)
                        array_publish(liste[-1])
            elif 36 <= i:
                a = x - (math.cos(i * (math.pi / 24)) * (float(data[i])))
                b = y - (math.sin(i * (math.pi / 24)) * (float(data[i])))
                if (not math.isinf(-a)):
                    if (not math.isinf(b)):
                        tuple = (a, b)
                        liste.append(tuple)
                        array_publish(liste[-1])
        tuple = (999.0, 999.0) # dizinin bittiş işareti olması için bu sayı gönderildi
        liste.append(tuple)
        array_publish(liste[-1])
        print('paylaşılan dizi boyutu : ' + str(len(liste)))
        print(str(liste))
        #pub.publish(len(liste)) # liste boyutu paylaşılır 
        # print len(liste) # listenin boyutu 3 burada bir topic ile paylaş 
                        
    Control()
    rospy.spin()