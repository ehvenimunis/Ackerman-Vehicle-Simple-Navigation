#!/usr/bin/env python
# -*-coding: utf-8 -*-

"""
# Bilgi : Robotu hareket ettirir ve hedefe ulaşınca durdurur. Bunun için object_detection.py ve RRT.py dosyalarını kullanır.
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
cur_pos = Point()
q_goal = Point()
paylas = bool()
x_cordinate = []
y_cordinate = []

def bool_callback(data):
    global paylas
    paylas = data.data
    if paylas == True:
        print("\nRRT Hesaplandı, hareket et!")
    else:
        print("\nRRT Hesaplanıyor, aracı durdur!")
        main_fonk(None, False)

def clbk_laser(msg):
    global regions_
    '''the len of msg.range is 720, the range of the angle is from -45 degree to 45 degree.'''
    regions_ = {
        'bright': min(min(msg.ranges[0:150]), 10),
        'right': min(min(msg.ranges[150:290]), 10),
        'front': min(min(msg.ranges[290:430]), 10), # 52.5 derece önünü görür
        'left': min(min(msg.ranges[430:570]), 10),
        'bleft': min(min(msg.ranges[570:720]), 10),
        'onfront': min(min(msg.ranges[120:600]), 10),
        'onleft': min(min(msg.ranges[0:120]), 10),
        'onright': min(min(msg.ranges[600:720]), 10),
        'onfull': min(min(msg.ranges[0:720]), 10)
    }


# def array_callback(data):
#     global size
#     size = data.data
#     print ("RRT ile gelen point sayisi : " + str(size))

def main_fonk(data, a):
    global x, y, paylas
    global cur_pos, q_goal, angular_speed, linear_speed, linear_unit, angular_unit
    global cur_action, next_action_time
    
    linear_speed = 0.5
    angular_speed = 0.5
    linear_unit = 0.5
    rate = rospy.Rate(20)
    angular_unit = math.pi / 6.

    # Initialize points
    cur_pos.x = x
    cur_pos.y = y
    start_position = cur_pos
    q_goal.z = 0.0

    # Bayraklar
    cur_action = 'gf'  # 'tl': sola dön; 'tr': sağa dön; 'fw': düz ilerle

    # Set parameters for actions
    next_action_time = rospy.Time.now()  # zaman hareketi başlıyor
    safe_distance = 2.0  # bir engele yakalaşılacak min nokta
    goal_tolarence = 1.5  # hedefe olan tolerans
    
    if a == True:
        print("\nAraç hedefe yöneliyor!")
        i = 0
        for p in data:
            q_goal.x = p[0]
            q_goal.y = p[1]
            if paylas == False:
                break
            # eğer hedefe uzaksam ve ros kapanmadı ise döngüye başla (kontrol döngüsü)
            while get_distance_between(cur_pos, q_goal) > goal_tolarence and not rospy.is_shutdown():
                global regions_
                get_goal_direction() # yön bul

                ackerman = AckermannDrive()
                ackerman.steering_angle_velocity = 0.0
                ackerman.acceleration = 0.5
                ackerman.jerk = 0.0
                if cur_action == 'tr':
                    ackerman.speed = linear_speed
                    ackerman.steering_angle = -angular_speed
                elif cur_action == 'tl':
                    ackerman.speed = linear_speed
                    ackerman.steering_angle = angular_speed
                elif cur_action == 'gf':
                    ackerman.steering_angle = 0.0
                    ackerman.speed = linear_speed
                elif cur_action == 'lgf':
                    ackerman.steering_angle = 0.0
                    ackerman.steering_angle = angular_speed + 0.25 #geriye dönerken yönünü daha fazla çevirir
                    ackerman.speed = -linear_speed
                elif cur_action == 'rgf':
                    ackerman.steering_angle = 0.0
                    ackerman.steering_angle = -angular_speed - 0.25 #geriye dönerken yönünü daha fazla çevirir
                    ackerman.speed = -linear_speed
                else:
                    print("[Hata Durumu]")
                    break

                cmd_vel_pub.publish(ackerman)
                rate.sleep()

                # pozisyon güncellendi
                cur_pos.x = x
                cur_pos.y = y

                # hedefe ulaşmak kontrol ediliyor
                if get_distance_between(cur_pos, q_goal) <= goal_tolarence:
                    i = i +1
                    print i, ". hedefe ulaşıldı!!!"
                    break
                
    elif paylas == False:
        a = False
        print("Araç yeni hedef bekliyor!")
        cur_action = 'stop'
        ackerman = AckermannDrive()
        ackerman.steering_angle_velocity = 0.0
        ackerman.acceleration = 0.0
        ackerman.jerk = 0.0
        if cur_action == 'stop':
            ackerman.speed = 0.0
            ackerman.steering_angle = 0.0
            cmd_vel_pub.publish(ackerman)

def get_goal_direction():
    global rotation
    global regions_
    global cur_pos, q_goal, angular_speed, linear_speed, linear_unit, angular_unit
    global cur_action, next_action_time

    # robotu hedefe yöneltir.
    robot_r = math.atan2(q_goal.y - cur_pos.y, q_goal.x - cur_pos.x)
    yaw_err = robot_r - rotation
    #print "hedefe dön"
    if regions_['onfull'] >= 1.0:
        if yaw_err < -0.1:
            if (2*math.pi*(-1) < yaw_err < math.pi*(-1)):
                cur_action = 'tl'
                next_action_time = rospy.Time.now() + rospy.Duration(angular_unit / angular_speed)
            else:
                cur_action = 'tr'
                next_action_time = rospy.Time.now() + rospy.Duration(angular_unit / angular_speed)
        elif yaw_err > 0.1:
            if (2*math.pi < yaw_err < math.pi):
                cur_action = 'tr'
                next_action_time = rospy.Time.now() + rospy.Duration(angular_unit / angular_speed)
            else :
                cur_action = 'tl'
                next_action_time = rospy.Time.now() + rospy.Duration(angular_unit / angular_speed)
        else:
            #print "yön doğru - > ileri"
            cur_action = 'gf'
            next_action_time = rospy.Time.now() + rospy.Duration(linear_unit / linear_speed)
        next_action_time = rospy.Time.now() + rospy.Duration(abs(yaw_err) / angular_speed)
    else:          
        if regions_['right'] < regions_['left']: # sol taraf boş ise
            cur_action = 'rgf'
            next_action_time = rospy.Time.now() + rospy.Duration(linear_unit / linear_speed)
        else:
            cur_action = 'lfg'
            next_action_time = rospy.Time.now() + rospy.Duration(linear_unit / linear_speed)

        # cur_action = 'rgf'
        # next_action_time = rospy.Time.now() + rospy.Duration(linear_unit / linear_speed)


def rrt_callback(data):
    global x_cordinate, y_cordinate
    x_cordinate.append(data.data[0])
    y_cordinate.append(data.data[1])

    if x_cordinate[-1] == 999.0: # eğer verinin tamamı geldi ise 
        #print ("data akışı tamamlandı")
        print ("Gelen dizi boyutu : " + str(len(x_cordinate) - 1 )) # -1 değeri konuldu çünkü 999.0 değeri mevcut 

        path_plan = []
        for i in range(len(x_cordinate)): # multi array oluştur
            tuple = (x_cordinate[i], y_cordinate[i])
            path_plan.append(tuple)

        path_plan.pop() # en sonda bulunan 999.0 çıksın 
        x_cordinate = []
        y_cordinate = []

        main_fonk(path_plan, True) # hedef verildi

def gps_callback( data):
    longitude = data.longitude
    latitude = data.latitude
    get_local_coord(latitude, longitude)

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

def callbackimu(msg):
    global roll, pitch, yaw, rotation
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    rotation = yaw

def get_distance_between(p1, p2): #iki nokta arasi uzaklık hesaplar
    return math.sqrt(math.pow((p1.x - p2.x), 2) + math.pow((p1.y - p2.y), 2))
     
# düğüm sonlanınca robotu durdurur.
def shutdown():
    rospy.loginfo("Stopping the robot...")
    rospy.sleep()

if __name__ == '__main__':
    rospy.init_node('ackermann_control', anonymous=False)
    cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
    scan_sub = rospy.Subscriber('laser/scan', LaserScan, clbk_laser)
    imu_sub = rospy.Subscriber('/imu', Imu, callbackimu)
    gps_sub = rospy.Subscriber("gps/fix", NavSatFix, gps_callback, queue_size=1)
    rrt_sub = rospy.Subscriber("rrt", Float32MultiArray, rrt_callback)
    bol_1 = rospy.Subscriber('start/stop', Bool, bool_callback)
    rospy.on_shutdown(shutdown)
    rospy.spin()






