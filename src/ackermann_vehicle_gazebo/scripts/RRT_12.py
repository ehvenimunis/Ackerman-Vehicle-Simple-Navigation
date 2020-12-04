#!/usr/bin/env python
# -*-coding: utf-8 -*-

"""
# Bilgi : Başlangıç noktası ile hedef arasında RRT algoritması kullanılarak noktalar üreterek paylaşır. Engel değerlendirmesi yapar!! Object detection dosyası ile senkron haberleşir. 
# Hazırlayan : Muhammed Salih Aydoğan
# Numara : 161201039
# İstanbul Medeniyet Üniversitesi | Elektrik Elektronik Mühendisliği
# Tarih : 30/03/2019
"""

import rospy, math, sys, random
from math import *
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Float32MultiArray, Int16, Bool

class Node(object):
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent

XDIM = 30
YDIM = 30
delta = 0.5 # yeni düğüm için yakınlık
GOAL_RADIUS = 0.5
engel_r = 1.5 # engelin boyutu 
NUMNODES = 15000
Goal_X = 25
Goal_Y = 25

count = 0
x_cordinate = []
y_cordinate = []
data_complate = Bool()

def dist(p1, p2):  # distance between two points
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def point_circle_collision(p1, p2, radius):
    distance = dist(p1, p2)
    if (distance <= radius):
        return True
    return False

def step_from_to(p1, p2):  # eğer p2 noktası p1 noktasına yakınsa onu döndürür
    if dist(p1, p2) < delta:  # delta değeri 10
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + delta * cos(theta), p1[1] + delta * sin(theta)

def get_random_clear():
    while True:
        p = random.random() * XDIM, random.random() * YDIM  # random.random() şunu yapmakatadır : Random float x, 0.0 <= x < 1.0
        return p

def reset():
    global count
    count = 0

def main(x, y):
    global count
    a = []

    initPoseSet = False  # başlangıç noktası bulunmadı
    initialPoint = Node(None, None)  # başlangıç noktası
    goalPoseSet = False  # hedef noktası bulunmadı
    goalPoint = Node(None, None)  # hedef noktası
    #currentState = 'init'  # işe başla

    nodes = []
    print("başlangıç belirlendi")

    start = (x[0], y[0]) # başlangıç
    print(str(start))
    initialPoint = Node(start, None)
    nodes.append(initialPoint)  # Start in the center
    initPoseSet = True  # başlangıç bulundu

    goal = (Goal_X, Goal_Y) # hedef
    goalPoint = Node(goal, None)
    goalPoseSet = True  # hedef nokta bulundu

    daire_cordinate = []
    for i in range(len(x)):
        tuple = (x[i], y[i])
        daire_cordinate.append(tuple)

    daire_cordinate.pop(0)
    daire_cordinate.pop()

    # if (len(daire_cordinate == 0)): # hiçbir engel yoksa hayali bir engel koy 
    #     tuple = (10.0, 10.0) # hayali bir engel ekliyoruz
    #     daire_cordinate.append(tuple)

    #print(str(daire_cordinate))

    currentState = 'buildTree'  # ağaç oluştur
    reset()  # engel oluşturutldu

    rate = rospy.Rate(0.1) # ROS Rate at 0.05 Hz

    while True:
        if currentState == 'init':
            print('goal point not yet set')  # hedef noktası henüz belirlenmedi
        elif currentState == 'goalFound':  # hedef bulundu
            print "Goal Reached"
            currNode = goalNode.parent
            i = 0
            while currNode.parent != None:
                x = currNode.point[0]
                y = currNode.point[1]

                x_y_data = [x, y]
                a.append(x_y_data)

                i+=1
                currNode = currNode.parent
                
                if currNode.parent == None: # yani dizinin sonu geldi ise
                    a = a[::-1]
                    tuple = (999.0, 999.0) # bir işaret olarak eleman ekledik.
                    a.append(tuple)
                    print[list(a)]
                    array_publish(a[::], i+1) # bulunduğu konum hariç hepsini tersten yazdırarak publish eder (topic name : rrt)

            bol_1.publish(True)
            rate.sleep() # 5 saniyede bir defa döngüye girer 
            bol.publish(True)
            break
            
        elif currentState == 'buildTree':  # ağaç oluşturma kısmı
            count = count + 1
            if count < NUMNODES:  # eğer sayaç 5000 den küçükse ağaç oluşturmaya devam et
                foundNext = False  # gelecek düğümü bulmak için bayrak
                while foundNext == False:
                    rand = get_random_clear()  # yeni düğüm eklenir
                    parentNode = nodes[0]  # start noktası ilk eleman
                    for p in nodes:
                        if dist(p.point, rand) <= dist(parentNode.point, rand):
                            newPoint = step_from_to(p.point, rand)  # iki noktanın yakınlığına bakar
                            if icinde_mi(newPoint[0], newPoint[1], daire_cordinate, engel_r) == False:
                                parentNode = p
                                foundNext = True  # gelecek düğüm bulundu

                newnode = step_from_to(parentNode.point, rand)  # iki noktanın yakınlığına bakar
                nodes.append(Node(newnode, parentNode))

                if point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS):  # eğer yeni düğüm hedefe ulaştı ise
                    goalNode = nodes[-1]
                    currentState = 'goalFound'
                    

            else:  # sayaç değeri yeterli gelmedi ise bulunamadı mesajı yayımla
                print("Ran out of nodes... :(")
                return;

        # handle events
        else:
            currentState = 'init'
            initPoseSet = False
            goalPoseSet = False  # hedef nokta iste
            reset()  # count sıfır olur

# icinde_mi(x, y, cember_merkez_x, cember_merkez_y, yaricap)
def icinde_mi(a, b, c, e):
    for p in c:
        f = pow(a-p[0], 2) + pow(b-p[1], 2)
        mesafe = pow(f, 0.5)
        if(mesafe <= e):
            return True
        else:
            pass
    
    return False

def shutdown():
    # düğüm sonlanınca robotu durdurur.
    rospy.loginfo("           Stopping the robot...")
    rospy.sleep()

def array_publish(data, count):
    i = 0
    for i in range(count):
        array = Float32MultiArray(data=data[i])
        cmd.publish(array)

def location_array_callback(data):
    global x_cordinate
    global y_cordinate
    global data_complate
    x_cordinate.append(data.data[0]) 
    y_cordinate.append(data.data[1])
    msg = Bool()
    msg.data = False
    if(x_cordinate[-1] == float(999.0) and y_cordinate[-1] == float(999.0)): # eğer gelen verinin sonu geldi ise 
        data_complate = True 
        if data_complate == True: # eğer data geldi ise 
            bol.publish(msg) # false
            bol_1.publish(msg)
            print "\ndata geldi"
            main(x_cordinate, y_cordinate)
            x_cordinate = []
            y_cordinate = []
    else:
        data_complate = False


if __name__ == '__main__':
    bol = rospy.Publisher('basla', Bool, queue_size=10) # verinin alındığı ile alakalı uyarı mesajı yayımlar
    bol_1 = rospy.Publisher('basla_1', Bool, queue_size=10) # verinin alındığı ile alakalı uyarı mesajı yayımlar
    rospy.init_node('RRT', anonymous=False)
    #pub = rospy.Publisher('array_size', Int16, queue_size=10)
    cmd = rospy.Publisher('rrt', Float32MultiArray, queue_size=10)
    engel = rospy.Subscriber('location', Float32MultiArray, location_array_callback) # bu topi engellerin konumu dizi olarak alır. ilk elemanı arabanın konumu 
    rospy.spin()