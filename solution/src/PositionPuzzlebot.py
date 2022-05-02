#!/usr/bin/env python

from turtle import pos
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import pi
import numpy as np

wl = 0.00
wr = 0.00
r = 0.05
l = 0.19

P_X = 0.00
P_Y = 0.00
angle = 0.00

def wr_callback(msg):
    wr = msg.data

def wl_callback(msg):
    wl = msg.data

def stop():
    print("Stopping")
    msg = Twist()
    msg.linear.x = 0


rospy.Subscriber('/wr', Float32, wr_callback)
rospy.Subscriber('/wl', Float32, wl_callback)

j_pub = rospy.Publisher('/chatter', String,queue_size=10)
w_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)

rospy.init_node("Position_Puzzlebot")
rate = rospy.Rate(10)

rospy.on_shutdown(stop)


current_time = rospy.get_time()
last_time = rospy.get_time()


msg = Twist()
msg.linear.x = 0
msg.linear.y = 0
msg.linear.z = 0
msg.angular.x = 0
msg.angular.y = 0
msg.angular.z = 0






def stop():
    print("Stopping")
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    w_pub.publish(msg)



while not rospy.is_shutdown():

    current_time = rospy.get_time()
    dt = current_time - last_time
    last_time = current_time

    msg.linear.x = 0.1
    msg.angular.z = 0.1

    print(wl)
    print(wr)


    angle = angle + r*((wr-wl)/l)* dt
    P_X = P_X + r*((wr+wl)/2)* dt*np.cos(angle)
    P_Y = P_Y + r*((wr-wl)/2)* dt*np.sin(angle)

    resultados = "Angulo:" + str(angle)+ "PosX:" +str(P_X) +"PosY:"+ str(P_Y)

    w_pub.publish(msg)
    j_pub.publish(resultados)
    rate.sleep()






                