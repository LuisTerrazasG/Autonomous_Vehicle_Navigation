#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from math import pi
from std_msgs.msg import String
import numpy as np

class square:
        def __init__(self):

            self.wr = 0.00
            self.wl = 0.00
            self.r = 0.05
            self.l = 0.19

            self.P_X = 0.00
            self.P_Y = 0.00
            self.angle = 0.00


            rospy.Subscriber('/wr', Float32, self.wr_callback)
            rospy.Subscriber('/wl', Float32, self.wl_callback)

            self.w_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
            self.j_pub = rospy.Publisher('/odometry', Pose2D,queue_size=10)

            rospy.init_node("Square")
            self.rate = rospy.Rate(10)

            rospy.on_shutdown(self.stop)


        def wr_callback(self,msg):
            self.wr = msg.data

        def wl_callback(self,msg):
            self.wl = msg.data

        def run(self):

            distance = 0.0
            angle = 0.0
            current_time = rospy.get_time()
            last_time = rospy.get_time()
            state  = 0
            count = 1
            no_sides = 4


            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0

            odometry = Pose2D()
            odometry.theta = self.angle
            odometry.x = self.P_X
            odometry.y = self.P_Y

            while not rospy.is_shutdown():
                current_time = rospy.get_time()
                dt = current_time - last_time
                last_time = current_time
                

                msg.linear.x = 0.1
                msg.angular.z = 0.1

                print(self.wl)
                print(self.wr)


                self.angle = self.angle + self.r*((self.wr-self.wl)/self.l)* dt
                self.P_X = self.P_X + self.r*((self.wr+self.wl)/2)* dt*np.cos(self.angle)
                self.P_Y = self.P_Y + self.r*((self.wr-self.wl)/2)* dt*np.sin(self.angle)

                odometry.theta = self.angle
                odometry.x = self.P_X
                odometry.y = self.P_Y

                self.w_pub.publish(msg)
                self.j_pub.publish(odometry)
                self.rate.sleep()


        def stop(self):
            print("Stopping")
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.w_pub.publish(msg)

if __name__ == "__main__":

    sq = square()
    try:
        sq.run()
    except rospy.ROSInterruptException:
        None