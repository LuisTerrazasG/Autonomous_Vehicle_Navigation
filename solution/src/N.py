#!/usr/bin/env python
import cv2
import sys
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


video_caputre = cv2.VideoCapture(0)
bridge = CvBridge()
img = None

def end_callback():
    print("Shutting down")
    cv2.destroyAllWindows()

def img_callback(msg):
    global img
    img = msg.src
    

    
rospy.init_node("Webcam_node_publisher")

rospy.Subscriber("/video_source/raw", Image,img_callback)
image_pub = rospy.Publisher("/video_source/raw", Image, queue_size=10)
rospy.on_shutdown(end_callback)
rate = rospy.Rate(10)

def main(args):
    try:
        if not video_caputre.isOpened():
            print("Unable to load camera")
            return
        print("Publishing webcam image)")
        while not rospy.is_shutdown():
            image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
            rate.sleep()
    except rospy.ROSInterruptException:
            rospy.logerr("ROS Interruption Exception")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        main(sys.argv)
    else:
        print("Interruption")