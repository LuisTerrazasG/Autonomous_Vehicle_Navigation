#!/usr/bin/env python

import cv2 as cv
import numpy as np
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

cap = cv.VideoCapture(0)
count_5_list = []
count_5 = False

wr = 0.00
wl = 0.00
velocidad = 0.3
frame = None

def nothing(x):
    pass

# Trackbar
cv.namedWindow("frame")
cv.createTrackbar("Green", "frame", 0, 255, nothing)
cv.createTrackbar("Yellow", "frame", 0, 255, nothing)
cv.createTrackbar("Red", "frame", 0, 255, nothing)

img_hsv = np.zeros((250, 500, 3), np.uint8)

# Callbacks para cambiar valores de wheel right and left
def wr_callback( msg):
    wr = msg.data


def wl_callback( msg):
    wl = msg.data


def stop():
    print("Stopping")
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    cmd_pub.publish(msg)

def img_callback(msg):
	global frame
	bridge = cv_bridge.CvBridge()
	frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

rospy.Subscriber('/wr', Float32, wr_callback)
rospy.Subscriber('/wl', Float32, wl_callback)
rospy.Subscriber("video_source/raw",Image,img_callback)

cmd_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
img_pub = rospy.Publisher("Image_out",Image,queue_size=10)

rospy.init_node("Image_Temp")
rate = rospy.Rate(10)
rospy.on_shutdown(stop)

msg = Twist()
msg.linear.x = 0.2
msg.linear.y = 0
msg.linear.z = 0
msg.angular.x = 0
msg.angular.y = 0
msg.angular.z = 0

while True:
    #Captura de frame del video
    #_, frame = cap.read()
    #frame = cv.resize(frame,(400,400))

    #Creacion de imagenes en grises
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    imgBlur = cv.GaussianBlur(gray,(3,3),0)

    resized = cv.resize(imgBlur, (400, 400), interpolation=cv.INTER_AREA)

    cv.imshow("Image", resized)

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    hsv_blur = cv.GaussianBlur(hsv,(3,3),0)

    #verde
    mask1 = cv.inRange(hsv, (30, 0, 0), (75, 255, 255))

    #Amarillo
    mask2 = cv.inRange(hsv, (15, 0, 0), (36, 255, 255))

    # Rojo
    mask3 = cv.inRange(hsv, (170,0,0), (200, 255, 255))
    ## final mask and masked
    mask_tmp = cv.bitwise_or(mask1, mask3)
    mask = cv.bitwise_or(mask_tmp,mask2)

    target = cv.bitwise_and(frame, frame, mask=mask)

    gray_target = cv.cvtColor(target, cv.COLOR_BGR2GRAY)

    ret, thresh1 = cv.threshold(gray_target, 10, 255, cv.THRESH_BINARY_INV)

    #cv.imshow("target.png", target)
    #cv.imshow("Img thresh", thresh1)

    kernel = np.ones((3, 3), np.uint8)

    dilatation_dst = cv.dilate(thresh1, kernel, iterations=3)
    #cv.imshow("dilatation", dilatation_dst)

    # Setup SimpleBlobDetector parameters.
    params = cv.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 400

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.65

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    detector = cv.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(dilatation_dst)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),
                                          cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    frame_hsv = cv.cvtColor(im_with_keypoints,cv.COLOR_BGR2HSV)

    num_keypoint = len(keypoints)

    for i in range(num_keypoint):
        if keypoints[i].pt[0] != 0:
            x_blob = keypoints[i].pt[0]
            y_blob = keypoints[i].pt[1]
            pixel_center = frame_hsv[int(x_blob), int(y_blob)]
            cv.circle(im_with_keypoints,(int(x_blob),int(y_blob)), 5 ,(255,0,0),3)
        hue_value = pixel_center[0]

        #print(pixel_center[0])
        if hue_value < cv.getTrackbarPos("Green", "frame"):
            color = "GREEN"
            if count_5 == True:
                print(color)
                msg.linear.x = 0.2
                cmd_pub.publish(msg)

        elif hue_value < cv.getTrackbarPos("Yellow", "frame"):
            color = "YELLOW"
            if count_5 == True:
                print(color)
                msg.linear.x = velocidad/3
                cmd_pub.publish(msg)

        elif hue_value < cv.getTrackbarPos("Red", "frame"):
            color = "RED"
            if count_5 == True:
                print(color)
                msg.linear.x = 0.0
                cmd_pub.publish(msg)
        else:
            color = "NONE"
        count_5_list.append(color)
        if len(count_5_list) == 25:

            for i in count_5_list:
                for j in count_5_list:
                    if i != j:
                        count_5_list = []
                        count_5 = True
                    else:
                        count_5 = False
            count_5_list = []

        else:
            pass
    global frame
    bridge = cv_bridge.CvBridge()
    img_back = bridge.cv2_to_imgmsg(frame, "bgr8")
    img_pub.publish(img_back)
    rate.sleep()

    # Show keypoints
    cv.imshow("Keypoints", im_with_keypoints)



    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
