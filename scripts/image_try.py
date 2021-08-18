#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import redis
import numpy as np
import base64
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from rclpy.node import Node
import threading
from sensor_msgs.msg import CompressedImage
import message_filters
import matplotlib.pyplot as plt

'''bridge = CvBridge()

def image_callback(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        a = 1
    else:
        # print(1)
        time = msg.header.stamp
        cv2.imwrite('image.jpg', cv2_img)
        rospy.sleep(0.01)

# rospy.init_node('image_listener')
# image_topic = "/reachy/camera/image_raw"
# #rospy.Subscriber(image_topic, Image, image_callback)

# redis_connection = redis.Redis(host='192.168.118.16', port=6379, db=0, password='DTL@b2021')
# pubsub = redis_connection.pubsub()

def image_publisher():
    cap = cv2.VideoCapture('image.jpg')

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Frame default resolution: (" + str(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")")

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            if ret:
                print(frame.shape)
                # print(1)
                retval, buffer = cv2.imencode('.jpg', frame, encode_param)
                jpg_as_text = base64.b64encode(buffer)
                redis_connection.publish('video', jpg_as_text)

    except (KeyboardInterrupt, SystemExit):
        # When everything done, release the capture
        cap.release()'''

rospy.init_node('image_listener')

class ImageSender():
    def __init__(self):
        super(ImageSender, self).__init__()
        self.i = 0

        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

        self.image_bridge = CvBridge()

        self.redis_connection = redis.Redis(host='192.168.118.16', port=6379, db=0, password='DTL@b2021')
        self.pubsub = self.redis_connection.pubsub()

        self.left_image_subscriber = message_filters.Subscriber('/reachy/camera_l/image_raw/compressed', CompressedImage)
        self.right_image_subscriber = message_filters.Subscriber('/reachy/camera_r/image_raw/compressed', CompressedImage)
        #print (self.left_image_subscriber)
        #self.left_image_subscriber
        #self.right_image_subscriber

        #self.time_sinc_subscriber = message_filters.ApproximateTimeSynchronizer([self.left_image_subscriber, self.right_image_subscriber], 30, 0.4)
        self.time_sinc_subscriber = message_filters.TimeSynchronizer([self.left_image_subscriber, self.right_image_subscriber], 10)
        self.time_sinc_subscriber.registerCallback(self.deal_callback)

    def deal_callback(self, left_image, right_image):
        #print('pub')
        #print(left_image, right_image)
        l_image = np.asarray(bytearray(left_image.data), dtype="uint8")
        l_image = cv2.imdecode(l_image, cv2.IMREAD_COLOR)
    
        # cv2.imshow(l_)

        r_image = np.asarray(bytearray(right_image.data), dtype="uint8")
        r_image = cv2.imdecode(r_image, cv2.IMREAD_COLOR)

        res_image = np.concatenate((r_image, l_image), axis=1)
        #np.zeros((max(r_image.height, l_image.height), r_image.width + l_image.width))

        retval, buffer = cv2.imencode('.jpg', res_image, self.encode_param)
        jpg_as_text = base64.b64encode(buffer)
        cv2.imwrite('image.jpg', res_image)
        self.redis_connection.publish('video', jpg_as_text)

while True:
    try:
        image = ImageSender()
        rospy.spin()
    except (KeyboardInterrupt, SystemExit):
        exit

