#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
from cv_bridge import CvBridge

class faceDetection:
    def __init__(self):
        # Load the cascade
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    
        rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)
        self.bridge = CvBridge()

    def callback(self,ros_data):
        img = self.bridge.compressed_imgmsg_to_cv2(ros_data,desired_encoding="bgr8")
        # Convert into grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect faces
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        # Draw rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
            print(f"{x} {y} {w} {h}")
        # Display the output
        cv2.imshow('img', img)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("face_detection_node",anonymous=True)
    fd = faceDetection()
    rospy.spin()
    cv2.destroyAllWindows()
    
