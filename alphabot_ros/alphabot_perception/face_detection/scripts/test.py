#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
from cv_bridge import CvBridge

SIZE = 2
(im_width,im_height) = (308,410)

class faceDetection:
    def __init__(self):
        # Load the cascade
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    
        rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback,queue_size=1)
        self.bridge = CvBridge()

    def callback(self,ros_data):
        img = self.bridge.compressed_imgmsg_to_cv2(ros_data,desired_encoding="bgr8")

        # Flip frame
        img = cv2.flip(img,1)
        
        # Convert into grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Scale down for speed
        mini = cv2.resize(gray, (int(gray.shape[1] /SIZE), int(gray.shape[0]/SIZE)))
        # Detect faces
        faces = self.face_cascade.detectMultiScale(gray)
        if len(faces) != 0:
            # Draw rectangle around the faces
            face_i = faces[0]
            (x,y,w,h) = [v*SIZE for v in face_i]
            face = gray[y:y+h,x:x+w]
            #face_resize = cv2.resize(face,(im_width,im_height))
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 3)
            #print(f"{x} {y} {w} {h}")
            # Display the output
        cv2.imshow('img', img)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("face_detection_node",anonymous=True)
    fd = faceDetection()
    rospy.spin()
    cv2.destroyAllWindows()
    
