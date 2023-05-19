#!/usr/bin/env python3

PAN = 0
TILT = 1
TIME_DELAY = 0.05

from time import sleep
from PCA9685 import PCA9685
from sensor_msgs.msg import JointState
import rospy

class pantilt(PCA9685):
    def __init__(self):
        super().__init__()
        self.setPWMFreq(50)
        self.pan_angle = 90
        self.tilt_angle = 90
        rospy.Subscriber("/alphabot/servo_cmd",JointState,self.callback,queue_size=1)
        self.setRotationAngle(0,90) #bottom
        self.setRotationAngle(1,90) #top

    def callback(self,msg):
        pan_setpoint = int(msg.position[0])
        tilt_setpoint = int(msg.position[1])
        if pan_setpoint > self.pan_angle:
            for i in range(self.pan_angle,pan_setpoint,1):
                self.setRotationAngle(PAN,i)
                sleep(TIME_DELAY) 
        else:
            for i in range(self.pan_angle,pan_setpoint,-1):
                    self.setRotationAngle(PAN,i)
                    sleep(TIME_DELAY) 
        if tilt_setpoint > self.tilt_angle:
            for i in range(self.tilt_angle,int(tilt_setpoint),1):
                self.setRotationAngle(TILT,i)
                sleep(TIME_DELAY)
        else:
            for i in range(self.tilt_angle,int(tilt_setpoint),-1):
                self.setRotationAngle(TILT,i)
                sleep(TIME_DELAY)
        self.pan_angle = pan_setpoint
        self.tilt_angle = tilt_setpoint
    
    def shutdown(self):
        if self.pan_angle > 90:
            for i in range(self.pan_angle,90,-1):
                self.setRotationAngle(PAN,i)
                sleep(TIME_DELAY) 
        else:
            for i in range(self.pan_angle,90,1):
                self.setRotationAngle(PAN,i)
                sleep(TIME_DELAY) 
        if self.tilt_angle > 90:
            for i in range(self.tilt_angle,90,-1):
                self.setRotationAngle(TILT,i)
                sleep(TIME_DELAY)
        else:
            for i in range(self.tilt_angle,90,1):
                self.setRotationAngle(TILT,i)
                sleep(TIME_DELAY)
        self.exit_PCA9685()

if __name__ == "__main__":
    rospy.init_node("pantilt_node",anonymous=True)
    pt = pantilt()
    rospy.on_shutdown(pt.shutdown)
    rospy.spin()
