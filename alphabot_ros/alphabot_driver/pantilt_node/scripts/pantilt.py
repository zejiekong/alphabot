#!/usr/bin/env python3

PAN = 0 # 20 (right) - 160 (left)
TILT = 1 # 20 (up) - 160 (down)
TIME_DELAY = 0.05

from time import sleep
from PCA9685 import PCA9685
from sensor_msgs.msg import JointState
import rospy
import threading

class pantilt(PCA9685):
    def __init__(self):
        super().__init__()
        self.setPWMFreq(50)
        self.servo_angle = [90,90]
        rospy.Subscriber("/alphabot/servo_cmd",JointState,self.callback,queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.setRotationAngle(0,90) #bottom
        self.setRotationAngle(1,90) #top
        rospy.spin()


    def moveServo(self,servo_id,servo_goal):
        while(self.servo_angle[servo_id] != servo_goal):
            next_angle = (servo_goal + self.servo_angle[servo_id])//2
            print(f"{servo_id}: {next_angle}")
            if(abs(servo_goal - next_angle) <= 1):
                next_angle = servo_goal
            if(next_angle < 20):
                next_angle = 20
            elif(next_angle > 160):
                next_angle = 160
            self.setRotationAngle(servo_id,next_angle)
            print(f"move {servo_id}")
            self.servo_angle[servo_id] = next_angle
            sleep(TIME_DELAY)
        

    def callback(self,msg):
        pan_setpoint = int(msg.position[0])
        tilt_setpoint = int(msg.position[1])
        pan_thread = threading.Thread(target=self.moveServo,args=(PAN,pan_setpoint,)) 
        tilt_thread = threading.Thread(target=self.moveServo,args=(TILT,tilt_setpoint,))
        pan_thread.start()
        tilt_thread.start()
        pan_thread.join()
        tilt_thread.join()
            
    def shutdown(self):
        pan_thread = threading.Thread(target=self.moveServo,args=(PAN,90,)) 
        tilt_thread = threading.Thread(target=self.moveServo,args=(TILT,90,))
        pan_thread.start()
        tilt_thread.start()
        pan_thread.join()
        tilt_thread.join()
        self.exit_PCA9685()

if __name__ == "__main__":
    rospy.init_node("pantilt_node",anonymous=True)
    pt = pantilt()
    rospy.spin()
