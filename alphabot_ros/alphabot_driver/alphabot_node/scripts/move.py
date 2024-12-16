#!/usr/bin/env python3
"""
Move robot based on cmd_vel messages
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from AlphaBot2 import AlphaBot2 

class alphabot(AlphaBot2):
    def __init__(self):
        super().__init__()
        self.subscriber = rospy.Subscriber("/cmd_vel",Twist,self.command_callback)        
    
    def command_callback(self,msg):
        print(msg)
        pass

if __name__ == "__main__":
    rospy.init_node("alphabot_node",anonymous=True)
    alphabot()
    rospy.spin()
    pass
