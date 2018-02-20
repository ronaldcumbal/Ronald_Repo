#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class pepperController():
    def __init__(self):
	      # Init Nodes
        self.sub = rospy.Subscriber("/joy", Joy, self.joyCallback)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_speech = rospy.Publisher('/speech', String, queue_size=10)
        self.pub_searching = rospy.Publisher('/searching_people', String, queue_size=10)
        rospy.init_node('pepper_joystick', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.vel = Twist()
        self.controlActive = True;
        self.maxVelX = 0.3
        self.minVelX = -0.3
        self.maxVelZ = 0.3
        self.minVelZ = -0.3

    def joyCallback(self, data):
        self.driveBody(data.axes)
        self.readButtonDown(data.buttons)
                    
    def driveBody(self,axes):
        if self.controlActive == True:
            self.vel.linear.x = round(((self.maxVelX-(self.minVelX))*(axes[1]-(-1)))/(1-(-1))+(self.minVelX),1)
            self.vel.angular.z = round(((self.maxVelZ-(self.minVelZ))*(axes[3]-(-1)))/(1-(-1))+(self.minVelZ),1)
            self.pub_vel.publish(self.vel)

    def stopBody(self):
	      self.vel.linear.x =0.0
	      self.vel.linear.y =0.0
	      self.vel.linear.z =0.0
	      self.vel.angular.z = 0.0
	      self.pub_vel.publish(self.vel)

    def readButtonDown(self, buttons):
        if buttons[0] == 1: # A
            pass
        if buttons[1] == 1: # B
            pass
        if buttons[2] == 1: # X
            pass
        if buttons[3] == 1: # Y
            pass
        if buttons[4] == 1: # LB
            pass
        if buttons[5] == 1: # RB
            pass
        if buttons[6] == 1: # BACK
            self.quitController()
        if buttons[7] == 1: # START
            self.initRobot()
        if buttons[8] == 1: # CENTER
            pass
        if buttons[9] == 1: # Left Joystick
            pass 
        if buttons[10] == 1: # Right Joystick
            pass 
        if buttons[11] == 1: # < PAD
            pass 
        if buttons[11] == 1: # > PAD
            pass 
        if buttons[11] == 1: # ^ PAD
            pass 
        if buttons[11] == 1: # v PAD
            pass 

    def initRobot(self):
#        self.pub_speech.publish("Controller Enabled")
        self.controlActive = True

    def quitController(self):
#        self.pub_speech.publish("Controller Disabled")
        self.controlActive = False
        self.stopBody()

    def runningLoop(self):
        while not rospy.is_shutdown():
            print "Working"
            self.rate.sleep()        

if __name__ == "__main__":
    control = pepperController()
    try:
        control.runningLoop()
    except KeyboardInterrupt:
        print "Interrupted by user, shutting down"
        rospy.loginfo("Shutting down")
