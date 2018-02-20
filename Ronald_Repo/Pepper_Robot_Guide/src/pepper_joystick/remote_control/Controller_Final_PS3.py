#!/usr/bin/env python
from naoqi import ALProxy
import pygame
robotIP = "145.94.240.73"
PORT = 9559    
robotConnected = True

class xboxcontroller():
    def __init__(self):
        if robotConnected:        
            self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
            self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
            self.textToSpeachProxy = ALProxy("ALTextToSpeech", robotIP, PORT)
            self.ledsProxy = ALProxy("ALLeds", robotIP, PORT)
            self.basicAwarenessProxy = ALProxy("ALBasicAwareness", robotIP, PORT)
        # Initialize variables
        self.motionProxy.setExternalCollisionProtectionEnabled("All",False)  
        self.joysticks = []
        self.clock = pygame.time.Clock()
        self.keepPlaying = True
        self.xVel = self.yVel = self.tVel = 0.0 
        self.maxVel = 0.9
        self.minVel = -0.9  
        self.headPitch = 0.0  # Updated at initial head command
        self.headYaw = 0.0 # Updated at initial head command
        self.shoulderPitch = 1.5  # Updated at initial arm command
        self.shoulderRoll = 0.0
        self.elbowRoll = 0.5
        self.maxShoulderPitch = 1.6
        self.minShoulderPitch = -1.0
        self.maxShoulderRoll = 0.05
        self.minShoulderRoll = -1.5
        self.maxElbowRoll = -0.05
        self.minElbowRoll = -1.5
        self.block = False
        self.rightArm = True
        self.startControl = False

        # Check for controllers connected
        for i in range(0, pygame.joystick.get_count()):
            self.joysticks.append(pygame.joystick.Joystick(i))    # create a Joystick object
            self.joysticks[-1].init()                   # initialize them all (-1 means loop forever)
            print "Detected joystick '",self.joysticks[-1].get_name(),"'"

    def runningCode(self):
        print "Controller ready"
        while self.keepPlaying:
            self.clock.tick(60)
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.readStick(event.axis,event.value)
                if event.type == pygame.JOYBUTTONDOWN: 
                    self.readButtonDown(event.button)
            if robotConnected and self.startControl:
                self.sendArmPosition(self.shoulderPitch,self.shoulderRoll,self.elbowRoll)
                
    def initRobot(self):
        if robotConnected:
            self.startControl = True
#            self.stopMoving()
            self.basicAwarenessProxy.pauseAwareness()        
#            self.postureProxy.goToPosture("StandInit", 0.1) #Speed 0.0 and 1.0
#            self.motionProxy.setExternalCollisionProtectionEnabled("All",False)      
#            self.ledsProxy.fadeRGB('ChestLeds',0.0,1.0,1.0, 0.5) # Green
            self.motionProxy.setMoveArmsEnabled(True, False)
            print "Awareness off"
        
    def pauseRobot(self):
        if robotConnected:
            self.startControl = False
#            self.stopMoving()
            self.basicAwarenessProxy.resumeAwareness()
            self.basicAwarenessProxy.setEngagementMode("FullyEngaged")
            self.basicAwarenessProxy.setTrackingMode("Head")            
#            self.motionProxy.setExternalCollisionProtectionEnabled("All",True)
#            self.ledsProxy.fadeRGB('ChestLeds',1.0,1.0,1.0, 0.5)
            print "Awareness on" 
            
    def readStick(self,axis,value):
        if self.startControl == True:
            if axis in [1, 0]:
                self.driveBody(axis,value)
            if axis == 13 and self.block == False:
                self.driveForearms(axis,value)
            if axis in [2, 3] and self.block == False:
                self.driveArms(axis,value)

    def readPad(self,value):
        if self.startControl == True:
            self.driveHead(value)
    
    def readButtonDown(self, value):
	self.readPad(value)
	if value == 0:
	    self.textToSpeachProxy.say("Hello")
        if value == 15: # Square
            self.pauseRobot()
        if value == 13: # Circle
            self.stopMoving()
        if value == 12: # Triangle
            self.initRobot()            
        if value == 10: # L1
            self.rightArm = False
            self.motionProxy.setStiffnesses('RArm',0.0);
            self.motionProxy.setStiffnesses('LArm',1.0);
        if value == 11: # R1
            self.rightArm = True
            self.motionProxy.setStiffnesses('RArm',1.0);
            self.motionProxy.setStiffnesses('LArm',0.0);
        if value == 3: # Start
            self.quitController()
        if value == 2: # right joy stick
            if self.block == False:        
                self.block = True
                print "Block true"
                self.ledsProxy.fadeRGB('ChestLeds',1.0,1.0,1.0, 0.5)
            else:
                self.block = False                   
                print "Block false"
                self.ledsProxy.fadeRGB('ChestLeds',0.0,0.0,1.0, 0.5)        
    def driveHead(self,value):
        if self.startControl and robotConnected:        
            if value == 4:  #Up:
                self.motionProxy.changeAngles("HeadPitch", -0.2, 0.1)
            if value == 6:  #down:
                self.motionProxy.changeAngles("HeadPitch", 0.2, 0.1)
            if value == 7:  #left
                self.motionProxy.changeAngles("HeadYaw", 0.2, 0.1)
            if value == 5:  #right
                self.motionProxy.changeAngles("HeadYaw", -0.2, 0.1)
     
    def driveArms(self,axis,value):
        if axis == 3: # Theta axis
            self.shoulderPitch = self.limitRange(self.minShoulderPitch,self.maxShoulderPitch,-1,0,value)
        if self.rightArm == True:
            if axis == 2 and value > 0: # Theta axis
                self.shoulderRoll = -self.limitRange(self.minShoulderRoll,self.maxShoulderRoll,-1,0,value)
        else:
            if axis == 2 and value < 0: # Theta axis
                self.shoulderRoll = -self.limitRange(self.minShoulderRoll,self.maxShoulderRoll,-1,0,value)
        self.sendArmPosition(self.shoulderPitch,self.shoulderRoll,self.elbowRoll)
      
    def driveForearms(self,axis,value):
		if self.rightArm == True:
			self.elbowRoll = -self.limitRange(self.minElbowRoll,self.maxElbowRoll,1,-1,value)
		if self.rightArm == False:
			self.elbowRoll = self.limitRange(self.minElbowRoll,self.maxElbowRoll,1,-1,value)
		self.sendArmPosition(self.shoulderPitch,self.shoulderRoll,self.elbowRoll)

    def driveBody(self,axis,value):
        self.basicAwarenessProxy.pauseAwareness()
        self.headPitch = 0.0 
        self.headYaw = 0.0 
        if axis == 1: # X axis
            self.xVel = -self.limitRange(self.minVel,self.maxVel,-1,1,value)
        if axis == 0: # Theta axis
            self.tVel = -self.limitRange(self.minVel,self.maxVel,-1,1,value)
        self.sendVelocity(self.xVel,self.yVel,self.tVel)       
        self.sendArmPosition(self.shoulderPitch,self.shoulderRoll,self.elbowRoll)

    def blockArms(self):
        pass

    def limitRange(self, minOut, maxOut, minIn, maxIn, inputValue):
        outputValue = ((maxOut - minOut)*(inputValue-minIn))/(maxIn-minIn)+minOut        
        return round(outputValue,1)

    def stopMoving(self):
        self.xVel = self.yVel = self.tVel = 0.0
        if robotConnected:        
            self.sendVelocity(self.xVel,self.yVel,self.tVel)

    def sendVelocity(self,xVel,yVel,tVel):
        #print  "Vel:", self.xVel, self.yVel, self.tVel
        if robotConnected:        
            self.motionProxy.moveToward(xVel,yVel,tVel)

    def sendArmPosition(self,shoulderPitch,shoulderRoll,elbowRoll):
        #print  "Arm:", self.shoulderPitch, self.shoulderRoll, self.elbowRoll
        if self.rightArm == True:
            names = ["RShoulderPitch","RShoulderRoll","RElbowRoll","LElbowRoll"]
            angles = [shoulderPitch, shoulderRoll, elbowRoll,-0.2] 
        else:
            names = ["LShoulderPitch","LShoulderRoll","LElbowRoll","RElbowRoll"]
            angles = [shoulderPitch, shoulderRoll, elbowRoll,0.2] 
        if robotConnected:
            self.motionProxy.setAngles(names, angles, 0.1)
            
    def sendShakeHand(self):
        if robotConnected:        
            self.motionProxy.closeHand("RHand")    
            self.motionProxy.openHand("RHand") 
           
    def quitController(self):
        self.keepPlaying = False 
        if robotConnected:
            self.stopMoving()
            self.basicAwarenessProxy.resumeAwareness()
            self.motionProxy.setExternalCollisionProtectionEnabled("All",True)
            self.ledsProxy.fadeRGB('ChestLeds',1.0,1.0,1.0, 0.5) 

if __name__ == "__main__":
    pygame.init()
    control = xboxcontroller()
    control.runningCode()
    pygame.quit()
                       
