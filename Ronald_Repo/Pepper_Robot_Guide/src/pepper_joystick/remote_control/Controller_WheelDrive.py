#!/usr/bin/env python
from naoqi import ALProxy
import pygame
robotIP = "192.168.0.186"
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
        self.startControl = False

        for i in range(0, pygame.joystick.get_count()):
            self.joysticks.append(pygame.joystick.Joystick(i))    # create a Joystick object
            self.joysticks[-1].init()                   # initialize them all (-1 means loop forever)
            print "Detected joystick '",self.joysticks[-1].get_name(),"'"
    
    def initRobot(self):
        if robotConnected:
            self.startControl = True
#            self.stopMoving()
            self.basicAwarenessProxy.pauseAwareness()        
#            self.postureProxy.goToPosture("StandInit", 0.1) # Speed Relative speed between 0.0 and 1.0
#            self.motionProxy.setExternalCollisionProtectionEnabled("All",False)      
#            self.ledsProxy.fadeRGB('ChestLeds',0.0,1.0,1.0, 0.5) # Green
#            self.motionProxy.setStiffnesses('RArm',0.0);
#            self.motionProxy.setStiffnesses('LArm',0.0);

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
#            self.motionProxy.setStiffnesses('RArm',1.0);
#            self.motionProxy.setStiffnesses('LArm',1.0);

    def runningCode(self):
        print "Controller ready"
        while self.keepPlaying:
            self.clock.tick(60)
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.readStick(event.axis,event.value)
                if event.type == pygame.JOYHATMOTION: 
                    self.readPad(event.value)
                if event.type == pygame.JOYBUTTONDOWN: 
                    self.readButtonDown(event.button)
            
    def readStick(self,axis,value):
        if self.startControl == True:
            if axis in [1, 0, 3, 4]:
                self.driveBody(axis,value)

    def readPad(self,value):
        if self.startControl == True:
            self.driveHead(value)
    
    def readButtonDown(self, value):
        if value == 0: # A
            self.pauseRobot()
        if value == 1: # B
            self.stopMoving()
        if value == 2: # X
            self.initRobot()            
        if value == 3: # Y
            pass
        if value == 4: # LB
            pass
        if value == 5: # RB
            pass
        if value == 6: # Back
            self.quitController()
        if value == 7: # Start
            pass
        if value == 8: # left joy stick
            pass
        if value == 9: # right joy stick
            pass                    
        
    def driveHead(self,value):
        if self.startControl and robotConnected:        
            if value == (0,1):  #Up:
                self.motionProxy.changeAngles("HeadPitch", -0.2, 0.1)
            if value == (0,-1):  #down:
                self.motionProxy.changeAngles("HeadPitch", 0.2, 0.1)
            if value == (-1,0):  #left
                self.motionProxy.changeAngles("HeadYaw", 0.2, 0.1)
            if value == (1,0):  #right
                self.motionProxy.changeAngles("HeadYaw", -0.2, 0.1)    
                
    def driveBody(self,axis,value):
        self.basicAwarenessProxy.pauseAwareness()
        self.headPitch = 0.0 
        self.headYaw = 0.0 
        self.shoulderPitch = 1.5  
        self.shoulderRoll = 0.0
        self.elbowRoll = 0.5
        if axis == 1: # X axis
            self.xVel = -self.limitRange(self.minVel,self.maxVel,-1,1,value)
        if axis == 3: # Theta axis
            self.tVel = -self.limitRange(self.minVel,self.maxVel,-1,1,value)
        self.sendVelocity(self.xVel,self.yVel,self.tVel)

    def limitRange(self, minOut, maxOut, minIn, maxIn, inputValue):
        outputValue = ((maxOut - minOut)*(inputValue-minIn))/(maxIn-minIn)+minOut        
        return round(outputValue,1)

    def stopMoving(self):
        self.xVel = self.yVel = self.tVel = 0.0
        if robotConnected:        
            self.sendVelocity(self.xVel,self.yVel,self.tVel)

    def armsDown(self):
        self.shoulderPitch = 1.5
        self.shoulderRoll = 0.0
        self.elbowRoll = 0.5
            
    def sendVelocity(self,xVel,yVel,tVel):
#        print  "Vel:", self.xVel, self.yVel, self.tVel
        if robotConnected:        
            self.motionProxy.moveToward(xVel,yVel,tVel)

    def quitController(self):
        self.keepPlaying = False 
        if robotConnected:
            self.stopMoving()
            self.basicAwarenessProxy.resumeAwareness()
            self.motionProxy.setExternalCollisionProtectionEnabled("All",True)
            self.ledsProxy.fadeRGB('ChestLeds',1.0,1.0,1.0, 0.5)
            self.motionProxy.setStiffnesses('RArm',1.0);
            self.motionProxy.setStiffnesses('LArm',1.0);

if __name__ == "__main__":
    pygame.init()
    control = xboxcontroller()
    control.runningCode()
    pygame.quit()
                       
