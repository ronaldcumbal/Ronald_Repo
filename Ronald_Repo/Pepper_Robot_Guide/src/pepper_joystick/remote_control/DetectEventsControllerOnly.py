#!/usr/bin/env python
import pygame
 
class xboxcontroller():
    def __init__(self):
        # Initialize variables
        self.joysticks = []
        self.clock = pygame.time.Clock()
        self.keepPlaying = True
        self.xVel = self.yVel = self.tVel = 0.0    
        self.maxVel = 0.9
        self.minVel = -0.9   
    
        # Check for controllers connected
        for i in range(0, pygame.joystick.get_count()):
            # create an Joystick object in our list
            self.joysticks.append(pygame.joystick.Joystick(i))
            # initialize them all (-1 means loop forever)
            self.joysticks[-1].init()
            # print a statement telling what the name of the controller is
            print "Detected joystick '",self.joysticks[-1].get_name(),"'"

    def running(self):
        while self.keepPlaying:
            self.clock.tick(60)
            for event in pygame.event.get():
		if event.type == pygame.JOYAXISMOTION:
			if event.axis<20:	
				print event.joy, event.axis, event.value
		#if event.type == pygame.JOYHATMOTION:
			#print event.joy, event.hat, event.value
                if event.type == pygame.JOYBUTTONUP:
			pass
			#print event.joy, event.button
                if event.type == pygame.JOYBUTTONDOWN:
			pass	
			#print event.joy, event.button 

if __name__ == "__main__":
    pygame.init()
    control = xboxcontroller()
    control.running()
    #basicAwarenessProxy = ALProxy("ALBasicAwareness", robotIP, PORT)
    #basicAwarenessProxy.stopAwareness()
    pygame.quit()  
                  


                     
#            elif event.type == pygame.JOYBUTTONDOWN:
#                print "JOYBUTTONDOWN '",joysticks[event.joy].get_name(),"' button",event.button
#
#            elif event.type == pygame.JOYBUTTONUP:
#                print "JOYBUTTONUP '",joysticks[event.joy].get_name(),"' button",event.button

# event.type == pygame.JOYBUTTONDOWN
# event.type == pygame.JOYBUTTONUP
# event.button == 0   A  
# event.button == 1   B
# event.button == 2   X  
# event.button == 3   Y
# event.button == 4   LB
# event.button == 5   RB
# event.button == 6   BACK
# event.button == 7   START
# event.button == 8   left joy stick
# event.button == 9   right joy stick

# event.type == pygame.JOYAXISMOTION
# event.axis == 0 left joy stick (center)
# event.axis == 1 left joy stick (moved)
# event.axis == 2 LT & RT
# event.axis == 3g right joy stick (center)
# event.axis == 4 riht joy stick (moved)
 
# vent.type == pygame.JOYHATMOTION 
# event.hat == 0 Pade
