#! /usr/bin/env python

from naoqi import ALProxy, ALBroker, ALModule
from pepper_people_detector.msg import AwarenessState
from pepper_people_detector.msg import HeadPose  
from std_msgs.msg import String
import rospy

robotIP = "192.168.0.186"
PORT = 9559

class GestureControl(ALModule):

    def __init__(self):
        ALModule.__init__(self, "GestureControl")
        #Start ROS node
        rospy.init_node('NaoqiControl2', anonymous=True) 
        self.awarness_sub= rospy.Subscriber("/awarenessState", AwarenessState, self.awarnessCallback)
        self.speech_pub = rospy.Publisher("/speech", String, queue_size=1)
        self.headPose_pub = rospy.Publisher("/guiding_headPose", HeadPose, queue_size=1)
        # Naoqi Modules
        self.basicAwarenessProxy = ALProxy("ALBasicAwareness", robotIP, PORT)
        self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        self.autonomousLifeProxy = ALProxy("ALAutonomousLife", robotIP, PORT)
        self.rate = rospy.Rate(2)
        # Initialize Pepper mode
        self.motionProxy.setOrthogonalSecurityDistance(0.15);
        self.motionProxy.setStiffnesses('LArm',1.0);
        self.motionProxy.setStiffnesses('RArm',1.0);
        self.headPose = HeadPose()
        self.triggeredOnce = False

    def awarnessCallback(self,msg):
        if msg.state == 0:
            self.basicAwarenessProxy.stopAwareness()
            self.basicAwarenessProxy.pauseAwareness()
        if msg.state == 1:
            self.autonomousLifeProxy.stopAll()
            self.basicAwarenessProxy.startAwareness()
            self.basicAwarenessProxy.resumeAwareness()
            self.basicAwarenessProxy.setEngagementMode("FullyEngaged")
            self.basicAwarenessProxy.setTrackingMode("Head")
        if msg.state == 2:
            self.autonomousLifeProxy.stopAll()
            self.basicAwarenessProxy.startAwareness()
            self.basicAwarenessProxy.resumeAwareness()
            self.basicAwarenessProxy.setEngagementMode("FullyEngaged")
            self.basicAwarenessProxy.setTrackingMode("WholeBody")


    def headPosition(self):
        useSensors    = False
        yaw = self.motionProxy.getAngles("HeadYaw", useSensors)
        pitch = self.motionProxy.getAngles("HeadPitch", useSensors)
        self.headPose.yaw = yaw[0]
        self.headPose.pitch = pitch[0]
        if self.headPose.yaw >= 1.3 and self.triggeredOnce == False:
            self.headPose.pose = "left"
            self.headPose_pub.publish(self.headPose)
            self.triggeredOnce = True
        if self.headPose.yaw >= -0.2 and self.headPose.yaw <= 0.2 and self.triggeredOnce == False:
            self.headPose.pose = "center"
            self.headPose_pub.publish(self.headPose)
            self.triggeredOnce = True
        if self.headPose.yaw > 0.2 and self.headPose.yaw < 1.3:
            self.triggeredOnce = False

    def standby(self):
        while not rospy.is_shutdown():
            self.headPosition()
            self.rate.sleep()

if __name__== "__main__":
        controlBroker = ALBroker("controlBroker", "0.0.0.0", 0, robotIP, PORT)
        controlEvent = GestureControl()
        try:
            controlEvent.standby()
            rospy.loginfo("Shutting down")
            controlBroker.shutdown()
        except rospy.ROSInterruptException:
            print "Interrupted by user, shutting down"

