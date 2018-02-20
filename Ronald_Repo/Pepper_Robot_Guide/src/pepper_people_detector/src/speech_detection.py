#! /usr/bin/env python

import rospy
from naoqi import ALProxy, ALBroker, ALModule
import actionlib
import time
from std_msgs.msg import String
from pepper_people_detector.msg import PeoplePose
from pepper_navigation.msg import listenAction
from pepper_navigation.msg import listenResult


robotIP = "192.168.0.186"
PORT = 9559

class SpeechControl(ALModule):

    def __init__(self):
        ALModule.__init__(self, "speechEvent")

        #Start ROS node
        rospy.init_node('SpeechControl', anonymous=True) 
        self.rate = rospy.Rate(10)
        self.speech_pub           = rospy.Publisher("/speech", String, queue_size=1)
        self.word_recog_pub       = rospy.Publisher("/word_recog", String, queue_size=1)
        self.partner_detected_sub = rospy.Subscriber("/partner_confirmed", PeoplePose,
                                                      self.partnerCallback)
        self.speech_recog_sub     = rospy.Subscriber("/speech_recog", String,
                                                      self.speechRecogCallback)
        # Naoqi Modules
        self.speechRecognitionProxy = ALProxy("ALSpeechRecognition", robotIP, PORT)
        self.ledsProxy = ALProxy("ALLeds", robotIP, PORT)
        self.memoryProxy = ALProxy("ALMemory", robotIP, PORT)

        #Retreive Events from memory with keys
        self.memoryProxy.subscribeToEvent("WordRecognized", "speechEvent", "onWordRecognized")

        # Initialization of ASR
        self.speechRecognitionProxy.setLanguage("English")
        self.speechRecognitionProxy.pause(True)
        self.vocabWord = ["yes", "no"]
        self.vocabNum = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12",
                         "13","14","15","16","17","18","19","20","21","22","23","24"]
        self.is_speech_reco_started = False
        
        # Parametes
#        self.Answer= String()
        self.triggeredOnce = False
        self.wordDetected = False

    def speechRecogCallback(self, msg):
        if self.triggeredOnce == False:
            if msg.data == "number":
                self.speechRecognitionProxy.setVocabulary(self.vocabNum, False)
            if msg.data == "word":
                self.speechRecognitionProxy.setVocabulary(self.vocabWord, False)
            self.triggeredOnce = True
            self.startSpeechReco()

    def startSpeechReco(self):
        if not self.is_speech_reco_started:
#            self.listen_timer = rospy.Timer(rospy.Duration(10), self.timerCallback)
            self.speechRecognitionProxy.pause(False)
            self.speechRecognitionProxy.subscribe("Test_ASR")
            self.is_speech_reco_started = True
            print "ASR started"

    def stopSpeechReco(self):
        if self.is_speech_reco_started:
#            self.listen_timer.shutdown()
            self.speechRecognitionProxy.unsubscribe("Test_ASR")
            self.speechRecognitionProxy.pause(True)
            self.is_speech_reco_started = False
            print "stop ASR"

    def partnerCallback(self,msg):
        pass

    def timerCallback(self, event):
        self.stopSpeechReco()
        # Send back response from action request
        self.result.word = "None"
        self.server.set_succeeded(self.result)
        self.talk("Could not understand your answer")
        self.wordDetected = False
        print "timerCallback"

    def onWordRecognized(self, key, value, msg):
        print "word: ", value
        if value[1] > 0.4:
            self.stopSpeechReco()
            self.Answer = value[0]
            self.wordDetected = True
            self.triggeredOnce = False
            time.sleep(0.2)
            self.talk(self.Answer)
            time.sleep(0.2)
            self.word_recog_pub.publish(self.Answer)

    def talk(self,text):
        self.speech_pub.publish(text)

    def standby(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__== "__main__":
        speechBroker = ALBroker("speechBroker", "0.0.0.0", 0,robotIP, PORT)
        speechEvent = SpeechControl()
        try:
            speechEvent.standby()
            rospy.loginfo("Shutting down")
            speechBroker.shutdown()
        except rospy.ROSInterruptException:
            print "Interrupted by user, shutting down"

