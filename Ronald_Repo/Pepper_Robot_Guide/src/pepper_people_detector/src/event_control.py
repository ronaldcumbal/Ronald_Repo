#! /usr/bin/env python

from naoqi import ALProxy, ALBroker, ALModule
from pepper_people_detector.msg import PeoplePoseArray
from pepper_people_detector.msg import PeoplePose
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import rospy

#        robotIP = rospy.get_param("robotIP")
robotIP = "192.168.0.186"
PORT = 9559

class EventWatcher(ALModule):

    def __init__(self):
        ALModule.__init__(self, "peopleEvent")
        #Start ROS node
        rospy.init_node('NaoqiControl', anonymous=True) 
        self.peolpe_detected_pub = rospy.Publisher("/people_detected", 
                                                    PeoplePoseArray, queue_size=1)
        self.partner_detected_pub = rospy.Publisher("/partner_confirmed", 
                                                    PeoplePose, queue_size=1)
        self.people_vis_pub = rospy.Publisher("/people_vis", 
                                                    PointCloud, queue_size=1)
        # Naoqi Modules
        self.peoplePerceptionProxy = ALProxy("ALPeoplePerception", robotIP, PORT)
        self.faceDetectionProxy = ALProxy("ALFaceDetection", robotIP, PORT)
#        self.ledsProxy = ALProxy("ALLeds", robotIP, PORT)

        #Retreive Events from memory with keys
        self.memoryProxy = ALProxy("ALMemory", robotIP, PORT)
        self.memoryProxy.subscribeToEvent("FaceDetected","peopleEvent","faceDetectedCallback")
        self.memoryProxy.subscribeToEvent("PeoplePerception/PeopleDetected",
                                     "peopleEvent","peopleDetectedCallback")
        # Parameters/Variables
        self.peoplePoseArray = PeoplePoseArray()
        self.peoplePose = PeoplePose()
        self.peopleCloud = PointCloud()
        self.peoplePoint = Point32()
        self.rate = rospy.Rate(10)

        # Initialize Pepper mode
        self.peoplePerceptionProxy.resetPopulation() 
        self.peoplePerceptionProxy.setMaximumDetectionRange(5.0)
#        self.ledsProxy.fadeRGB('ChestLeds',1.0,1.0,1.0,1.0)
        self.partnerName = "Partner"
        self.peopleDetected = False
        self.partnerDetected = False

    def peopleDetectedCallback(self, key, value, msg):
        numPeople = len(value[1])   #[PersonData_1, PersonData_2, ... PersonData_n] 
        del self.peoplePoseArray.people[:]
        del self.peopleCloud.points[:]
        self.peoplePoseArray.header.frame_id = "some_frame" 
        self.peoplePoseArray.header.stamp = rospy.Time.now()
        # For Visualization
        self.peopleCloud.header.stamp = rospy.Time.now()
        self.peopleCloud.header.frame_id = "base_link"
        for n in range(numPeople):
            PersonId = value[1][n][0]
            peoplePosition = self.memoryProxy.getData("PeoplePerception/Person/" + 
                                                      str(PersonId) + "/PositionInRobotFrame")
            angles = self.memoryProxy.getData("PeoplePerception/Person/" + 
                                              str(PersonId) + "/AnglesYawPitch")
            colorShirt = self.memoryProxy.getData("PeoplePerception/Person/" + 
                                                  str(PersonId) + "/ShirtColor")
            self.peoplePose.id=PersonId
            self.peoplePose.shirtcolor=colorShirt
            self.peoplePose.yaw=angles[0]
            self.peoplePose.yaw=angles[1]
            self.peoplePose.pose.position.x=peoplePosition[0]
            self.peoplePose.pose.position.y=peoplePosition[1]
            self.peoplePose.pose.position.z=0.0
            self.peoplePoseArray.people.append(self.peoplePose)
            # For Visualization
            self.peoplePoint.x = peoplePosition[0] 
            self.peoplePoint.y = peoplePosition[1]
            self.peoplePoint.z = 0.0;
            self.peopleCloud.points.append(self.peoplePoint)

        self.peopleDetected = True
        self.partnerDetected = False
        self.publish(self.peopleDetected, self.partnerDetected)

    def faceDetectedCallback(self, key, value, msg):
        if value: #sometimes this list is empty
            FaceInfo = value[1]
            ExtraInfo_N = FaceInfo[0][1]
            faceId = ExtraInfo_N[0]
            scoreRecog = ExtraInfo_N[1]
            faceLabel = ExtraInfo_N[2]
            if faceLabel == self.partnerName:
                self.peopleDetected = True
                self.partnerDetected = True
                self.publish(self.peopleDetected, self.partnerDetected)

    def publish(self, people, partner):
        if people:
            self.people_vis_pub.publish(self.peopleCloud)
            if partner:
#                self.ledsProxy.fadeRGB('ChestLeds',0.0,1.0,0.0,0.01) #Green
                self.partner_detected_pub.publish(self.peoplePose)
            else:
#                self.ledsProxy.fadeRGB('ChestLeds',0.0,0.0,1.0,1.0) #Blue
                self.peolpe_detected_pub.publish(self.peoplePoseArray)
                del self.peoplePoseArray.people[:]
                del self.peopleCloud.points[:]
        else:
            del self.peoplePoseArray.people[:]
            self.peolpe_detected_pub.publish(self.peoplePoseArray)

    def standby(self):
        while not rospy.is_shutdown():
            self.partnerName = "Partner"
#            self.ledsProxy.fadeRGB('ChestLeds',1.0,1.0,1.0,1.0)
            self.peopleDetected = False
            self.partnerDetected = False
            self.publish(self.peopleDetected, self.partnerDetected)
            self.rate.sleep()

if __name__== "__main__":
        eventBroker = ALBroker("eventBroker", "0.0.0.0", 0,robotIP, PORT)
        peopleEvent = EventWatcher()
        try:
            peopleEvent.standby()
            rospy.loginfo("Shutting down")
            eventBroker.shutdown()
        except rospy.ROSInterruptException:
            print "Interrupted by user, shutting down"

