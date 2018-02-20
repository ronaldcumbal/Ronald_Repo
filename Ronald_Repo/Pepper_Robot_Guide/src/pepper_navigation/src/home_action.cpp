#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pepper_people_detector/AwarenessState.h>
#include <pepper_people_detector/PeoplePoseArray.h>
#include <pepper_people_detector/PeoplePose.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <pepper_navigation/sendLocationAction.h>
#include <pepper_navigation/listenAction.h>
#include <pepper_navigation/guiding.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/String.h>

class StateMachine
{

public:
  StateMachine()
  {
    speech_pub_     = nh_.advertise<std_msgs::String>("/speech",1); 
    awaremess_pub_  = nh_.advertise<pepper_people_detector::AwarenessState>("/awarenessState",1);
    speech_recog_pub_=nh_.advertise<std_msgs::String>("/speech_recog", 1);
    joint_pub_      = nh_.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>(
                      "/pepper_robot/pose/joint_angles",1);

    resultGoal_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, 
                        boost::bind(&StateMachine::goalResultCallback, this, _1));
    partner_sub_    = nh_.subscribe<pepper_people_detector::PeoplePose>("/partner_confirmed",1,
                      boost::bind(&StateMachine::partnerCallback, this, _1));
    guiding_sub_    = nh_.subscribe<pepper_navigation::guiding>("/guiding_status", 1, 
                      boost::bind(&StateMachine::guidingStatusCallback, this, _1));
    word_recog_sub_ = nh_.subscribe<std_msgs::String>("/word_recog", 1, boost::bind(
                      &StateMachine::wordRecognizedCallback, this, _1));

    timer_          = nh_.createTimer(ros::Duration(30.0), &StateMachine::timerCallback, this);
    timer_.stop();
    count_ = 0;
    going_room_ = false;
    going_home_ = false;
    guiding_ = false;
    triggeredOnce_ = false;
    armRaised_ = false;
    headTurningFront_ = false;
    headTurningLeft_  = false;
  }

  void guidingStatusCallback(const pepper_navigation::guiding::ConstPtr& msg) {
      if(msg->value == true) {guiding_ = true;}
      else {guiding_ = false;}
  }

  void timerCallback(const ros::TimerEvent& event) {
      triggeredOnce_ = false;
      timer_.stop();
  }

  void partnerCallback(const pepper_people_detector::PeoplePose::ConstPtr& pose) {
      bool with_partner;
      nh_.getParam("/with_partner", with_partner);
      if(!guiding_ && with_partner) {
          count_++;
          if(count_>5){askQuestion(); count_ = 0;}    
      }
   }

  void askQuestion(){
      if(!guiding_ && !triggeredOnce_) {
          triggeredOnce_ = true;
          talk("Hi, I'm Pepper.");
          ros::Duration(0.5).sleep();
          talk("I can guide you to your room");
          ros::Duration(1.0).sleep();
          talk("What room number are you looking for?");
//          sendListeningRequest();
          sendKeyboardRequest();
      }
  }

  void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
      nh_.getParam("/move_base/SFMPlanner/going_room", going_room_);
      nh_.getParam("/move_base/SFMPlanner/going_home", going_home_);
      if (msg->status.status == 3) {
          if (going_room_){roomReached();}
          if (going_home_){standby();}
      }
  }

  void sendListeningRequest() {
      awareness(1);
      std_msgs::String msg;
      msg.data = "number";
      speech_recog_pub_.publish(msg);
  } 

  void sendKeyboardRequest(){
      int room_number;
      std::cin >> room_number;
      if(room_number != 0){  
          nh_.setParam("/room_number", room_number);
          sendRoomGoal(room_number);

          std::stringstream strs;
          strs << room_number;
          std::string temp_str = strs.str();
          char* char_type = (char*) temp_str.c_str();
          talk(char_type);
      }

  }

  void wordRecognizedCallback(const std_msgs::String::ConstPtr& msg) {
      int room_number;  
      std::stringstream(msg->data) >> room_number;
      if(room_number != 0){  
          nh_.setParam("/room_number", room_number);
          sendRoomGoal(room_number);
      }
  }

  void roomReached() {
      talk("This is your room"); 
      raiseArm();
      ros::Duration(0.5).sleep();
      talk("I have to go back now");
      ros::Duration(1.0).sleep();
      talk("Have a good day");
      ros::Duration(0.5).sleep();
      talk("Goodbye");
      ros::Duration(2.0).sleep();
      returnHome();
  }

  void sendRoomGoal(const int number) {
      awareness(0);
      sendLocationClient_ locationClient("send_location_server", true);
      locationClient.waitForServer();
      ROS_INFO("home_action_server: send_goal server started, sending goal.");

      pepper_navigation::sendLocationGoal room; 
      room.room_number = number; 
      locationClient.sendGoal(room);
  } 

  void returnHome(){
      awareness(0);
      sendLocationClient_ locationClient("send_location_server", true);
      locationClient.waitForServer();
      ROS_INFO("home_action_server: send_goal server started, sending goal.");

      pepper_navigation::sendLocationGoal room; 
      room.room_number = 0;
      locationClient.sendGoal(room);
  }

  void standby() {
      bool with_partner;
      nh_.getParam("/with_partner", with_partner);
      nh_.setParam("Guiding/partnerName", "Partner");
      ros::Duration(10.0).sleep();
      awareness(1); 
  }

  void talk(const char* text) {
      std_msgs::String msg;
      msg.data = text;
      speech_pub_.publish(msg);
  }

  void awareness(const int condition) {
      pepper_people_detector::AwarenessState msg;
      msg.state = condition;
      awaremess_pub_.publish(msg);
  }

  void raiseArm() {
      if(!armRaised_) {
          naoqi_bridge_msgs::JointAnglesWithSpeed angles;
          angles.header.frame_id = "base_link";

          angles.joint_names.clear();
          angles.joint_angles.clear();
          angles.header.stamp = ros::Time::now();
          angles.joint_names.push_back("RShoulderPitch");
          angles.joint_angles.push_back(0.3);
          angles.joint_names.push_back("RShoulderRoll");
          angles.joint_angles.push_back(-0.02);
          angles.joint_names.push_back("RElbowRoll");
          angles.joint_angles.push_back(0.25);
          angles.joint_names.push_back("RWristYaw");
          angles.joint_angles.push_back(0.75);
          angles.speed = 0.2;
          joint_pub_.publish(angles);
          armRaised_ = true;

          ros::Duration(2.0).sleep();
          angles.joint_names.clear();
          angles.joint_angles.clear();
          angles.header.stamp = ros::Time::now();
          angles.joint_names.push_back("RShoulderPitch");
          angles.joint_angles.push_back(1.5);
          angles.joint_names.push_back("RShoulderRoll");
          angles.joint_angles.push_back(0.0);
          angles.joint_names.push_back("RElbowRoll");
          angles.joint_angles.push_back(0.0);
          angles.joint_names.push_back("RWristYaw");
          angles.joint_angles.push_back(0.0);
          angles.speed = 0.2;
          joint_pub_.publish(angles);
          armRaised_ = false;
      }
  }

private: 
  bool going_room_;
  bool going_home_;
  bool guiding_;
  bool triggeredOnce_;
  bool  armRaised_;
  int count_;
  bool headTurningFront_;
  bool headTurningLeft_;
 
  ros::NodeHandle nh_; 
  ros::Timer timer_;
  ros::Publisher speech_pub_;
  ros::Publisher awaremess_pub_;
  ros::Publisher speech_recog_pub_;
  ros::Publisher joint_pub_;
  ros::Subscriber resultGoal_sub_;
  ros::Subscriber partner_sub_;
  ros::Subscriber guiding_sub_;
  ros::Subscriber word_recog_sub_;

  typedef actionlib::SimpleActionClient<pepper_navigation::sendLocationAction> sendLocationClient_;

}; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "home_action_node");
  StateMachine state;
  state.standby();
  ros::spin(); 
  return 0;
}
