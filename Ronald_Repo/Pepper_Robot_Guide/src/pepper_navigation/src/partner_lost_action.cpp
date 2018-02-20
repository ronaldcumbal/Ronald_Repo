#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <pepper_people_detector/PeoplePose.h>
#include <pepper_people_detector/AwarenessState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pepper_navigation/partnerLostAction.h>
#include <pepper_navigation/sendLocationAction.h>
#include <pepper_navigation/listenAction.h>
#include <pepper_navigation/found.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>

class PartnerLost
{
  public:
  PartnerLost(): partnerLostServer_(nh_,"partner_lost_server",
                   boost::bind(&PartnerLost::partnerLostExecute, this, _1), false)
  {
      // Paramters      
      partner_detected_once_ = true;
      partner_detected_ = false;
      partner_lost_ = false;
      searching_ = false;
      answer_received_ = false;

      // Subscribers and Publisher
      speech_pub_       = nh_.advertise<std_msgs::String>("/speech",1); 
      reset_sub_        = nh_.subscribe<std_msgs::String>(
                          "/reset",1,boost::bind(&PartnerLost::resetCallback, this, _1)); 
      cancelGoal_pub_   = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
      cancelLost_pub_   = nh_.advertise<actionlib_msgs::GoalID>("/partner_lost_server/cancel", 1);
      awaremess_pub_    = nh_.advertise<pepper_people_detector::AwarenessState>("/awarenessState",1); 
      partner_found_pub_= nh_.advertise<pepper_navigation::found>("/partner_found",1);


      speech_recog_pub_ = nh_.advertise<std_msgs::String>("/speech_recog", 1);
      partner_pose_sub_ = nh_.subscribe<pepper_people_detector::PeoplePose>(
                          "/partner_confirmed", 1, 
                          boost::bind(&PartnerLost::posePartnerCallback, this, _1));
      word_recog_sub_   = nh_.subscribe<std_msgs::String>(
                          "/word_recog", 1, boost::bind(
                          &PartnerLost::wordRecognizedCallback, this, _1));
      partnerLostServer_.start();
  }

  void posePartnerCallback(const pepper_people_detector::PeoplePose::ConstPtr& msg){
      if(partner_lost_) {
          partner_detected_ = true;
          askPartner();
          partner_detected_once_ = false;
      }
  }

  void wordRecognizedCallback(const std_msgs::String::ConstPtr& msg) {
      if(msg->data == "yes"){
          sendRoomGoal();
      }
      else if( msg->data == "no"){  
          sendHomeGoal();
          talk("Okay. I'll go home now. Goodbye.");
      }
  }

  void partnerLostExecute(const pepper_navigation::partnerLostGoalConstPtr& pose){
      partner_lost_ = true;
      actionlib_msgs::GoalID emptyID;
      cancelGoal_pub_.publish(emptyID);

      // Change the status and result of the action
      if (partnerLostServer_.isPreemptRequested() || !ros::ok())
      {ROS_INFO("Partner_lost_action: Preempted"); partnerLostServer_.setPreempted();}
      partnerLostResult_.recieved_condition = true;
      partnerLostServer_.setSucceeded(partnerLostResult_);

      // wait to send next goal
      ros::Duration(1.0).sleep();
      talk("Where did you go?");

      //Change variable in planner 
      nh_.setParam("/partner_lost", true);
      nh_.setParam("/with_partner", false);
      nh_.setParam("/move_base/SFMPlanner/going_room", false);
      nh_.setParam("/move_base/SFMPlanner/going_home", false);

      //Send last known position
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = pose->target_pose.header.frame_id;
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = pose->target_pose.pose.position.x;
      goal.target_pose.pose.position.y = pose->target_pose.pose.position.y;
      goal.target_pose.pose.position.z = pose->target_pose.pose.position.y;
      goal.target_pose.pose.orientation.x = pose->target_pose.pose.orientation.x;
      goal.target_pose.pose.orientation.y = pose->target_pose.pose.orientation.y;
      goal.target_pose.pose.orientation.z = pose->target_pose.pose.orientation.z;
      goal.target_pose.pose.orientation.w = pose->target_pose.pose.orientation.w;

      MoveBaseClient_ goalClient("move_base", true);
      ROS_INFO("partner_lost_action: waiting for action server to start.");
      goalClient.waitForServer();
      ROS_INFO("partner_lost_action: Sending goal");
      goalClient.sendGoal(goal);

      searching_ = true;

      goalClient.waitForResult();
      if(goalClient.getState()==actionlib::SimpleClientGoalState::SUCCEEDED) {
          if(partner_detected_==false && searching_==true) {
              awareness(2);
              ros::Duration(60.0).sleep(); // Time Pepper waits to find Partner       
              std::cout<< "Done checking, sending home" << std::endl;
//                sendHomeGoal();
          }
      }
  }

  void askPartner(){
      if(partner_detected_once_ && searching_) {
          talk("There you are, would you like to continue?");
          actionlib_msgs::GoalID emptyID;
          cancelLost_pub_.publish(emptyID);
          cancelGoal_pub_.publish(emptyID);
          partnerFound();
     }
  } 

  void partnerFound(){
      partner_detected_once_ = false;
//      sendListeningRequest();
      sendKeyboardRequest();
      partner_lost_ =false;
      partner_detected_ = true;

      pepper_navigation::found msg;
      msg.condition = true;
      partner_found_pub_.publish(msg);
  }

  void sendRoomGoal(){
      std::cout<< "sendRoomGoal()" << std::endl;
      searching_ = false;
      nh_.getParam("/room_number", room_number);
      sendLocationClient_ locationClient("send_location_server", true);
      locationClient.waitForServer();
      ROS_INFO("partner_lost_server: send_goal server started, sending goal.");

      pepper_navigation::sendLocationGoal room; 
      room.room_number = room_number;
      locationClient.sendGoal(room);

      partner_detected_ = false;
      partner_lost_ = false;

  }

  void sendHomeGoal(){
      std::cout<< "sendHomeGoal()" << std::endl;

      searching_ = false;
      if(!partner_detected_) {
          talk("I can't find you");
      }

      //cancel Goals
      actionlib_msgs::GoalID emptyID;
      cancelLost_pub_.publish(emptyID);

      // Send Action
      sendLocationClient_ locationClient("send_location_server", true);
      locationClient.waitForServer();
      ROS_INFO("partner_lost_server: send_goal server started, sending goal.");
      pepper_navigation::sendLocationGoal room; 
      room.room_number = 0;
      locationClient.sendGoal(room);
  }

  void sendListeningRequest() {
      std_msgs::String msg;
      msg.data = "word";
      speech_recog_pub_.publish(msg);
  }  

  void sendKeyboardRequest(){
      int answer;
      std::cin >> answer;

      if(answer == 1){
          talk("yes");
          sendRoomGoal();
      }
      else if( answer == 0){  
          talk("no");
          sendHomeGoal();
          talk("Okay. I'll go home now. Have a good day. Goodbye.");
      }
  }


  void talk(const char* text){
      std_msgs::String msg;
      msg.data = text;
      speech_pub_.publish(msg);
  }

  void resetCallback(const std_msgs::String::ConstPtr& msg) {
      partner_detected_once_ = true;
      partner_detected_ = false;
      partner_lost_ = false;
      searching_ = false;
      answer_received_ = false;
  }

  void awareness(const int condition) {
      pepper_people_detector::AwarenessState msg;
      msg.state = condition;
      awaremess_pub_.publish(msg);
  }

  private: 
  ros::NodeHandle nh_; 
  ros::Publisher speech_pub_;
  ros::Publisher cancelGoal_pub_;
  ros::Publisher cancelLost_pub_;
  ros::Publisher speech_recog_pub_;
  ros::Publisher awaremess_pub_;
  ros::Publisher partner_found_pub_;
  ros::Subscriber partner_pose_sub_;
  ros::Subscriber word_recog_sub_;
  ros::Subscriber reset_sub_;

  actionlib::SimpleActionServer<pepper_navigation::partnerLostAction> partnerLostServer_; 
  pepper_navigation::partnerLostResult partnerLostResult_;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient_; 
  typedef actionlib::SimpleActionClient<pepper_navigation::sendLocationAction> sendLocationClient_;
  bool partner_detected_once_;
  bool partner_detected_;
  bool partner_lost_;
  bool searching_;
  bool answer_received_;
  int room_number;
  std::string partner_pose_topic_;
}; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "partner_lost_node");
  PartnerLost state;
  ros::spin();
  return 0;
}

