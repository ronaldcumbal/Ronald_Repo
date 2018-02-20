#include <ros/ros.h>
#include <iostream>
#include <stdlib.h> 
#include <time.h> 
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pepper_navigation/sendLocationAction.h>
#include <pepper_navigation/lost.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/String.h>

class SendLocation
{
  public:
    SendLocation(): sendLocationServer_(nh_,"send_location_server",
                     boost::bind(&SendLocation::sendLocationExecute, this, _1), false)
    {
        reset_pub_        = nh_.advertise<std_msgs::String>("/reset",1); 
        speech_pub_       = nh_.advertise<std_msgs::String>("/speech",1);
        partner_lost_pub_ = nh_.advertise<pepper_navigation::lost>("/partner_lost",1); 
        joint_pub_        = nh_.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>(
                            "/pepper_robot/pose/joint_angles",1);
        sendLocationServer_.start();

        armShown_ = false;
        headTurningFront_ = true;
        headTurningLeft_  = false;
    }

    void setLocations(std::vector<geometry_msgs::Pose> &location){
        geometry_msgs::Pose pose;

//        pose.position.x=21.2; pose.position.y=0.0; pose.orientation.z=0.999999999; pose.orientation.w = 0.0421423458; location.push_back(pose);
        pose.position.x=-8.50; pose.position.y=0.0; pose.orientation.z=0.965000000; pose.orientation.w = -0.2622570; location.push_back(pose);
//        pose.position.x=-12.5; pose.position.y=0.74; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);
        pose.position.x=33.02; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 1
        pose.position.x=29.15; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 2
        pose.position.x=25.29; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 3
        pose.position.x=21.33; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 4
        pose.position.x=17.50; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 5
        pose.position.x=17.50; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 6
        pose.position.x=13.61; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 7
        pose.position.x=13.61; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 8
        pose.position.x=9.62; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 9
        pose.position.x=9.62; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 10
        pose.position.x=5.90; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 11
        pose.position.x=5.90; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 12
        pose.position.x=2.00; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 13
        pose.position.x=2.00; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 14
        pose.position.x=-2.00; pose.position.y=0.0; pose.orientation.z=-0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 15
        pose.position.x=-2.00; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 16
        pose.position.x=-5.86; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 17
        pose.position.x=-9.74; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 18
        pose.position.x=-13.64; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 19
        pose.position.x=-17.57; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 20
        pose.position.x=-21.39; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 21
        pose.position.x=-25.31; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 22
        pose.position.x=-29.18; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 23
        pose.position.x=-33.12; pose.position.y=0.0; pose.orientation.z=0.707106781; pose.orientation.w = 0.707106781; location.push_back(pose);// 24
}

  void sendLocationExecute(const pepper_navigation::sendLocationGoalConstPtr& goal)
  {
      bool partner_present;
      bool going_home;
      bool going_room;
      int input = goal->room_number;
      if (sendLocationServer_.isPreemptRequested() || !ros::ok()) {
          ROS_INFO("send Goal: Preempted");
          sendLocationServer_.setPreempted();
      }

      if(goal->room_number == 0) {
          partner_present = false;
          going_home = true;
          going_room = false;
          resetVaribles(going_home,going_room,partner_present);
      }
      else {
          partner_present = true;
          going_home = false;
          going_room = true;
          resetVaribles(going_home,going_room,partner_present);
      }

      turnFront();
      sendRoomGoal(partner_present,input);
      sendLocationResult_.recieved_condition = true;
      sendLocationServer_.setSucceeded(sendLocationResult_);
  }

  void sendRoomGoal(bool &partner_present, int & ind)
  {
      setLocations(locations_);
      move_base_msgs::MoveBaseGoal goal;
      geometry_msgs::Pose pose = locations_[ind];
      goal.target_pose.header.frame_id = "/map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = pose.position.x;
      goal.target_pose.pose.position.y = pose.position.y;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = pose.orientation.z;
      goal.target_pose.pose.orientation.w = pose.orientation.w;

      MoveBaseClient_ goalClient("move_base", true);
      ROS_INFO("state_machine: Waiting for server: sendRoomGoal");
      goalClient.waitForServer();
      ROS_INFO("state_machine: Sending sendRoomGoal");
      goalClient.sendGoal(goal);

      if (partner_present){
          ros::Duration(2.0).sleep();
          talk("Could you walk on my left side");
          showLeft();
          ros::Duration(3.0).sleep();
          talk("Please, follow me");
      }
  }

  void talk(const char* text)
  {
      std_msgs::String msg;
      msg.data = text;
      speech_pub_.publish(msg);
  }

  void turnFront() {
      if(!headTurningFront_){
          naoqi_bridge_msgs::JointAnglesWithSpeed angles;
          angles.header.frame_id = "base_link";
          angles.header.stamp = ros::Time::now();
          angles.joint_names.push_back("HeadPitch");
          angles.joint_angles.push_back(0.0);
          angles.joint_names.push_back("HeadYaw");
          angles.joint_angles.push_back(0.0);
          angles.speed = 0.2;
          joint_pub_.publish(angles);
          headTurningFront_ = true;
          headTurningLeft_  = false;
      }
  }

  void turnLeft() {
      if(!headTurningLeft_) {
          naoqi_bridge_msgs::JointAnglesWithSpeed angles;
          angles.header.frame_id = "base_link";
          angles.header.stamp = ros::Time::now();
          angles.joint_names.push_back("HeadPitch");
          angles.joint_angles.push_back(-0.5);
          angles.joint_names.push_back("HeadYaw");
          angles.joint_angles.push_back(1.5);
          angles.speed = 0.2;
          joint_pub_.publish(angles);
          headTurningFront_ = false;
          headTurningLeft_  = true;
      }
  }

  void showLeft() {
      if(!armShown_) {
          naoqi_bridge_msgs::JointAnglesWithSpeed angles;
          angles.header.frame_id = "base_link";
          angles.header.stamp = ros::Time::now();
          angles.joint_names.push_back("HeadPitch");
          angles.joint_angles.push_back(-0.2);
          angles.joint_names.push_back("HeadYaw");
          angles.joint_angles.push_back(1.1);
          angles.joint_names.push_back("LElbowRoll");
          angles.joint_angles.push_back(-0.50);
          angles.joint_names.push_back("LShoulderPitch");
          angles.joint_angles.push_back(0.93);
          angles.joint_names.push_back("LShoulderRoll");
          angles.joint_angles.push_back(0.41);
          angles.joint_names.push_back("LWristYaw");
          angles.joint_angles.push_back(-0.91);
          angles.speed = 0.1;
          joint_pub_.publish(angles);
          armShown_ = true;

          ros::Duration(2.0).sleep();
          angles.joint_names.clear();
          angles.joint_angles.clear();
          angles.header.stamp = ros::Time::now();
          angles.joint_names.push_back("HeadPitch");
          angles.joint_angles.push_back(-0.2);
          angles.joint_names.push_back("HeadYaw");
          angles.joint_angles.push_back(0.0);
          angles.joint_names.push_back("LElbowRoll");
          angles.joint_angles.push_back(0.0);
          angles.joint_names.push_back("LShoulderPitch");
          angles.joint_angles.push_back(1.64);
          angles.joint_names.push_back("LShoulderRoll");
          angles.joint_angles.push_back(0.0);
          angles.joint_names.push_back("LWristYaw");
          angles.joint_angles.push_back(-0.14);
          angles.speed = 0.1;
          joint_pub_.publish(angles);
          armShown_ = false;
      }
  }

  void resetVaribles(const bool& going_home, const bool& going_room,const bool& partner_present){
      std_msgs::String msg;
      msg.data = "reset";
      reset_pub_.publish(msg);

      pepper_navigation::lost data;
      data.condition = false;
      partner_lost_pub_.publish(data);

      nh_.setParam("/move_base/SFMPlanner/going_home", going_home);
      nh_.setParam("/move_base/SFMPlanner/going_room", going_room);
      nh_.setParam("/with_partner", partner_present);
      nh_.setParam("/partner_lost", false);
  }

  private: 
    ros::NodeHandle nh_; 
    ros::Publisher speech_pub_;
    ros::Publisher joint_pub_;
    ros::Publisher reset_pub_;
    ros::Publisher partner_lost_pub_;
    std::vector<geometry_msgs::Pose> locations_;
      
    bool armShown_;
    bool headTurningLeft_;
    bool headTurningFront_;


    //Action Clients
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient_; 
    actionlib::SimpleActionServer<pepper_navigation::sendLocationAction> sendLocationServer_; 
    pepper_navigation::sendLocationResult sendLocationResult_;
}; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node");
  SendLocation state;
  ros::spin(); 
  return 0;
}
