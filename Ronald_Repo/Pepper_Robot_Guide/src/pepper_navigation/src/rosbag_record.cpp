#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <pepper_navigation/lost.h>
#include <pepper_navigation/found.h>
#include <pepper_navigation/distance.h>
#include <pepper_navigation/guiding.h>
#include <pepper_navigation/progress.h>


class Record
{
public:
  Record(){

      robotVel_sub_     = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, 
                              boost::bind(&Record::robotVelCallback, this, _1));
      partnerPose_sub_  = nh_.subscribe<geometry_msgs::PointStamped>("/partner_pose", 1,
                              boost::bind(&Record::partnerPoseCallback, this, _1));
      partnerFound_sub_ = nh_.subscribe<pepper_navigation::found>("/partner_found", 1,
                              boost::bind(&Record::partnerFoundCallback, this, _1));
      partnerLost_sub_  = nh_.subscribe<pepper_navigation::lost>("/partner_lost", 1,
                              boost::bind(&Record::partnerLostCallback, this, _1));
      partnerLost_sub_  = nh_.subscribe<pepper_navigation::lost>("/partner_lost", 1,
                              boost::bind(&Record::partnerLostCallback, this, _1));
      progress_sub_     = nh_.subscribe<pepper_navigation::progress>("/guiding_progress",1,
                              boost::bind(&Record::progressCallback, this, _1));
      guiding_sub_      = nh_.subscribe<pepper_navigation::guiding>("/guiding_status", 1,
                              boost::bind(&Record::guidingStatusCallback, this, _1));



      robotPose_pub_    = nh_.advertise<geometry_msgs::Point>("/bag_robotPose",1); 
      robotVel_pub_     = nh_.advertise<geometry_msgs::Vector3>("/bag_robotVel",1); 
      partnerLocal_pub_ = nh_.advertise<geometry_msgs::Point>("/bag_partnerLocalPose",1);
      partnerGlobal_pub_= nh_.advertise<geometry_msgs::Point>("/bag_partnerGlobalPose",1);
      partnerFound_pub_ = nh_.advertise<geometry_msgs::Point>("/bag_partnerFound",1);
      partnerLost_pub_  = nh_.advertise<geometry_msgs::Point>("/bag_partnerLost",1);

      guiding_ = false;

  }

  void robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
          geometry_msgs::Vector3 vel;
          vel.x = msg->linear.x; 
          vel.y = msg->linear.y;
          vel.z = msg->linear.z;
          robotVel_pub_.publish(vel);
  }

  void partnerPoseCallback(const geometry_msgs::PointStamped::ConstPtr& msg){

      double x_l = msg -> point.x; 
      double y_l = msg -> point.y; 

      geometry_msgs::Point localPose;
      localPose.x = x_l; 
      localPose.y = y_l;
      localPose.z = 0.0;
      partnerLocal_pub_.publish(localPose);

      tf::Stamped<tf::Pose> robot = getRobotTransform("map","base_link");
      double ang = tf::getYaw(robot.getRotation());
      double x_g = x_l*cos(ang)-y_l*sin(ang) + robot.getOrigin().x();
      double y_g = x_l*sin(ang)+y_l*cos(ang) + robot.getOrigin().y();

      geometry_msgs::Point globalPose;
      globalPose.x = x_g; 
      globalPose.y = y_g;
      globalPose.z = 0.0;
      partnerGlobal_pub_.publish(globalPose);
  }

  void partnerFoundCallback(const pepper_navigation::found::ConstPtr& msg){

      geometry_msgs::Point data;
      data.x = 0.0; 
      data.y = 0.0;
      data.z = 0.0;
      partnerFound_pub_.publish(data);
  }

  void partnerLostCallback(const pepper_navigation::lost::ConstPtr& msg){

      geometry_msgs::Point data;
      data.x = 0.0; 
      data.y = 0.0;
      data.z = 0.0;
      partnerLost_pub_.publish(data);
  }

  void getRobotPose(){
          tf::Stamped<tf::Pose> robot_pose = getRobotTransform("map","base_link");

          geometry_msgs::Point robotPose;
          robotPose.x = robot_pose.getOrigin().x(); 
          robotPose.y = robot_pose.getOrigin().y();;
          robotPose.z = 0.0;
          robotPose_pub_ .publish(robotPose);
  }
  
  void guidingStatusCallback(const pepper_navigation::guiding::ConstPtr& msg){
      guiding_ = msg->value;
  }

  void progressCallback(const pepper_navigation::progress::ConstPtr& msg) {
      prog_ = msg->value;
      getRobotPose();
  }

  void distanceCallback(const pepper_navigation::distance::ConstPtr& msg) {
      dist_ = msg->value;
  } 

  tf::Stamped<tf::Pose> getRobotTransform(const std::string& target_frame_,
                                          const std::string& source_frame_){
      tf::StampedTransform transform;
      tf::Stamped<tf::Pose> pose;
      try {
          tf_.waitForTransform(target_frame_,source_frame_, ros::Time(0),ros::Duration(5.0));
          tf_.lookupTransform( target_frame_,source_frame_, ros::Time(0), transform);
          pose.stamp_ = ros::Time(0);
          pose.frame_id_ = target_frame_;
          pose.setData(transform);
      } catch (tf::TransformException ex) {
          ROS_ERROR("Error on Transformation (rosbagTransform) : %s", ex.what());
      }
      return pose;
  }

private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_;

  ros::Subscriber robotVel_sub_;
  ros::Subscriber partnerPose_sub_;
  ros::Subscriber partnerFound_sub_;
  ros::Subscriber partnerLost_sub_;
  ros::Subscriber progress_sub_;
  ros::Subscriber guiding_sub_;

  ros::Publisher robotPose_pub_;
  ros::Publisher robotVel_pub_;
  ros::Publisher partnerLocal_pub_;
  ros::Publisher partnerGlobal_pub_;
  ros::Publisher partnerFound_pub_;
  ros::Publisher partnerLost_pub_;

  bool guiding_;
  double prog_;
  double dist_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosbag_record");
  Record obj;
  ros::spin();
  return 0;

}
