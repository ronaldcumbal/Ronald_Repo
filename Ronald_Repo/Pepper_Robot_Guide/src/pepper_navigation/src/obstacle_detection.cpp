#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <pepper_people_detector/PeoplePoseArray.h>
#include "pepper_people_detector/PeoplePose.h"
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>

class ObstacleDetection
{
  public:
  ObstacleDetection(){
      nh_.getParam("/simulation", simulation_);
      nh_.getParam("/obstacle_detection/laser", laser_topic_); ///pepper_robot/laser

      if (!simulation_){
          nh_.getParam("/move_base/SFMPlanner/robot_base_frame", robot_base_frame_);
          nh_.getParam("/move_base/SFMPlanner/pedest_pose_topic", pedest_pose_topic_);
          nh_.getParam("/move_base/SFMPlanner/obstacle_pose_topic",obstacle_pose_topic_);

      } else {
          nh_.getParam("/robot_0/move_base/SFMPlanner/robot_base_frame", robot_base_frame_);
          nh_.getParam("/robot_0/move_base/SFMPlanner/pedest_pose_topic", pedest_pose_topic_);
          nh_.getParam("/robot_0/move_base/SFMPlanner/obstacle_pose_topic", obstacle_pose_topic_);
      }
      obstacle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>(obstacle_pose_topic_, 1);
      obstacle_vis_pub_  = nh_.advertise<sensor_msgs::PointCloud>("/obstacle_vis", 1);
      pedest_pose_sub_   = nh_.subscribe<pepper_people_detector::PeoplePoseArray>(
                           pedest_pose_topic_, 1, 
                           boost::bind(&ObstacleDetection::posePedestCallback, this, _1));
      laser_sub_         = nh_.subscribe<sensor_msgs::LaserScan>(
                           laser_topic_,1,boost::bind(
                           &ObstacleDetection::laserCallback, this, _1));
     counter_ = 0;
  }

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
      // Pepper Front Laser - Change interval for different lasers
      double ind = 0.0;
      double ang = 0.0;
      double dist = 0.0;
      double x = 0.0;
      double y = 0.0;

      geometry_msgs::Pose pose;
      pose.position.x = 0.0; 
      pose.position.y = 0.0;
      pose.position.z = 0.0; 

      obstaclePose_.poses.clear();
      obstaclePose_.header.stamp = ros::Time::now();
      obstaclePose_.header.frame_id = robot_base_frame_;

      if (!simulation_){//not simulation
          for(int k = 23; k <= 37; k++){
              dist = msg->ranges[k];
              ind = k-23;     
              ang = -0.523598333 + (ind *0.0698); // 60 + i*4
              x = dist * cos(ang);  
              y = dist * sin(ang);
              if (notPeople(x,y,peoplePoseArray_)){
                 pose.position.x = x;
                 pose.position.y = y;
                 obstaclePose_.poses.push_back(pose);
              }
          }
      }
      else{
          for(int k = 0; k < 15; k++){
              dist = msg->ranges[k];
              ind = k;    
              ang = -0.52359789 + (ind *0.0747998282313);
              x = dist * cos(ang);  
              y = dist * sin(ang);
              if (notPeople(x,y,peoplePoseArray_)){
                 pose.position.x = x;
                 pose.position.y = y;
                 obstaclePose_.poses.push_back(pose);
              }
          }
      }
      publishObstaclePose(obstaclePose_);
  }

  void posePedestCallback(const pepper_people_detector::PeoplePoseArray::ConstPtr& poses){

      counter_++;
      if (counter_ > 50){
      peoplePoseArray_.people.clear();
      counter_ = 0;
      }
      for (int i=0; i < poses->people.size(); ++i){
        pepper_people_detector::PeoplePose poseMsg = poses->people[i];
        pepper_people_detector::PeoplePose poseNew;
        peoplePoseArray_.people.push_back(poseMsg);
      }
  }

  bool notPeople(double& x, double& y, pepper_people_detector::PeoplePoseArray& peoplePoses){
      double x_min;
      double y_min;
      double x_max;
      double y_max;
      double range = 0.20;
      bool res = true;
      pepper_people_detector::PeoplePose peoplePoseHelper;

      // First check if obstacle is not a person by comparing poses.
      for (int i=0; i < peoplePoses.people.size(); ++i){
          peoplePoseHelper = peoplePoses.people[i];
          x_min = peoplePoseHelper.pose.position.x - range;
          x_max = peoplePoseHelper.pose.position.x + range;
          y_min = peoplePoseHelper.pose.position.y - range;
          y_max = peoplePoseHelper.pose.position.y + range;
          if( (x_min < x && x < x_max) || (y_min < y && y < y_max) ){
              res = false;
          }
      }
      return res;
  }

  void publishObstaclePose(geometry_msgs::PoseArray& ObstaclePoses){
      obstacle_pose_pub_.publish(ObstaclePoses);

      sensor_msgs::PointCloud obstacle_vis;
      geometry_msgs::Point32 helperPoint;
      geometry_msgs::Pose helperPose;

      obstacle_vis.header.frame_id = robot_base_frame_;
      obstacle_vis.header.stamp = ros::Time::now();

      //Visualization only:
      for(int i=0; i<ObstaclePoses.poses.size(); i++){
          helperPose = ObstaclePoses.poses[i];
          helperPoint.x = helperPose.position.x;
          helperPoint.y = helperPose.position.y;
          helperPoint.z = helperPose.position.z;
          obstacle_vis.points.push_back(helperPoint);
      }
      obstacle_vis_pub_.publish(obstacle_vis);

  }

  private: 
    // Node
    ros::NodeHandle nh_; 
    ros::Subscriber laser_sub_;
    ros::Subscriber pedest_pose_sub_;
    ros::Publisher obstacle_pose_pub_;
    ros::Publisher obstacle_vis_pub_;
    tf::TransformListener tf_;
   
    bool simulation_;
    int counter_;
    std::string robot_base_frame_;
    std::string laser_topic_;
    std::string obstacle_pose_topic_;
    std::string pedest_pose_topic_;
    geometry_msgs::PoseArray obstaclePose_;
    pepper_people_detector::PeoplePose peoplePose_;
    pepper_people_detector::PeoplePoseArray peoplePoseArray_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detection");
  ObstacleDetection state;
  ros::spin();
  return 0;
}

/*

  geometry_msgs::PoseArray SFMPlanner::getObstacles(const costmap_2d::Costmap2D& costmap,
                                                  const std::string frame)
  {
    geometry_msgs::PoseArray array;
    geometry_msgs::Pose pose;

    array.header.stamp = ros::Time::now();
    array.header.frame_id = frame; 

    for(int j = 0; j < costmap.getSizeInCellsX(); j++){
      for(int k = 0; k < costmap.getSizeInCellsY(); k++){
        if(costmap.getCost(j,k)==100){
          pose.position.x = (j - ((costmap.getSizeInCellsX()-1)/2))*costmap.getResolution();
          pose.position.y = (k - ((costmap.getSizeInCellsX()-1)/2))*costmap.getResolution();
          pose.position.z = 0.0;
          array.poses.push_back(pose);
        }
      }
    }
    return array;
  }

      geometry_msgs::PoseArray getObstacles(const costmap_2d::Costmap2D& costmap,
                                            const std::string frame);


*/

