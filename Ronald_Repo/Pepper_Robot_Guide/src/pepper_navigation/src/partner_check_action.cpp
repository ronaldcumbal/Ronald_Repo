#include <math.h>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point32.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pepper_navigation/guiding.h>
#include <pepper_navigation/progress.h>
#include <pepper_navigation/distance.h>
#include <pepper_navigation/lost.h>
#include <pepper_navigation/partnerLostAction.h>
#include <pepper_people_detector/PeoplePose.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>

class PartnerCheck
{
  public:
  PartnerCheck(): R(2,2), Q(4,4), A(4,4), H(2,4), I(4,4), P_old(4,4), P_pred(4,4), 
                  K(4,2), P_cor(4,4), S(4,4)
      {

      nh_.getParam("/move_base/SFMPlanner/robot_base_frame",robot_base_frame_);
      nh_.getParam("/move_base/SFMPlanner/global_frame",global_frame_);

      speech_pub_       = nh_.advertise<std_msgs::String>("/speech",1); 
      partner_lost_pub_ = nh_.advertise<pepper_navigation::lost>("/partner_lost",1);
      partner_point_pub_= nh_.advertise<geometry_msgs::PointStamped>("/partner_vis",1);
      partner_pose_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/partner_pose",1);
      partner_cov_pub_  = nh_.advertise<sensor_msgs::PointCloud>("/partner_cov",1);

//      partnerpose_pub_  = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/partner_pose");

      joint_pub_        = nh_.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>(
                        "/pepper_robot/pose/joint_angles",1);

      reset_sub_    = nh_.subscribe<std_msgs::String>(
                      "/reset",1,boost::bind(&PartnerCheck::resetCallback, this, _1)); 
      laser_sub_    = nh_.subscribe<sensor_msgs::LaserScan>(
                      "/pepper_robot/laser",1,boost::bind(
                      &PartnerCheck::laserCallback, this, _1));
      guiding_sub_  = nh_.subscribe<pepper_navigation::guiding>(
                      "/guiding_status", 1, boost::bind(
                      &PartnerCheck::guidingStatusCallback, this, _1));
      progress_sub_ = nh_.subscribe<pepper_navigation::progress>(
                      "/guiding_progress", 1, boost::bind(
                      &PartnerCheck::progressCallback, this, _1));
      distance_sub_ = nh_.subscribe<pepper_navigation::distance>(
                      "/guiding_distance", 1, boost::bind(
                      &PartnerCheck::distanceCallback, this, _1));
      sonar_sub_    = nh_.subscribe<sensor_msgs::Range>(
                      "/pepper_robot/sonar/back", 1, boost::bind(
                      &PartnerCheck::sonarCallback, this, _1));

      posture_timer_= nh_.createTimer(ros::Duration(8),&PartnerCheck::postureTimerCallback, this);
      reset_timer_  = nh_.createTimer(ros::Duration(2),&PartnerCheck::resetTimerCallback, this);
      reset_timer_.stop();

      partnerDetected_  = true;
      partnerLost_      = false;
      triggered_once_   = false;
      say_once_         = false;
      headTurningFront_ = false;
      headTurningLeft_  = false;
      uncertainty_ = 0.0;

      //Kalman Filter Paramters
      R << 0.1,0,0,0.1;                                 //measurement noise
      Q << 0.1,0,0,0, 0,0.1,0,0, 0,0,1,0, 0,0,0,1;  //process noise
      // System Equations
      A << 1,0,1,0, 0,1,0,1, 0,0,1,0, 0,0,0,1; 
      I << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;  
      H << 1,0,0,0, 0,1,0,0;
      // Initial values
      P_old <<  1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1; 
      P_pred << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0; 
      P_cor <<  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0; 
      x_old <<  0, 0, 0, 0; 
      x_pred << 0,0,0,0;
      x_cor <<  0,0,0,0;
      K << 0,0, 0,0, 0,0, 0,0;
      S << 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0;
      z << 0.0 , 0.0;

      x_f_ = 0.0;
      y_f_ = 0.0;

  }

  void guidingStatusCallback(const pepper_navigation::guiding::ConstPtr& msg){
      guiding_ = msg->value;
      if(guiding_ && dist_ > 1.0 && prog_ < 0.85 && !partnerLost_) {
          partnerControl();
      } else {
          turnFront();
      }
  }

  void partnerControl() {
      if(uncertainty_ > 10) {
          probablyLost();
          partnerDetected_ = false;
      } else {
          triggered_once_  = false;
          partnerDetected_ = true;
      }
  }

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
      left_x_ = 0.0; 
      left_y_ = 0.0; 

      // LEFT LASER
      double left_dist = 0.0;
      int count = 0;
      double sum = 0.0;
      double ind = 0.0;
      double left_ang = 0.0;
      double left_ind = 0.0;

      for(int k = 46; k <= 60; k++){
          if(msg->ranges[k]<1.2){
              ind = ind + k-46;     //0-14
              count++;
              sum = sum + msg->ranges[k];
          }
      }

      if(count!=0){
          left_ind = ind/count;
          left_dist = sum/(count+1) + 0.10;
          left_ang = 1.04719 + (left_ind *0.0698); // 60 + i*4  
          // Results
          left_x_ = left_dist * cos(left_ang);
          left_y_ = left_dist * sin(left_ang);
          left_count_ = count;
          left_pose_prob_ = left_count_/9.0;
      }
      else {
          left_pose_prob_ = 0.0;
      }

     probabilityComputation(left_pose_prob_, sonar_pose_prob_);      

  }

  void probabilityComputation(const double& left_prob, const double& sonar_prob) {
      double x, y;      

      //Check which one looks closer to the human shape
      if(fabs(left_prob-1.0) < fabs(sonar_prob-1.0)) {
          x = left_x_;
          y = left_y_;
          pose_prob_ = left_pose_prob_;
          prob_side_ = 1;  
      }
      else if(fabs(left_prob-1.0) > fabs(sonar_prob-1.0)){
          x = -sonar_;
          y = 0.2;
          pose_prob_ = sonar_pose_prob_;
          prob_side_ = 0;  // Turn left has the highest probability
      }
      else{
          pose_prob_ = 0.0;
          x = 0.0;
          y = 0.0;
      }
          publishLaserPartnerPose(x, y);
          filterPosition(x, y);
  }

  void publishLaserPartnerPose(double& x, double& y){
      // For visualization only: Publish the position as a point
      geometry_msgs::PointStamped point;
      point.header.stamp = ros::Time::now();
      point.header.frame_id = "base_link";
      point.point.x = x ;
      point.point.y = y;
      point.point.z = 0.0;
      partner_point_pub_.publish(point);

    }

  void sonarCallback(const sensor_msgs::Range::ConstPtr& msg) {
      double limit = 1.0;      
      if(msg->range < limit){
          sonar_ = msg->range;
          sonar_pose_prob_ = fabs(limit - msg->range); // closer, best chance is the Partner
      }else{
          sonar_ = 0.0;
          sonar_pose_prob_ = 0.0;
      }
  }  

  void probablyLost(){
      if (!triggered_once_ && !partnerLost_) {
          getRobotPose();
          partnerLost();
          triggered_once_ = true;
          posture_timer_.stop();
      }
  }

  void postureTimerCallback(const ros::TimerEvent& event){
      say_once_ = false;
      if (!headTurningLeft_){
          correctPosture();
      } 
  }

  void resetTimerCallback(const ros::TimerEvent& event){
      say_once_ = false;
      reset_timer_.stop();
  }

  void partnerLost(){
      if(guiding_ && !partnerLost_) {

          // Publish topic and change server parameter
          partnerDetected_ = false;
          partnerLost_ = true;
          nh_.setParam("/partner_lost", partnerLost_);

          search();
          talk("Seems I lost you?");
          turnFront();

      	  pepper_navigation::lost msg;
          msg.condition = partnerLost_;
          partner_lost_pub_.publish(msg);

          ros::Duration(1.0).sleep();
          partnerLostClient_ lostClient("partner_lost_server", true);
          lostClient.waitForServer();
          ROS_DEBUG("sfm_planner: Partner_Lost server started, sending goal.");
          lostClient.sendGoal(last_pose_);
      }
  }

  void search(){
      nh_.setParam("/partner_side", prob_side_);
      if(prob_side_==1){turnLeft(); ros::Duration(1.0).sleep(); turnFront();}
  }

  void getRobotPose(){
      robot_pose_ = getRobotTransform("map","base_link");
      last_pose_.target_pose.header.frame_id = robot_pose_.frame_id_;
      last_pose_.target_pose.header.stamp =    ros::Time(0);
      last_pose_.target_pose.pose.position.x = robot_pose_.getOrigin().x();
      last_pose_.target_pose.pose.position.y = robot_pose_.getOrigin().y();
      last_pose_.target_pose.pose.position.z = 0.0;
      
      double robot_th = tf::getYaw(robot_pose_.getRotation());
      double final_th;

      if(robot_th<0){
          final_th = robot_th + 3.14159 ;
      } else {
          final_th = robot_th - 3.14159 ;
      }
      last_pose_.target_pose.pose.orientation = getQuaterion(final_th);

  }

  void distanceCallback(const pepper_navigation::distance::ConstPtr& msg) {
      dist_ = msg->value;
  } 

  void progressCallback(const pepper_navigation::progress::ConstPtr& msg) {
      prog_ = msg->value;
      if(guiding_ && !partnerLost_) {
          interact();
      }
  }

  void interact() {
      if(!say_once_) {
          ros::Duration(1.5).sleep();
          if (prog_ > 0.40 && prog_ < 0.43){
              talk("We are not far away"); 
              say_once_ = true; 
              reset_timer_.start();}
          if (prog_ > 0.60 && prog_ < 0.63){
              talk("Only halfway to go"); 
              say_once_ = true; 
              reset_timer_.start();}
          if (prog_ > 0.83 && prog_ < 0.85){
              talk("Here we are");
              say_once_ = true; 
              reset_timer_.start();}
      }
  }

  void resetCallback(const std_msgs::String::ConstPtr& msg) {
      partnerDetected_  = true;
      partnerLost_      = false;
      triggered_once_   = false;
      posture_timer_.start();
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
          ROS_ERROR("Error on Transformation (getRobotTransform: check) : %s", ex.what());
      }
      return pose;
  }

  geometry_msgs::Quaternion getQuaterion(const double &rad){
      tf::Quaternion quaternion;
      quaternion = tf::createQuaternionFromYaw(rad);
      geometry_msgs::Quaternion qMsg;
      tf::quaternionTFToMsg(quaternion, qMsg);
      return qMsg;
  }

  void talk(const char* text) {
  	  std_msgs::String msg;
      msg.data = text;
      speech_pub_.publish(msg);
  }

  void turnLeft() {
      if(!headTurningLeft_) {
          naoqi_bridge_msgs::JointAnglesWithSpeed angles;
          angles.header.frame_id = "base_link";
          angles.header.stamp = ros::Time::now();
          angles.joint_names.push_back("HeadPitch");
          angles.joint_angles.push_back(-0.3);
          angles.joint_names.push_back("HeadYaw");
          angles.joint_angles.push_back(1.5);
          angles.speed = 0.2;
          joint_pub_.publish(angles);
          headTurningFront_ = false;
          headTurningLeft_  = true;
      }
  }

  void turnFront() {
      if(!headTurningFront_) {
          naoqi_bridge_msgs::JointAnglesWithSpeed angles;
          angles.header.frame_id = "base_link";
          angles.header.stamp = ros::Time::now();
          angles.joint_names.push_back("HeadPitch");
          angles.joint_angles.push_back(-0.3);
          angles.joint_names.push_back("HeadYaw");
          angles.joint_angles.push_back(0.0);
          angles.speed = 0.2;
          joint_pub_.publish(angles);
          headTurningFront_ = true;
          headTurningLeft_  = false;
      }
  }

  void correctPosture() {
      naoqi_bridge_msgs::JointAnglesWithSpeed angles;
      angles.header.frame_id = "base_link";
      angles.header.stamp = ros::Time::now();
      angles.joint_names.push_back("HeadPitch");
      angles.joint_angles.push_back(-0.2);
      angles.joint_names.push_back("HeadYaw");
      angles.joint_angles.push_back(0.0);
      angles.speed = 0.1;
      joint_pub_.publish(angles);
      headTurningFront_ = true;
      headTurningLeft_  = false;
  }

  void filterPosition(double& x_l,double& y_l){

      if (x_l == 0 && y_l == 0){               // no measurements
          // Prediction equations:
          x_pred = A*x_old;
          P_pred = A*P_old*A.transpose()+Q;
          // Update equations/Correction Step:
          S = H*P_pred*H.transpose() + R;
          K = P_pred * H.transpose()*S.inverse();
          x_cor = x_pred;               // take the measurements out
          P_cor = P_pred;
          // Save new state as old state for next iteration:
          x_old = x_cor;
          P_old = P_cor;
          x_f_ = x_cor[0];
          y_f_ = x_cor[1];
      }else{                                  //correct with measurements
          z << x_l,y_l;
          // Prediction equations:
          x_pred = A*x_old;
          P_pred = A*P_old*A.transpose()+Q;
          // Update equations/Correction Step:
          S = H*P_pred*H.transpose() + R;
          K = P_pred * H.transpose()*S.inverse();
          x_cor = x_pred + K * (z - (H*x_pred));
          P_cor = (I - K * H) * P_pred;
          // Save new state as old state for next iteration:
          x_old = x_cor;
          P_old = P_cor;
          x_f_ = x_cor[0];
          y_f_ = x_cor[1];
      }

      // Assign the current covariance
      uncertainty_ = P_old(0,0);

      // For visualization only: 
      //Publish the position as a point
      geometry_msgs::PointStamped partnerPoint;
      partnerPoint.header.stamp = ros::Time::now();
      partnerPoint.header.frame_id = "base_link";
      partnerPoint.point.x = x_f_;
      partnerPoint.point.y = y_f_;
      partnerPoint.point.z = 0.0;

      //Publish covariance as a dynamic circule around partner position 
      sensor_msgs::PointCloud pointCloud;
      pointCloud.header.frame_id = "base_link";
      pointCloud.header.stamp = ros::Time::now();

      //Visualization only:
      double num_points = 50;
      double mag;
      for(int i = 0; i < num_points; i++){
          geometry_msgs::Point32 helperPoint;
          mag =  0.005*uncertainty_;
          if (mag > 1.0){mag = 1.0;}
          helperPoint.x = mag*cos(i*(6.28/num_points)) + x_f_;
          helperPoint.y = mag*sin(i*(6.28/num_points)) + y_f_;
          helperPoint.z = 0.0;
          pointCloud.points.push_back(helperPoint);
      }

      // Clear info when its not guiding
      if(guiding_ && dist_ > 0.3 && prog_ < 0.85) {
          partner_pose_pub_.publish(partnerPoint);
          partner_cov_pub_.publish(pointCloud);
    }
  }

  private: 
    // Node
    ros::NodeHandle nh_; 
    ros::Publisher speech_pub_;
    ros::Publisher joint_pub_;
    ros::Publisher partner_pose_pub_;
    ros::Publisher partner_cov_pub_;
    ros::Publisher partner_point_pub_;
    ros::Publisher partner_lost_pub_;
    ros::Subscriber reset_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber guiding_sub_;
    ros::Subscriber robot_pose_sub_; 
    ros::Subscriber distance_sub_; 
    ros::Subscriber progress_sub_; 
    ros::Subscriber sonar_sub_;
    ros::Timer posture_timer_;
    ros::Timer reset_timer_;
    tf::TransformListener tf_;

    std::string global_frame_;
    std::string robot_base_frame_;

    tf::Stamped<tf::Pose> robot_pose_;
    pepper_navigation::partnerLostGoal last_pose_;

    // Probability Function
    double sonar_pose_prob_;
    double left_pose_prob_;
    double pose_prob_;
    double left_x_;
    double left_y_;
    double left_count_;
    double prog_;
    double dist_;
    double sonar_;
    double uncertainty_;
    int prob_side_;

    // Common parameters
    bool partnerDetected_;
    bool partnerLost_;
    bool headTurningFront_;
    bool headTurningLeft_;
    bool guiding_;
    bool say_once_;
    bool triggered_once_;

    typedef actionlib::SimpleActionClient<pepper_navigation::partnerLostAction> partnerLostClient_;

    // Kalman Filter
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;
    // System Equations
    Eigen::MatrixXd A;
    Eigen::MatrixXd H;
    Eigen::MatrixXd I;  
    // Initial values
    Eigen::MatrixXd P_old; 
    Eigen::MatrixXd P_pred; 
    Eigen::MatrixXd K;
    Eigen::MatrixXd P_cor;
    Eigen::MatrixXd S;
    Eigen::Vector4d x_cor;
    Eigen::Vector4d x_old; 
    Eigen::Vector4d x_pred;
    Eigen::Vector2d z;
    // Output
    double x_f_;
    double y_f_;
}; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine");
  PartnerCheck state;
  ros::spin();
  return 0;
}

