#include <iostream>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_goal_node");
  ros::NodeHandle node;
  //Change the correct topic move_base_simple/goal accordingly
  ros::Publisher goal_pub;
  goal_pub = node.advertise<geometry_msgs::PoseStamped>("robot_0/move_base_simple/goal", 100);

  while (ros::ok())
  {
    std::cout<<"Select a room please:\n";
    std::cout<<" 1. HB 12.030 \n";
    std::cout<<" 2. HB 12.040 \n";
    std::cout<<" 3. HB 12.050 \n";
    std::cout<<" 4. HB 12.060 \n";
    std::cout<<" 5. HB 12.070 \n";
    std::cout<<" 6. HB 12.080 \n";
    std::cout<<" 7. HB 12.090 \n";
    std::cout<<" 8. HB 12.100 \n";
    std::cout<<" 9. HB 12.110 \n";
    std::cout<<"10. HB 12.120 \n";
    std::cout<<"11. HB 12.130 \n";
    std::cout<<"12. HB 12.140 \n";
    std::cout<<"13. HB 12.150 \n";
    std::cout<<"14. HB 12.160 \n";
    std::cout<<"15. HB 12.230 \n";
    std::cout<<"16. HB 12.240 \n";
    std::cout<<"17. HB 12.250 \n";
    std::cout<<"18. HB 12.260 \n";
    std::cout<<"19. HB 12.270 \n";
    std::cout<<"20. HB 12.280 \n";
    std::cout<<"21. HB 12.290 \n";
    std::cout<<"22. HB 12.300 \n";
    std::cout<<"23. HB 12.310 \n";
    std::cout<<"24. HB 12.320 \n";

    int input;
    double x,y,z,rad;
    std::cin >> input;
   
    switch (input) {
    case 0:  //Home
      x=-8.00; y=0.0; z=0.0; rad=-2.7 ; break;
    case 1: //HB 12.030
      x=18.20; y=0.0; z=0.0; rad=1.57 ; break;
    case 2: //Hb 12.040
      x=14.15; y=0.0; z=0.0; rad=1.57 ; break;
    case 3: //HB 12.050
      x=10.08; y=0.0; z=0.0; rad=1.57 ; break;
    case 4: //HB 12.060
      x=6.01; y=0.0; z=0.0; rad=1.57 ; break;
    case 5: //HB 12.070
      x=1.98; y=0.0; z=0.0; rad=1.57 ; break;
    case 6: //HB 12.080
      x=-2.06; y=0.0; z=0.0; rad=1.57 ; break;
    case 7: //HB 12.090
      x=-6.14; y=0.0; z=0.0; rad=1.57 ; break;
    case 8: //HB 12.100
      x=-10.15; y=0.0; z=0.0; rad=1.57 ; break;
    case 9: //HB 12.110
      x=-14.20; y=0.0; z=0.0; rad=1.57 ; break;
    case 10: //HB 12.120
      x=-18.26; y=0.0; z=0.0; rad=1.57 ; break;
    case 11: //HB 12.130
      x=-22.34; y=0.0; z=0.0; rad=1.57 ; break;
    case 12: //HB 12.140
      x=-26.34; y=0.0; z=0.0; rad=1.57 ; break;
    case 13: //HB 12.150
      x=-30.41; y=0.0; z=0.0; rad=1.57 ; break;
    case 14: //HB 12.160
      x=-34.43; y=0.0; z=0.0; rad=1.57 ; break;
    //
    case 15: //HB 12.230
      x=34.44; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 16: //HB 12.240
      x=30.37; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 17: //HB 12.250
      x=26.35; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 18: //HB 12.260
      x=22.27; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 19: //HB 12.270
      x=18.21; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 20: //HB 12.280
      x=14.17; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 21: //HB 12.290
      x=10.14; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 22: //HB 12.300
      x=6.09; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 23: //HB 12.310
      x=2.02; y=-0.0; z=0.0; rad=-1.57 ; break;
    case 24: //HB 12.320
      x=-2.01; y=-0.0; z=0.0; rad=-1.57 ; break;
    default:
      std::cout<<"Error, bad input, quitting\n"; break;
    }

	  // Send a goal to move_base, 1st option
    geometry_msgs::PoseStamped goal_location;
    goal_location.header.stamp = ros::Time::now();
    goal_location.header.frame_id = "/map";  
    goal_location.pose.position.x=x;
    goal_location.pose.position.y=y;
    goal_location.pose.position.z=z;

	  // Convert the Euler angle to quaternion
	  tf::Quaternion quaternion;
	  quaternion = tf::createQuaternionFromYaw(rad);

	  geometry_msgs::Quaternion qMsg;
	  tf::quaternionTFToMsg(quaternion, qMsg);
	  goal_location.pose.orientation = qMsg;

    ROS_INFO("Published goal for room %d: %f %f %f", input, goal_location.pose.position.x, goal_location.pose.position.y, goal_location.pose.position.z); 

    ros::Rate loop_rate(10);
    goal_pub.publish(goal_location);

    ros::spinOnce();
    loop_rate.sleep();
  }
  //terminal example
  //rostopic pub /robot_0/move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 1, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'

  return 0;
}
