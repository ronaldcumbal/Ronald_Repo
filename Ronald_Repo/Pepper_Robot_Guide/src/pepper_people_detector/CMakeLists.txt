cmake_minimum_required(VERSION 2.8.3)
project(pepper_people_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PeoplePose.msg
  PeoplePoseArray.msg
  AwarenessState.msg
  HeadPose.msg
  WordRecognized.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pepper_people_detector
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime tf
)

#add_executable(speech_server src/speech_server.cpp)
#target_link_libraries(speech_server ${catkin_LIBRARIES})

#add_executable(partner_detector_sim src/partner_detector_sim.cpp)
#target_link_libraries(partner_detector_sim ${catkin_LIBRARIES})

#add_executable(people_detector_sim src/people_detector_sim.cpp)
#target_link_libraries(people_detector_sim ${catkin_LIBRARIES})

