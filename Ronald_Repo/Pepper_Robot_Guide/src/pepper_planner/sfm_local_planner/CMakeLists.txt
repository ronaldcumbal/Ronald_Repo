cmake_minimum_required(VERSION 2.8.3)
project(sfm_local_planner)

find_package(catkin REQUIRED
            COMPONENTS
            nav_core
            base_local_planner
            costmap_2d
            roscpp
            tf
            nav_msgs
            pluginlib
            dynamic_reconfigure
            cmake_modules
#            pcl_conversions
        )

find_package(Eigen REQUIRED)
#find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
#    ${EIGEN3_INCLUDE_DIRS}
#    ${PCL_INCLUDE_DIRS}
    )

link_directories(${catkin_LIBRARY_DIRS} ${Eigen_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
    cfg/SFMPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES smf_local_planner
    CATKIN_DEPENDS
        pluginlib
        roscpp
        dynamic_reconfigure
    DEPENDS Eigen
)

add_library(sfm_local_planner src/sfm_planner.cpp)
target_link_libraries(sfm_local_planner ${catkin_LIBRARIES})

#add_dependencies(sfm_local_planner smf_local_planner_gencfg)
#add_dependencies(sfm_local_planner nav_msgs_generate_messages_cpp)

install(TARGETS sfm_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
