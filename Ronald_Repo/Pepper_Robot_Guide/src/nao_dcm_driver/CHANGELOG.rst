^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_dcm_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2016-10-14)
------------------
* Do not search naoqi libraries in ROS paths, go directly in SDK path
* Fix install rule.
  It made catkin_make fail on a system without NAOqi C++ sdk. The error was:
  -- Cannot find NAOqi C++ sdk; C++ nodes will NOT be built
  CMake Error at nao_dcm_robot/nao_dcm_driver/CMakeLists.txt:55 (install):
  install TARGETS given target nao_dcm_driver which does not exist in this
  directory.
* light nao_dcm to Control NAO without NAOqi's motion module
* Contributors: Sam Pfeiffer, margueda, sambrose
