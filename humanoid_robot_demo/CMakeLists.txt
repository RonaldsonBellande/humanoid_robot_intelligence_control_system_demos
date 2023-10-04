cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_demo)


if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    humanoid_robot_controller_msgs
    humanoid_robot_walking_module_msgs
    humanoid_robot_action_module_msgs
    cmake_modules
    humanoid_robot_math
    humanoid_robot_ball_detector
  )

  find_package(Boost REQUIRED COMPONENTS thread)
  find_package(Eigen3 REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
endif()


find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(
  YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(
  YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    humanoid_robot_controller_msgs
    humanoid_robot_walking_module_msgs
    humanoid_robot_action_module_msgs
    cmake_modules
    humanoid_robot_math
    humanoid_robot_ball_detector
    DEPENDS Boost EIGEN3
  )
else()
  ament_package()
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(
  op_demo_node
  src/demo_node.cpp
  src/soccer/soccer_demo.cpp
  src/soccer/ball_tracker.cpp
  src/soccer/ball_follower.cpp
  src/action/action_demo.cpp
  src/vision/vision_demo.cpp
  src/vision/face_tracker.cpp
)

add_dependencies(
  op_demo_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  op_demo_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(
  self_test_node
  src/test_node.cpp
  src/soccer/soccer_demo.cpp
  src/soccer/ball_tracker.cpp
  src/soccer/ball_follower.cpp
  src/action/action_demo.cpp
  src/vision/vision_demo.cpp
  src/vision/face_tracker.cpp
  src/test/button_test.cpp
  src/test/mic_test.cpp
)

add_dependencies(
  self_test_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  self_test_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(
  TARGETS op_demo_node self_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY data launch list
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
