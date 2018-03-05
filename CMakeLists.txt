cmake_minimum_required(VERSION 2.8.3)
project(humanoid_fsr)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES humanoid_fsr
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf 
  DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(humanoid_fsr src/humanoid_fsr_driver.cpp src/humanoid_fsr.cpp)
target_link_libraries(humanoid_fsr ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(humanoid_fsr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


