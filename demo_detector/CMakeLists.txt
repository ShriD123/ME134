cmake_minimum_required(VERSION 2.8.3)
project(detector_demo)

## Set the package dependencies: Find catkin macros/libraries and
## other required packages.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  urdfdom_py
  message_generation)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

add_message_files(
  FILES
  SingleDetection.msg
  ManyDetections.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
## Uncomment if the package has a setup.py
# catkin_python_setup()

## Catkin configuration.  We don't have dependent projects.
catkin_package()

###########
## Build ##
###########

## Specify locations of header files
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
## Add cmake target dependencies of the executables

## Specify libraries to link a library or executable target against
