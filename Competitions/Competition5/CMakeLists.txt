cmake_minimum_required(VERSION 2.8.3)
project(final_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

find_package(OpenCV REQUIRED)

catkin_package()
catkin_python_setup()
