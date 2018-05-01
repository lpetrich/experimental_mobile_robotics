cmake_minimum_required(VERSION 2.8.3)
project(game_navigation_layers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  dynamic_reconfigure
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/HidingLayer.cfg  
  cfg/SeekingLayer.cfg
)

catkin_package(
    CATKIN_DEPENDS costmap_2d dynamic_reconfigure
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## add cpp library
add_library(${PROJECT_NAME} 
            src/game_hiding_layer.cpp
            src/hiding_layer.cpp
            src/game_seeking_layer.cpp
            src/seeking_layer.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS game_navigation_layers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
