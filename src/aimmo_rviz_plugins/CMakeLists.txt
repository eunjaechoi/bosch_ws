cmake_minimum_required(VERSION 2.8.3)
project(aimmo_rviz_plugins)

# # Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Qt
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

# Ogre
find_package(PkgConfig)
pkg_check_modules(OGRE OGRE)

# Catkin
find_package(catkin REQUIRED COMPONENTS
  rviz
  roscpp
  tf
  pcl_ros
  pcl_msgs
)

catkin_package(
  DEPENDS
  rviz
  INCLUDE_DIRS
  common
  polar_grid_display
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  rviz
  tf
  pcl_ros
  pcl_msgs
)

# Include Directory
include_directories(
  common
  polar_grid_display
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Compile
file(GLOB_RECURSE COMMON_SRCS
  common/*.cpp
)

set(POLAR_GRID_DISPLAY_SRCS
  polar_grid_display/polar_grid_display.cpp
)

add_library(${PROJECT_NAME}
  ${COMMON_SRCS}
  ${POLAR_GRID_DISPLAY_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${rviz_DEFAULT_PLUGIN_LIBRARIES}
  ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
