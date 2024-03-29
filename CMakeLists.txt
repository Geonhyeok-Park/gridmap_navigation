cmake_minimum_required(VERSION 3.0.2)
project(gridmap_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  grid_map_core
  grid_map_ros
  grid_map_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  laser_geometry
  pcl_ros
)

find_package(PCL REQUIRED COMPONENTS common)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS 
  dynamic_reconfigure 
  grid_map_core 
  grid_map_ros 
  grid_map_msgs 
  message_generation 
  nav_msgs 
  roscpp 
  sensor_msgs 
  std_msgs 
  tf 
  tf2_ros
  laser_geometry
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_global
  src/global_planner/global_planner_ros.cpp 
  src/global_planner/dijkstra_search.cpp 
  src/global_planner/elastic_bands.cpp
)
add_library(${PROJECT_NAME}_local
  src/local_planner/local_planner_ros.cpp
  src/local_planner/dwa.cpp
)

add_library(${PROJECT_NAME}
  src/global_planner/costmap_ros.cpp
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(gridmap_global_planner 
 src//global_planner_ros_node.cpp 
 )

 add_executable(gridmap_local_planner
 src/local_planner_ros_node.cpp
 )

 add_executable(global_planner_node
 src/global_planner/global_planner_node.cpp
 )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(gridmap_nav_planner ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
 target_link_libraries(gridmap_global_planner
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}_global
 )

 target_link_libraries(gridmap_local_planner
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}_local
)

target_link_libraries(global_planner_node
${catkin_LIBRARIES}
${PROJECT_NAME}
)