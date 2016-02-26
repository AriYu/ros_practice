cmake_minimum_required(VERSION 2.8.3)
project(interactive_marker_tf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  cmake_modules
  interactive_markers
  visualization_msgs
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${PCL_COMMON_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp interactive_markers visualization_msgs tf 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  )


## Declare a C++ executable
add_executable(interactive_marker_tf src/interactive_marker_tf.cpp)
target_link_libraries(interactive_marker_tf ${catkin_LIBRARIES})

