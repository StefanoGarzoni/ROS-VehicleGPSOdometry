cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  message_generation
)

## Declare ROS messages
add_message_files(
  FILES
  secotor_times.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare this catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    message_runtime
)

## Specify include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Build odometer node
add_executable(odometer src/odometer.cpp)
add_dependencies(odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometer ${catkin_LIBRARIES})

## Build gps_odometer node
add_executable(gps_odometer src/gps_odometer.cpp)
add_dependencies(gps_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_odometer ${catkin_LIBRARIES})

## Build sector_times node
add_executable(sector_times src/sector_times.cpp)
add_dependencies(sector_times ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sector_times ${catkin_LIBRARIES})
