cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  sector_times.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometer src/odometer.cpp)
add_executable(gps_odometer src/gps_odometer.cpp)
add_executable(sector_times src/sector_times.cpp)

add_dependencies(odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gps_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sector_times ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odometer ${catkin_LIBRARIES})
target_link_libraries(gps_odometer ${catkin_LIBRARIES})
target_link_libraries(sector_times ${catkin_LIBRARIES})
