cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Usa C++14
add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_generation
)

## ----- Messaggi custom -----
add_message_files(
  FILES
    sector_times.msg        # correzione del refuso 'secotor'
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

## ------- Executable: odometer -------
add_executable(odometer_node src/odometer.cpp)
add_dependencies(odometer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometer_node ${catkin_LIBRARIES})

## ------- Executable: gps_odometer -------
add_executable(gps_odometer_node src/gps_odometer.cpp)
add_dependencies(gps_odometer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_odometer_node ${catkin_LIBRARIES})

## ------- Executable: sector_times -------
add_executable(sector_times_node src/sector_times.cpp)
add_dependencies(sector_times_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sector_times_node ${catkin_LIBRARIES})
