# Robotics-Projects

Comandi di avvio:
cd catkin_ws
catkin_make
source devel/setup.bash
tmux new -s first_project
terminale 1: roscore
terminale 2: roslaunch first_project launch.launch  
terminale 3: rosbag play --clock project.bag 

Comandi utili:
rostopic echo -n5 /gps_odom
rostopic echo -n5 /sector_times
