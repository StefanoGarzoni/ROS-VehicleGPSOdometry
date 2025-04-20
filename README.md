# Robotics-Projects
------------------------------------------------------- 
chat da cui ho preso le informazioni: https://chatgpt.com/share/6804f87a-ca24-8012-bd29-db79e5bc5a49

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

NOTE / Osservazioni/Problemi : 

sector_times pubblica 3 valori quindi divide in 3 settori
penso che i valori dovrebbero essere di più dato che dopo il terzo va avanti il rosbag
->ricontrollare che i metri di suddivisione siano giusti, 

gps_pose pubblica parecchi valori con 0...