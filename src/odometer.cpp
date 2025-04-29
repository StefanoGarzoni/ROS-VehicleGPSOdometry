/*************************************************************
 * Nodo 1 – odometer
 * Calcola l'odometria integrando velocità (km/h) e angolo
 * sterzo volante (deg) del topic /speedsteer.
 * Pubblica nav_msgs/Odometry su /odom e trasforma TF
 * odom -> base_link (frame "vehicle").
 *************************************************************/

 #include <ros/ros.h>
 #include <geometry_msgs/PointStamped.h>
 #include <nav_msgs/Odometry.h>
 #include <tf2_ros/transform_broadcaster.h>
 #include <tf2/LinearMath/Quaternion.h>
 
 //costanti veicolo
 double WHEELBASE = 1.765;
 double STEER_FACTOR = 32.0;
 
 //Variabili stato odom
 double x = 0.0, y = 0.0, yaw = 0.0;
 ros::Time last_stamp;                 // per dt
 
 // publisher globale
 ros::Publisher odom_pub;
 tf2_ros::TransformBroadcaster* tf_broadcaster;
 
 // conversione deg → rad
 inline double deg2rad(double d) { return d * M_PI / 180.0; }
 
 // Callback su /speedsteer
 void speedsteerCB(const geometry_msgs::PointStamped::ConstPtr& msg)
 {
     // velocità km/h → m/s
     double speed_mps = msg->point.y * (1000.0/3600.0);
 
     // Angolo volante (deg) / fattore = angolo ruote, deg-> rad
     double delta = deg2rad(msg->point.x / STEER_FACTOR);
 
     // Se primo messaggio, inizializza e ritorna
     if (last_stamp.isZero()) { last_stamp = msg->header.stamp; return; }
 
     // dt
     double dt = (msg->header.stamp - last_stamp).toSec();
     last_stamp = msg->header.stamp;
 
     // Cinematica biciclo: v*dt, yaw_rate = v * tan(delta)/L
     double yaw_rate = speed_mps * tan(delta) / WHEELBASE;
     yaw += yaw_rate * dt;
 
     // Integrazione posizione in 2D
     x += speed_mps * cos(yaw) * dt;
     y += speed_mps * sin(yaw) * dt;
 
     //pubblicazione Odometry
     nav_msgs::Odometry odom;
     odom.header.stamp = msg->header.stamp;
     odom.header.frame_id = "odom";
     odom.child_frame_id = "base_link";
 
     odom.pose.pose.position.x = x;
     odom.pose.pose.position.y = y;
     odom.pose.pose.position.z = 0.0;
 
     tf2::Quaternion q;
     q.setRPY(0, 0, yaw);
     odom.pose.pose.orientation.x = q.x();
     odom.pose.pose.orientation.y = q.y();
     odom.pose.pose.orientation.z = q.z();
     odom.pose.pose.orientation.w = q.w();
 
     odom.twist.twist.linear.x = speed_mps;
     odom.twist.twist.angular.z = yaw_rate;
 
     odom_pub.publish(odom);
 
     //TF broadcast
     geometry_msgs::TransformStamped tf_msg;
     tf_msg.header = odom.header;
     tf_msg.child_frame_id = "base_link";
     tf_msg.transform.translation.x = x;
     tf_msg.transform.translation.y = y;
     tf_msg.transform.translation.z = 0.0;
     tf_msg.transform.rotation.x = q.x();
     tf_msg.transform.rotation.y = q.y();
     tf_msg.transform.rotation.z = q.z();
     tf_msg.transform.rotation.w = q.w();
     tf_broadcaster->sendTransform(tf_msg);
 }
 
 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "odometer");
     ros::NodeHandle nh("~");
 
     nh.param("wheelbase", WHEELBASE, WHEELBASE);
     nh.param("steer_factor", STEER_FACTOR, STEER_FACTOR);
  
     odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
 
     tf_broadcaster = new tf2_ros::TransformBroadcaster();
 
     ros::Subscriber sub = nh.subscribe("/speedsteer", 50, speedsteerCB);
     ROS_INFO_STREAM("odometer node started");
 
     ros::spin();
     return 0;
 }
 
