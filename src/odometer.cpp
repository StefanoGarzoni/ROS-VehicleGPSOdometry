// File: odometer.cpp

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

// Parametri veicolo
static const double WHEELBASE = 1.765;       // Distanza asse anteriore-posteriore [m]
static const double STEERING_FACTOR = 32.0;  // Fattore di conversione sterzo

// Stato globale
double x = 0.0, y = 0.0, yaw = 0.0;
ros::Time last_time;
ros::Publisher odom_pub;

void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  double speed_ms = msg->point.y * 1000.0 / 3600.0;  // km/h→m/s
  double steering_rad = msg->point.x / STEERING_FACTOR * M_PI/180.0;
  double angular_velocity = speed_ms * std::tan(steering_rad) / WHEELBASE;

  if (last_time.isZero()) {
    last_time = msg->header.stamp;
    return;
  }
  double dt = (msg->header.stamp - last_time).toSec();
  last_time = msg->header.stamp;

  x += speed_ms * std::cos(yaw) * dt;
  y += speed_ms * std::sin(yaw) * dt;
  yaw += angular_velocity * dt;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "vehicle";
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  geometry_msgs::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(yaw/2.0);
  q.w = std::cos(yaw/2.0);
  odom_msg.pose.pose.orientation = q;
  odom_msg.twist.twist.linear.x = speed_ms;
  odom_msg.twist.twist.angular.z = angular_velocity;
  odom_pub.publish(odom_msg);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion qt; qt.setRPY(0,0,yaw);
  transform.setRotation(qt);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "vehicle"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometer");
  ros::NodeHandle nh;
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  ros::Subscriber sub = nh.subscribe("/speedsteer", 10, speedSteerCallback);
  ros::spin();
  return 0;
}

/*
Modifiche apportate:
- Nessuna: questo file implementa esattamente le indicazioni delle slide per il nodo 'odometer':
  sottoscrive /speedsteer, pubblica /odom di tipo nav_msgs/Odometry e trasforma odom→vehicle.
*/
