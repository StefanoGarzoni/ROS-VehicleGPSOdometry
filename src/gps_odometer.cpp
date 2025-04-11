// Inclusione delle librerie base ROS
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

// Libreria per conversioni geografiche
#include <GeographicLib/LocalCartesian.hpp>

ros::Publisher gps_odom_pub; // Publisher per l'odometria GPS
tf::TransformBroadcaster* tf_broadcaster; // Broadcaster TF

// Oggetto per convertire da lat/lon/alt a coordinate ENU
GeographicLib::LocalCartesian* geo_converter = nullptr;

// Stato precedente per calcolare heading
double last_x = 0.0;
double last_y = 0.0;
bool is_first_fix = true;

// Callback per i messaggi GPS
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // Estrai latitudine, longitudine e altitudine dal messaggio
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    ros::Time current_time = msg->header.stamp;

    // Se è il primo messaggio, lo uso come riferimento per il sistema ENU
    if (is_first_fix) {
        geo_converter = new GeographicLib::LocalCartesian(lat, lon, alt);
        is_first_fix = false;
        last_x = last_y = 0.0; // la posizione relativa iniziale è (0, 0)
        return;
    }

    // Converto in coordinate locali ENU
    double x, y, z;
    geo_converter->Forward(lat, lon, alt, x, y, z);

    // Calcolo l'orientamento (yaw) basato su differenza tra pose
    double dx = x - last_x;
    double dy = y - last_y;

    double yaw = atan2(dy, dx); // orientamento del veicolo (in radianti)
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(yaw);

    // Costruisco il messaggio di odometria
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0; // ignoriamo la quota

    odom.pose.pose.orientation = orientation;

    odom.child_frame_id = "gps";
    // twist non disponibile: il GPS fornisce solo posizione

    gps_odom_pub.publish(odom); // Pubblico odometria GPS

    // Trasformazione TF (odom → gps)
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "gps";
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = orientation;

    tf_broadcaster->sendTransform(transform); // Invia la trasformazione

    // Salva per il prossimo step
    last_x = x;
    last_y = y;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    ros::NodeHandle nh;

    // Publisher e broadcaster
    gps_odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 10);
    tf_broadcaster = new tf::TransformBroadcaster;

    // Sottoscrizione al GPS
    ros::Subscriber gps_sub = nh.subscribe("/swiftnav/front/gps_pose", 100, gpsCallback);

    ros::spin(); // Loop principale
    return 0;
}
