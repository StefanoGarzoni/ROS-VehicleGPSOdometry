// ROS base
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/sector_times.h>

// Libreria per conversione coordinate
#include <GeographicLib/LocalCartesian.hpp>

ros::Publisher sector_pub;

// Converter per coordinate GPS → ENU
GeographicLib::LocalCartesian* geo_converter = nullptr;

// Stato interno
double last_x = 0.0, last_y = 0.0;
ros::Time sector_start_time;
double accumulated_speed = 0.0;
int speed_count = 0;
int current_sector = 0;
bool gps_ready = false;

double sector_length = 50.0; // metri

// Posizione attuale in ENU (aggiornata dalla callback GPS)
double current_x = 0.0;
double current_y = 0.0;

// Callback GPS → aggiorna posizione attuale
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    if (!geo_converter) {
        geo_converter = new GeographicLib::LocalCartesian(lat, lon, alt);
        sector_start_time = msg->header.stamp;
        gps_ready = true;
        return;
    }

    geo_converter->Forward(lat, lon, alt, current_x, current_y, std::ignore);
}

// Callback velocità → aggiorna media e tempo settore
void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (!gps_ready) return;

    double speed_kmh = msg->point.y;
    double speed = speed_kmh / 3.6; // m/s
    accumulated_speed += speed;
    speed_count++;

    // Calcola distanza dal punto di partenza del settore
    double dx = current_x - last_x;
    double dy = current_y - last_y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance >= sector_length) {
        ros::Time now = msg->header.stamp;
        double time = (now - sector_start_time).toSec();

        // Crea messaggio custom
        first_project::sector_times msg_out;
        msg_out.current_sector = current_sector;
        msg_out.current_sector_time = time;
        msg_out.current_sector_mean_speed = (speed_count > 0) ? accumulated_speed / speed_count : 0.0;

        sector_pub.publish(msg_out);

        // Passa al settore successivo
        current_sector++;
        last_x = current_x;
        last_y = current_y;
        sector_start_time = now;
        accumulated_speed = 0.0;
        speed_count = 0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    ros::NodeHandle nh;

    sector_pub = nh.advertise<first_project::sector_times>("/sector_times", 10);

    ros::Subscriber gps_sub = nh.subscribe("/swiftnav/front/gps_pose", 100, gpsCallback);
    ros::Subscriber speed_sub = nh.subscribe("/speedsteer", 100, speedCallback);

    ros::spin();
    return 0;
}
