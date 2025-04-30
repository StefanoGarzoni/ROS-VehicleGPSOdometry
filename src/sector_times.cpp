/*************************************************************
 * Nodo 3 – sector_times
 * Calcola i tempi per ciascun settore di una pista basandosi
 * su dati GPS e velocità.
 *************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <first_project/sector_times.h>
#include <cmath>

constexpr double DEFAULT_FIRST_SECTOR_LENGTH = 1500.0; // metri
constexpr double DEFAULT_SECOND_SECTOR_LENGTH = 2000.0; // metri
constexpr double DEFAULT_THIRD_SECTOR_LENGTH = 2293.0; // metri

// Stato corrente
double last_e = 0;
double last_n = 0;
bool first_fix = true;

double current_sector_length;
double first_sector_length;
double second_sector_length;
double third_sector_length;
int current_sector = 1;
const int last_sector = 3;
ros::Time sector_start_time;

double accumulated_distance = 0.0;
double accumulated_speed = 0.0;
int speed_samples = 0;

ros::Publisher sector_pub;

// Helpers
inline double haversine2D(double dx, double dy) { return std::sqrt(dx * dx + dy * dy); }

struct ECEF { double x, y, z; };
struct ENU { double e, n, u; };

constexpr double a = 6378137.0;
constexpr double b = 6356752.0;
constexpr double e2 = (a * a - b * b) / (a * a);

// Conversioni:
inline double d2r(double deg) { return deg * M_PI / 180.0; }

ECEF lla2ecef(double lat, double lon, double alt) {
    double phi = d2r(lat);
    double lambda = d2r(lon);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);

    double N = a / std::sqrt(1.0 - e2 * sin_phi * sin_phi);
    double x = (N + alt) * cos_phi * cos(lambda);
    double y = (N + alt) * cos_phi * sin(lambda);
    double z = (N * (1 - e2) + alt) * sin_phi;
    return {x, y, z};
}

ENU ecef2enu(const ECEF& p, const ECEF& ref, double lat_r, double lon_r) {
    double phi = d2r(lat_r);
    double lambda = d2r(lon_r);

    double dx = p.x - ref.x;
    double dy = p.y - ref.y;
    double dz = p.z - ref.z;

    double sin_phi = sin(phi);
    double cos_phi = cos(phi);
    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);

    double e = -sin_lambda * dx + cos_lambda * dy;
    double n = -sin_phi * cos_lambda * dx - sin_phi * sin_lambda * dy + cos_phi * dz;
    double u = cos_phi * cos_lambda * dx + cos_phi * sin_lambda * dy + sin_phi * dz;

    return {e, n, u};
}

// Riferimento
bool ref_ready = false;
ECEF ref_ecef;
double lat_r, lon_r, alt_r;

// Callback GPS
void gpsCB(const sensor_msgs::NavSatFixConstPtr& msg) {
    // Inizializzazione riferimento
    if (!ref_ready) {
        lat_r = msg->latitude;
        lon_r = msg->longitude;
        alt_r = msg->altitude;

        ref_ecef = lla2ecef(lat_r, lon_r, alt_r);
        ref_ready = true;

        sector_start_time = msg->header.stamp;
        ROS_INFO_STREAM("Riferimento GPS inizializzato.");
    }

    ECEF ecef = lla2ecef(msg->latitude, msg->longitude, msg->altitude);
    ENU enu = ecef2enu(ecef, ref_ecef, lat_r, lon_r);

    if (first_fix) {
        last_e = enu.e;
        last_n = enu.n;
        first_fix = false;

        // Determina settore iniziale
        if (accumulated_distance < DEFAULT_FIRST_SECTOR_LENGTH) {
            current_sector = 1;
            current_sector_length = DEFAULT_FIRST_SECTOR_LENGTH;
        } else if (accumulated_distance < (DEFAULT_FIRST_SECTOR_LENGTH + DEFAULT_SECOND_SECTOR_LENGTH)) {
            current_sector = 2;
            current_sector_length = DEFAULT_SECOND_SECTOR_LENGTH;
        } else {
            current_sector = 3;
            current_sector_length = DEFAULT_THIRD_SECTOR_LENGTH;
        }

        ROS_INFO_STREAM("Settore iniziale determinato: " << current_sector);
        return;
    }

    double dx = enu.e - last_e;
    double dy = enu.n - last_n;
    double dist = haversine2D(dx, dy);
    accumulated_distance += dist;

    last_e = enu.e;
    last_n = enu.n;
}

// Callback velocità /speedsteer
void speedCB(const geometry_msgs::PointStamped::ConstPtr& msg) {
    double speed_mps = msg->point.y * (1000.0 / 3600.0);
    accumulated_speed += speed_mps;
    speed_samples++;
}

// Timer per il controllo del settore
void timerCB(const ros::TimerEvent&) {
    if (accumulated_distance >= current_sector_length) {
        ros::Duration sector_time = ros::Time::now() - sector_start_time;

        // Filtra tempi di settore troppo brevi
        if (sector_time.toSec() < 1.0) {
            ROS_WARN_STREAM("Tempo settore troppo breve, ignorato.");
            return;
        }

        double mean_speed = (speed_samples > 0) ? accumulated_speed / speed_samples : 0.0;

        first_project::sector_times msg;
        msg.current_sector = current_sector;
        msg.current_sector_time = sector_time.toSec();
        msg.current_sector_mean_speed = mean_speed;

        sector_pub.publish(msg);

        // Reset contatori per il prossimo settore
        if (current_sector == last_sector) {
            current_sector = 1;
        } else {
            current_sector++;
        }

        switch (current_sector) {
            case 1:
                current_sector_length = first_sector_length;
                break;
            case 2:
                current_sector_length = second_sector_length;
                break;
            case 3:
                current_sector_length = third_sector_length;
                break;
        }

        accumulated_distance = 0.0;
        accumulated_speed = 0.0;
        speed_samples = 0;
        sector_start_time = ros::Time::now();

        ROS_INFO_STREAM("Settore completato. Passaggio al settore: " << current_sector);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    ros::NodeHandle nh("~");

    nh.param("first_sector_length", first_sector_length, DEFAULT_FIRST_SECTOR_LENGTH);
    nh.param("second_sector_length", second_sector_length, DEFAULT_SECOND_SECTOR_LENGTH);
    nh.param("third_sector_length", third_sector_length, DEFAULT_THIRD_SECTOR_LENGTH);

    sector_pub = nh.advertise<first_project::sector_times>("/sector_times", 10);

    ros::Subscriber gps_sub = nh.subscribe("/swiftnav/front/gps_pose", 50, gpsCB);
    ros::Subscriber spd_sub = nh.subscribe("/speedsteer", 50, speedCB);

    // Timer a 10 Hz per il controllo dei settori
    ros::Timer t = nh.createTimer(ros::Duration(0.1), timerCB);

    ROS_INFO_STREAM("Nodo sector_times avviato.");

    ros::spin();
    return 0;
}
