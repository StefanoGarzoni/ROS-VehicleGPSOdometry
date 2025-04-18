// File: sector_times.cpp

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/secotor_times.h>
#include <cmath>

// Stato velocià
double latest_speed_ms = 0.0;
bool speed_received = false;

// Parametri di circuito (da settare manualmente nello slide; qui esempio)
int num_sectors = 3;
double track_length = 300.0;           // Lunghezza [m]
double sector_length = track_length / num_sectors;

// Stato settore
int current_sector = 1;
ros::Time last_sector_time;
double cum_distance = 0.0;

// Parametri GPS→ENU
static const double a = 6378137.0, b = 6356752.0;
static const double e_sq = (a*a - b*b)/(a*a);
double lat_r=0, lon_r=0, alt_r=0, x_r=0, y_r=0, z_r=0;
bool ref_set = false;

void llaToECEF(double lat, double lon, double alt,
               double& x, double& y, double& z)
{
  double phi = lat * M_PI/180.0;
  double lambda = lon * M_PI/180.0;
  double N = a / std::sqrt(1 - e_sq * std::sin(phi)*std::sin(phi));
  x = (N + alt)*std::cos(phi)*std::cos(lambda);
  y = (N + alt)*std::cos(phi)*std::sin(lambda);
  z = ((b*b)/(a*a)*N + alt)*std::sin(phi);
}

void ecefToENU(double x, double y, double z,
               double& east, double& north, double& up)
{
  if (!ref_set) {
    lat_r = lat_r; lon_r = lon_r; alt_r = alt_r;
    llaToECEF(lat_r, lon_r, alt_r, x_r, y_r, z_r);
    ref_set = true;
  }
  double dx = x - x_r, dy = y - y_r, dz = z - z_r;
  double phi = lat_r*M_PI/180.0, lambda = lon_r*M_PI/180.0;
  double R[3][3] = {
    { -std::sin(lambda),  std::cos(lambda), 0 },
    { -std::sin(phi)*std::cos(lambda), -std::sin(phi)*std::sin(lambda), std::cos(phi) },
    {  std::cos(phi)*std::cos(lambda),  std::cos(phi)*std::sin(lambda), std::sin(phi) }
  };
  east  = R[0][0]*dx + R[0][1]*dy + R[0][2]*dz;
  north = R[1][0]*dx + R[1][1]*dy + R[1][2]*dz;
  up    = R[2][0]*dx + R[2][1]*dy + R[2][2]*dz;
}

ros::Publisher sector_pub;

void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  latest_speed_ms = msg->point.y * 1000.0 / 3600.0;
  speed_received = true;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  if (!speed_received) return;

  double x_e, y_e, z_e;
  llaToECEF(msg->latitude, msg->longitude, msg->altitude, x_e, y_e, z_e);
  double east, north, up;
  ecefToENU(x_e, y_e, z_e, east, north, up);

  static double prev_e=0, prev_n=0;
  static ros::Time prev_t;
  if (prev_t.isZero()) {
    prev_e = east; prev_n = north;
    prev_t = msg->header.stamp;
    last_sector_time = msg->header.stamp;
    return;
  }
  double dt = (msg->header.stamp - prev_t).toSec();
  double dx = east - prev_e, dy = north - prev_n;
  double dist = std::sqrt(dx*dx + dy*dy);
  cum_distance += dist;
  prev_e = east; prev_n = north; prev_t = msg->header.stamp;

  if (cum_distance >= sector_length * current_sector && current_sector <= num_sectors) {
    ros::Duration d = msg->header.stamp - last_sector_time;
    double sector_time = d.toSec();
    double mean_speed = sector_length / sector_time;
    first_project::secotor_times out;
    out.current_sector = current_sector;
    out.current_sector_time = sector_time;
    out.current_sector_mean_speed = mean_speed;
    sector_pub.publish(out);
    current_sector++;
    last_sector_time = msg->header.stamp;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sector_times");
  ros::NodeHandle nh;
  sector_pub = nh.advertise<first_project::secotor_times>("/sector_times", 10);
  ros::Subscriber s1 = nh.subscribe("/speedsteer", 10, speedCallback);
  ros::Subscriber s2 = nh.subscribe("/swiftnav/front/gps_pose", 10, gpsCallback);
  ros::spin();
  return 0;
}

/*
Modifiche apportate:
- Nessuna modifica strutturale: il nodo rispetta le slide
  (sottoscrive /speedsteer e /swiftnav/front/gps_pose,
   pubblica /sector_times con il custom message).
- I parametri num_sectors e track_length sono esempi di divisione in settori,
  poiché le slide non dettagliano la logica di suddivisione.
*/
