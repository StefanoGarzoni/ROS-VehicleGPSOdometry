// File: gps_odometer.cpp

// Include ROS core
#include <ros/ros.h>
// Messaggio NavSatFix per leggere GPS
#include <sensor_msgs/NavSatFix.h>
// Messaggio Odometry per pubblicare /gps_odom
#include <nav_msgs/Odometry.h>
// TF broadcaster per inviare il transform odom→gps
#include <tf/transform_broadcaster.h>
// Funzioni matematiche
#include <cmath>

// Parametri Terra per conversione geodetica
static const double a = 6378137.0;      // Semi-asse maggiore (m)
static const double b = 6356752.0;      // Semi-asse minore (m)
static const double e_sq = (a*a - b*b)/(a*a); // Eccentricità al quadrato

// Coordinate di riferimento geografiche (verranno inizializzate al primo fix)
double lat_r = 0.0, lon_r = 0.0, alt_r = 0.0;
// Coordinate di riferimento in ECEF (inizializzate in gps_to_odom)
double x_r = 0.0, y_r = 0.0, z_r = 0.0;
// Flag per riferimento inizializzato
bool ref_initialized = false;

// Publisher globale per /gps_odom
ros::Publisher gps_odom_pub;

// Converte lat/lon/alt → ECEF
void llaToECEF(double lat, double lon, double alt,
               double& x, double& y, double& z)
{
  double phi = lat * M_PI/180.0;
  double lambda = lon * M_PI/180.0;
  double N = a / std::sqrt(1 - e_sq * std::sin(phi)*std::sin(phi));
  x = (N + alt) * std::cos(phi) * std::cos(lambda);
  y = (N + alt) * std::cos(phi) * std::sin(lambda);
  z = ((b*b)/(a*a) * N + alt) * std::sin(phi);
}

// Funzione gps_to_odom: riceve NavSatFix e restituisce ENU + yaw
void gps_to_odom(const sensor_msgs::NavSatFix::ConstPtr& msg,
                 double& x_enu, double& y_enu, double& z_enu, double& yaw)
{
  double x_ecef, y_ecef, z_ecef;
  // 1) Lat/Lon/Alt → ECEF
  llaToECEF(msg->latitude, msg->longitude, msg->altitude,
            x_ecef, y_ecef, z_ecef);

  if (!ref_initialized) {
    // Salva il primo fix come punto di riferimento
    lat_r = msg->latitude;
    lon_r = msg->longitude;
    alt_r = msg->altitude;
    llaToECEF(lat_r, lon_r, alt_r, x_r, y_r, z_r);
    ref_initialized = true;
  }

  // 2) Differenziale ECEF relativo
  double dx = x_ecef - x_r;
  double dy = y_ecef - y_r;
  double dz = z_ecef - z_r;

  // 3) Costruzione matrice di rotazione ECEF→ENU
  double phi_r = lat_r * M_PI/180.0;
  double lambda_r = lon_r * M_PI/180.0;
  double R[3][3] = {
    { -std::sin(lambda_r),                std::cos(lambda_r),               0 },
    { -std::sin(phi_r)*std::cos(lambda_r), -std::sin(phi_r)*std::sin(lambda_r), std::cos(phi_r) },
    {  std::cos(phi_r)*std::cos(lambda_r),  std::cos(phi_r)*std::sin(lambda_r), std::sin(phi_r) }
  };
  // 4) Applicazione rotazione
  x_enu = R[0][0]*dx + R[0][1]*dy + R[0][2]*dz;
  y_enu = R[1][0]*dx + R[1][1]*dy + R[1][2]*dz;
  z_enu = R[2][0]*dx + R[2][1]*dy + R[2][2]*dz;

  // 5) Stima yaw dal delta tra posizioni successive (2D)
  static double prev_x = 0.0, prev_y = 0.0;
  static ros::Time prev_time;
  yaw = 0.0;
  if (!prev_time.isZero()) {
    double dt = (msg->header.stamp - prev_time).toSec();
    if (dt > 0.0) {
      yaw = std::atan2(y_enu - prev_y, x_enu - prev_x);
    }
  }
  prev_x = x_enu;
  prev_y = y_enu;
  prev_time = msg->header.stamp;
}

// Callback su /swiftnav/front/gps_pose
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  double x_enu, y_enu, z_enu, yaw;
  // Richiama la funzione gps_to_odom
  gps_to_odom(msg, x_enu, y_enu, z_enu, yaw);

  // Costruisce il messaggio Odometry
  nav_msgs::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "gps";
  odom.pose.pose.position.x = x_enu;
  odom.pose.pose.position.y = y_enu;
  odom.pose.pose.position.z = z_enu;
  // Quaternion da yaw
  geometry_msgs::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(yaw/2.0);
  q.w = std::cos(yaw/2.0);
  odom.pose.pose.orientation = q;
  // Twist lasciato a zero (solo posizione)
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
  // Pubblica su /gps_odom
  gps_odom_pub.publish(odom);

  // Invia TF odom→gps
  static tf::TransformBroadcaster br;
  tf::Transform t;
  t.setOrigin(tf::Vector3(x_enu, y_enu, z_enu));
  tf::Quaternion qt; qt.setRPY(0, 0, yaw);
  t.setRotation(qt);
  br.sendTransform(tf::StampedTransform(t, msg->header.stamp, "odom", "gps"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_odometer");           // Inizializza nodo
  ros::NodeHandle nh;                              // NodeHandle pubblico

  // Publisher per /gps_odom
  gps_odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 10);
  // Subscriber per il GPS frontale
  ros::Subscriber sub = nh.subscribe("/swiftnav/front/gps_pose", 10, gpsCallback);

  ros::spin();                                     // Loop ROS
  return 0;
}

/*
Modifiche apportate:
- Estratta la logica di conversione GPS→odometria in una funzione gps_to_odom() fedele alle slide,
  con parametri lat_r, lon_r, alt_r e innesco del riferimento al primo messaggio.
- Rimosso l’uso di parametri da launch file per lat_r/lon_r/alt_r: ora inizializzati dinamicamente
  al primo fix GPS come indicato (“set manually to the first value from GPS”).
- Il callback invoca gps_to_odom(), semplificando la struttura e riflettendo esattamente la slide.
- Child frame ID rinominato in "gps" per chiarire il TF odom→gps come richiesto.
*/
