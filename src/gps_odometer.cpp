/*************************************************************
 * Nodo 2 – gps_odometer
 * Converte NavSatFix in nav_msgs/Odometry via gps_to_odom:
 *   LLA -> ECEF -> ENU (referenza primo fix)
 * Stima heading dai vettori ENU consecutivi (2D).
 * Parametri: lat_r, lon_r, alt_r (se non passati, usa il primo fix).
 *************************************************************/

 #include <ros/ros.h>
 #include <sensor_msgs/NavSatFix.h>
 #include <nav_msgs/Odometry.h>
 #include <tf2_ros/transform_broadcaster.h>
 #include <tf2/LinearMath/Quaternion.h>
 
 // WGS‑84
 constexpr double a = 6378137.0;           // semi‑asse maggiore (m)
 constexpr double b = 6356752.0;           // semi‑asse minore (m)
 constexpr double e2 = (a*a - b*b) / (a*a);
 
 /// ----------- Conversioni -------------------------------
 struct ECEF { double x, y, z; };
 struct ENU  { double e, n, u; };
 
 /// gradi → radianti
 inline double d2r(double deg){ return deg*M_PI/180.0; }
 
 /// LLA -> ECEF (slide Formulas) :contentReference[oaicite:36]{index=36}&#8203;:contentReference[oaicite:37]{index=37}
 ECEF lla2ecef(double lat, double lon, double alt)
 {
     double phi = d2r(lat);
     double lambda = d2r(lon);
     double sin_phi = sin(phi);
     double cos_phi = cos(phi);
 
     double N = a / sqrt(1.0 - e2 * sin_phi*sin_phi);
     double x = (N + alt) * cos_phi * cos(lambda);
     double y = (N + alt) * cos_phi * sin(lambda);
     double z = (N*(1-e2) + alt) * sin_phi;
     return {x,y,z};
 }
 
 /// ECEF -> ENU (slide Formulas) :contentReference[oaicite:38]{index=38}&#8203;:contentReference[oaicite:39]{index=39}
 ENU ecef2enu(const ECEF& p, const ECEF& ref, double lat_r, double lon_r)
 {
     double phi = d2r(lat_r);
     double lambda = d2r(lon_r);
 
     double dx = p.x - ref.x;
     double dy = p.y - ref.y;
     double dz = p.z - ref.z;
 
     double sin_phi = sin(phi), cos_phi = cos(phi);
     double sin_lambda = sin(lambda), cos_lambda = cos(lambda);
 
     double e = -sin_lambda*dx + cos_lambda*dy;
     double n = -sin_phi*cos_lambda*dx - sin_phi*sin_lambda*dy + cos_phi*dz;
     double u =  cos_phi*cos_lambda*dx + cos_phi*sin_lambda*dy + sin_phi*dz;
     return {e,n,u};
 }
 
 /// -------------- Variabili globali -----------------------
 bool ref_ready = false;
 ECEF ref_ecef;
 double lat_r, lon_r, alt_r;    // parametri/valori ref
 
 double last_e=0, last_n=0;     // per heading
 bool first_pose = true;
 
 ros::Publisher gps_odom_pub;
 tf2_ros::TransformBroadcaster* tf_broadcaster;
 
 /// ----------- Funzione richiesta: gps_to_odom ------------
 void gps_to_odom(const sensor_msgs::NavSatFixConstPtr& msg)
 {
     // 1. Costruisce ECEF del fix
     ECEF ecef = lla2ecef(msg->latitude, msg->longitude, msg->altitude);
 
     // 2. Inizializza riferimento se necessario
     if(!ref_ready)
     {
         if(std::isnan(lat_r)) { lat_r = msg->latitude; }
         if(std::isnan(lon_r)) { lon_r = msg->longitude; }
         if(std::isnan(alt_r)) { alt_r = msg->altitude; }
         ref_ecef = lla2ecef(lat_r, lon_r, alt_r);
         ref_ready = true;
         ROS_INFO_STREAM("Reference LLA set to ["<<lat_r<<","<<lon_r<<","<<alt_r<<"]");
     }
 
     // 3. ECEF -> ENU
     ENU enu = ecef2enu(ecef, ref_ecef, lat_r, lon_r);
 
     // 4. Heading (solo dopo il primo punto)
     double heading = 0.0;
     if(!first_pose)
     {
         heading = atan2(enu.e - last_e, enu.n - last_n); // yaw rad
     }
     last_e = enu.e; last_n = enu.n; first_pose = false;
 
     // 5. Pubblica Odometry
     nav_msgs::Odometry odom;
     odom.header.stamp = msg->header.stamp;
     odom.header.frame_id = "odom";
     odom.child_frame_id = "gps_link";
 
     odom.pose.pose.position.x = enu.e;
     odom.pose.pose.position.y = enu.n;
     odom.pose.pose.position.z = enu.u;
 
     tf2::Quaternion q;
     q.setRPY(0,0,heading);
     odom.pose.pose.orientation.x = q.x();
     odom.pose.pose.orientation.y = q.y();
     odom.pose.pose.orientation.z = q.z();
     odom.pose.pose.orientation.w = q.w();
 
     gps_odom_pub.publish(odom);
 
     // TF
     geometry_msgs::TransformStamped tf_msg;
     tf_msg.header = odom.header;
     tf_msg.child_frame_id = "gps_link";
     tf_msg.transform.translation.x = enu.e;
     tf_msg.transform.translation.y = enu.n;
     tf_msg.transform.translation.z = enu.u;
     tf_msg.transform.rotation.x = q.x();
     tf_msg.transform.rotation.y = q.y();
     tf_msg.transform.rotation.z = q.z();
     tf_msg.transform.rotation.w = q.w();
     tf_broadcaster->sendTransform(tf_msg);
 }
 
 /// -------------- main ------------------------------------
 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "gps_odometer");
     ros::NodeHandle nh("~");  // namespace privato
 
     // parametri: se non forniti -> NaN -> usa primo fix
     nh.param("lat_r", lat_r, std::numeric_limits<double>::quiet_NaN());
     nh.param("lon_r", lon_r, std::numeric_limits<double>::quiet_NaN());
     nh.param("alt_r", alt_r, std::numeric_limits<double>::quiet_NaN());
 
     gps_odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom",10);
     tf_broadcaster = new tf2_ros::TransformBroadcaster();
 
     ros::Subscriber sub = nh.subscribe("/swiftnav/front/gps_pose", 50, gps_to_odom);
     ROS_INFO_STREAM("gps_odometer node started");
 
     ros::spin();
     return 0;
 }
 