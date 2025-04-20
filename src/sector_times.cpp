/*************************************************************
 * Nodo 3 – sector_times
 * Divide il circuito in settori di lunghezza parametrica
 * (default 250 m) sul percorso ENU ricavato dal GPS.
 * Pubblica il messaggio custom sector_times.
 * NOTA: in assenza di way‑points reali nel bag, l'approccio
 * più generico è basarsi sulla distanza percorsa.
 *************************************************************/

 #include <ros/ros.h>
 #include <sensor_msgs/NavSatFix.h>
 #include <geometry_msgs/PointStamped.h>
 #include <first_project/sector_times.h>
 
 constexpr double DEFAULT_SECTOR_LENGTH = 250.0; // metri
 
 // Stato corrente
 double last_e = 0, last_n = 0;
 bool   first_fix = true;
 
 double sector_length;
 int    current_sector = 0;
 ros::Time sector_start_time;
 
 double accumulated_distance = 0.0;
 double accumulated_speed    = 0.0;
 int    speed_samples        = 0;
 
 ros::Publisher sector_pub;
 
 // ------- Helpers --------------------------------------------------
 inline double haversine2D(double dx, double dy)
 {
     return sqrt(dx*dx + dy*dy);
 }
 
 // Dichiarazione di funzione convertitore LLA -> ENU
 // Riutilizziamo la logica di gps_odometer: per semplicità
 // copiamo qui le minime funzioni (sole 2).
 struct ECEF { double x,y,z; }; struct ENU { double e,n; };
 constexpr double a = 6378137.0, b = 6356752.0;
 constexpr double e2 = (a*a - b*b)/(a*a);
 inline double d2r(double d){ return d*M_PI/180.0; }
 ECEF lla2ecef(double lat,double lon,double alt){
    double phi=d2r(lat),lam=d2r(lon);
    double sinp=sin(phi), cosp=cos(phi);
    double N=a/sqrt(1-e2*sinp*sinp);
    return {(N+alt)*cosp*cos(lam),(N+alt)*cosp*sin(lam),(N*(1-e2)+alt)*sinp};
 }
 ENU ecef2enu(const ECEF&p,const ECEF&r,double lat_r,double lon_r){
    double phi=d2r(lat_r),lam=d2r(lon_r);
    double dx=p.x-r.x,dy=p.y-r.y,dz=p.z-r.z;
    return {
      -sin(lam)*dx+cos(lam)*dy,
      -sin(phi)*cos(lam)*dx -sin(phi)*sin(lam)*dy +cos(phi)*dz
    };
 }
 
 // Riferimento
 bool ref_ready=false; ECEF ref_ecef; double lat_r,lon_r,alt_r;
 
 // ------- Callback GPS ------------------------------------------------
 void gpsCB(const sensor_msgs::NavSatFixConstPtr& msg)
 {
     // init ref
     if(!ref_ready){
        lat_r=msg->latitude; lon_r=msg->longitude; alt_r=msg->altitude;
        ref_ecef=lla2ecef(lat_r,lon_r,alt_r);
        ref_ready=true; sector_start_time = msg->header.stamp;
     }
 
     ECEF ecef=lla2ecef(msg->latitude,msg->longitude,msg->altitude);
     ENU enu=ecef2enu(ecef,ref_ecef,lat_r,lon_r);
 
     if(first_fix){
         last_e = enu.e; last_n = enu.n; first_fix=false; return;
     }
 
     double dx=enu.e-last_e, dy=enu.n-last_n;
     double dist = haversine2D(dx,dy);
     accumulated_distance += dist;
 
     last_e=enu.e; last_n=enu.n;
 }
 
 // ------- Callback velocità /speedsteer -----------------------------
 void speedCB(const geometry_msgs::PointStamped::ConstPtr& msg)
 {
     double speed_mps = msg->point.y * (1000.0/3600.0);
     accumulated_speed += speed_mps;
     speed_samples++;
 }
 
 // ------- Timer per controllo settore -------------------------------
 void timerCB(const ros::TimerEvent& ev)
 {
     if(accumulated_distance >= sector_length)
     {
         ros::Duration sector_time = ros::Time::now() - sector_start_time;
         double mean_speed = (speed_samples>0) ? accumulated_speed/speed_samples : 0.0;
 
         first_project::sector_times msg;
         msg.current_sector = current_sector;
         msg.current_sector_time = sector_time.toSec();
         msg.current_sector_mean_speed = mean_speed;
 
         sector_pub.publish(msg);
 
         // Reset contatori per il prossimo settore
         current_sector++;
         accumulated_distance = 0.0;
         accumulated_speed = 0.0;
         speed_samples = 0;
         sector_start_time = ros::Time::now();
     }
 }
 
 int main(int argc,char** argv)
 {
     ros::init(argc,argv,"sector_times");
     ros::NodeHandle nh("~");
 
     nh.param("sector_length", sector_length, DEFAULT_SECTOR_LENGTH);
 
     sector_pub = nh.advertise<first_project::sector_times>("/sector_times",10);
 
     ros::Subscriber gps_sub   = nh.subscribe("/swiftnav/front/gps_pose", 50, gpsCB);
     ros::Subscriber spd_sub   = nh.subscribe("/speedsteer", 50, speedCB);
 
     // Timer a 10 Hz per check settore
     ros::Timer t = nh.createTimer(ros::Duration(0.1), timerCB);
 
     ROS_INFO_STREAM("sector_times node started");
 
     ros::spin();
     return 0;
 }
 