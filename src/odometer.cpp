// Includo le librerie ROS base
#include <ros/ros.h>

// Include per i messaggi in arrivo: velocità e sterzo
#include <geometry_msgs/PointStamped.h>

// Include per il messaggio di odometria da pubblicare
#include <nav_msgs/Odometry.h>

// Include per trasmettere trasformazioni tf
#include <tf/transform_broadcaster.h>

// Parametri del veicolo (distanze in metri)
const double WHEELBASE = 1.765;   // distanza tra asse anteriore e posteriore

// Variabili globali per posizione e orientamento
double x = 0.0;      // posizione lungo asse X
double y = 0.0;      // posizione lungo asse Y
double theta = 0.0;  // orientamento (angolo rispetto all’asse X)

// Publisher per l'odometria
ros::Publisher odom_pub;

// Broadcaster per il TF (odom → vehicle)
tf::TransformBroadcaster* tf_broadcaster;

// Tempo dell’ultima misura, per calcolare il deltaT
ros::Time last_time;

// Callback chiamata ogni volta che arriva un messaggio su /speedsteer
void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    ros::Time current_time = msg->header.stamp;

    // Calcolo il tempo trascorso (in secondi)
    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    // Estraggo la velocità in km/h e la converto in m/s
    double speed_kmh = msg->point.y;
    double speed = speed_kmh / 3.6;

    // Estraggo l'angolo di sterzo in gradi e lo converto in radianti
    double steer_deg = msg->point.x;
    double steer_rad = steer_deg * M_PI / 180.0;

    // Calcolo della velocità angolare (yaw rate)
    double omega = speed * tan(steer_rad) / WHEELBASE;

    // Integrazione delle equazioni di movimento (modello bicicletta)
    x += speed * cos(theta) * dt;
    y += speed * sin(theta) * dt;
    theta += omega * dt;

    // Converto theta in quaternion per l’orientamento
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    // Preparo e invio il messaggio di trasformazione (TF)
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "vehicle";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    tf_broadcaster->sendTransform(odom_trans);

    // Creo il messaggio di odometria da pubblicare
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // Posizione e orientamento
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Frame di riferimento del robot
    odom.child_frame_id = "vehicle";

    // Velocità lineare e angolare
    odom.twist.twist.linear.x = speed;
    odom.twist.twist.angular.z = omega;

    // Pubblico il messaggio
    odom_pub.publish(odom);
}

int main(int argc, char** argv) {
    // Inizializzo il nodo ROS
    ros::init(argc, argv, "odometer");

    // Creo un NodeHandle per comunicare con ROS
    ros::NodeHandle nh;

    // Creo il publisher per /odom
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    // Inizializzo il broadcaster tf
    tf_broadcaster = new tf::TransformBroadcaster;

    // Imposto il tempo iniziale
    last_time = ros::Time::now();

    // Sottoscrizione al topic /speedsteer
    ros::Subscriber sub = nh.subscribe("/speedsteer", 100, speedsteerCallback);

    // Avvia il ciclo principale ROS
    ros::spin();

    return 0;
}
