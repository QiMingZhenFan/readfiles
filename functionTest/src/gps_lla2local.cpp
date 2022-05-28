#include <iostream>
#include <exception>
#include <cmath>
#include <string>
#include <thread>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace GeographicLib;

/*
int main() {
  try {
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Geocentric& earth = Geocentric::WGS84();
    const double lat0 = 48 + 50/60.0, lon0 = 2 + 20/60.0; // Paris
    LocalCartesian proj(lat0, lon0, 0, earth);
    {
      // Sample forward calculation
      double lat = 50.9, lon = 1.8, h = 0; // Calais
      double x, y, z;
      proj.Forward(lat, lon, h, x, y, z);
      cout << x << " " << y << " " << z << "\n";
    }
    {
      // Sample reverse calculation
      double x = -38e3, y = 230e3, z = -4e3;
      double lat, lon, h;
      proj.Reverse(x, y, z, lat, lon, h);
      cout << lat << " " << lon << " " << h << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
*/

class GPS_Covert{
public:
    const Geocentric& earth = Geocentric::WGS84();
    ros::Subscriber subGPS;
    ros::Publisher  pubGPSOdometry;
    ros::NodeHandle &nh;
    string gps_topic;
    bool fisrt_gps_come = false;
    LocalCartesian proj;
    double x0 = 0, y0 = 0, z0 = 0;
    void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg){
        if(!fisrt_gps_come){
            //TODO check if reliable
            // proj.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);

            // use local centor in xodr file
            proj.Reset(30, 120, 0);
            proj.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, x0, y0, z0);
            fisrt_gps_come = true;
            return;
        }
        static double x = 0, y = 0, z = 0;
        proj.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, x, y, z);
        // ROS_INFO_STREAM("ori l: " << setprecision(10) << proj.LatitudeOrigin() << "\tl: " << proj.LongitudeOrigin() << "\ta: " << proj.HeightOrigin());
        // ROS_INFO_STREAM("now l: " << setprecision(10) << gpsMsg->latitude << "\tl: " << gpsMsg->longitude << "\ta: " << gpsMsg->altitude);
        // ROS_INFO_STREAM("deta l: " << setprecision(10) << gpsMsg->latitude - proj.LatitudeOrigin()
        //                                      << "\tl: " << gpsMsg->longitude - proj.LongitudeOrigin()
        //                                       << "\ta: " << gpsMsg->altitude - proj.HeightOrigin());
        // ROS_INFO_STREAM("x: " << x << "\ty: " << y << "\tz: " << z);
        nav_msgs::Odometry gpsOdom;
        gpsOdom.header.stamp = gpsMsg->header.stamp;
        gpsOdom.header.frame_id = "map";
        gpsOdom.child_frame_id = "gps";

        gpsOdom.pose.pose.position.x = x - x0;
        gpsOdom.pose.pose.position.y = y - y0;
        gpsOdom.pose.pose.position.z = z - z0;
        gpsOdom.pose.pose.orientation.w = 1.0;
        gpsOdom.pose.pose.orientation.x = 0.0;
        gpsOdom.pose.pose.orientation.y = 0.0;
        gpsOdom.pose.pose.orientation.z = 0.0;
        pubGPSOdometry.publish(gpsOdom);
    }

    GPS_Covert(ros::NodeHandle &h, string topic):nh(h), gps_topic(topic){
        subGPS = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 2000, &GPS_Covert::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        pubGPSOdometry = nh.advertise<nav_msgs::Odometry>("/gps_convert_odom", 20);
    }   

/*
    void Recv() {
        int a = 2;
        std::thread t(Process, a);
        std::thread t(Process, this, a);
        std::thread t(&Process, this, a);
        std::thread t(&GPS_Covert::Process, this, a);
        std::thread t(&(GPS_Covert::Process), this, a);
        // Do something
        t.join();
    }
    void Process(int val) {
        std::cout << "Process, val=" << val << "\n";
    }
*/
};


int main(int argc, char** argv){
    ros::init(argc, argv, "gps_lla2local");
    ros::NodeHandle nh;
    string gps_topic;

    if(nh.getParam("gps_topic", gps_topic) == false){
        ROS_ERROR_STREAM("please set gps_topic!");
        return -1;
    }

    GPS_Covert con(nh, gps_topic);
    ros::spin();
}