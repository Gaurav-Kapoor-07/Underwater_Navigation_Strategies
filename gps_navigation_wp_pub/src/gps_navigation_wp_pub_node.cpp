#include <ros/node_handle.h>
#include <ros/console.h>

#include <gps_navigation/gps_navigation.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gps_navigation_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ROS_INFO("gps_navigation_wp_pub_node: started");
    gps_navigation::GPSNavigation hc(n, np);
    while (ros::ok()) {
        ros::spin();
    }
}
