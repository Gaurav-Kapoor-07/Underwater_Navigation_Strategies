#include <ros/node_handle.h>
#include <ros/console.h>

#include <point_and_shoot_surface/point_and_shoot_surface.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "point_and_shoot_surface_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ROS_INFO("point_and_shoot_surface: started");
    point_and_shoot_surface::PointAndShootSurface hc(n, np);
    while (ros::ok()) {
        ros::spin();
    }
}
