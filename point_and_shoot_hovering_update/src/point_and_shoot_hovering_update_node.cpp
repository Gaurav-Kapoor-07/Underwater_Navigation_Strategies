#include <ros/node_handle.h>
#include <ros/console.h>
#include <point_and_shoot_hovering_update/point_and_shoot_hovering_update.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "point_and_shoot_hovering_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ROS_INFO("point_and_shoot_hovering: started");
    point_and_shoot_hovering::PointAndShootHovering hc(n, np);
    while (ros::ok()) {
        ros::spin();
    }
}
