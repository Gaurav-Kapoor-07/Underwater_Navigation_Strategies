#include <ros/node_handle.h>
#include <ros/console.h>

#include <point_and_shoot/point_and_shoot.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "point_and_shoot_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ROS_INFO("point_and_shoot_node started");
    point_and_shoot::PointAndShoot hc(n, np);
    while (ros::ok()) {
        ros::spin();
    }
}
