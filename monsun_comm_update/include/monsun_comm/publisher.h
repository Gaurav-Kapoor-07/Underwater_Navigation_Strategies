#pragma once

#include <monsun_comm/buffer_send.h>
#include <monsun_comm/message_types.h>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <zmq.hpp>

#include <geometry_msgs/Vector3.h>
#include <monsun_msgs/DataTypesTest.h>
#include <monsun_msgs/GpsPath.h>
#include <monsun_msgs/GpsWaypoint.h>
#include <monsun_msgs/Xsens700Position.h>
#include <monsun_msgs/Xsens700Time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

namespace monsun_comm {

class MonsunPublisher {
public:
    MonsunPublisher(ros::NodeHandle const& handle, ros::NodeHandle const& privateHandle);

private:
    void init_zmq_publisher();
    void init_ros_pub_sub();

    void send_monsun_msg(SendBuffer const& buf);

    void heartbeat_cb(const ros::TimerEvent& event);
    void data_types_test_cb(monsun_msgs::DataTypesTest::ConstPtr const& msg);

    void xsens_imu_cb(geometry_msgs::Vector3ConstPtr const& msg);
    void xsens_gps_cb(monsun_msgs::Xsens700PositionConstPtr const& msg);
    void xsens_time_cb(monsun_msgs::Xsens700TimeConstPtr const& msg);

    void ms5803_temp_cb(std_msgs::Float64ConstPtr const& msg);
    void ms5803_press_cb(std_msgs::Float64ConstPtr const& msg);
    void ms5803_depth_cb(std_msgs::Float64ConstPtr const& msg);

    // void gui_waypoint_cb(monsun_msgs::GpsWaypointConstPtr const& msg);
    // void gui_path_cb(monsun_msgs::GpsPathConstPtr const& msg);

    // GPS waypoints
    void gps_waypoint_cb(monsun_msgs::GpsWaypointConstPtr const& msg);
    void gps_path_cb(monsun_msgs::GpsPathConstPtr const& msg);

    void bat_7v4_cb(std_msgs::Float64ConstPtr const& msg);
    void bat_11v1_cb(std_msgs::Float64ConstPtr const& msg);

    void heading_ctrl_heading_cb(std_msgs::Float64ConstPtr const& msg);
    void heading_ctrl_speed_cb(std_msgs::Float64ConstPtr const& msg);

    ros::NodeHandle nhs;
    ros::NodeHandle nhp;

    zmq::context_t ctx;
    zmq::socket_t zpub;

    int id = 0;

    int num_messages = 0;
    ros::Publisher debug_num_messages_com;
    ros::Time last_pub_time;

    // monsun_comm
    ros::Timer heartbeat_timer;
    ros::Subscriber data_types_test_com;

    // Xsens
    ros::Subscriber xsens_imu_com;
    ros::Subscriber xsens_pos_com;
    ros::Subscriber xsens_time_com;

    // MS5803
    ros::Subscriber ms5803_temp_com;
    ros::Subscriber ms5803_press_com;
    ros::Subscriber ms5803_depth_com;

    // // GUI
    // ros::Subscriber gui_waypoint_com;
    // ros::Subscriber gui_path_com;

    // GPS waypoints
    ros::Subscriber gps_waypoint_com;
    ros::Subscriber gps_path_com;

    // Battery Monitor
    ros::Subscriber bat_7v4_com;
    ros::Subscriber bat_11v1_com;

    // heading_ctrl
    ros::Subscriber heading_ctrl_heading;
    ros::Subscriber heading_ctrl_speed;
};

} // namespace monsun_comm
