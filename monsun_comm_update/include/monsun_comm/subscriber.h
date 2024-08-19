#pragma once

#include <map>

#include <monsun_comm/buffer_receive.h>
#include <monsun_comm/message_types.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <zmq.hpp>

namespace monsun_comm {

struct RemoteHandle {
    // used to republish remote topics
    ros::NodeHandle nhr;

    // monsun_comm
    ros::Publisher heartbeat_com;
    ros::Publisher data_types_test_com;

    // Xsens
    ros::Publisher xsens_imu_com;
    ros::Publisher xsens_gps_com;
    ros::Publisher xsens_time_com;

    // MS5803
    ros::Publisher ms5803_temp_com;
    ros::Publisher ms5803_press_com;
    ros::Publisher ms5803_depth_com;

    // GUI
    ros::Publisher gui_waypoint_com;
    ros::Publisher gui_path_com;
    ros::Publisher gui_area_com;

    // GPS waypoints
    ros::Publisher gps_waypoint_com;
    ros::Publisher gps_path_com;
    // ros::Publisher gps_area_com;

    // Battery monitor
    ros::Publisher bat_7v4_com;
    ros::Publisher bat_11v1_com;

    // heading_ctrl
    ros::Publisher heading_ctrl_heading_com;
    ros::Publisher heading_ctrl_speed_com;
};

class MonsunSubscriber {
public:
    MonsunSubscriber(ros::NodeHandle n, zmq::context_t& ctx);
    void spin();

private:
    void zmq_subscribe();
    void handle_data(zmq::message_t const& msg);

    void create_remote_handle(uint32_t id);

    void heartbeat_cb(ReceiveBuffer& /*unused*/, RemoteHandle& h);
    void data_types_test_cb(ReceiveBuffer& buf, RemoteHandle& h);

    void xsens_imu_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void xsens_gps_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void xsens_time_cb(ReceiveBuffer& buf, RemoteHandle& h);

    void ms5803_temp_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void ms5803_press_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void ms5803_depth_cb(ReceiveBuffer& buf, RemoteHandle& h);

    void gui_waypoint_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void gui_path_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void gui_area_cb(ReceiveBuffer& buf, RemoteHandle& h);

    // GPS waypoints
    
    void gps_waypoint_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void gps_path_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void gps_area_cb(ReceiveBuffer& buf, RemoteHandle& h);

    void bat_7v4_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void bat_11v1_cb(ReceiveBuffer& buf, RemoteHandle& h);

    // heading_ctrl

    void heading_ctrl_heading_cb(ReceiveBuffer& buf, RemoteHandle& h);
    void heading_ctrl_speed_cb(ReceiveBuffer& buf, RemoteHandle& h);

    ros::NodeHandle nhs; // public
    ros::NodeHandle nhp; // private

    zmq::socket_t zsub;

    int num_messages = 0;
    ros::Publisher debug_num_messages_pub;
    ros::Time last_pub_time;

    std::map<int, RemoteHandle> remote_handles;
};

} // namespace monsun_comm
