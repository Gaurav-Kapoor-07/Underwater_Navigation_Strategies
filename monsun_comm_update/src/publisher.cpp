#include <monsun_comm/portdef.h>
#include <monsun_comm/publisher.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <std_msgs/Int32.h>

namespace monsun_comm {
static void write(SendBuffer* buf, MsgHeader const& header);

MonsunPublisher::MonsunPublisher(ros::NodeHandle const& handle,
                                 ros::NodeHandle const& privateHandle)
    : nhs(handle), nhp(privateHandle), zpub(ctx, ZMQ_PUB)
{
    try {
        init_zmq_publisher();
    }
    catch(zmq::error_t& err) {
        ROS_FATAL("publisher: zmq socket error: %s", err.what());
        ros::shutdown();
        return;
    }
    init_ros_pub_sub();
    heartbeat_timer = handle.createTimer(ros::Duration(10.0), &MonsunPublisher::heartbeat_cb, this);
}

void MonsunPublisher::init_zmq_publisher()
{
    // do not block when sending
    zpub.setsockopt(ZMQ_SNDTIMEO, 0);
    // do not block on shutdown
    zpub.setsockopt(ZMQ_LINGER, 0);

    // set up monsun_comm publisher
    int port = nhp.param<int>("port", CommPort::PortPublisher);
    if(port <= 0)
        port = CommPort::PortPublisher;
    std::string ep = std::string("tcp://0.0.0.0:").append(std::to_string(port));
    zpub.bind(ep);
    ROS_INFO("publisher: publishing on: %s", ep.c_str());
}

void MonsunPublisher::init_ros_pub_sub()
{
    if(!nhp.getParam("id", id)) {
        ROS_WARN("publisher: no id given, using 0");
    }
    else if(id < 0 || id > INT32_MAX) {
        ROS_WARN("publisher: invalid id given, using 0");
        id = 0;
    }
    else {
        ROS_INFO("publisher: id: %i", id);
    }

    // init debug publishers
    debug_num_messages_com = nhp.advertise<std_msgs::Int32>("num_messages", 10);

    // Testing
    data_types_test_com = nhp.subscribe("data_types_test",
                                        10,
                                        &MonsunPublisher::data_types_test_cb,
                                        this,
                                        ros::TransportHints().tcpNoDelay());

    // Xsens
    xsens_imu_com = nhs.subscribe(nhp.param<std::string>("xsens_imu_topic", "/xsens/imu"),
                                  10,
                                  &MonsunPublisher::xsens_imu_cb,
                                  this,
                                  ros::TransportHints().tcpNoDelay());
    xsens_pos_com = nhs.subscribe(nhp.param<std::string>("xsens_pos_topic", "/xsens/gps"),
                                  10,
                                  &MonsunPublisher::xsens_gps_cb,
                                  this,
                                  ros::TransportHints().tcpNoDelay());
    xsens_time_com = nhs.subscribe(nhp.param<std::string>("xsens_time_topic", "/xsens/time"),
                                   10,
                                   &MonsunPublisher::xsens_time_cb,
                                   this,
                                   ros::TransportHints().tcpNoDelay());

    // MS 5803
    ms5803_temp_com =
        nhs.subscribe(nhp.param<std::string>("ms5803_temp_topic", "/ms5803/temperature"),
                      10,
                      &MonsunPublisher::ms5803_temp_cb,
                      this,
                      ros::TransportHints().tcpNoDelay());
    ms5803_press_com =
        nhs.subscribe(nhp.param<std::string>("ms5803_press_topic", "/ms5803/pressure"),
                      10,
                      &MonsunPublisher::ms5803_press_cb,
                      this,
                      ros::TransportHints().tcpNoDelay());
    ms5803_depth_com = nhs.subscribe(nhp.param<std::string>("ms5803_depth_topic", "/ms5803/depth"),
                                     10,
                                     &MonsunPublisher::ms5803_depth_cb,
                                     this,
                                     ros::TransportHints().tcpNoDelay());

    // // GUI
    // gui_waypoint_com = nhs.subscribe(nhp.param<std::string>("gui_waypoint_topic", "/ui/waypoint"),
    //                                  10,
    //                                  &MonsunPublisher::gui_waypoint_cb,
    //                                  this,
    //                                  ros::TransportHints().tcpNoDelay());
    // gui_path_com = nhs.subscribe(nhp.param<std::string>("gui_path_topic", "/ui/path"),
    //                              10,
    //                              &MonsunPublisher::gui_path_cb,
    //                              this,
    //                              ros::TransportHints().tcpNoDelay());

    // GPS waypoints
    gps_waypoint_com = nhs.subscribe(nhp.param<std::string>("gps_waypoint_topic", "/gps/waypoint"),
                                     10,
                                     &MonsunPublisher::gps_waypoint_cb,
                                     this,
                                     ros::TransportHints().tcpNoDelay());
    gps_path_com = nhs.subscribe(nhp.param<std::string>("gps_path_topic", "/gps/path"),
                                 10,
                                 &MonsunPublisher::gps_path_cb,
                                 this,
                                 ros::TransportHints().tcpNoDelay());

    // Battery Monitor
    bat_7v4_com = nhs.subscribe(nhp.param<std::string>("bat_7v4_topic", "/battery/bat7v4"),
                                10,
                                &MonsunPublisher::bat_7v4_cb,
                                this,
                                ros::TransportHints().tcpNoDelay());
    bat_11v1_com = nhs.subscribe(nhp.param<std::string>("bat_11v1_topic", "/battery/bat11v1"),
                                 10,
                                 &MonsunPublisher::bat_11v1_cb,
                                 this,
                                 ros::TransportHints().tcpNoDelay());

    // heading_ctrl
    heading_ctrl_heading = nhs.subscribe(nhp.param<std::string>("heading_ctrl_heading_topic", "/heading_ctrl/heading"),
                                10,
                                &MonsunPublisher::heading_ctrl_heading_cb,
                                this,
                                ros::TransportHints().tcpNoDelay());
    heading_ctrl_speed = nhs.subscribe(nhp.param<std::string>("heading_ctrl_speed_topic", "/heading_ctrl/speed"),
                                 10,
                                 &MonsunPublisher::heading_ctrl_speed_cb,
                                 this,
                                 ros::TransportHints().tcpNoDelay());
}

void MonsunPublisher::send_monsun_msg(SendBuffer const& buf)
{
    try {
        zpub.send(buf.data(), buf.len(), 0);
    }
    catch(zmq::error_t& err) {
        if(err.num() != EINTR)
            ROS_ERROR("publisher: zmq socket error: %s", err.what());
        return;
    }
    num_messages += 1;
    auto now = ros::Time::now();
    if((now - last_pub_time).toSec() > 10.0) {
        std_msgs::Int32 num_msg;
        num_msg.data = num_messages;
        debug_num_messages_com.publish(num_msg);
        last_pub_time = now;
    }
}

// Testing

void MonsunPublisher::data_types_test_cb(monsun_msgs::DataTypesTestConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = DataTypesTest;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

// Heartbeat

void MonsunPublisher::heartbeat_cb(const ros::TimerEvent& /*event*/)
{
    MsgHeader header;
    header.msgType = EmptyMsg;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    send_monsun_msg(buf);
}

// Xsens

void MonsunPublisher::xsens_imu_cb(geometry_msgs::Vector3ConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = XsensImu;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

void MonsunPublisher::xsens_gps_cb(monsun_msgs::Xsens700PositionConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = XsensPosition;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

void MonsunPublisher::xsens_time_cb(monsun_msgs::Xsens700TimeConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = XsensTime;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

// Pressure Sensor

void MonsunPublisher::ms5803_temp_cb(std_msgs::Float64ConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = Ms5803Temp;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

void MonsunPublisher::ms5803_press_cb(std_msgs::Float64ConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = Ms5803Press;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

void MonsunPublisher::ms5803_depth_cb(std_msgs::Float64ConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = Ms5803Depth;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

// // GUI

// void MonsunPublisher::gui_waypoint_cb(monsun_msgs::GpsWaypointConstPtr const& msg)
// {
//     MsgHeader header;
//     header.msgType = GpsWaypoint;
//     header.source = id;
//     SendBuffer buf;
//     write(&buf, header);
//     buf.write_ros(*msg);
//     send_monsun_msg(buf);
// }

// void MonsunPublisher::gui_path_cb(monsun_msgs::GpsPathConstPtr const& msg)
// {
//     MsgHeader header;
//     header.msgType = GpsPath;
//     header.source = id;
//     SendBuffer buf;
//     write(&buf, header);
//     buf.write_ros(*msg);
//     send_monsun_msg(buf);
// }

// GPS waypoints

void MonsunPublisher::gps_waypoint_cb(monsun_msgs::GpsWaypointConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = GpsWaypoint;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

void MonsunPublisher::gps_path_cb(monsun_msgs::GpsPathConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = GpsPath;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

// Battery Monitor

void MonsunPublisher::bat_7v4_cb(std_msgs::Float64ConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = Bat7v4;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

void MonsunPublisher::bat_11v1_cb(std_msgs::Float64ConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = Bat11v1;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

// heading_ctrl

void MonsunPublisher::heading_ctrl_heading_cb(std_msgs::Float64ConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = HeadingCtrlHeading;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

void MonsunPublisher::heading_ctrl_speed_cb(std_msgs::Float64ConstPtr const& msg)
{
    MsgHeader header;
    header.msgType = HeadingCtrlSpeed;
    header.source = id;
    SendBuffer buf;
    write(&buf, header);
    buf.write_ros(*msg);
    send_monsun_msg(buf);
}

static void write(SendBuffer* buf, MsgHeader const& header)
{
    buf->write(header.version);
    buf->write(header.msgType);
    buf->write(header.source);
    buf->write(header.relais);
    buf->write(header.origin);
}

} // namespace monsun_comm

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "monsun_publisher");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    monsun_comm::MonsunPublisher mnPub(n, np);
    ROS_INFO("monsun_publisher started");
    ros::spin();
}
