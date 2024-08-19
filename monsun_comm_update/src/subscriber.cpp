#include <monsun_comm/portdef.h>
#include <monsun_comm/subscriber.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <std_msgs/Int32.h>

// ROS message definitions
#include <geometry_msgs/Vector3.h>
#include <monsun_msgs/DataTypesTest.h>
#include <monsun_msgs/GpsPath.h>
#include <monsun_msgs/GpsWaypoint.h>
#include <monsun_msgs/Xsens700Position.h>
#include <monsun_msgs/Xsens700Time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

namespace monsun_comm {
static bool read(MsgHeader* header, ReceiveBuffer& buf);

MonsunSubscriber::MonsunSubscriber(ros::NodeHandle n, zmq::context_t& ctx)
    : nhs(n), nhp("~"), zsub(ctx, zmq::socket_type::sub)
{
    // init debug publishers
    debug_num_messages_pub = nhp.advertise<std_msgs::Int32>("num_messages", 10);

    // subscribe to all ros messages sent via zmq
    zmq_subscribe();
}

void MonsunSubscriber::spin()
{
    while(ros::ok()) {
        zmq::message_t msg;
        bool recved = false;
        try {
            recved = zsub.recv(&msg, 0);
        }
        catch(zmq::error_t& err) {
            if(err.num() != EINTR) {
                ROS_FATAL("subscriber: recv error: %s", err.what());
                break; // stop spinning
            }
        }
        if(recved)
            handle_data(msg);
    }
}

/// Set up monsun_comm ZeroMQ subscriber. In case of an error shut down the node handles and quit.
void MonsunSubscriber::zmq_subscribe()
{
    // prepare config string (protocol + endpoint)
    std::string endpoint;
    if(!nhp.getParam("endpoint", endpoint) || endpoint.empty()) {
        ROS_WARN("subscriber: no endpoint given");
        endpoint = "127.0.0.1:";
        endpoint.append(std::to_string(PortPublisher));
    }
    else {
        // add a port number if none is given
        if(endpoint.find(':') == endpoint.npos)
            endpoint.append(":").append(std::to_string(PortPublisher));
    }
    std::string pub_address("tcp://");
    pub_address.append(endpoint);
    zsub.setsockopt(ZMQ_RCVTIMEO, 200);
    zsub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    zsub.connect(pub_address.c_str());
    ROS_INFO("subscriber: connecting to: %s", pub_address.c_str());
}

void MonsunSubscriber::handle_data(zmq::message_t const& msg)
{
    ReceiveBuffer buf(msg.data(), msg.size());

    MsgHeader header;
    if(!read(&header, buf)) {
        ROS_ERROR("subscriber: unexpected EOF in monsun_comm message header");
        return;
    }
    ROS_INFO_ONCE("subscriber: connection established");

    if(header.version != MN_COMM_VERS) {
        ROS_ERROR_ONCE("subscriber: message version mismatch (this: %u / other: %u)",
                       MN_COMM_VERS,
                       header.version);
        return;
    }

    if(header.relais) {
        ROS_INFO("subscriber: received relais message");
        return;
    }

    if(remote_handles.count(header.source) == 0) {
        ROS_INFO("subscriber: new connection: id %u", header.source);
        create_remote_handle(header.source);
    }

    RemoteHandle& h = remote_handles[header.source];
    switch(header.msgType) {
    case EmptyMsg:
        heartbeat_cb(buf, h);
        break;
    case DataTypesTest:
        data_types_test_cb(buf, h);
        break;
    // Xsens
    case XsensImu:
        xsens_imu_cb(buf, h);
        break;
    case XsensPosition:
        xsens_gps_cb(buf, h);
        break;
    case XsensTime:
        xsens_time_cb(buf, h);
        break;
    // MS5803
    case Ms5803Temp:
        ms5803_temp_cb(buf, h);
        break;
    case Ms5803Press:
        ms5803_press_cb(buf, h);
        break;
    case Ms5803Depth:
        ms5803_depth_cb(buf, h);
        break;
    // // GUI
    // case GpsWaypoint:
    //     gui_waypoint_cb(buf, h);
    //     break;
    // case GpsPath:
    //     gui_path_cb(buf, h);
    //     break;
    // case GpsArea:
    //     gui_area_cb(buf, h);
        // break;
    
    // GPS waypoints

    case GpsWaypoint:
        gps_waypoint_cb(buf, h);
        break;
    case GpsPath:
        gps_path_cb(buf, h);
        break;
    // case GpsArea:
    //     gps_area_cb(buf, h);
    //     break;

    // Battery
    case Bat7v4:
        bat_7v4_cb(buf, h);
        break;
    case Bat11v1:
        bat_11v1_cb(buf, h);
        break;

    // heading_ctrl
    case HeadingCtrlHeading:
        heading_ctrl_heading_cb(buf, h);
        break;
    case HeadingCtrlSpeed:
        heading_ctrl_speed_cb(buf, h);
        break;

    default:
        ROS_WARN("subscriber: unknown message type");
        return;
    }

    num_messages += 1;
    auto now = ros::Time::now();
    if((now - last_pub_time).toSec() > 10.0) {
        std_msgs::Int32 num_msg;
        num_msg.data = num_messages;
        debug_num_messages_pub.publish(num_msg);
        last_pub_time = now;
    }
}

void MonsunSubscriber::create_remote_handle(uint32_t id)
{
    // prepare republishing of remote ros topics
    // create new sub-namespace with the id of the remote monsun
    auto ns = std::string{};
    if(id == 100) {
        // special case: the command and control computer has the ID 100
        ns = std::string("cc");
    }
    else {
        ns = std::string("mn").append(std::to_string(id));
    }

    remote_handles[id] = RemoteHandle{};
    auto& h = remote_handles[id];
    h.nhr = ros::NodeHandle(nhs, ns);

    // monsun publisher
    h.heartbeat_com = h.nhr.advertise<std_msgs::Empty>("monsun_publisher/heartbeat", 10);
    h.data_types_test_com =
        h.nhr.advertise<monsun_msgs::DataTypesTest>("monsun_publisher/data_types_test", 10);

    // Xsens
    h.xsens_imu_com = h.nhr.advertise<geometry_msgs::Vector3>("xsens/imu", 10);
    h.xsens_gps_com = h.nhr.advertise<monsun_msgs::Xsens700Position>("xsens/position", 10);
    h.xsens_time_com = h.nhr.advertise<monsun_msgs::Xsens700Time>("xsens/time", 10);

    // MS5803
    h.ms5803_temp_com = h.nhr.advertise<std_msgs::Float64>("ms5803/temperature", 10);
    h.ms5803_press_com = h.nhr.advertise<std_msgs::Float64>("ms5803/pressure", 10);
    h.ms5803_depth_com = h.nhr.advertise<std_msgs::Float64>("ms5803/depth", 10);

    // // GUI
    // h.gui_waypoint_com = h.nhr.advertise<monsun_msgs::GpsWaypoint>("ui/waypoint", 10);
    // h.gui_path_com = h.nhr.advertise<monsun_msgs::GpsPath>("ui/path", 10);
    // h.gui_area_com = h.nhr.advertise<monsun_msgs::GpsPath>("ui/area", 10);

    // GPS waypoints
    h.gps_waypoint_com = h.nhr.advertise<monsun_msgs::GpsWaypoint>("gps/waypoint", 10);
    h.gps_path_com = h.nhr.advertise<monsun_msgs::GpsPath>("gps/path", 10);
    // h.gps_area_com = h.nhr.advertise<monsun_msgs::GpsPath>("gps/area", 10);

    // Battery Monitor
    h.bat_7v4_com = h.nhr.advertise<std_msgs::Float64>("battery/bat7v4", 10);
    h.bat_11v1_com = h.nhr.advertise<std_msgs::Float64>("battery/bat11v1", 10);

    // heading_ctrl
    h.heading_ctrl_heading_com = h.nhr.advertise<std_msgs::Float64>("heading_ctrl/heading", 10);
    h.heading_ctrl_speed_com = h.nhr.advertise<std_msgs::Float64>("heading_ctrl/speed", 10);
}

void MonsunSubscriber::heartbeat_cb(ReceiveBuffer& /*unused*/, RemoteHandle& h)
{
    h.heartbeat_com.publish(std_msgs::Empty());
}

void MonsunSubscriber::data_types_test_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    monsun_msgs::DataTypesTest msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: data_types_test_cb");
        return;
    }
    h.data_types_test_com.publish(msg);
}

void MonsunSubscriber::xsens_imu_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    geometry_msgs::Vector3 msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: xsens_imu_cb");
        return;
    }
    h.xsens_imu_com.publish(msg);
}

void MonsunSubscriber::xsens_gps_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    monsun_msgs::Xsens700Position msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: xsens_gps_cb");
        return;
    }
    h.xsens_gps_com.publish(msg);
}

void MonsunSubscriber::xsens_time_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    monsun_msgs::Xsens700Time msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: xsens_time_cb");
        return;
    }
    h.xsens_time_com.publish(msg);
}

void MonsunSubscriber::ms5803_temp_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    std_msgs::Float64 msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: ms5803_temp_cb");
        return;
    }
    h.ms5803_temp_com.publish(msg);
}

void MonsunSubscriber::ms5803_press_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    std_msgs::Float64 msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: ms5803_press_cb");
        return;
    }
    h.ms5803_press_com.publish(msg);
}

void MonsunSubscriber::ms5803_depth_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    std_msgs::Float64 msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: ms5803_depth_cb");
        return;
    }
    h.ms5803_depth_com.publish(msg);
}

// void MonsunSubscriber::gui_waypoint_cb(ReceiveBuffer& buf, RemoteHandle& h)
// {
//     monsun_msgs::GpsWaypoint msg;
//     if(!buf.read_ros(msg)) {
//         ROS_ERROR("deserialization error: gui_waypoint_cb");
//         return;
//     }
//     h.gui_waypoint_com.publish(msg);
// }

// void MonsunSubscriber::gui_path_cb(ReceiveBuffer& buf, RemoteHandle& h)
// {
//     monsun_msgs::GpsPath msg;
//     if(!buf.read_ros(msg)) {
//         ROS_ERROR("deserialization error: gui_path_cb");
//         return;
//     }
//     h.gui_path_com.publish(msg);
// }

// void MonsunSubscriber::gui_area_cb(ReceiveBuffer& buf, RemoteHandle& h)
// {
//     monsun_msgs::GpsPath msg;
//     if(!buf.read_ros(msg)) {
//         ROS_ERROR("deserialization error: gui_area_cb");
//         return;
//     }
//     h.gui_area_com.publish(msg);
// }

// GPS waypoints

void MonsunSubscriber::gps_waypoint_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    monsun_msgs::GpsWaypoint msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: gps_waypoint_cb");
        return;
    }
    h.gps_waypoint_com.publish(msg);
}

void MonsunSubscriber::gps_path_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    monsun_msgs::GpsPath msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: gps_path_cb");
        return;
    }
    h.gps_path_com.publish(msg);
}

void MonsunSubscriber::bat_7v4_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    std_msgs::Float64 msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: bat_7v4_cb");
        return;
    }
    h.bat_7v4_com.publish(msg);
}

void MonsunSubscriber::bat_11v1_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    std_msgs::Float64 msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: bat_11v1_cb");
        return;
    }
    h.bat_11v1_com.publish(msg);
}

// heading_ctrl

void MonsunSubscriber::heading_ctrl_heading_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    std_msgs::Float64 msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: heading_ctrl_heading_cb");
        return;
    }
    h.heading_ctrl_heading_com.publish(msg);
}

void MonsunSubscriber::heading_ctrl_speed_cb(ReceiveBuffer& buf, RemoteHandle& h)
{
    std_msgs::Float64 msg;
    if(!buf.read_ros(msg)) {
        ROS_ERROR("deserialization error: heading_ctrl_speed_cb");
        return;
    }
    h.heading_ctrl_speed_com.publish(msg);
}

static bool read(MsgHeader* header, ReceiveBuffer& buf)
{
    if(!buf.read(&header->version))
        return false;
    if(!buf.read(&header->msgType))
        return false;
    if(!buf.read(&header->source))
        return false;
    if(!buf.read(&header->relais))
        return false;
    if(!buf.read(&header->origin))
        return false;
    return true;
}

} // namespace monsun_comm

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "monsun_subscriber");
    ros::NodeHandle n;

    zmq::context_t ctx;

    monsun_comm::MonsunSubscriber mn_sub(n, ctx);
    ROS_INFO("monsun_subscriber started");
    mn_sub.spin();
}
