#pragma once

#include <monsun_util/types.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>

namespace heading_ctrl {

struct Reference {
    f64 heading = 0.0; // north
    f64 speed = 0.0;
};

class HeadingCtrl {
public:
    HeadingCtrl(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle);

private:
    void ctrl_cb(geometry_msgs::Vector3ConstPtr const& meas);

    void heading_cb(std_msgs::Float64ConstPtr const& msg);
    void speed_cb(std_msgs::Float64ConstPtr const& msg) { ref.speed = msg->data; }
    // void depth_cb(std_msgs::Float64ConstPtr const& msg) { depth_ctrl_ref_pub.publish(*msg); }

    bool enable_cb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);
    void enable(bool enable);
    // bool depth_ctrl_en(bool enable);

    ros::NodeHandle nhs;
    ros::NodeHandle nhp;

    ros::Publisher debug_en_pub;
    ros::Publisher surge_pub;
    // ros::Publisher depth_ctrl_ref_pub;
    // ros::ServiceClient depth_ctrl_enable_cl;

    ros::Subscriber heading_sensor_sub;

    ros::Subscriber heading_ref_sub;
    ros::Subscriber speed_ref_sub;
    // ros::Subscriber depth_ref_sub;

    ros::ServiceServer enable_ss;
    bool enabled_ = false;

    Reference ref;
    f64 gain_heading_ = 0.0;

    // // Timeout feature

    // ros::Time last_timepoint_timeout_s;
    // f64 timeout_s = 60.0;
    // f64 dt_timeout_s = 0.0;
};

} // namespace heading_ctrl
