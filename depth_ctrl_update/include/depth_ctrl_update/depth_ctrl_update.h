#pragma once

#include <monsun_util/types.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>

struct CtrlGains {
    f64 heave = 0.0;
    f64 roll = 0.0; // roll, pitch are inverted
    f64 pitch = 0.0;
    f64 offset_heave = 0.0;
};

class Measurement {
public:
    f64 depth() { return depth_; }
    f64 roll() { return roll_; }
    f64 pitch() { return pitch_; }
    void set_depth(f64 const& depth)
    {
        depth_ = depth;
        new_depth = true;
    }
    void set_roll_pitch(f64 const& roll, f64 const& pitch)
    {
        roll_ = roll;
        pitch_ = pitch;
        new_att = true;
    }
    bool new_meas() { return new_depth && new_att; }
    void reset()
    {
        new_depth = false;
        new_att = false;
    }

private:
    f64 depth_ = 0.0;
    f64 roll_ = 0.0;
    f64 pitch_ = 0.0;
    bool new_depth = false;
    bool new_att = false;
};

class DepthCtrlNode {
public:
    DepthCtrlNode();

private:
    void depth_sensor_cb(std_msgs::Float64ConstPtr const& msg);
    void imu_sensor_cb(geometry_msgs::Vector3ConstPtr const& msg);
    void depth_sp_cb(std_msgs::Float64ConstPtr const& msg) { depth_sp = msg->data; }
    void fb_ctrl();

    bool enable_cb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

    ros::NodeHandle nhp; // private
    ros::NodeHandle nhs; // public

    ros::Publisher heave_pub;
    ros::Publisher debug_en_pub;
    ros::Publisher debug_dt_max_pub;
    ros::Publisher debug_dt_min_pub;

    ros::Subscriber depth_sensor_sub;
    ros::Subscriber imu_sensor_sub;
    ros::Subscriber depth_sp_sub;

    ros::ServiceServer enable_ss;
    bool enabled = false;

    ros::Time last_timepoint;
    f64 dt_max = std::numeric_limits<f64>::min();
    f64 dt_min = std::numeric_limits<f64>::max();

    CtrlGains gains;
    Measurement meas;
    f64 depth_sp = 0.0;

    ros::Time last_timepoint_timeout_s;
    f64 timeout_s = 60.0;
    f64 dt_timeout_s = 0.0;
};
