#include <heading_ctrl_update/heading_ctrl_update.h>

#include <ros/console.h>
#include <ros/node_handle.h>

namespace heading_ctrl {

HeadingCtrl::HeadingCtrl(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle)
    : nhs(handle), nhp(private_handle)
{
    debug_en_pub = nhp.advertise<std_msgs::Int32>("debug/enabled", 1);

    // send commands to engine
    {
        std::string surge_topic;
        if(nhp.getParam("surge_topic", surge_topic)) {
            surge_pub = nhs.advertise<geometry_msgs::Twist>(surge_topic, 1);
        }
        else {
            surge_pub = nhp.advertise<geometry_msgs::Twist>("surge", 1);
        }
    }
    // // forward depth sp to depth controller
    // {
    //     std::string depth_sp_topic;
    //     if(nhp.getParam("depth_sp", depth_sp_topic)) {
    //         depth_ctrl_ref_pub = nhs.advertise<std_msgs::Float64>(depth_sp_topic, 1);
    //     }
    //     // TODO(Uli) simplify: use default, always call nhs.advertise
    //     else {
    //         depth_ctrl_ref_pub = nhp.advertise<std_msgs::Float64>("/depth_ctrl/depth", 1);
    //     }
    // }
    // // enable depth ctrl when needed
    // depth_ctrl_enable_cl = nhs.serviceClient<std_srvs::SetBool>(
    //     nhp.param<std::string>("depth_ctrl_en_srv", "/depth_ctrl/enable"));

    // measurement subscriber
    heading_sensor_sub = nhs.subscribe(nhp.param<std::string>("imu_topic", "/xsens/imu"),
                                       1,
                                       &HeadingCtrl::ctrl_cb,
                                       this,
                                       ros::TransportHints().tcpNoDelay());

    // reference subscribers
    heading_ref_sub = nhp.subscribe(
        "heading", 1, &HeadingCtrl::heading_cb, this, ros::TransportHints().tcpNoDelay());
    // depth_ref_sub =
    //     nhp.subscribe("depth", 1, &HeadingCtrl::depth_cb, this, ros::TransportHints().tcpNoDelay());
    speed_ref_sub =
        nhp.subscribe("speed", 1, &HeadingCtrl::speed_cb, this, ros::TransportHints().tcpNoDelay());

    // enable
    enable_ss = nhp.advertiseService("enable", &HeadingCtrl::enable_cb, this);

    // set controller parameters
    nhp.param<f64>("gain_heading", gain_heading_, 0.01667);

    // Timeout feature

    last_timepoint_timeout_s = ros::Time(0, 0);

    nhp.getParam("timeout_s", timeout_s);
}

/// Main heading control loop.
/// This callback is triggered at 10 Hz, the rate of the IMU measurements. It
/// calculates the control signals and sends them to the engine.
void HeadingCtrl::ctrl_cb(geometry_msgs::Vector3ConstPtr const& meas)
{
    if(!enabled_) {
        return;
    }

    f64 e = ref.heading - meas->z;

    // normalize angle
    if(e <= -180.0) {
        e += 360;
    }
    else if(e > 180.0) {
        e -= 360;
    }

    // Limit thrust to the equivalent thrust at an error of 30 degrees
    // Robbe thruster: Kp = 0.01667 --> F_lim = 0.50
    // T100 thruster:  Kp = 0.00833 --> F_lim = 0.25
    if(e < -30.0)
        e = -30.0;
    else if(e > 30.0)
        e = 30.0;

    f64 thrust = gain_heading_ * e;

    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = ref.speed;
    cmd_msg.angular.z = thrust;
    surge_pub.publish(cmd_msg);

    // Timeout feature

    ros::Time now = ros::Time::now();

    dt_timeout_s = (now - last_timepoint_timeout_s).toSec();

    if (last_timepoint_timeout_s == ros::Time(0, 0)) {
        // first run: just save timepoint to have dt in next run
        last_timepoint_timeout_s = now;
        return;
    }

    if (dt_timeout_s >= timeout_s) {
        ROS_INFO("heading_ctrl: Timeout, disabling!");
        enable(false);
        // stop motors
        geometry_msgs::Twist cmd_msg;
        surge_pub.publish(cmd_msg);
        // // stop depth controller
        // if(!depth_ctrl_en(false)) {
        //     // Depth control is probably not running
        //     ROS_WARN("heading_ctrl: could not disable depth_ctrl");
        // }
        last_timepoint_timeout_s = ros::Time(0, 0);
        dt_timeout_s = 0.0;
    }
}

void HeadingCtrl::heading_cb(std_msgs::Float64ConstPtr const& msg)
{
    f64 setpoint = msg->data;
    if(setpoint <= -180.0) {
        setpoint += 360.0;
    }
    else if(setpoint > 180.0) {
        setpoint -= 360.0;
    }
    ref.heading = setpoint;
}

bool HeadingCtrl::enable_cb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
{
    res.success = true;

    // switch on
    if(req.data) {
        if(enabled_) {
            ROS_INFO("heading_ctrl is already on");
            return true;
        }
        // // start depth controller
        // if(!depth_ctrl_en(true)) {
        //     ROS_ERROR("heading_ctrl: could not enable depth_ctrl");
        //     res.success = false;
        // }
        // Enabling despite depth control not being available to allow
        // for easier testing
        enable(true);
        return true;
    }

    // switch off
    if(!enabled_) {
        ROS_INFO("heading_ctrl is already off");
        return true;
    }
    enable(false);
    // stop motors
    geometry_msgs::Twist cmd_msg;
    surge_pub.publish(cmd_msg);
    // // stop depth controller
    // if(!depth_ctrl_en(false)) {
    //     // Depth control is probably not running
    //     ROS_WARN("heading_ctrl: could not disable depth_ctrl");
    // }
    return true;
}

void HeadingCtrl::enable(bool enable)
{
    enabled_ = enable;
    if(enable) {
        ROS_INFO("heading_ctrl on");
    }
    else {
        ROS_INFO("heading_ctrl off");
    }
    std_msgs::Int32 en_msg;
    en_msg.data = enable ? 1 : 0;
    debug_en_pub.publish(en_msg);
}

// bool HeadingCtrl::depth_ctrl_en(bool enable)
// {
//     std_srvs::SetBool depth_en;
//     depth_en.request.data = enable;
//     if(!depth_ctrl_enable_cl.call(depth_en) || !depth_en.response.success) {
//         return false;
//     }
//     return true;
// }

} // namespace heading_ctrl

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "heading_ctrl");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ROS_INFO("heading_ctrl started");
    heading_ctrl::HeadingCtrl hc(n, np);
    while(ros::ok()) { ros::spin(); }
}
