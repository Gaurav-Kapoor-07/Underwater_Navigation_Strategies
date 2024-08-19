#include <depth_ctrl_update/depth_ctrl_update.h>

DepthCtrlNode::DepthCtrlNode()
    : nhp("~")
    , nhs()
{
    // read and set default parameters
    {
        f64 gain_heave;
        if (!nhp.getParam("gain_heave", gain_heave)) {
            ROS_FATAL("depth_ctrl: missing parameter gain_heave");
            ros::shutdown();
            return;
        }
        gains.heave = gain_heave;
    }
    {
        f64 gain_roll;
        if (!nhp.getParam("gain_roll", gain_roll)) {
            ROS_FATAL("depth_ctrl: missing parameter gain_roll");
            ros::shutdown();
            return;
        }
        gains.roll = -gain_roll; // invert once
    }
    {
        f64 gain_pitch;
        if (!nhp.getParam("gain_pitch", gain_pitch)) {
            ROS_FATAL("depth_ctrl: missing parameter gain_pitch");
            ros::shutdown();
            return;
        }
        gains.pitch = -gain_pitch; // invert once
    }
    gains.offset_heave = nhp.param("offset_heave", 0.25);

    // set up heave command publisher
    std::string cmd_topic;
    if (nhp.getParam("cmd_topic", cmd_topic)) {
        heave_pub = nhs.advertise<geometry_msgs::Twist>(cmd_topic, 1);
    }
    else {
        heave_pub = nhp.advertise<geometry_msgs::Twist>("heave", 1);
    }

    // debug publishers
    debug_en_pub = nhp.advertise<std_msgs::Int32>("debug/enabled", 1);
    debug_dt_max_pub = nhp.advertise<std_msgs::Float64>("debug/dt_max", 1);
    debug_dt_min_pub = nhp.advertise<std_msgs::Float64>("debug/dt_min", 1);

    // measurement subscribers
    depth_sensor_sub = nhp.subscribe(nhp.param<std::string>("depth_sensor", "depth_sensor"), 1,
        &DepthCtrlNode::depth_sensor_cb, this, ros::TransportHints().tcpNoDelay());
    imu_sensor_sub = nhp.subscribe(nhp.param<std::string>("imu_sensor", "imu_sensor"), 1, &DepthCtrlNode::imu_sensor_cb,
        this, ros::TransportHints().tcpNoDelay());

    // configuration subscribers
    depth_sp_sub = nhp.subscribe(nhp.param<std::string>("depth_topic", "depth"), 1, &DepthCtrlNode::depth_sp_cb, this,
        ros::TransportHints().tcpNoDelay());

    enable_ss = nhp.advertiseService("enable", &DepthCtrlNode::enable_cb, this);
    last_timepoint = ros::Time(0, 0);
    
    last_timepoint_timeout_s = ros::Time(0, 0);

    nhp.getParam("timeout_s", timeout_s);
}

void DepthCtrlNode::depth_sensor_cb(std_msgs::Float64ConstPtr const& msg)
{
    if (!enabled) {
        return;
    }
    meas.set_depth(msg->data);
    if (meas.new_meas()) {
        fb_ctrl();
    }
}

void DepthCtrlNode::imu_sensor_cb(geometry_msgs::Vector3ConstPtr const& msg)
{
    if (!enabled) {
        return;
    }
    meas.set_roll_pitch(msg->x, msg->y);
    if (meas.new_meas()) {
        fb_ctrl();
    }
}

void DepthCtrlNode::fb_ctrl()
{
    ros::Time now = ros::Time::now();
    meas.reset();

    f64 u_heave = (depth_sp - meas.depth()) * gains.heave;
    // HACK to support out of water testing: ignore offset at small set points
    if (depth_sp >= 0.10) {
        u_heave += gains.offset_heave;
    }
    f64 u_roll = meas.roll() * gains.roll; // roll, pitch gains already inverted
    f64 u_pitch = meas.pitch() * gains.pitch;
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.z = u_heave;
    cmd_msg.angular.x = u_roll;
    cmd_msg.angular.y = u_pitch;
    heave_pub.publish(cmd_msg);

    if (last_timepoint == ros::Time(0, 0) && last_timepoint_timeout_s == ros::Time(0, 0)) {
        // first run: just save timepoint to have dt in next run
        last_timepoint = now;
        last_timepoint_timeout_s = now;
        return;
    }
    f64 dt = (now - last_timepoint).toSec();
    last_timepoint = now;

    if (dt > dt_max) {
        dt_max = dt;
        std_msgs::Float64 msg;
        msg.data = dt_max;
        debug_dt_max_pub.publish(msg);
    }

    if (dt < dt_min) {
        dt_min = dt;
        std_msgs::Float64 msg;
        msg.data = dt_min;
        debug_dt_min_pub.publish(msg);
    }

    dt_timeout_s = (now - last_timepoint_timeout_s).toSec();

    if (dt_timeout_s >= timeout_s) {
        // disable
        enabled = false;
        // reset partial measurement
        meas.reset();
        // reset chached timepoint
        last_timepoint = ros::Time(0, 0);
        // stop motors
        geometry_msgs::Twist cmd_msg_timeout_s;
        heave_pub.publish(cmd_msg_timeout_s);
        ROS_INFO("depth_ctrl off");
        std_msgs::Int32 msg;
        msg.data = 0;
        debug_en_pub.publish(msg);
        last_timepoint_timeout_s = ros::Time(0, 0);
        dt_timeout_s = 0.0;
    }
}

bool DepthCtrlNode::enable_cb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
{
    // enable
    if (req.data) {
        if (enabled) {
            ROS_ERROR("depth_ctrl already on");
            res.success = false;
            return true;
        }
        enabled = true;
        ROS_INFO("depth_ctrl on");
        std_msgs::Int32 msg;
        msg.data = 1;
        debug_en_pub.publish(msg);
        res.success = true;
        return true;
    }
    // disable
    if (!enabled) {
        ROS_WARN("depth_ctrl already off");
        res.success = false;
        return true;
    }
    enabled = false;
    // reset partial measurement
    meas.reset();
    // reset chached timepoint
    last_timepoint = ros::Time(0, 0);
    // stop motors
    geometry_msgs::Twist cmd_msg;
    heave_pub.publish(cmd_msg);
    ROS_INFO("depth_ctrl off");
    std_msgs::Int32 msg;
    msg.data = 0;
    debug_en_pub.publish(msg);
    res.success = true;
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "depth_ctrl");
    DepthCtrlNode node;
    ROS_INFO("depth_ctrl started");
    ros::spin();
}
