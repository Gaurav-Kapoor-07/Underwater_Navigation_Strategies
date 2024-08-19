#include <gps_navigation/gps_navigation.h>
#include <cmath>

static const double RAD2DEG = 180.0 / M_PI;

namespace gps_navigation {

GPSNavigation::GPSNavigation(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle)
    : n(handle), np(private_handle)
{
    // read/set default parameters
    heading_pub = n.advertise<std_msgs::Float64>(
        np.param<std::string>("set_heading_topic", "set_heading"), 1);
    speed_pub =
        n.advertise<std_msgs::Float64>(np.param<std::string>("set_speed_topic", "set_speed"), 1);

    // heading_ctrl/enable service
    // {
    //     std::string cmd_en_pub_name;
    //     if(np.getParam("cmd_enable_topic", cmd_en_pub_name)) {
    //         cmd_en_pub = n.advertise<std_msgs::Bool>(cmd_en_pub_name, 10); // TODO service
    //     }
    //     else {
    //         cmd_en_pub = np.advertise<std_msgs::Bool>("cmd_enable", 10);
    //     }
    // }

    debug_distance_pub = np.advertise<std_msgs::Float64>("debug/target_distance", 10);

    // depth_pub = n.advertise<std_msgs::Float64>(
    //     np.param<std::string>("set_depth_topic", "set_depth"), 1);

    np.param<bool>("enable", enable_, false);
    np.param<double>("speed", speed_, 1.0);
    np.param<double>("threshold_distance_wp_m", threshold_distance_wp_m, 5.0);

    // set up configuration subscribers
    current_pos_sub = np.subscribe(np.param<std::string>("current_pos_topic", "current_pos"),
                                   1,
                                   &GPSNavigation::current_pos_cb,
                                   this,
                                   ros::TransportHints().tcpNoDelay());
    target_pos_sub = n.subscribe(np.param<std::string>("target_pos_topic", "target_pos"),
                                 1,
                                 &GPSNavigation::target_pos_cb,
                                 this,
                                 ros::TransportHints().tcpNoDelay());
    // gps_navigation_node/enable service
    // enable_sub = np.subscribe(np.param<std::string>("enable_topic", "enable"),
    //                           10,
    //                           &GPSNavigation::enable_cb,
    //                           this,
    //                           ros::TransportHints().tcpNoDelay());
    speed_sub = np.subscribe(np.param<std::string>("target_speed_topic", "target_speed"),
                             1,
                             &GPSNavigation::speed_cb,
                             this,
                             ros::TransportHints().tcpNoDelay());

   // gps_navigation_node/enable service
   enable_gps_navigation_node = np.advertiseService("enable", &GPSNavigation::enable_gps_navigation_node_callback, this);
   
   // heading_ctrl/enable service
   enable_heading_ctrl = n.serviceClient<std_srvs::SetBool>("heading_ctrl/enable");  

    // depth_ctrl/enable service
    enable_depth_ctrl = n.serviceClient<std_srvs::SetBool>("depth_ctrl/enable"); 
}

void GPSNavigation::current_pos_cb(monsun_msgs::Xsens700Position::ConstPtr const& msg)
{
    current_lon_ = msg->lon;
    current_lat_ = msg->lat;

    if(enable_) {
        
        std_msgs::Float64 heading_msg;
        std_msgs::Float64 speed_msg;

        // if (msg->fixind == 1)
        // {
        ROS_INFO_THROTTLE(15, "Current: long: %f lat: %f", current_lon_, current_lat_);
        ROS_INFO_THROTTLE(15, "Target: long: %f lat: %f", target_lon_, target_lat_);
        double angle = atan2(71.5 * (target_lon_ - current_lon_), 111.3 * (target_lat_ - current_lat_));
        // // wrap  angle to (-pi,pi]
        // if(angle > M_PI) {
        //     angle -= M_PI * 2.0;
        // }
        // if(angle <= -M_PI) {
        //     angle += M_PI * 2.0;
        // }
        // angle *= -1;
        angle *= RAD2DEG;
        // TODO consider fix_ind

        double dist = sqrt(pow(71.5 * (target_lon_ - current_lon_), 2) +
                        pow(111.3 * (target_lat_ - current_lat_), 2));
        ROS_INFO("distance to target: %f m", dist * 1000.0);
        {
            // publish distance to debug publisher to simplify follow-up analysis
            std_msgs::Float64 m;
            m.data = dist * 1000.0;
            debug_distance_pub.publish(m);
        }

        // std_msgs::Float64 heading_msg;
        // std_msgs::Float64 speed_msg;

        // Treshhold of 5m radius circle around target point - inside cirlce forward speed is set to
        // zero
        if(dist > threshold_distance_wp_m / 1000.0) {
        // if(dist > 0.005) {
            heading_msg.data = angle;
            speed_msg.data = speed_;
            heading_pub.publish(heading_msg);
            speed_pub.publish(speed_msg);
        }
        else if(waypt_list_.size() > 0) {
            get_new_target();
        }
        else {
            heading_msg.data = angle;
            speed_msg.data = 0.0;
            heading_pub.publish(heading_msg);
            speed_pub.publish(speed_msg);
                        
            // disable heading_ctrl
        
            std_srvs::SetBool heading_ctrl_srv;
            heading_ctrl_srv.request.data = false;
            enable_heading_ctrl.call(heading_ctrl_srv);

            // disable depth_ctrl

            std_srvs::SetBool depth_ctrl_srv;
            depth_ctrl_srv.request.data = false;
            enable_depth_ctrl.call(depth_ctrl_srv);
        }
        // }
        // else
        // {
        //     ROS_INFO("GPS fix zero, stopping and coming to the surface!");

        //     speed_msg.data = 0.0;
        //     speed_pub.publish(speed_msg);
            
        //     // std_msgs::Float64 depth_msg;
        //     // depth_msg.data = 0.0;
        //     // depth_pub.publish(depth_msg);

        //     // disable depth_ctrl

        //     std_srvs::SetBool depth_ctrl_srv;
        //     depth_ctrl_srv.request.data = false;
        //     enable_depth_ctrl.call(depth_ctrl_srv);
        // }  
    }
}

void GPSNavigation::target_pos_cb(monsun_msgs::GpsPath::ConstPtr const& msg)
{
    waypt_list_.clear();
    for(uint8_t i = 0; i < msg->wpts.size(); i++) { waypt_list_.push_back(msg->wpts[i]); }
    ROS_INFO("Received new waypoint list! Length: %zu", waypt_list_.size());
    get_new_target();
}

void GPSNavigation::get_new_target()
{
    ROS_INFO("Getting new waypoint!");
    target_lon_ = waypt_list_.front().lon;
    target_lat_ = waypt_list_.front().lat;
    waypt_list_.pop_front();
}

// gps_navigation_node/enable service
// void GPSNavigation::enable_cb(std_msgs::Bool::ConstPtr const& msg)
// {
//     if(msg->data) {
//         ROS_INFO("gps_navigation on");
//         enable_ = true;
//         std_msgs::Bool en_msg;
//         en_msg.data = true;
//         // heading_ctrl/enable service
//         // cmd_en_pub.publish(en_msg);
//         std_srvs::SetBool heading_ctrl_srv;
//         heading_ctrl_srv.request.data = true;
//         enable_heading_ctrl.call(heading_ctrl_srv);
//     }
//     else {
//         ROS_INFO("gps_navigation off");
//         enable_ = false;
//         std_msgs::Float64 speed_msg;
//         speed_msg.data = 0.0;
//         speed_pub.publish(speed_msg);
//         // stop heading control node after this new setting has been applied
//         // TODO heading control needs a stop topic/service that surfaces and stops the forward
//         // motion
//         delay_stop_timer =
//             np.createTimer(ros::Duration(5.0), &GPSNavigation::delay_stop_cb, this, true);
//     }
// }

bool GPSNavigation::enable_gps_navigation_node_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    std_srvs::SetBool heading_ctrl_srv;
    
    // gps_navigation enable
    if (req.data) 
    {
        if (enable_) 
        {
            ROS_ERROR("gps_navigation already on");
            res.success = false;
            return true;
        }

        heading_ctrl_srv.request.data = true;

        if (enable_heading_ctrl.call(heading_ctrl_srv))
        {
            enable_ = true;
            ROS_INFO("gps_navigation on");
            // heading_ctrl/enable service
            res.success = true;
            return true;
        }
    }

    // gps_navigation disable
    if (!enable_) 
    {
        ROS_WARN("gps_navigation already off");
        res.success = false;
        return true;
    }

    enable_ = false;
    ROS_INFO("gps_navigation off");
    res.success = true;
    std_msgs::Float64 speed_msg;
    speed_msg.data = 0.0;
    speed_pub.publish(speed_msg);
    // stop heading control node after this new setting has been applied
    // TODO heading control needs a stop topic/service that surfaces and stops the forward
    // motion
    delay_stop_timer =
        np.createTimer(ros::Duration(5.0), &GPSNavigation::delay_stop_cb, this, true); 
    return true;
}

void GPSNavigation::delay_stop_cb(ros::TimerEvent const& /*unused*/)
{
    // we should have stopped by now
    ROS_INFO("GPS behaviour finished");
    // std_msgs::Bool en_msg;
    // en_msg.data = false;
    // heading_ctrl/enable service
    // cmd_en_pub.publish(en_msg);
    std_srvs::SetBool heading_ctrl_srv;
    heading_ctrl_srv.request.data = false;
    enable_heading_ctrl.call(heading_ctrl_srv);
}

void GPSNavigation::speed_cb(std_msgs::Float64::ConstPtr const& msg)
{
    double tmp_speed = speed_;
    speed_ = msg->data;
    ROS_INFO("speed set from %f to %f", tmp_speed, speed_);
}

} // namespace gps_navigation
