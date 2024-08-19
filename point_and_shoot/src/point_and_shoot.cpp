#include <point_and_shoot/point_and_shoot.h>
#include <cmath>

namespace point_and_shoot {

PointAndShoot::PointAndShoot(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle)
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

    np.param<bool>("enable", enable_, false);
    np.param<double>("speed", speed_, 1.0);
    np.param<double>("heading_threshold_degs", heading_threshold_degs_, 1.0);
    np.param<double>("timeout_secs", timeout_secs_, 30.0);
    np.param<double>("come_up_duration_secs", come_up_duration_secs_, 5.0);

    // set up configuration subscribers
    heading_sensor_sub = n.subscribe(np.param<std::string>("imu_topic", "/xsens/imu"),
                                1,
                                &PointAndShoot::heading_sensor_sub_cb,
                                this,
                                ros::TransportHints().tcpNoDelay());
    current_pos_sub = np.subscribe(np.param<std::string>("current_pos_topic", "current_pos"),
                                   1,
                                   &PointAndShoot::current_pos_cb,
                                   this,
                                   ros::TransportHints().tcpNoDelay());
    target_pos_sub = n.subscribe(np.param<std::string>("target_pos_topic", "target_pos"),
                                 1,
                                 &PointAndShoot::target_pos_cb,
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
                             &PointAndShoot::speed_cb,
                             this,
                             ros::TransportHints().tcpNoDelay());

   // point_and_shoot_node/enable service
   enable_point_and_shoot_node = np.advertiseService("enable", &PointAndShoot::enable_point_and_shoot_node_callback, this);
   
   // depth_ctrl/enable service
   enable_depth_ctrl = n.serviceClient<std_srvs::SetBool>("depth_ctrl/enable");    
   
   // heading_ctrl/enable service
   enable_heading_ctrl = n.serviceClient<std_srvs::SetBool>("heading_ctrl/enable"); 

   last_timepoint_timeout_s = ros::Time(0, 0);  
}

void PointAndShoot::heading_sensor_sub_cb(geometry_msgs::Vector3ConstPtr const& meas)
{
    current_heading_ = meas->z;
}

void PointAndShoot::current_pos_cb(monsun_msgs::Xsens700Position::ConstPtr const& msg)
{
    current_lon_ = msg->lon;
    current_lat_ = msg->lat;

    if(enable_) {
        ROS_INFO("Current: long: %f lat: %f", current_lon_, target_lat_);
        ROS_INFO("Target: long: %f lat: %f", target_lon_, target_lat_);
        double angle = atan2((target_lon_ - current_lon_), (target_lat_ - current_lat_));
        // wrap  angle to (-pi,pi]
        if(angle > M_PI) {
            angle -= M_PI * 2.0;
        }
        if(angle <= -M_PI) {
            angle += M_PI * 2.0;
        }
        angle *= -1;
        angle = angle * (180.0 / M_PI);
        // TODO consider fix_ind

        double dist = sqrt(pow(71.5 * (target_lon_ - current_lon_), 2) +
                           pow(111.3 * (target_lat_ - current_lat_), 2));
        ROS_INFO("dist to target: %f km", dist);
    
        // publish distance to debug publisher to simplify follow-up analysis
        std_msgs::Float64 m;
        m.data = dist * 1000.0;
        debug_distance_pub.publish(m);

        std_msgs::Float64 heading_msg;
        std_msgs::Float64 speed_msg;

        std_srvs::SetBool depth_ctrl_srv;

        // Treshhold of 5m radius circle around target point - inside cirlce forward speed is set to
        // zero
        if(dist > 0.005) {
            if (abs(current_heading_ - angle) > heading_threshold_degs_)  {
                heading_msg.data = angle;
                speed_msg.data = 0.0;
                heading_pub.publish(heading_msg);
                speed_pub.publish(speed_msg);
            }
            else    {
                k += 1;
                if (k == 1)
                {
                    depth_ctrl_srv.request.data = true;
                    enable_depth_ctrl.call(depth_ctrl_srv);
                }
                
                heading_msg.data = angle;
                speed_msg.data = speed_;
                heading_pub.publish(heading_msg);
                speed_pub.publish(speed_msg);
                
                ros::Time now = ros::Time::now();
                dt_timeout_s = (now - last_timepoint_timeout_s).toSec();
                if (last_timepoint_timeout_s == ros::Time(0, 0)) {
                // first run: just save timepoint to have dt in next run
                last_timepoint_timeout_s = now;
                return;
                }
                if (dt_timeout_s >= timeout_secs_) {
                    depth_ctrl_srv.request.data = false;
                    enable_depth_ctrl.call(depth_ctrl_srv);
                    k = 0;
                }
                last_timepoint_timeout_s = ros::Time(0, 0);
                dt_timeout_s = 0.0;
                
                ros::Duration(come_up_duration_secs_).sleep();
            }
        }
        else if(waypt_list_.size() > 0) {
            get_new_target();
        }
        else {
            heading_msg.data = angle;
            speed_msg.data = 0.0;
            heading_pub.publish(heading_msg);
            speed_pub.publish(speed_msg);
        }
    }
}

void PointAndShoot::target_pos_cb(monsun_msgs::GpsPath::ConstPtr const& msg)
{
    waypt_list_.clear();
    for(uint8_t i = 0; i < msg->wpts.size(); i++) { waypt_list_.push_back(msg->wpts[i]); }
    ROS_INFO("Received new waypoint list! Length: %zu", waypt_list_.size());
    get_new_target();
}

void PointAndShoot::get_new_target()
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

bool PointAndShoot::enable_point_and_shoot_node_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    std_srvs::SetBool heading_ctrl_srv;
    std_srvs::SetBool depth_ctrl_srv;
    
    // point_and_shoot enable
    if (req.data) 
    {
        if (enable_) 
        {
            ROS_ERROR("point_and_shoot already on");
            res.success = false;
            return true;
        }

        heading_ctrl_srv.request.data = true;
        depth_ctrl_srv.request.data = false;

        if (enable_heading_ctrl.call(heading_ctrl_srv) && enable_depth_ctrl.call(depth_ctrl_srv))
        {
            enable_ = true;
            ROS_INFO("point_and_shoot on");
            // heading_ctrl/enable service
            res.success = true;
            return true;
        }
    }

    // point_and_shoot disable
    if (!enable_) 
    {
        ROS_WARN("point_and_shoot already off");
        res.success = false;
        return true;
    }

    enable_ = false;
    ROS_INFO("point_and_shoot off");
    res.success = true;
    std_msgs::Float64 speed_msg;
    speed_msg.data = 0.0;
    speed_pub.publish(speed_msg);
    // stop heading control node after this new setting has been applied
    // TODO heading control needs a stop topic/service that surfaces and stops the forward
    // motion
    delay_stop_timer =
        np.createTimer(ros::Duration(5.0), &PointAndShoot::delay_stop_cb, this, true); 
    return true;
}

void PointAndShoot::delay_stop_cb(ros::TimerEvent const& /*unused*/)
{
    // we should have stopped by now
    ROS_INFO("Point and Shoot behaviour finished");
    // std_msgs::Bool en_msg;
    // en_msg.data = false;
    // heading_ctrl/enable service
    // cmd_en_pub.publish(en_msg);
    std_srvs::SetBool heading_ctrl_srv;
    heading_ctrl_srv.request.data = false;
    enable_heading_ctrl.call(heading_ctrl_srv);
}

void PointAndShoot::speed_cb(std_msgs::Float64::ConstPtr const& msg)
{
    double tmp_speed = speed_;
    speed_ = msg->data;
    ROS_INFO("speed set from %f to %f", tmp_speed, speed_);
}

} // namespace gps_navigation
