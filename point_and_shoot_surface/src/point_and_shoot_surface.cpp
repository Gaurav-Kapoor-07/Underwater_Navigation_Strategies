#include <point_and_shoot_surface/point_and_shoot_surface.h>
// #include <cmath>

static const double RAD2DEG = 180.0 / M_PI;

namespace point_and_shoot_surface {

PointAndShootSurface::PointAndShootSurface(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle)
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
    // np.param<double>("come_up_duration_secs", come_up_duration_secs_, 5.0);
    np.param<double>("dist_to_waypoint_threshold_m", dist_to_waypoint_threshold_m_, 10.0);
    np.param<double>("assumed_speed_m_p_s", assumed_speed_m_p_s_, 0.1);

    // set up configuration subscribers
    heading_sensor_sub = n.subscribe(np.param<std::string>("imu_topic", "/xsens/imu"),
                                1,
                                &PointAndShootSurface::heading_sensor_sub_cb,
                                this,
                                ros::TransportHints().tcpNoDelay());
    current_pos_sub = np.subscribe(np.param<std::string>("current_pos_topic", "current_pos"),
                                   1,
                                   &PointAndShootSurface::current_pos_cb,
                                   this,
                                   ros::TransportHints().tcpNoDelay());
    target_pos_sub = n.subscribe(np.param<std::string>("target_pos_topic", "target_pos"),
                                 1,
                                 &PointAndShootSurface::target_pos_cb,
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
                             &PointAndShootSurface::speed_cb,
                             this,
                             ros::TransportHints().tcpNoDelay());

   // point_and_shoot_surface_node/enable service
   enable_point_and_shoot_surface_node = np.advertiseService("enable", &PointAndShootSurface::enable_point_and_shoot_surface_node_callback, this);
   
   // depth_ctrl/enable service
//    enable_depth_ctrl = n.serviceClient<std_srvs::SetBool>("depth_ctrl/enable");    
   
   // heading_ctrl/enable service
   enable_heading_ctrl = n.serviceClient<std_srvs::SetBool>("heading_ctrl/enable"); 

   last_timepoint_timeout_s = ros::Time(0, 0);  
}

void PointAndShootSurface::heading_sensor_sub_cb(geometry_msgs::Vector3ConstPtr const& meas)
{
    current_heading_ = meas->z;
}

bool PointAndShootSurface::check_heading()
{
    double angle_lower_limit = 0.0;
    double angle_upper_limit = 0.0;
    
    if (angle_init > -90.0 && angle_init <= 90.0) // 1st or 4th quadrant
    {   
        if (angle_init >= 0.0 && angle_init <= 90.0)
        {
            ROS_INFO_THROTTLE(15, "point_and_shoot_surface: initial heading is in the 1st quadrant");
        }
        else
        {
            ROS_INFO_THROTTLE(15, "point_and_shoot_surface: initial heading is in the 4th quadrant");
        }
        
        angle_lower_limit = angle_init - 90.0;
        angle_upper_limit = angle_init + 90.0;

        ROS_INFO_THROTTLE(15, "point_and_shoot_surface: heading lower limit: %f degrees, heading upper limit: %f degrees", angle_lower_limit, angle_upper_limit);

        if (angle >= 0.0)
        {
            if (angle <= angle_upper_limit)
            {
                ROS_INFO_THROTTLE(15, "point_and_shoot_surface: heading calculated is in the limits");
                return true;
            }
            else
            {
                ROS_WARN("point_and_shoot_surface: heading calculated is out the limits!");
                return false;
            }
        }

        else
        {
            if (angle >= angle_lower_limit)
            {
                ROS_INFO_THROTTLE(15, "point_and_shoot_surface: heading calculated is in the limits");
                return true;
            }
            else
            {
                ROS_WARN("point_and_shoot_surface: heading calculated is out the limits!");
                return false;
            }
        }
    }

    else
    {
        if (angle_init > 90.0 && angle_init <= 180.0) // 2nd quadrant
        {
            ROS_INFO_THROTTLE(15, "point_and_shoot_surface: initial heading is in the 2nd quadrant");
            
            angle_lower_limit = angle_init - 90.0;
            angle_upper_limit = angle_init - 270.0;
        }
        else // 3rd quadrant
        {
            ROS_INFO_THROTTLE(15, "point_and_shoot_surface: initial heading is in the 3rd quadrant");

            angle_lower_limit = angle_init + 270.0;
            angle_upper_limit = angle_init + 90.0;
        }

        ROS_INFO_THROTTLE(15, "point_and_shoot_surface: heading lower limit: %f degrees, heading upper limit: %f degrees", angle_lower_limit, angle_upper_limit);

        if (angle >= 0.0)
        {
            if (angle >= angle_lower_limit)
            {
                ROS_INFO_THROTTLE(15, "point_and_shoot_surface: heading calculated is in the limits");
                return true;
            }
            else
            {
                ROS_WARN("point_and_shoot_surface: heading calculated is out the limits!");
                return false;
            }
        }

        else
        {
            if (angle <= angle_upper_limit)
            {
                ROS_INFO_THROTTLE(15, "point_and_shoot_surface: heading calculated is in the limits");
                return true;
            }
            else
            {
                ROS_WARN("point_and_shoot_surface: heading calculated is out the limits!");
                return false;
            }
        }
    }
}

void PointAndShootSurface::current_pos_cb(monsun_msgs::Xsens700Position::ConstPtr const& msg)
{
    if(enable_) {
        if (j == 0)
        {    
            // ROS_INFO("Current: long: %f lat: %f", current_lon_, target_lat_);
            // ROS_INFO("Target: long: %f lat: %f", target_lon_, target_lat_);
            // angle = atan2((target_lon_ - current_lon_), (target_lat_ - current_lat_));
            // // wrap  angle to (-pi,pi]
            // if(angle > M_PI) {
            //     angle -= M_PI * 2.0;
            // }
            // if(angle <= -M_PI) {
            //     angle += M_PI * 2.0;
            // }
            // angle *= -1;
            // angle = angle * (180.0 / M_PI);
            // // TODO consider fix_ind

            current_lon_ = msg->lon;
            current_lat_ = msg->lat;
            
            ROS_INFO_THROTTLE(15, "point_and_shoot_surface: current: lat: %f long: %f", current_lat_, current_lon_);
            ROS_INFO_THROTTLE(15, "point_and_shoot_surface: target: lat: %f long: %f", target_lat_, target_lon_);

            // TODO Fix calculation: We want the forward azimuth
            angle = atan2((target_lon_ - current_lon_), (target_lat_ - current_lat_));

            // // wrap  angle to (-pi,pi]
            // if(angle > M_PI) {
            //     angle -= M_PI * 2.0;
            // }
            // else if(angle <= -M_PI) {
            //     angle += M_PI * 2.0;
            // }

            angle *= RAD2DEG;
        
            ROS_INFO("point_and_shoot_surface: computed heading: %f degrees", angle);

            if (o == 1)
            {
                angle_init = angle;
                ROS_INFO("point_and_shoot_surface: initial heading: %f degrees", angle_init);
                o = 0;
            }

            dist = sqrt(pow(71.5 * (target_lon_ - current_lon_), 2) +
                            pow(111.3 * (target_lat_ - current_lat_), 2));
            ROS_INFO("point_and_shoot_surface: dist to target: %f m", dist * 1000.0);
            
            // publish distance to debug publisher to simplify follow-up analysis
            std_msgs::Float64 m;
            m.data = dist * 1000.0;
            debug_distance_pub.publish(m);       
        }
        std_msgs::Float64 heading_msg;
        std_msgs::Float64 speed_msg;

        // std_srvs::SetBool depth_ctrl_srv;

        // Treshhold of 3m radius circle around target point - inside cirlce forward speed is set to
        // zero
        // if(dist > 0.003) {
        if(counter < total_count && check_heading()) {

            ROS_INFO_THROTTLE(15, "point_and_shoot_surface: counter and heading okay!");

            if(dist > dist_to_waypoint_threshold_m_ / 1000.0) {

                ROS_INFO_THROTTLE(15, "point_and_shoot_surface: aiming the waypoint!");

                if (abs(current_heading_ - angle) > heading_threshold_degs_ && l == 0)  {

                    ROS_INFO_THROTTLE(15, "point_and_shoot_surface: pointing towards the waypoint!");

                    ROS_INFO("point_and_shoot_surface: heading error: %f degrees", current_heading_ - angle);

                    heading_msg.data = angle;
                    speed_msg.data = 0.0;
                    heading_pub.publish(heading_msg);
                    speed_pub.publish(speed_msg);
                }
                else    {

                        /// Only works in P&S surface as it has GPS 
                        current_lon_ = msg->lon;
                        current_lat_ = msg->lat;
                        
                        dist = sqrt(pow(71.5 * (target_lon_ - current_lon_), 2) +
                            pow(111.3 * (target_lat_ - current_lat_), 2));
                        ROS_INFO_THROTTLE(15, "point_and_shoot_surface: dist to target: %f m", dist * 1000.0);
                        ///

                        // k += 1;
                        // if (k == 1)
                        // {
                        //     depth_ctrl_srv.request.data = true;
                        //     enable_depth_ctrl.call(depth_ctrl_srv);
                        // }

                        if (p == 1)
                        {   
                            dist_to_waypoint_m = dist * 1000.0;
                            
                            ROS_INFO("point_and_shoot_surface: distance to waypoint = %f m!", dist_to_waypoint_m);

                            time_to_waypoint_s = dist_to_waypoint_m / assumed_speed_m_p_s_;

                            ROS_INFO("point_and_shoot_surface: time to waypoint = %f s!", time_to_waypoint_s);

                            total_count = round(time_to_waypoint_s / timeout_secs_);
                            ROS_INFO("point_and_shoot_surface: total count: %d", total_count);
                            
                            p = 0;
                        }    

                        j = 1;
                        l = 1;

                        ROS_INFO_THROTTLE(15, "point_and_shoot_surface: shooting towards the waypoint!");

                        heading_msg.data = angle;
                        speed_msg.data = speed_;
                        heading_pub.publish(heading_msg);
                        speed_pub.publish(speed_msg);

                        ros::Time now = ros::Time::now();
                        
                        if (last_timepoint_timeout_s == ros::Time(0, 0)) {
                            // first run: just save timepoint to have dt in next run
                            last_timepoint_timeout_s = now;
                            ROS_INFO("point_and_shoot_surface: initializing timeout");
                            return;
                        }
                        
                        dt_timeout_s = (now - last_timepoint_timeout_s).toSec();

                        if (dt_timeout_s >= timeout_secs_) {
                            // depth_ctrl_srv.request.data = false;
                            // enable_depth_ctrl.call(depth_ctrl_srv);
                            // k = 0;
                            last_timepoint_timeout_s = ros::Time(0, 0);
                            dt_timeout_s = 0.0;
                            // ros::Duration(come_up_duration_secs_).sleep();
                            j = 0;
                            l = 0;
                            counter += 1;
                            ROS_INFO("point_and_shoot_surface: timeout, correcting heading!");
                            ROS_INFO("point_and_shoot_surface: count left: %d", total_count - counter);
                        }
                }
            }
            else
            {
                counter = 0;
                total_count = 0;
                ROS_WARN("point_and_shoot_surface: skipping the waypoint as it is near than the threshold distance!");
            }
        }
        else if(waypt_list_.size() > 0) {
            get_new_target();
            counter = 0;
            total_count = 1;
            p = 1;
            o = 1;
        }
        else {
            heading_msg.data = angle;
            speed_msg.data = 0.0;
            heading_pub.publish(heading_msg);
            speed_pub.publish(speed_msg);
            ROS_INFO("point_and_shoot_surface: stopping and disabling heading_ctrl as all waypoints reached!");
            
            // disable heading_ctrl
            delay_stop_timer = np.createTimer(ros::Duration(5.0), &PointAndShootSurface::delay_stop_cb, this, true); 
        }
    }
}

void PointAndShootSurface::target_pos_cb(monsun_msgs::GpsPath::ConstPtr const& msg)
{
    waypt_list_.clear();
    for(uint8_t i = 0; i < msg->wpts.size(); i++) { waypt_list_.push_back(msg->wpts[i]); }
    ROS_INFO("point_and_shoot_surface: received new waypoint list! Length: %zu", waypt_list_.size());
    get_new_target();
}

void PointAndShootSurface::get_new_target()
{
    ROS_INFO("point_and_shoot_surface: getting new waypoint!");
    target_lon_ = waypt_list_.front().lon;
    target_lat_ = waypt_list_.front().lat;
    waypt_list_.pop_front();
}

// gps_navigation_node/_ service
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

bool PointAndShootSurface::enable_point_and_shoot_surface_node_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    std_srvs::SetBool heading_ctrl_srv;
    // std_srvs::SetBool depth_ctrl_srv;
    
    // point_and_shoot_Surface enable
    if (req.data) 
    {
        if (enable_) 
        {
            ROS_ERROR("point_and_shoot_surface: already on");
            res.success = false;
            return true;
        }

        heading_ctrl_srv.request.data = true;
        // depth_ctrl_srv.request.data = false;

        if (enable_heading_ctrl.call(heading_ctrl_srv))
        {
            enable_ = true;
            ROS_INFO("point_and_shoot_surface: on");
            // heading_ctrl/enable service
            res.success = true;
            return true;
        }
    }

    // point_and_shoot_surface disable
    if (!enable_) 
    {
        ROS_WARN("point_and_shoot_surface: already off");
        res.success = false;
        return true;
    }

    enable_ = false;
    ROS_INFO("point_and_shoot_surface: off");
    res.success = true;
    std_msgs::Float64 speed_msg;
    speed_msg.data = 0.0;
    speed_pub.publish(speed_msg);
    // stop heading control node after this new setting has been applied
    // TODO heading control needs a stop topic/service that surfaces and stops the forward
    // motion
    delay_stop_timer =
        np.createTimer(ros::Duration(5.0), &PointAndShootSurface::delay_stop_cb, this, true); 
    return true;
}

void PointAndShootSurface::delay_stop_cb(ros::TimerEvent const& /*unused*/)
{
    // we should have stopped by now
    ROS_INFO("point_and_shoot_surface: behaviour finished");
    // std_msgs::Bool en_msg;
    // en_msg.data = false;
    // heading_ctrl/enable service
    // cmd_en_pub.publish(en_msg);
    std_srvs::SetBool heading_ctrl_srv;
    heading_ctrl_srv.request.data = false;
    enable_heading_ctrl.call(heading_ctrl_srv);
}

void PointAndShootSurface::speed_cb(std_msgs::Float64::ConstPtr const& msg)
{
    double tmp_speed = speed_;
    speed_ = msg->data;
    ROS_INFO("point_and_shoot_surface: speed set from %f to %f", tmp_speed, speed_);
}

} // namespace point_and_shoot_surface
