#include <point_and_shoot_hovering_update/point_and_shoot_hovering_update.h>

static const double RAD2DEG = 180.0 / M_PI;

namespace point_and_shoot_hovering {

PointAndShootHovering::PointAndShootHovering(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle)
    : n(handle), np(private_handle)
{
    // read/set default parameters
    heading_pub = n.advertise<std_msgs::Float64>(
        np.param<std::string>("set_heading_topic", "set_heading"), 1);

    speed_pub =
        n.advertise<std_msgs::Float64>(np.param<std::string>("set_speed_topic", "set_speed"), 1);

    debug_distance_pub = np.advertise<std_msgs::Float64>("debug/target_distance", 10);

    np.param<bool>("enable", enable_, false);
    np.param<double>("speed", speed_, 1.0);
    np.param<double>("heading_threshold_degs", heading_threshold_degs_, 1.0);
    np.param<double>("timeout_secs", timeout_secs_, 30.0);

    np.param<double>("dist_to_waypoint_threshold_m", dist_to_waypoint_threshold_m_, 10.0);
    np.param<double>("assumed_speed_m_p_s", assumed_speed_m_p_s_, 0.1);
    
    // set up configuration subscribers
    heading_sensor_sub = n.subscribe(np.param<std::string>("imu_topic", "/xsens/imu"),
                                1,
                                &PointAndShootHovering::heading_sensor_sub_cb,
                                this,
                                ros::TransportHints().tcpNoDelay());
    current_pos_sub = np.subscribe(np.param<std::string>("current_pos_topic", "current_pos"),
                                   1,
                                   &PointAndShootHovering::current_pos_cb,
                                   this,
                                   ros::TransportHints().tcpNoDelay());
    target_pos_sub = n.subscribe(np.param<std::string>("target_pos_topic", "target_pos"),
                                 1,
                                 &PointAndShootHovering::target_pos_cb,
                                 this,
                                 ros::TransportHints().tcpNoDelay());

    speed_sub = np.subscribe(np.param<std::string>("target_speed_topic", "target_speed"),
                             1,
                             &PointAndShootHovering::speed_cb,
                             this,
                             ros::TransportHints().tcpNoDelay());

   // point_and_shoot_hovering_node/enable service
   enable_point_and_shoot_hovering_node = np.advertiseService("enable", &PointAndShootHovering::enable_point_and_shoot_hovering_node_callback, this);
   
   // depth_ctrl/enable service
   enable_depth_ctrl = n.serviceClient<std_srvs::SetBool>("depth_ctrl/enable");    
   
   // hovering/enable service
   enable_hovering = n.serviceClient<std_srvs::SetBool>("hovering/enable"); 

   last_timepoint_timeout_s = ros::Time(0, 0);  
}

void PointAndShootHovering::heading_sensor_sub_cb(geometry_msgs::Vector3ConstPtr const& meas)
{
    current_heading_ = meas->z;
}

bool PointAndShootHovering::check_heading()
{
    double angle_lower_limit = 0.0;
    double angle_upper_limit = 0.0;
    
    if (angle_init > -90.0 && angle_init <= 90.0) // 1st or 4th quadrant
    {   
        if (angle_init >= 0.0 && angle_init <= 90.0)
        {
            ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: initial heading is in the 1st quadrant");
        }
        else
        {
            ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: initial heading is in the 4th quadrant");
        }
        
        angle_lower_limit = angle_init - 90.0;
        angle_upper_limit = angle_init + 90.0;

        ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: heading lower limit: %f degrees, heading upper limit: %f degrees", angle_lower_limit, angle_upper_limit);

        if (angle >= 0.0)
        {
            if (angle <= angle_upper_limit)
            {
                ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: heading calculated is in the limits");
                return true;
            }
            else
            {
                ROS_WARN("point_and_shoot_hovering: heading calculated is out the limits!");
                return false;
            }
        }

        else
        {
            if (angle >= angle_lower_limit)
            {
                ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: heading calculated is in the limits");
                return true;
            }
            else
            {
                ROS_WARN("point_and_shoot_hovering: heading calculated is out the limits!");
                return false;
            }
        }
    }

    else
    {
        if (angle_init > 90.0 && angle_init <= 180.0) // 2nd quadrant
        {
            ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: initial heading is in the 2nd quadrant");
            
            angle_lower_limit = angle_init - 90.0;
            angle_upper_limit = angle_init - 270.0;
        }
        else // 3rd quadrant
        {
            ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: initial heading is in the 3rd quadrant");

            angle_lower_limit = angle_init + 270.0;
            angle_upper_limit = angle_init + 90.0;
        }

        ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: heading lower limit: %f degrees, heading upper limit: %f degrees", angle_lower_limit, angle_upper_limit);

        if (angle >= 0.0)
        {
            if (angle >= angle_lower_limit)
            {
                ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: heading calculated is in the limits");
                return true;
            }
            else
            {
                ROS_WARN("point_and_shoot_hovering: heading calculated is out the limits!");
                return false;
            }
        }

        else
        {
            if (angle <= angle_upper_limit)
            {
                ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: heading calculated is in the limits");
                return true;
            }
            else
            {
                ROS_WARN("point_and_shoot_hovering: heading calculated is out the limits!");
                return false;
            }
        }
    }
}

double PointAndShootHovering::angle_normalize(double angle_input)
{
    if(angle_input <= -180.0) {
        angle_input += 360.0;
    }
    else if(angle_input > 180.0) {
        angle_input -= 360.0;
    }

    return angle_input;
}

void PointAndShootHovering::current_pos_cb(monsun_msgs::Xsens700Position::ConstPtr const& msg)
{
    if(enable_) {
            
        gps_fix = msg->fixind;
        
        if (j == 0 && gps_fix == 1)
        {  
            current_lon_ = msg->lon;
            current_lat_ = msg->lat;
            
            ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: current: lat: %f long: %f", current_lat_, current_lon_);
            ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: target: lat: %f long: %f", target_lat_, target_lon_);

            // TODO Fix calculation: We want the forward azimuth
            // angle = atan2(71.5 * (target_lon_ - current_lon_), 111.3 * (target_lat_ - current_lat_));

            angle = atan2(65.88 * (target_lon_ - current_lon_), 111.27 * (target_lat_ - current_lat_));

            angle *= RAD2DEG;
            
            ROS_INFO("point_and_shoot_hovering: computed heading: %f degrees", angle);

            if (o == 1)
            {
                angle_init = angle;
                ROS_INFO("point_and_shoot_hovering: initial heading: %f degrees", angle_init);
                o = 0;
            }

            // dist = sqrt(pow(71.5 * (target_lon_ - current_lon_), 2) +
            //                 pow(111.3 * (target_lat_ - current_lat_), 2)) * 1000.0;

            dist = sqrt(pow(65.88 * (target_lon_ - current_lon_), 2) +
                        pow(111.27 * (target_lat_ - current_lat_), 2)) * 1000.0;

            ROS_INFO("dist to target: %f m", dist);
            
            if (r == 1)
            {
                calculated_timeout_secs_ = dist / assumed_speed_m_p_s_;

                ROS_INFO("point_and_shoot_hovering: calculated timeout = %f secs", calculated_timeout_secs_);

                if (calculated_timeout_secs_ < timeout_secs_)
                {
                    timeout_secs_ = calculated_timeout_secs_;
                } 

                ROS_INFO("point_and_shoot_hovering: timeout applied = %f secs", timeout_secs_);
            
                r = 0;
            }
        
            if (q == 1)
            {
                ROS_WARN("point_and_shoot_hovering: current: lat: %f long = %f", current_lat_, current_lon_);
                ROS_WARN("point_and_shoot_hovering: previous: lat: %f long = %f", previous_lat, previous_lon);
                
                // dist_travelled_m = sqrt(pow(71.5 * (current_lon_ - previous_lon), 2) +
                    // pow(111.3 * (current_lat_ - previous_lat), 2)) * 1000.0;

                dist_travelled_m = sqrt(pow(65.88 * (current_lon_ - previous_lon), 2) +
                    pow(111.27 * (current_lat_ - previous_lat), 2)) * 1000.0;
                
                ROS_WARN("point_and_shoot_hovering: distance travelled = %f m", dist_travelled_m);
                
                calculated_speed = dist_travelled_m / timeout_secs_;

                ROS_WARN("point_and_shoot_hovering: calculated speed = %f mps", calculated_speed);

                calculated_timeout_secs_ = dist / calculated_speed;

                ROS_WARN("point_and_shoot_hovering: calculated timeout = %f secs", calculated_timeout_secs_);

                if (calculated_timeout_secs_ < timeout_secs_)
                {
                    timeout_secs_ = calculated_timeout_secs_;
                }

                ROS_WARN("point_and_shoot_hovering: timeout applied = %f secs", timeout_secs_);
    
                ROS_WARN("point_and_shoot_hovering: previous calculated heading = %f", previous_calculated_heading);
                
                // gps_heading = atan2(71.5 * (current_lon_ - previous_lon), 111.3 * (current_lat_ - previous_lat));

                gps_heading = atan2(65.88 * (current_lon_ - previous_lon), 111.27 * (current_lat_ - previous_lat));

                ROS_WARN("point_and_shoot_hovering: actual GPS heading = %f degrees", gps_heading);

                heading_drift_gps = angle_normalize(gps_heading - previous_calculated_heading); 
        
                ROS_WARN("point_and_shoot_hovering: gps heading drift from point heading= %f degrees", heading_drift_gps);
                
                q = 0;
            }

            previous_lon = current_lon_;
            previous_lat = current_lat_;
            previous_calculated_heading = angle;

            // publish distance to debug publisher to simplify follow-up analysis
            std_msgs::Float64 m;
            m.data = dist;
            debug_distance_pub.publish(m);
        }

        std_msgs::Float64 heading_msg;
        std_msgs::Float64 speed_msg;

        std_srvs::SetBool depth_ctrl_srv;

        if (q == 0)
        {
            if(check_heading()) {   
            // if(check_heading() || s == 1) {   

                ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: heading okay!");

                if(dist > dist_to_waypoint_threshold_m_) { 

                    ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: aiming the waypoint!");
                    
                    if (abs(angle_normalize(current_heading_ - angle)) > heading_threshold_degs_ && l == 0)  {

                        ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: pointing towards the waypoint!");

                        ROS_INFO("point_and_shoot_hovering: heading error: %f degrees", angle_normalize(current_heading_ - angle));

                        heading_msg.data = angle_normalize(angle - heading_drift_gps);
                        speed_msg.data = 0.0;
                        heading_pub.publish(heading_msg);
                        speed_pub.publish(speed_msg);
                    }
                    else    {
                        if (k == 0)
                        {
                            depth_ctrl_srv.request.data = true;
                            enable_depth_ctrl.call(depth_ctrl_srv);
                            k = 1;
                        }

                        j = 1;
                        l = 1;

                        ROS_INFO_THROTTLE(15, "point_and_shoot_hovering: shooting towards the waypoint!");

                        heading_msg.data = angle_normalize(angle - heading_drift_gps);

                        heading_pub.publish(heading_msg);

                        speed_msg.data = speed_;
                        speed_pub.publish(speed_msg);
                        
                        ros::Time now = ros::Time::now();
                        
                        if (last_timepoint_timeout_s == ros::Time(0, 0)) {
                        // first run: just save timepoint to have dt in next run
                        last_timepoint_timeout_s = now;
                        return;
                        }

                        dt_timeout_s = (now - last_timepoint_timeout_s).toSec();
                            
                        if (dt_timeout_s >= timeout_secs_) {
                            depth_ctrl_srv.request.data = false;
                            enable_depth_ctrl.call(depth_ctrl_srv);
                            k = 0;
                            last_timepoint_timeout_s = ros::Time(0, 0);
                            dt_timeout_s = 0.0;
                        
                            j = 0;
                            l = 0;
                            q = 1;
                            // s = 0;

                            ROS_WARN("point_and_shoot_hovering: coming up and continuing when gps fix!");
                        }
                    }
                }
                else
                {
                    ROS_WARN("point_and_shoot_hovering: skipping the waypoint as it is near than the threshold distance!");
                }
            }
            else if(waypt_list_.size() > 0) {
                get_new_target();
                o = 1;
            }
            else {
                // depth_ctrl_srv.request.data = false;
                // enable_depth_ctrl.call(depth_ctrl_srv);
                heading_msg.data = angle_normalize(angle - heading_drift_gps);
                speed_msg.data = 0.0;
                heading_pub.publish(heading_msg);
                speed_pub.publish(speed_msg);

                // disable hovering
                // delay_stop_timer = np.createTimer(ros::Duration(5.0), &PointAndShootHovering::delay_stop_cb, this, true);
                
                std_srvs::SetBool hovering_srv;
                hovering_srv.request.data = false;
                enable_hovering.call(hovering_srv);
            }
        }
    }
}

void PointAndShootHovering::target_pos_cb(monsun_msgs::GpsPath::ConstPtr const& msg)
{
    waypt_list_.clear();
    for(uint8_t i = 0; i < msg->wpts.size(); i++) { waypt_list_.push_back(msg->wpts[i]); }
    ROS_INFO("point_and_shoot_hovering: received new waypoint list! Length: %zu", waypt_list_.size());
    get_new_target();
}

void PointAndShootHovering::get_new_target()
{
    ROS_INFO("point_and_shoot_hovering: getting new waypoint!");
    target_lon_ = waypt_list_.front().lon;
    target_lat_ = waypt_list_.front().lat;
    waypt_list_.pop_front();
}

bool PointAndShootHovering::enable_point_and_shoot_hovering_node_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    std_srvs::SetBool hovering_srv;
    std_srvs::SetBool depth_ctrl_srv;
    
    // point_and_shoot_hovering enable
    if (req.data) 
    {
        if (enable_) 
        {
            ROS_ERROR("point_and_shoot_hovering: already on");
            res.success = false;
            return true;
        }

        hovering_srv.request.data = true;
        depth_ctrl_srv.request.data = false;

        if (enable_hovering.call(hovering_srv) && enable_depth_ctrl.call(depth_ctrl_srv))
        {
            enable_ = true;
            ROS_INFO("point_and_shoot_hovering: on");

            k = 0;
            j = 0;
            l = 0;
            o = 1;
            q = 0;
            r = 1;
            // s = 1;
            
            res.success = true;
            return true;
        }
    }

    // point_and_shoot_hovering disable
    if (!enable_) 
    {
        ROS_WARN("point_and_shoot_hovering: already off");
        res.success = false;
        return true;
    }

    enable_ = false;
    ROS_INFO("point_and_shoot_hovering: off");
    res.success = true;
    std_msgs::Float64 speed_msg;
    speed_msg.data = 0.0;
    speed_pub.publish(speed_msg);
    // stop heading control node after this new setting has been applied
    // TODO heading control needs a stop topic/service that surfaces and stops the forward
    // motion
    delay_stop_timer =
        np.createTimer(ros::Duration(5.0), &PointAndShootHovering::delay_stop_cb, this, true); 
    return true;
}

void PointAndShootHovering::delay_stop_cb(ros::TimerEvent const& /*unused*/)
{
    // we should have stopped by now
    ROS_INFO("point_and_shoot_hovering: behaviour finished");

    std_srvs::SetBool hovering_srv;
    hovering_srv.request.data = false;
    enable_hovering.call(hovering_srv);
}

void PointAndShootHovering::speed_cb(std_msgs::Float64::ConstPtr const& msg)
{
    double tmp_speed = speed_;
    speed_ = msg->data;
    ROS_INFO("point_and_shoot_hovering: speed set from %f to %f", tmp_speed, speed_);
}

} // namespace point_and_shoot_hovering
