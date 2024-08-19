#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Vector3.h>
#include <monsun_msgs/Xsens700Position.h>
#include <cmath>
#include <string>
#include <vector>

using namespace std;

class WifiFollower
{
private:
    ros::NodeHandle n;
    ros::Subscriber master_xsens_gps_subscriber;
    ros::Subscriber follower_xsens_gps_subscriber;
    ros::Subscriber master_course_subscriber;
    ros::Subscriber master_speed_subscriber;

    ros::Publisher follower_course_publisher;
    ros::Publisher follower_speed_publisher;

    ros::ServiceServer enable_wifi_follower;
    bool wifi_follower_enabled = false;

    ros::ServiceClient enable_heading_ctrl;
    ros::ServiceClient enable_follower_gps_navigation_node;

    int id_master{1};
    double master_lat_{0.0};
    double master_lon_{0.0};
    double follower_lat_{0.0};
    double follower_lon_{0.0};
    double bearing_follower_wrt_master{0.0};
    const double RAD2DEG = 180.0 / M_PI;
    double distance_follower_wrt_master{0.0};
    double master_follower_distance_setpoint_m{10.0};
    double master_follower_distance_setpoint_threshold_m{1.0};
    double turn_left_angle_deg{10.0};
    double turn_right_angle_deg{10.0};
    double speed_up{0.2};
    double slow_down{0.2};
    double follower_setpoint_bearing_threshold_deg{5.0};
    double master_course_{0.0};
    double master_speed_{0.0};
    double follower_setpoint_bearing{0.0};
    double wifi_comm_lost_sec{30.0};
    double wifi_comm_master_gps_freq{10.0};
    vector<int> wifi_comm_lost_vector;
    int flag{0};
    int j{1};
    int k{1};

public:
    WifiFollower()
    {
        n.getParam("wifi_follower/id_master", id_master);
        n.getParam("wifi_follower/master_follower_distance_setpoint_m", master_follower_distance_setpoint_m);
        n.getParam("wifi_follower/master_follower_distance_setpoint_threshold_m", master_follower_distance_setpoint_threshold_m);
        n.getParam("wifi_follower/turn_left_angle_deg", turn_left_angle_deg);
        n.getParam("wifi_follower/turn_right_angle_deg", turn_right_angle_deg);
        n.getParam("wifi_follower/speed_up", speed_up);
        n.getParam("wifi_follower/slow_down", slow_down);
        n.getParam("wifi_follower/follower_setpoint_bearing_threshold_deg", follower_setpoint_bearing_threshold_deg);
        n.getParam("wifi_follower/wifi_comm_lost_sec", wifi_comm_lost_sec);
        n.getParam("wifi_follower/wifi_comm_master_gps_freq", wifi_comm_master_gps_freq);
        
        enable_wifi_follower = n.advertiseService("wifi_follower/enable", &WifiFollower::enable_wifi_follower_callback, this);
        enable_heading_ctrl = n.serviceClient<std_srvs::SetBool>("heading_ctrl/enable");
        enable_follower_gps_navigation_node = n.serviceClient<std_srvs::SetBool>("heading_ctrl/enable");

        master_xsens_gps_subscriber = n.subscribe("mn" + std::to_string(id_master) + "/xsens/gps", 1000, &WifiFollower::master_xsens_gps_subscriber_callback, this);
        follower_xsens_gps_subscriber = n.subscribe("xsens/gps", 1000, &WifiFollower::follower_xsens_gps_subscriber_callback, this);
        master_course_subscriber = n.subscribe("mn" + std::to_string(id_master) + "/heading_ctrl/heading", 1000, &WifiFollower::master_course_subscriber_callback, this);
        master_speed_subscriber = n.subscribe("mn" + std::to_string(id_master) + "/heading_ctrl/speed", 1000, &WifiFollower::master_speed_subscriber_callback, this);
        
        follower_course_publisher = n.advertise<std_msgs::Float64>("heading_ctrl/heading", 1000);
        follower_speed_publisher = n.advertise<std_msgs::Float64>("heading_ctrl/speed", 1000);
    }
    
    bool enable_wifi_follower_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
        std_srvs::SetBool heading_ctrl_srv;
        
        // wifi_follower enable
        if (req.data) 
        {
            if (wifi_follower_enabled) 
            {
                ROS_ERROR("wifi_follower: already on");
                res.success = false;
                return true;
            }

            heading_ctrl_srv.request.data = true;

            if (enable_heading_ctrl.call(heading_ctrl_srv))
            {
                wifi_follower_enabled = true;
                ROS_INFO("wifi_follower: on");
                res.success = true;

                std_msgs::Float64 set_speed_msg;
                set_speed_msg.data = 0.0;
                follower_speed_publisher.publish(set_speed_msg);

                return true;
            }
        }

        // wifi_follower disable
        if (!wifi_follower_enabled) 
        {
            ROS_WARN("wifi_follower: already off");
            res.success = false;
            return true;
        }

        heading_ctrl_srv.request.data = false;

        if (enable_heading_ctrl.call(heading_ctrl_srv))
        {
            wifi_follower_enabled = false;
            ROS_INFO("wifi_follower: off");
            res.success = true;
        }

        return true;
    }

    void master_xsens_gps_subscriber_callback(const monsun_msgs::Xsens700Position::ConstPtr& msg_master_gps)
    {        
        if (!wifi_follower_enabled) 
        {
            return;
        }
        
        wifi_comm_lost_vector.push_back(0);

        master_lat_ = msg_master_gps->lat;
        master_lon_ = msg_master_gps->lon;

        ROS_INFO_THROTTLE(15, "wifi_follower: Master: lat: %f long: %f", master_lat_, master_lon_);
    }

    void master_course_subscriber_callback(const std_msgs::Float64::ConstPtr& master_course)
    {
        if (!wifi_follower_enabled) 
        {
            return;
        }
        
        master_course_ = master_course->data;
    }

    void master_speed_subscriber_callback(const std_msgs::Float64::ConstPtr& master_speed)
    {
        if (!wifi_follower_enabled) 
        {
            return;
        }
        
        master_speed_ = master_speed->data;
    }

    double check_angle_limits(double angle_deg)
    {
        if (angle_deg < -180.0)
        {
            angle_deg += 360.0;
        }

        else if (angle_deg > 180.0)
        {
            angle_deg -= 360.0;
        }

        return angle_deg;
    }
    
    void follower_xsens_gps_subscriber_callback(const monsun_msgs::Xsens700Position::ConstPtr& msg_gps)
    {         
        if (!wifi_follower_enabled) 
        {
            return;
        }

        wifi_comm_lost_vector.push_back(1);

        if (wifi_comm_lost_vector.size() == unsigned(int(wifi_comm_lost_sec * wifi_comm_master_gps_freq)))
        {
            for (size_t i = 0; i < wifi_comm_lost_vector.size(); i++)
            {
                flag = 1;
                flag *= wifi_comm_lost_vector.at(i);
            }
        }
        
        follower_lat_ = msg_gps->lat;
        follower_lon_ = msg_gps->lon;

        ROS_INFO_THROTTLE(15, "wifi_follower: Follower: lat: %f long: %f", follower_lat_, follower_lon_);

        std_msgs::Float64 heading_msg;
        std_msgs::Float64 speed_msg;

        std_srvs::SetBool follower_gps_navigation_node_srv;
        std_srvs::SetBool heading_ctrl_srv;

        if (flag == 0)
        {
            if (k == 1)
            {
                ROS_INFO("wifi_follower: WiFi communication with master active, following master");

                follower_gps_navigation_node_srv.request.data = false;
                heading_ctrl_srv.request.data = true;

                if (enable_follower_gps_navigation_node.call(follower_gps_navigation_node_srv))
                {
                    enable_heading_ctrl.call(heading_ctrl_srv);
                    ROS_INFO("wifi_follower: navigating independently to remaining GPS waypoints");
                }  
            }
            
            k = 0;
            
            distance_follower_wrt_master = sqrt(pow(71.5 * (follower_lon_ - master_lon_), 2) + pow(111.3 * (follower_lat_ - master_lat_), 2)); // in kilometers
            
            distance_follower_wrt_master *= 1000.0; // in meters
            
            ROS_INFO("wifi_follower: distance follower: %f m", distance_follower_wrt_master);
            
            if (distance_follower_wrt_master < master_follower_distance_setpoint_m - master_follower_distance_setpoint_threshold_m)
            {
                // turn right
                ROS_INFO("wifi_follower: follower too close to master, turning right");
                heading_msg.data = master_course_ + turn_right_angle_deg;
            }

            else if (distance_follower_wrt_master > master_follower_distance_setpoint_m + master_follower_distance_setpoint_threshold_m)
            {
                // turn left
                ROS_INFO("wifi_follower: follower too far away from master, turning left");
                heading_msg.data = master_course_ - turn_left_angle_deg;
            }
            
            else
            {
                // course_follower = course_master
                ROS_INFO("wifi_follower: follower master distance appropriate, following master course");
                heading_msg.data = master_course_;  
            }

            follower_course_publisher.publish(heading_msg); 

            bearing_follower_wrt_master = atan2((follower_lon_ - master_lon_), (follower_lat_ - master_lat_));

            bearing_follower_wrt_master *= RAD2DEG;
            
            ROS_INFO("wifi_follower: bearing follower: %f degrees", bearing_follower_wrt_master);
            
            follower_setpoint_bearing = check_angle_limits(master_course_ + 90.0); 

            if (bearing_follower_wrt_master > check_angle_limits(follower_setpoint_bearing + follower_setpoint_bearing_threshold_deg))
            {
                // speed up 
                ROS_INFO("wifi_follower: speeding up");
                speed_msg.data = master_speed_ + speed_up;
            }

            else if (bearing_follower_wrt_master < check_angle_limits(follower_setpoint_bearing - follower_setpoint_bearing_threshold_deg))
            {
                // slow down
                ROS_INFO("wifi_follower: slowing down");
                speed_msg.data = master_speed_ - slow_down;
            }
            
            else
            {
                // speed_follower = speed_master
                ROS_INFO("following master speed");
                speed_msg.data = master_speed_;
            }

            follower_speed_publisher.publish(speed_msg);

            j = 1;
        }

        else
        {
            if (j == 1)
            {
                ROS_WARN("wifi_follower: WiFi communication lost");
                
                follower_gps_navigation_node_srv.request.data = true;

                if (enable_follower_gps_navigation_node.call(follower_gps_navigation_node_srv))
                {
                    ROS_INFO("wifi_follower: navigating independently to remaining GPS waypoints");
                }   
            }

            j = 0;
            k = 1;
        }
    }     
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "wifi_follower");
    WifiFollower wififollower;
    ROS_INFO("wifi_follower: started");
    ros::spin();
    return 0;
}
