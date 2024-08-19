#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Vector3.h>
#include <monsun_msgs/Xsens700Position.h>
#include <cmath>
#include <string>
#include <vector>

class PumpModuleBehavior
{
private:
    ros::NodeHandle n;

    ros::ServiceServer enable_pump_module_behavior;
    bool pump_module_behavior_enabled = false;

    ros::ServiceClient enable_gps_navigation_node;
    ros::ServiceClient enable_depth_ctrl;
    ros::ServiceClient enable_pump_module;

    ros::Subscriber signal_gps_waypoint_reached_subscriber;
    ros::Subscriber ms5803_depth_subscriber;
    ros::Subscriber published_depth_subscriber;

    double depth_subscribed_min{0.0};
    double depth_subscribed_max{20.0};
    double depth_subscribed{0.0};
    double depth{0.0};

    double depth_published{0.0};
    double delta_required_depth{0.1};

    bool i = false;
    bool j = false;
    bool k = false;
    bool l = false;

public:
    PumpModuleBehavior()
    {   
        n.getParam("pump_module_behavior/depth_subscribed_min", depth_subscribed_min);
        n.getParam("pump_module_behavior/depth_subscribed_max", depth_subscribed_max);
        n.getParam("pump_module_behavior/delta_required_depth", delta_required_depth);

        enable_pump_module_behavior = n.advertiseService("pump_module_behavior/enable", &PumpModuleBehavior::enable_pump_module_behavior_callback, this);
        enable_gps_navigation_node = n.serviceClient<std_srvs::SetBool>("gps_navigation_node/enable");
        enable_depth_ctrl = n.serviceClient<std_srvs::SetBool>("depth_ctrl/enable");
        enable_pump_module = n.serviceClient<std_srvs::SetBool>("pump_module/enable");

        signal_gps_waypoint_reached_subscriber = n.subscribe("gps_navigation_node/signal_waypoint_reached", 1000, &PumpModuleBehavior::signal_gps_waypoint_reached_subscriber_callback, this);
        ms5803_depth_subscriber = n.subscribe("ms5803/depth", 1000, &PumpModuleBehavior::ms5803_depth_subscriber_callback, this);
        published_depth_subscriber = n.subscribe("heading_ctrl/depth", 1000, &PumpModuleBehavior::published_depth_subscriber_callback, this);
    }
    
    bool enable_pump_module_behavior_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
        std_srvs::SetBool gps_navigation_node_srv;
        std_srvs::SetBool depth_ctrl_srv;
       
        // pump_module_behavior enable
        if (req.data) 
        {
            if (pump_module_behavior_enabled) 
            {
                ROS_ERROR("pump_module_behavior: already on");
                res.success = false;
                return true;
            }

            gps_navigation_node_srv.request.data = true;
            depth_ctrl_srv.request.data = false;
            
            if (enable_gps_navigation_node.call(gps_navigation_node_srv) && enable_depth_ctrl.call(depth_ctrl_srv))
            {
                pump_module_behavior_enabled = true;
                ROS_INFO("pump_module_behavior: on");
                res.success = true;

                k = true;
                l = true;
                return true;
            }
        }

        // pump_module_behavior disable
        if (!pump_module_behavior_enabled) 
        {
            ROS_WARN("pump_module_behavior: already off");
            res.success = false;
            return true;
        }

        pump_module_behavior_enabled = false;
        ROS_INFO("pump_module_behavior: off");
        res.success = true;

        return true;
    }

    void signal_gps_waypoint_reached_subscriber_callback(const std_msgs::Bool::ConstPtr& msg_signal_gps_waypoint_reached)
    {        
        if (!pump_module_behavior_enabled) 
        {
            return;
        }
        
        std_srvs::SetBool gps_navigation_node_srv;
        std_srvs::SetBool depth_ctrl_srv;

        if (msg_signal_gps_waypoint_reached->data && k)
        {   
            gps_navigation_node_srv.request.data = false;
            depth_ctrl_srv.request.data = true;
            
            if (enable_gps_navigation_node.call(gps_navigation_node_srv) && enable_depth_ctrl.call(depth_ctrl_srv))
            {
                ROS_INFO("pump_module_behavior: going down to collect water"); 
                i = true;
                j = true;
            }

            k = false;
            l = true;
        }

        else if (l)
        {
            gps_navigation_node_srv.request.data = true;
            depth_ctrl_srv.request.data = false;
            
            if (enable_gps_navigation_node.call(gps_navigation_node_srv) && enable_depth_ctrl.call(depth_ctrl_srv))
            {
                ROS_INFO("pump_module_behavior: still on the surface"); 
            }

            k = true;
            l = false;
        }
    }

    void ms5803_depth_subscriber_callback(const std_msgs::Float64::ConstPtr& msg_subscribed_depth) 
    {
        if (!pump_module_behavior_enabled) 
        {
            return;
        }

        depth_subscribed = msg_subscribed_depth->data;

        std_srvs::SetBool pump_module_srv;

        if (depth_subscribed >= depth_subscribed_min && depth_subscribed <= depth_subscribed_max)
        {
            depth = depth_subscribed;
    
            if (std::abs(depth_published - depth) <= delta_required_depth && i)
            {
                pump_module_srv.request.data = true;

                if (enable_pump_module.call(pump_module_srv))
                {
                    ROS_INFO("pump_module_behavior: pump starting");
                } 

                i = false;
            }

            else if (j)
            {
                pump_module_srv.request.data = false;

                if (enable_pump_module.call(pump_module_srv))
                {
                    ROS_INFO("pump_module_behavior: pump stopping");
                } 

                j = false;
            }
        }
    }

    void published_depth_subscriber_callback(const std_msgs::Float64::ConstPtr& msg_published_depth)
    {
        if (!pump_module_behavior_enabled) 
        {
            return;
        }

        depth_published = msg_published_depth->data;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pump_module_behavior");
    PumpModuleBehavior pumpmodulebehavior;
    ROS_INFO("pump_module_behavior: started");
    ros::spin();
    return 0;
}
