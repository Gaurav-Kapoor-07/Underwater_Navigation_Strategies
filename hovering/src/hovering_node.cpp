#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include <pinger_sonar/PingerSonar.h>

class HoveringNode 
{
    private:
        ros::NodeHandle n;
        ros::Publisher set_depth_publisher;
        ros::Subscriber ms5803_depth_subscriber;
        ros::Subscriber pinger_subscriber;
        ros::ServiceServer enable_hovering;
        ros::ServiceClient enable_depth_ctrl;
        // ros::ServiceClient enable_heading_ctrl;
        bool hovering_enabled = false;
        double depth_subscribed_min{0.0};
        double depth_subscribed_max{20.0};
        double depth_subscribed{0.0};
        double depth{0.0};
        double distance_m{0.0};
        double confidence_p{0.0};
        double hovering_distance_m{1.0};
        double confidence_threshold_p{75.0};
        std::vector<double> distance_m_filtered;
        double distance_m_filtered_median{0.0};
        int distance_median_filter_size{10};

    public:
        HoveringNode() 
        { 
            n.getParam("hovering/hovering_distance_m", hovering_distance_m);
            n.getParam("hovering/confidence_threshold_p", confidence_threshold_p);
            n.getParam("hovering/distance_median_filter_size", distance_median_filter_size);
            n.getParam("hovering/depth_subscribed_min", depth_subscribed_min);
            n.getParam("hovering/depth_subscribed_max", depth_subscribed_max);
            enable_hovering = n.advertiseService("hovering/enable", &HoveringNode::enable_hovering_callback, this);
            enable_depth_ctrl = n.serviceClient<std_srvs::SetBool>("depth_ctrl/enable");
            // enable_heading_ctrl = n.serviceClient<std_srvs::SetBool>("heading_ctrl/enable");
            ms5803_depth_subscriber = n.subscribe("ms5803/depth", 1000, &HoveringNode::ms5803_depth_subscriber_callback, this);
            pinger_subscriber = n.subscribe("pinger", 1000, &HoveringNode::pinger_subscriber_callback, this);
            set_depth_publisher = n.advertise<std_msgs::Float64>("depth_ctrl/depth", 1000);
            // set_depth_publisher = n.advertise<std_msgs::Float64>("heading_ctrl/depth", 1000);
        }

        bool enable_hovering_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
        {
            std_srvs::SetBool depth_ctrl_srv;
            // std_srvs::SetBool heading_ctrl_srv;
            
            // hovering enable
            if (req.data) 
            {
                if (hovering_enabled) 
                {
                    ROS_ERROR("hovering already on");
                    res.success = false;
                    return true;
                }

                depth_ctrl_srv.request.data = true;
                // heading_ctrl_srv.request.data = true;

                if (enable_depth_ctrl.call(depth_ctrl_srv))
                // if (enable_heading_ctrl.call(heading_ctrl_srv))
                {
                    hovering_enabled = true;
                    ROS_INFO("hovering on");
                    res.success = true;
                    return true;
                }
            }

            // hovering disable
            if (!hovering_enabled) 
            {
                ROS_WARN("hovering already off");
                res.success = false;
                return true;
            }

            depth_ctrl_srv.request.data = false;
            // heading_ctrl_srv.request.data = false;

            if (enable_depth_ctrl.call(depth_ctrl_srv))
            // if (enable_heading_ctrl.call(heading_ctrl_srv))
            {
                hovering_enabled = false;
                ROS_INFO("hovering off");
                res.success = true;
            }

            return true;
        }
        
        void ms5803_depth_subscriber_callback(const std_msgs::Float64::ConstPtr& msg_depth) 
        {
            if (!hovering_enabled) 
                {
                    return;
                }

            depth_subscribed = msg_depth->data;

            if (depth_subscribed >= depth_subscribed_min && depth_subscribed <= depth_subscribed_max)
            {
                depth = depth_subscribed;
            } 
        }

        void pinger_subscriber_callback(const pinger_sonar::PingerSonar::ConstPtr& msg_pinger) 
        {
            if (!hovering_enabled) 
            {
                return;
            }

            distance_m = msg_pinger->distance_m;
            confidence_p = msg_pinger->confidence_p;
            
            if (confidence_p >= confidence_threshold_p)
            {
                distance_m_filtered.push_back(distance_m);
                
                if (distance_m_filtered.size() == unsigned(distance_median_filter_size))
                {
                    std::sort(distance_m_filtered.begin(), distance_m_filtered.end());
                    
                    if (distance_m_filtered.size() % 2 == 0)
                    {
                        distance_m_filtered_median = (distance_m_filtered.at(distance_m_filtered.size() / 2 - 1) + distance_m_filtered.at(distance_m_filtered.size() / 2)) / 2;
                    }

                    else
                    {
                        distance_m_filtered_median = distance_m_filtered.at((distance_m_filtered.size() - 1) / 2);
                    }

                    std_msgs::Float64 set_depth_msg;
    	            set_depth_msg.data = depth + distance_m_filtered_median - hovering_distance_m;
                    set_depth_publisher.publish(set_depth_msg);

                    distance_m_filtered.clear();
                }               
            }
        }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "hovering");
    HoveringNode hoveringnode;
    ROS_INFO("hovering started");
    ros::spin();
    return 0;
}