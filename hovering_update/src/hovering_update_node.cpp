#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <vector>
// #include <pinger_sonar/PingerSonar.h>
#include <pinger_sonar/PingerSonarDistanceSimple.h>
#include <cmath>

class HoveringUpdateNode 
{
    private:
        ros::NodeHandle n;
        ros::Publisher set_depth_publisher;
        // ros::Publisher set_speed_publisher;
        ros::Subscriber ms5803_depth_subscriber;
        ros::Subscriber pinger_subscriber;
        ros::ServiceServer enable_hovering;
        ros::ServiceClient enable_heading_ctrl;
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
        int distance_median_filter_size{15};
        // double delta_actual_hovering_distance_m{0.1};
        // double speed{0.0};
        double set_depth_m{0.0};

    public:
        HoveringUpdateNode() 
        { 
            n.getParam("hovering/hovering_distance_m", hovering_distance_m);
            n.getParam("hovering/confidence_threshold_p", confidence_threshold_p);
            n.getParam("hovering/distance_median_filter_size", distance_median_filter_size);
            n.getParam("hovering/depth_subscribed_min", depth_subscribed_min);
            n.getParam("hovering/depth_subscribed_max", depth_subscribed_max);
            // n.getParam("hovering/delta_actual_hovering_distance_m", delta_actual_hovering_distance_m);
            // n.getParam("hovering/speed", speed);
            enable_hovering = n.advertiseService("hovering/enable", &HoveringUpdateNode::enable_hovering_callback, this);
            enable_heading_ctrl = n.serviceClient<std_srvs::SetBool>("heading_ctrl/enable");
            ms5803_depth_subscriber = n.subscribe("ms5803/depth", 1000, &HoveringUpdateNode::ms5803_depth_subscriber_callback, this);
            pinger_subscriber = n.subscribe("pinger", 1000, &HoveringUpdateNode::pinger_subscriber_callback, this);
            set_depth_publisher = n.advertise<std_msgs::Float64>("heading_ctrl/depth", 1000);
            // set_speed_publisher = n.advertise<std_msgs::Float64>("heading_ctrl/speed", 1000);
        }

        bool enable_hovering_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
        {
            std_srvs::SetBool heading_ctrl_srv;
            
            // hovering enable
            if (req.data) 
            {
                if (hovering_enabled) 
                {
                    ROS_ERROR("hovering: already on");
                    res.success = false;
                    return true;
                }

                heading_ctrl_srv.request.data = true;

                if (enable_heading_ctrl.call(heading_ctrl_srv))
                {
                    hovering_enabled = true;
                    ROS_INFO("hovering: on");
                    res.success = true;

                    // std_msgs::Float64 set_speed_msg;
                    // set_speed_msg.data = 0.0;
                    // set_speed_publisher.publish(set_speed_msg);

                    return true;
                }
            }

            // hovering disable
            if (!hovering_enabled) 
            {
                ROS_WARN("hovering: already off");
                res.success = false;
                return true;
            }

            heading_ctrl_srv.request.data = false;

            if (enable_heading_ctrl.call(heading_ctrl_srv))
            {
                hovering_enabled = false;
                ROS_INFO("hovering: off");
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

        // void pinger_subscriber_callback(const pinger_sonar::PingerSonar::ConstPtr& msg_pinger) 
        void pinger_subscriber_callback(const pinger_sonar::PingerSonarDistanceSimple::ConstPtr& msg_pinger)
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

            // if (std::abs(distance_m - hovering_distance_m) <= delta_actual_hovering_distance_m)
            // {
            //     // ROS_INFO("hovering: publishing speed of %f to heading_ctrl/speed topic", speed);
                
            //     std_msgs::Float64 set_speed_msg;
            //     set_speed_msg.data = speed;
            //     set_speed_publisher.publish(set_speed_msg);
            // }
        }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "hovering");
    HoveringUpdateNode hoveringupdatenode;
    ROS_INFO("hovering: started");
    ros::spin();
    return 0;
}