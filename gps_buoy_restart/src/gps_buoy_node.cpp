#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Vector3.h>
#include <monsun_msgs/Xsens700Position.h>
#include <cmath>

using namespace std;

class GPSBuoyNode 
{
    private:
        ros::NodeHandle n;

        ros::Publisher monsun_gps_publisher;
        ros::Subscriber ms5803_depth_subscriber;
        ros::Subscriber xsens_gps_subscriber;
        ros::Subscriber xsens_imu_subscriber;
        ros::ServiceServer enable_gps_buoy;
        ros::ServiceClient enable_hovering;
        ros::ServiceClient enable_gps_navigation;

        bool gps_buoy_enabled = false;
        
        double depth_subscribed_min{0.0};
        double depth_subscribed_max{20.0};
        double depth_subscribed{0.0};
        double depth{0.0};
        double gps_cable_length_m{6.0};
        double current_heading_deg{0.0};
        double current_heading_rad{0.0};
        double current_buoy_lat_{0.0};
        double current_buoy_lon_{0.0};
        double two_d_distance{0.0};
        double delta_latitude_m{0.0};
        double delta_longitude_m{0.0};
        // double surface_depth_m{0.2};
        
        // Timeout feature
        ros::Time last_timepoint_timeout_s;
        double timeout_s{900.0};
        double dt_timeout_s{0.0};
        double surface_wait_s{6.0};
        
        int gps_fix{0};

    public:
        GPSBuoyNode() 
        { 
            n.getParam("gps_buoy/gps_cable_length_m", gps_cable_length_m);
            n.getParam("gps_buoy/timeout_s", timeout_s);
            // n.getParam("gps_buoy/surface_depth_m", surface_depth_m);
            n.getParam("gps_buoy/depth_subscribed_min", depth_subscribed_min);
            n.getParam("gps_buoy/depth_subscribed_max", depth_subscribed_max);
            n.getParam("gps_buoy/surface_wait_s", surface_wait_s);

            enable_gps_buoy = n.advertiseService("/gps_buoy/enable", &GPSBuoyNode::enable_gps_buoy_callback, this);
            enable_hovering = n.serviceClient<std_srvs::SetBool>("/hovering/enable");
            enable_gps_navigation = n.serviceClient<std_srvs::SetBool>("/gps_navigation_node/enable");
            
            ms5803_depth_subscriber = n.subscribe("/ms5803/depth", 1000, &GPSBuoyNode::ms5803_depth_subscriber_callback, this);
            xsens_imu_subscriber = n.subscribe("/xsens/imu", 1000, &GPSBuoyNode::xsens_imu_subscriber_callback, this);
            xsens_gps_subscriber = n.subscribe("/xsens/gps", 1000, &GPSBuoyNode::xsens_gps_subscriber_callback, this);

            monsun_gps_publisher = n.advertise<monsun_msgs::Xsens700Position>("/monsun_gps_estimated", 1000);
        }

        bool enable_gps_buoy_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
        {
            std_srvs::SetBool hovering_srv;
            std_srvs::SetBool gps_navigation_srv;
            
            // gps_buoy enable
            if (req.data) 
            {
                if (gps_buoy_enabled) 
                {
                    ROS_ERROR("gps_buoy: already on");
                    res.success = false;
                    return true;
                }

                hovering_srv.request.data = true;
                gps_navigation_srv.request.data = true;

                if (enable_hovering.call(hovering_srv) && enable_gps_navigation.call(gps_navigation_srv))
                {
                    gps_buoy_enabled = true;
                    ROS_INFO("gps_buoy: on");
                    res.success = true;

                    return true;
                }
            }

            // gps_buoy disable
            if (!gps_buoy_enabled) 
            {
                ROS_WARN("gps_buoy: already off");
                res.success = false;
                return true;
            }

            hovering_srv.request.data = false;
            gps_navigation_srv.request.data = false;

            if (enable_hovering.call(hovering_srv) && enable_gps_navigation.call(gps_navigation_srv))
            {
                gps_buoy_enabled = false;
                ROS_INFO("gps_buoy: off");
                res.success = true;
            }

            return true;
        }
        
        void ms5803_depth_subscriber_callback(const std_msgs::Float64::ConstPtr& msg_depth) 
        {
            if (!gps_buoy_enabled) 
                {
                    return;
                }

            depth_subscribed = msg_depth->data;

            if (depth_subscribed >= depth_subscribed_min && depth_subscribed <= depth_subscribed_max)
            {
                depth = depth_subscribed;
            } 
        }

        void xsens_imu_subscriber_callback(const geometry_msgs::Vector3::ConstPtr& msg_imu)
        {
            if (!gps_buoy_enabled) 
                {
                    return;
                }

            current_heading_deg = msg_imu->z;
            current_heading_rad = current_heading_deg * M_PI / 180.0;
        }

        void xsens_gps_subscriber_callback(const monsun_msgs::Xsens700Position::ConstPtr& msg_gps)
        {
            if (!gps_buoy_enabled) 
                {
                    return;
                }

            // Timeout feature

            ros::Time now = ros::Time::now();

            if (last_timepoint_timeout_s == ros::Time(0, 0)) {
                // first run: just save timepoint to have dt in next run
                last_timepoint_timeout_s = now;
                ROS_INFO("gps_buoy: initializing timeout");
                return;
            }
            
            dt_timeout_s = (now - last_timepoint_timeout_s).toSec();

            if (dt_timeout_s >= timeout_s) {
                ROS_INFO("gps_buoy: Timeout!");
                
                std_srvs::SetBool hovering_srv;
                std_srvs::SetBool gps_navigation_srv;

                hovering_srv.request.data = false;
                gps_navigation_srv.request.data = false;

                if (enable_hovering.call(hovering_srv) && enable_gps_navigation.call(gps_navigation_srv))
                {
                    ROS_INFO("gps_buoy: disabing GPS Navigation and Hovering");
                }

                ROS_INFO("gps_buoy: surfacing!");
                ros::Duration(surface_wait_s).sleep();

                hovering_srv.request.data = true;
                gps_navigation_srv.request.data = true;

                if (enable_hovering.call(hovering_srv) && enable_gps_navigation.call(gps_navigation_srv))
                {
                    ROS_INFO("gps_buoy: enabling GPS Navigation and Hovering");
                }
                
                last_timepoint_timeout_s = ros::Time(0, 0);
                dt_timeout_s = 0.0;
            }
                
            current_buoy_lat_ = msg_gps->lat;
            current_buoy_lon_ = msg_gps->lon;
            gps_fix = msg_gps->fixind;

            monsun_msgs::Xsens700Position monsun_gps; 

            if (gps_fix == 1)
            {
                two_d_distance = sqrt(pow(gps_cable_length_m, 2) - pow(depth, 2));
                
                delta_latitude_m = two_d_distance * cos(current_heading_rad);
                delta_longitude_m = two_d_distance * sin(current_heading_rad);

                monsun_gps.lat = current_buoy_lat_ + (delta_latitude_m / (111.3 * 1000.0));
                monsun_gps.lon = current_buoy_lon_ + (delta_longitude_m / (71.5 * 1000.0));
                monsun_gps.fixind = 1;

                monsun_gps_publisher.publish(monsun_gps);
            }
        }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "gps_buoy");
    GPSBuoyNode gpsbuoynode;
    ROS_INFO("gps_buoy: started");
    ros::spin();
    return 0;
}