#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <monsun_msgs/Xsens700Position.h>
#include <monsun_msgs/GpsPath.h>
#include <monsun_msgs/GpsWaypoint.h>

// heading_ctrl/enable service
#include <std_srvs/SetBool.h>

namespace gps_navigation {

class GPSNavigation {
public:
  GPSNavigation(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle);

private:
  void current_pos_cb(monsun_msgs::Xsens700Position::ConstPtr const& msg);
  void target_pos_cb(monsun_msgs::GpsPath::ConstPtr const& msg);
  // gps_navigation_node/enable service
  // void enable_cb(std_msgs::Bool::ConstPtr const& msg);
  bool enable_gps_navigation_node_callback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);
  void speed_cb(std_msgs::Float64::ConstPtr const& msg);
  void delay_stop_cb(ros::TimerEvent const&);
  void get_new_target();

  ros::NodeHandle n;
  ros::NodeHandle np;

  ros::Publisher heading_pub;
  ros::Publisher speed_pub;
  // heading_ctrl/enable service
  // ros::Publisher cmd_en_pub;
  ros::Publisher debug_distance_pub;
  // ros::Publisher depth_pub;
  ros::Timer delay_stop_timer;

  ros::Subscriber current_pos_sub;
  ros::Subscriber target_pos_sub;
  ros::Subscriber speed_sub;
  // gps_navigation_node/enable service
  // ros::Subscriber enable_sub;

  std::list<monsun_msgs::GpsWaypoint> waypt_list_;

  double current_lon_ = 0;
  double current_lat_ = 0;
  double target_lon_ = 0;
  double target_lat_ = 0;
  double speed_ = 0;
  double threshold_distance_wp_m = 0.0;
  bool enable_ = false;

  // gps_navigation_node/enable service 
  ros::ServiceServer enable_gps_navigation_node;
  
  // heading_ctrl/enable service
  ros::ServiceClient enable_heading_ctrl;

  // depth_ctrl/enable service
  ros::ServiceClient enable_depth_ctrl;
};

} // namespace gps_navigation
