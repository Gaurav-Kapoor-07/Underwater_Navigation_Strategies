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

#include <cmath>

using namespace std;

namespace point_and_shoot_surface {

class PointAndShootSurface {
public:
  PointAndShootSurface(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle);

private:
  void heading_sensor_sub_cb(geometry_msgs::Vector3ConstPtr const& meas);
  void current_pos_cb(monsun_msgs::Xsens700Position::ConstPtr const& msg);
  void target_pos_cb(monsun_msgs::GpsPath::ConstPtr const& msg);
  // gps_navigation_node/enable service
  // void enable_cb(std_msgs::Bool::ConstPtr const& msg);
  bool enable_point_and_shoot_surface_node_callback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);
  void speed_cb(std_msgs::Float64::ConstPtr const& msg);
  void delay_stop_cb(ros::TimerEvent const&);
  void pointing_achieved_sub_cb(std_msgs::Bool::ConstPtr const& signal);
  void get_new_target();
  bool check_heading();

  ros::NodeHandle n;
  ros::NodeHandle np;

  ros::Publisher heading_pub;
  ros::Publisher speed_pub;
  // heading_ctrl/enable service
  // ros::Publisher cmd_en_pub;
  ros::Publisher debug_distance_pub;
  ros::Timer delay_stop_timer;

  ros::Subscriber heading_sensor_sub;
  ros::Subscriber current_pos_sub;
  ros::Subscriber target_pos_sub;
  ros::Subscriber speed_sub;
  // gps_navigation_node/enable service
  // ros::Subscriber enable_sub;
  ros::Subscriber pointing_achieved_sub;

  std::list<monsun_msgs::GpsWaypoint> waypt_list_;

  // int k = 0;
  int j = 0;
  int l = 0;
  int p = 1;
  int o = 1;
  int q = 0;
  int r = 0;
  double angle = 0.0;
  double angle_init = 0.0;
  double dist = 0.0;
  double dist_travelled_m = 0.0;
  double dist_check = 0.0;
  double current_heading_ = 0.0;
  double heading_threshold_degs_ = 0.0;
  // double come_up_duration_secs_ = 0.0;
  double current_lon_ = 0;
  double current_lat_ = 0;
  double target_lon_ = 0;
  double target_lat_ = 0;
  double previous_lon = 0;
  double previous_lat = 0;
  double speed_ = 0;
  bool enable_ = false;
  bool pointing_achieved = false;
  double assumed_speed_m_p_s_ = 0.0;
  double dist_to_waypoint_threshold_m_ = 0.0;
  // int total_count = 1;
  // int counter = 0;

  // point_and_shoot_surface_node/enable service 
  ros::ServiceServer enable_point_and_shoot_surface_node;

  // depth_ctrl/enable service
  // ros::ServiceClient enable_depth_ctrl;  
  
  // heading_ctrl/enable service
  ros::ServiceClient enable_heading_ctrl;

  ros::Time last_timepoint_timeout_s;
  double timeout_secs_ = 0.0;
  double dt_timeout_s = 0.0;

  double calculated_speed = 0.0;
  double calculated_timeout_secs_ = 0.0;
};

} // namespace point_and_shoot_surface
