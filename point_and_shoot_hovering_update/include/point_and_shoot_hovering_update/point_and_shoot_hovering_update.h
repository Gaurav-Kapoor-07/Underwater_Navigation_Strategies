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

namespace point_and_shoot_hovering {

class PointAndShootHovering {
public:
  PointAndShootHovering(ros::NodeHandle const& handle, ros::NodeHandle const& private_handle);

private:
  void heading_sensor_sub_cb(geometry_msgs::Vector3ConstPtr const& meas);
  void current_pos_cb(monsun_msgs::Xsens700Position::ConstPtr const& msg);
  void target_pos_cb(monsun_msgs::GpsPath::ConstPtr const& msg);

  bool enable_point_and_shoot_hovering_node_callback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

  void speed_cb(std_msgs::Float64::ConstPtr const& msg);
  void delay_stop_cb(ros::TimerEvent const&);
  // void pointing_achieved_sub_cb(std_msgs::Bool::ConstPtr const& signal);
  void get_new_target();
  bool check_heading();
  double angle_normalize(double angle_input);

  ros::NodeHandle n;
  ros::NodeHandle np;

  ros::Publisher heading_pub;
  ros::Publisher speed_pub;
  ros::Publisher debug_distance_pub;
  ros::Timer delay_stop_timer;

  ros::Subscriber heading_sensor_sub;
  ros::Subscriber current_pos_sub;
  ros::Subscriber target_pos_sub;
  ros::Subscriber speed_sub;
  // ros::Subscriber pointing_achieved_sub;

  std::list<monsun_msgs::GpsWaypoint> waypt_list_;

  int k = 0;
  int j = 0;
  int l = 0;
  int o = 1;
  int q = 0;
  int r = 1;
  // int s = 1;
  int gps_fix = 0;
  double angle = 0.0;
  double angle_init = 0.0;
  double dist = 0.0;
  double dist_travelled_m = 0.0;
  double current_heading_ = 0.0;
  double heading_threshold_degs_ = 0.0;
  double current_lon_ = 0.0;
  double current_lat_ = 0.0;
  double target_lon_ = 0.0;
  double target_lat_ = 0.0;
  double previous_lon = 0.0;
  double previous_lat = 0.0;
  double heading_drift_gps = 0.0;
  double gps_heading = 0.0; 
  double previous_calculated_heading = 0.0;
  double speed_ = 0.0;

  bool enable_ = false;
  double assumed_speed_m_p_s_ = 0.0;
  double dist_to_waypoint_threshold_m_ = 0.0;

  // point_and_shoot_hovering_node/enable service 
  ros::ServiceServer enable_point_and_shoot_hovering_node;

  // depth_ctrl/enable service
  ros::ServiceClient enable_depth_ctrl;  
  
  // hovering/enable service
  ros::ServiceClient enable_hovering;

  ros::Time last_timepoint_timeout_s;
  double timeout_secs_ = 0.0;
  double dt_timeout_s = 0.0;

  double calculated_speed = 0.0;
  double calculated_timeout_secs_ = 0.0;
};

} // namespace point_and_shoot_hovering
