/*
 * UnicycleController.h
 *
 *  Created on: Feb 27, 2012
 *      Author: jdelacroix
 */

#ifndef ROS_KHEPERA3_CONTROLLER_LION_H_
#define ROS_KHEPERA3_CONTROLLER_LION_H_

#include <math.h>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <khepera3_driver/UnicycleControl.h>
#include <optitrack_driver/OptiTrackData.h>
#include <vicon_driver/ViconData.h>

class Lion {

 public:
  Lion();
  void run();

 private:
  ros::NodeHandle node_handle_;
  ros::ServiceClient service_client_;
  ros::Subscriber position_subscriber_;
  ros::Subscriber goal_subscriber_;

  int gazelle_id_;
  geometry_msgs::Point gazelle_pose_;

  int lion_id_;
  geometry_msgs::Pose lion_pose_;
  double lion_max_velocity_;

  std::string motion_capture_system_;

  double k3_max_linear_velocity_;
  double k3_max_angular_velocity_;
  double k3_linear_velocity_, k3_angular_velocity_;

  double angular_velocity_gain_, linear_velocity_gain_;
  double deadzone_radius_;

  bool updated_pose_info_;
  bool captured_gazelle_;

  void goal_callback(const geometry_msgs::Point &msg);
  void odometry_callback(const geometry_msgs::Pose &msg);
  void optitrack_callback(const optitrack_driver::OptiTrackData &msg);
  void vicon_callback(const vicon_driver::ViconData &msg);

  void compute_control_signal(void );
  double enforce_bounds(double value, double bound);
};

#endif /* ROS_KHEPERA3_CONTROLLER_LION_H_ */
