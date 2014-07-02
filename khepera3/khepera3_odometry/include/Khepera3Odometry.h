// Copyright (C) 2014 Georgia Tech Research Corporation
// see the LICENSE file included with this software

#ifndef KHEPERA3ODOMETRY_H_
#define KHEPERA3ODOMETRY_H_

#include <ros/ros.h>

#include <khepera3_driver/SensorData.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

class Khepera3Odometry {

private:

	ros::NodeHandle m_node_handle;
	ros::ServiceClient m_client;
	ros::Publisher m_odometry_publisher;

	int m_frequency;

	khepera3_driver::SensorData m_data_service;

	float m_wheel_radius;
	float m_wheel_base_length;

	float m_meters_per_tick;

	int m_previous_left_ticks, m_previous_right_ticks;

	geometry_msgs::Pose m_estimated_pose;


public:
	Khepera3Odometry();
	virtual ~Khepera3Odometry();
	void run();
};

#endif /* KHEPERA3ODOMETRY_H_ */
