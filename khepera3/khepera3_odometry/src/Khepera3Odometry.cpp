// Copyright (C) 2014 Georgia Tech Research Corporation
// see the LICENSE file included with this software

#include "Khepera3Odometry.h"

Khepera3Odometry::Khepera3Odometry() {

	if(!(ros::param::get("khepera3_odometry/frequency", m_frequency)))
		m_frequency = 2; // default frequency: 2Hz

	m_client = m_node_handle.serviceClient<khepera3_driver::SensorData>("khepera3_receive_data");
	m_odometry_publisher = m_node_handle.advertise<geometry_msgs::Pose>("khepera3/odometry", 1);

	m_estimated_pose.position.x = 0;
	m_estimated_pose.position.y = 0;
	m_estimated_pose.orientation.z = 0;

	m_wheel_radius = 0.021f;
	m_wheel_base_length = 0.0885f;
	m_meters_per_tick = (2*M_PI*m_wheel_radius)/2675;

	m_previous_right_ticks = 0;
	m_previous_left_ticks = 0;
}

Khepera3Odometry::~Khepera3Odometry() {
	// TODO Auto-generated destructor stub
}

void Khepera3Odometry::run() {
	ros::Rate loop_rate(m_frequency);
	int wheel_encoder_count, left_ticks, right_ticks;
	float d_right, d_left, d_center, phi;

	while(m_node_handle.ok()) {

		if(m_client.call(m_data_service)) {
//			ROS_INFO("Received %d bytes of data: %s", mReaderService.response.length, mReaderService.response.data.c_str());
//			parse(mReaderService.response.data.c_str());

			wheel_encoder_count = m_data_service.response.wheel_encoder_count;
			right_ticks = m_data_service.response.wheel_encoders[0];
			left_ticks = m_data_service.response.wheel_encoders[1];

//			ROS_DEBUG("(right_ticks,left_ticks) = (%d,%d)", right_ticks, left_ticks);

			d_right = (right_ticks-m_previous_right_ticks)*m_meters_per_tick;
			d_left = (left_ticks-m_previous_left_ticks)*m_meters_per_tick;

			d_center = (d_right + d_left)/2.0f;
			phi = (d_right - d_left)/m_wheel_base_length;

//			ROS_DEBUG("(d_right,d_left,d_center,phi) = (%0.6g,%0.6g,%0.6g,%0.6g)", d_right,d_left,d_center,phi);

			m_estimated_pose.position.x += d_center*cosf(m_estimated_pose.orientation.z);
			m_estimated_pose.position.y += d_center*sinf(m_estimated_pose.orientation.z);
			m_estimated_pose.orientation.z += phi;

			m_previous_right_ticks = right_ticks;
			m_previous_left_ticks = left_ticks;

			m_odometry_publisher.publish(m_estimated_pose);

		} else {
			ROS_ERROR("Failed to make service call: 'khepera3_receive_data'");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "khepera3_odometry");

  Khepera3Odometry odometry;

  odometry.run();

  return 0;
}
