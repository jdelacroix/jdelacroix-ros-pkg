// Copyright (C) 2014 Georgia Tech Research Corporation
// see the LICENSE file included with this software

#include "PIDRegulator.h"
#include <math.h>
using namespace std;

PIDRegulator::PIDRegulator() {

    mNodeHandle.param<double>("pid_gain_kp", mGainKp, 10.0);
    mNodeHandle.param<double>("pid_gain_ki", mGainKi, 0.001);
    mNodeHandle.param<double>("pid_gain_kd", mGainKd, 1.0);

    mNodeHandle.param<int>("khepera3/platform_id", mPlatformID, 0);

    mPreviousError = 0; mIntegratedError = 0;

	MAX_LINEAR = 0.3148; MAX_ANGULAR = 2.2763;

	mPose.position.x = 0; mPose.position.y = 0;

	mUpdatedPose = false;

	mOdometrySubscriber = mNodeHandle.subscribe("khepera3/odometry", 1, &PIDRegulator::mOdometryCallback, this);
	mOptiTrackSubscriber = mNodeHandle.subscribe("/optitrack/data", 1, &PIDRegulator::mOptiTrackCallback, this);
	mServiceClient = mNodeHandle.serviceClient<khepera3_driver::UnicycleControl>("khepera3_send_control");
}

void PIDRegulator::mOptiTrackCallback(const optitrack_driver::OptiTrackData &msg) {

    if(msg.id == mPlatformID) {
      ROS_INFO("[K3_%d] Received new pose at (%0.3g,%0.3g,%0.3g).", mPlatformID, msg.position.x, msg.position.y, msg.orientation.z);

      mPose.position.x = msg.position.x;
      mPose.position.y = msg.position.y;
      mPose.orientation.z = msg.orientation.z;

      mUpdatedPose = true;
    }
}

void PIDRegulator::mOdometryCallback(const geometry_msgs::Pose &msg) {

    ROS_INFO("Received new pose at (%0.3g,%0.3g,%0.3g).", msg.position.x, msg.position.y, msg.orientation.z);

    mPose.position.x = msg.position.x * 100; // m -> cm
    mPose.position.y = msg.position.y * 100; // m -> cm
    mPose.orientation.z = msg.orientation.z;

    mUpdatedPose = true;
}

void PIDRegulator::computeControl(void ) {

	double theta = mPose.orientation.z;

	double PI = 3.141592;
	double theta_d = 0;

	// PID regulator
	double e = atan2(cos(theta_d - theta),sin(theta_d-theta));

	double p = -mGainKp*e;
	double i = -mGainKi*(e+mIntegratedError);
	double d = -mGainKd*(e-mPreviousError);

	mPreviousError = e;
	mIntegratedError += e;

	mAngular = p + i + d;
	mLinear = 0.15;

	//ROS_INFO("Unbounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

	mLinear = enforceBound(mLinear, MAX_LINEAR);
	mAngular = enforceBound(mAngular, MAX_ANGULAR);

	//ROS_INFO("Bounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

	//ROS_INFO("Distance to goal: %0.3g cm for node %i", r, mPlatformID);


	khepera3_driver::UnicycleControl m_data_service;
	m_data_service.request.linear_velocity = mLinear;
	m_data_service.request.angular_velocity = mAngular;

	if(mServiceClient.call(m_data_service)) {
	  ROS_INFO("Sent control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);
	} else {
	  ROS_ERROR("Failed to make service call: 'khepera3_send_control'");
	}

}

double PIDRegulator::enforceBound(double value, double bound) {
	if(value > bound) {
		return bound;
	} else if(value < -bound) {
		return -bound;
	}
	return value;
}

void PIDRegulator::run() {

	ros::Rate mLoopRate(10); // 10Hz

	while(mNodeHandle.ok()) {
		if(mUpdatedPose) {
			computeControl();
			mUpdatedPose = false;
		}

		ros::spinOnce();
		mLoopRate.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "k3_pid_regulator");

	PIDRegulator mController;

	mController.run();

	return 1;
}
