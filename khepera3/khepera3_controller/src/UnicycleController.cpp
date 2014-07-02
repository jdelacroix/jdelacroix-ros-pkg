// Copyright (C) 2014 Georgia Tech Research Corporation
// see the LICENSE file included with this software

#include "UnicycleController.h"
#include <math.h>
using namespace std;

UnicycleController::UnicycleController() {

    mAngularGain = 0.5; mLinearGain = 0.025;
	MAX_LINEAR = 0.3148; MAX_ANGULAR = 2.2763;

	mUnicycleControllerPose.position.x = 0; mUnicycleControllerPose.position.y = 0;
	mGoalPose.x = 0; mGoalPose.y = 0;

	mUpdatedPose = false;

	mNodeHandle.param<std::string>("motion_capture_system", mPositioningSystem, "optitrack");
	mNodeHandle.param<int>("khepera3/platform_id", mUnicycleControllerID, 0);

	mOdometrySubscriber = mNodeHandle.subscribe("khepera3/odometry", 1, &UnicycleController::mOdometryCallback, this);
	mGoalSubscriber = mNodeHandle.subscribe("khepera3/set_goal", 1, &UnicycleController::mGoalCallback, this);
	if(mPositioningSystem.compare("optitrack") == 0) {
	  mPositionSubscriber = mNodeHandle.subscribe("/optitrack/data", 1, &UnicycleController::mOptiTrackCallback, this);
	} else {
	  mPositionSubscriber = mNodeHandle.subscribe("/vicon/data", 1, &UnicycleController::mViconCallback, this);
	}
	mServiceClient = mNodeHandle.serviceClient<khepera3_driver::UnicycleControl>("khepera3_send_control");
}

void UnicycleController::mOptiTrackCallback(const optitrack_driver::OptiTrackData &msg) {

    if(msg.id == mUnicycleControllerID) { // Gazelle
      ROS_INFO("[K3_%d] Received new pose at (%0.3g,%0.3g,%0.3g).", mUnicycleControllerID, msg.position.x, msg.position.y, msg.orientation.z);

      mUnicycleControllerPose.position.x = msg.position.x;
      mUnicycleControllerPose.position.y = msg.position.y;
      mUnicycleControllerPose.orientation.z = msg.orientation.z;

      mUpdatedPose = true;
    }
}

void UnicycleController::mViconCallback(const vicon_driver::ViconData &msg) {

  for(int i=0; i<msg.count; i++) {
    if(msg.id[i] == mUnicycleControllerID) {
      ROS_INFO("Received new pose at (%0.3g,%0.3g,%0.3g).", msg.position[i].x, msg.position[i].y, msg.orientation[i].z);

      mUnicycleControllerPose.position.x = msg.position[i].x;
      mUnicycleControllerPose.position.y = msg.position[i].y;
      mUnicycleControllerPose.orientation.z = msg.orientation[i].z;

      mUpdatedPose = true;
    }
  }
}

void UnicycleController::mGoalCallback(const geometry_msgs::Point &msg) {

    ROS_INFO("[K3_%d] Received new goal location at (%0.3g,%0.3g)\n.", mUnicycleControllerID, msg.x, msg.y);

    mGoalPose.x = msg.x;
    mGoalPose.y = msg.y;
}

void UnicycleController::mOdometryCallback(const geometry_msgs::Pose &msg) {

    ROS_INFO("Received new pose at (%0.3g,%0.3g,%0.3g).", msg.position.x, msg.position.y, msg.orientation.z);

    mUnicycleControllerPose.position.x = msg.position.x * 100; // m -> cm
    mUnicycleControllerPose.position.y = msg.position.y * 100; // m -> cm
    mUnicycleControllerPose.orientation.z = msg.orientation.z;

    mUpdatedPose = true;
}

void UnicycleController::computeControl(void ) {

	//double r = sqrt(pow(mPose.position.x,2)+pow(mPose.position.y,2));

	// h = [h_x;h_y];
	double h_x = cos(mUnicycleControllerPose.orientation.z);
	double h_y = sin(mUnicycleControllerPose.orientation.z);

	// g = [g_x;g_y]
	double g_x = mGoalPose.x;
	double g_y = mGoalPose.y;

	// p = [x;y];
	double x = mUnicycleControllerPose.position.x;
	double y = mUnicycleControllerPose.position.y;

	// r = norm(p-g);
	double r = sqrt(pow(x-g_x,2)+pow(y-g_y,2));

	// w = -k1*p'*J*u;
	// (g_x*h_y - g_y*h_x - h_y*x + h_x*y)/r^2
	mAngular = -mAngularGain*r*((g_x*h_y - g_y*h_x - h_y*x + h_x*y)/pow(r,2));

	// v = -k1*k2*p'*u;
	// -(h_x*(g_x - x) + h_y*(g_y - y))/r
	mLinear = -mAngularGain*mLinearGain*r*(-(h_x*(g_x - x) + h_y*(g_y - y))/r);

	//ROS_INFO("Unbounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

	mLinear = enforceBound(mLinear, MAX_LINEAR);
	mAngular = enforceBound(mAngular, MAX_ANGULAR);

	//ROS_INFO("Bounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

	//ROS_INFO("Distance to goal: %0.3g cm for node %i", r, mPlatformID);

	if(r < 10.0) // within 5 cm of the target.
	{
		//ROS_INFO("Within range of the goal: %0.3g cm", r);
		mLinear = 0.0; mAngular = 0.0;
	}

	khepera3_driver::UnicycleControl m_data_service;
	m_data_service.request.linear_velocity = mLinear;
	m_data_service.request.angular_velocity = mAngular;

	if(mServiceClient.call(m_data_service)) {
	  ROS_INFO("Sent control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);
	} else {
	  ROS_ERROR("Failed to make service call: 'khepera3_send_control'");
	}

}

double UnicycleController::enforceBound(double value, double bound) {
	if(value > bound) {
		return bound;
	} else if(value < -bound) {
		return -bound;
	}
	return value;
}

void UnicycleController::run() {

	ros::Rate mLoopRate(20); // 10Hz

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
	ros::init(argc, argv, "k3_unicycle_goto");

	UnicycleController mController;
	mController.run();

	return 1;
}
