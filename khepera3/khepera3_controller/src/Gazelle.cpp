/*
 * Gazelle.cpp
 *
 *  Created on: Mar 16, 2011
 *      Author: jdelacroix
 */
using namespace std;

#include "Gazelle.h"

Gazelle::Gazelle() {

	mNodeHandle.param<int>("khepera3/platform_id", mPlatformID, 0);

	mNodeHandle.param<double>("parameter_a", mParameterA, 0);
	mNodeHandle.param<double>("parameter_b", mParameterB, 1.0);
	mNodeHandle.param<double>("parameter_c", mParameterC, 0.1);

	mNodeHandle.param<int>("lion_count", mLionCount, 1);
	mNodeHandle.param<std::string>("motion_capture_system", mPositioningSystem, "optitrack");

	MAX_LINEAR = 0.3148; MAX_ANGULAR = 2.2763;

    mGazellePose.position.x = 0; mGazellePose.position.y = 0;

    mUpdatedPose = false;
    mCaptured = false;

    mNodeHandle.param<double>("gazelle/max_speed", mMaxGazelleSpeed, 0.0);
    MAX_LINEAR = mMaxGazelleSpeed;

    mNodeHandle.param<double>("linear_gain", mLinearGain, 0.0);
    mNodeHandle.param<double>("angular_gain", mAngularGain, 0.0);

    mNodeHandle.param<double>("delta", mDelta, 0.0);


	gettimeofday(&mPreviousTime, NULL);
	mPreviousTime.tv_sec = 0;

	//SUBSCRIPTION
	if(mPositioningSystem.compare("optitrack") == 0) {
	  mPositionSubscriber = mNodeHandle.subscribe("/optitrack/data", 1, &Gazelle::mOptiTrackCallback, this);
	} else if(mPositioningSystem.compare("vicon") == 0) {
	  mPositionSubscriber = mNodeHandle.subscribe("/vicon/data", 1, &Gazelle::mViconCallback, this);
	} else {
	  ROS_FATAL("Motion capture system %s not found!", mPositioningSystem.c_str());
	}

	//PUBLICATIONS
	//mGoalPublisher = mNodeHandle.advertise<geometry_msgs::Point>("khepera3/set_goal", 1);
    mServiceClient = mNodeHandle.serviceClient<khepera3_driver::UnicycleControl>("khepera3_send_control");
}

void Gazelle::mOptiTrackCallback(const optitrack_driver::OptiTrackData &msg) {

    if(msg.id == mPlatformID) { // Gazelle
      //ROS_INFO("[K3_%d] Received new pose at (%0.3g,%0.3g,%0.3g).", mPlatformID, msg.position.x, msg.position.y, msg.orientation.z);

      mGazellePose.position.x = msg.position.x;
      mGazellePose.position.y = msg.position.y;
      mGazellePose.orientation.z = msg.orientation.z;

      if(mLionPose.size() == mLionCount) {
          mUpdatedPose = true;
      }
    } else { // Lion
      //ROS_INFO("[K3_%d] Received LION pose at (%0.3g,%0.3g,%0.3g).", mPlatformID, msg.position.x, msg.position.y, msg.orientation.z);

      geometry_msgs::Point lion_pose;
      lion_pose.x = msg.position.x;
      lion_pose.y = msg.position.y;

      //mLionPose.insert(std::pair<int, geometry_msgs::Point>(msg.id, lion_pose));
      mLionPose[msg.id] = lion_pose;

      //mLionUpdate++;
    }
}

void Gazelle::mViconCallback(const vicon_driver::ViconData &msg) {

  for(int i=0; i<msg.count; i++) {
    if(msg.id[i] == mPlatformID) {
      ROS_INFO("Received new pose at (%0.3g,%0.3g,%0.3g).", msg.position[i].x, msg.position[i].y, msg.orientation[i].z);

      mGazellePose.position.x = msg.position[i].x;
      mGazellePose.position.y = msg.position[i].y;
      mGazellePose.orientation.z = msg.orientation[i].z;

      mUpdatedPose = true;
    } else {
      //ROS_INFO("Received new pose at (%0.3g,%0.3g,%0.3g).", msg.position.x, msg.position.y, msg.orientation.z);

      geometry_msgs::Point lion_pose;
      lion_pose.x = msg.position[i].x;
      lion_pose.y = msg.position[i].y;

      mLionPose.insert(std::pair<int, geometry_msgs::Point>(msg.id[i], lion_pose));

      //mLionUpdate++;
    }
  }
}

void Gazelle::computeControl(void ) {

    double x = mGazellePose.position.x;
    double y = mGazellePose.position.y;

    double f_x = 0;
    double f_y = 0;
    double r;

    ROS_INFO("# of LIONS detected %d", mLionPose.size());

    std::map<int, geometry_msgs::Point>::const_iterator it;
    for(it = mLionPose.begin(); it != mLionPose.end(); ++it) {
      geometry_msgs::Point lion_pose = it->second;

      double lx = lion_pose.x;
      double ly = lion_pose.y;

      r = sqrt(pow(x-lx,2)+pow(y-ly,2));
      double dx = x-lx;
      double dy = y-ly;

      f_x += -dx*(mParameterA-mParameterB*exp(-r/mParameterC));
      f_y += -dy*(mParameterA-mParameterB*exp(-r/mParameterC));

      ROS_INFO("[K3_%d] Read LION pose at (%0.3g,%0.3g).", mPlatformID, lion_pose.x, lion_pose.y);
    }

    ROS_INFO("[K3_%d] r, f_x, f_y = (%0.3g, %0.3g,%0.3g).", mPlatformID, r, f_x, f_y);

    double g_x, g_y;

    if(mPreviousTime.tv_sec != 0) {
      struct timeval current_time;
      gettimeofday(&current_time, NULL);

      double dt = (current_time.tv_usec - mPreviousTime.tv_usec)/1000000.0;

      ROS_INFO("[K3_%d] dt = %0.3g", mPlatformID, dt);

      g_x = x + dt*f_x; // Euler approximation
      g_y = y + dt*f_y;

      ROS_INFO("[K3_%d] goal at (%0.3g,%0.3g).", g_x, g_y);
    }

    // h = [h_x;h_y];
    double h_x = cos(mGazellePose.orientation.z);
    double h_y = sin(mGazellePose.orientation.z);

    // g = [g_x;g_y]
    //double g_x = goal_point.x;
    //double g_y = goal_point.y;

    // p = [x;y];
    x = mGazellePose.position.x;
    y = mGazellePose.position.y;

    // r = norm(p-g);
    r = sqrt(pow(x-g_x,2)+pow(y-g_y,2));

    // w = -k1*p'*J*u;
    // (g_x*h_y - g_y*h_x - h_y*x + h_x*y)/r^2
    mAngular = -mAngularGain*r*((g_x*h_y - g_y*h_x - h_y*x + h_x*y)/pow(r,2));

    // v = -k1*k2*p'*u;
    // -(h_x*(g_x - x) + h_y*(g_y - y))/r
    mLinear = -mAngularGain*mLinearGain*r*(-(h_x*(g_x - x) + h_y*(g_y - y))/r);
    //mLinear = mMaxGazelleSpeed;
    mLinear = mMaxGazelleSpeed*abs(f_y);

    //ROS_INFO("Unbounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

    mLinear = enforceBound(mLinear, MAX_LINEAR);
    mAngular = enforceBound(mAngular, MAX_ANGULAR);

    //ROS_INFO("Bounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

    //ROS_INFO("Distance to goal: %0.3g cm for node %i", r, mPlatformID);
    double dr = sqrt(pow(x-mLionPose[5].x,2)+pow(y-mLionPose[5].y,2));
    if(dr < mDelta) // within 5 cm of the target.
    {
        //ROS_INFO("Within range of the goal: %0.3g cm", r);
        mLinear = 0.0; mAngular = 0.0;
        mCaptured = true;
    }

    khepera3_driver::UnicycleControl m_data_service;
    m_data_service.request.linear_velocity = mLinear;
    m_data_service.request.angular_velocity = mAngular;

    if(mServiceClient.call(m_data_service)) {
      ROS_INFO("[K3_%d] Sent control signal (v,w): (%0.3g,%0.3g).", mPlatformID, mLinear, mAngular);
    } else {
      ROS_ERROR("Failed to make service call: 'khepera3_send_control'");
    }

    gettimeofday(&mPreviousTime, NULL);
}

double Gazelle::enforceBound(double value, double bound) {
    if(value > bound) {
        return bound;
    } else if(value < -bound) {
        return -bound;
    }
    return value;
}

void Gazelle::run() {
	ros::Rate mLoopRate(20); // 5Hz

	while(mNodeHandle.ok()) {
		if(mUpdatedPose && !mCaptured) {
			computeControl();
			//mUpdatedPose = false;
		}
		ros::spinOnce();
		mLoopRate.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "k3_nav_gazelle");
	Gazelle mController;

	mController.run();

	return 1;
}
