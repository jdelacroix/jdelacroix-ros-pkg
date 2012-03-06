/*
 * PIDRegulator.h
 *
 *  Created on: Feb 27, 2012
 *      Author: jdelacroix
 */

#ifndef PIDREGULATOR_H_
#define PIDREGULATOR_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <khepera3_driver/UnicycleControl.h>
#include <optitrack_driver/OptiTrackData.h>
#include <math.h>

class PIDRegulator {

	private:

		ros::NodeHandle mNodeHandle;
	    ros::ServiceClient mServiceClient;
	    ros::Subscriber mOdometrySubscriber;
	    ros::Subscriber mOptiTrackSubscriber;

		double mLinear, mAngular;
		geometry_msgs::Pose mPose;

		int mPlatformID;

		double MAX_LINEAR;
		double MAX_ANGULAR;

		double mGainKp, mGainKi, mGainKd;

		double mPreviousError, mIntegratedError;

		bool mUpdatedPose;

		void mOdometryCallback(const geometry_msgs::Pose &msg);
		void mGoalCallback(const geometry_msgs::Point &msg);
		void mOptiTrackCallback(const optitrack_driver::OptiTrackData &msg);

		void computeControl(void );

		double enforceBound(double value, double bound);


	public:

		PIDRegulator();
//		~PIDRegulator();

		void run();

};

#endif /* PIDREGULATOR_H_ */
