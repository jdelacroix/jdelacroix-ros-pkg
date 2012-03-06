/*
 * UnicycleController.h
 *
 *  Created on: Feb 27, 2012
 *      Author: jdelacroix
 */

#ifndef UNICYCLECONTROLLER_H_
#define UNICYCLECONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <khepera3_driver/UnicycleControl.h>
#include <optitrack_driver/OptiTrackData.h>
#include <vicon_driver/ViconData.h>
#include <math.h>
#include <string>

class UnicycleController {

	private:

		ros::NodeHandle mNodeHandle;
		ros::Subscriber mGoalSubscriber;
	    ros::ServiceClient mServiceClient;
	    ros::Subscriber mOdometrySubscriber;
	    ros::Subscriber mPositionSubscriber;

		double mLinear, mAngular;
		geometry_msgs::Point mGoalPose;
		geometry_msgs::Pose mUnicycleControllerPose;

		std::string mPositioningSystem;
		int mUnicycleControllerID;

		double MAX_LINEAR;
		double MAX_ANGULAR;

		double mAngularGain, mLinearGain;

		bool mUpdatedPose;

		void mOdometryCallback(const geometry_msgs::Pose &msg);
		void mGoalCallback(const geometry_msgs::Point &msg);
		void mOptiTrackCallback(const optitrack_driver::OptiTrackData &msg);
		void mViconCallback(const vicon_driver::ViconData &msg);

		void computeControl(void );

		double enforceBound(double value, double bound);


	public:

		UnicycleController();
//		~UnicycleController();

		void run();

};

#endif /* UNICYCLECONTROLLER_H_ */
