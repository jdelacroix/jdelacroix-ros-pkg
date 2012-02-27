/*
 * UnicycleController.h
 *
 *  Created on: Mar 12, 2011
 *      Author: jdelacroix
 */

#ifndef UNICYCLECONTROLLER_H_
#define UNICYCLECONTROLLER_H_

#include <ros/ros.h>
#include <vicon_driver/ViconBatchData.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
//#include <khepera3_nav/Control.h>
#include <khepera3_driver/KheperaIIIMovement.h>
#include <math.h>

class UnicycleController {

	private:

		ros::NodeHandle mNodeHandle;
		ros::Subscriber mGoalSubscriber;
		ros::Subscriber mPositionSubscriber;
		ros::Publisher  mControlPublisher;

		double mGainK1, mGainK2;
		double mLinear, mAngular;
		geometry_msgs::Point mGoal;
		geometry_msgs::Pose mPose;

		double MAX_LINEAR; 		// 0.10
		double MAX_ANGULAR;  	// 0.22

		int mPlatformID;

		bool mUpdatedPose;

		void mPositionCallback(const vicon_driver::ViconBatchData &msg);
		void mGoalCallback(const geometry_msgs::Point &msg);

		void computeControl(void );

		double enforceBound(double value, double bound);


	public:

		UnicycleController();
//		~UnicycleController();

		void run();

};

#endif /* UNICYCLECONTROLLER_H_ */
