#ifndef FIGUREEIGHT_H_
#define FIGUREEIGHT_H_

#include <ros/ros.h>

#include <optitrack_driver/OptiTrackData.h>
#include <vicon_driver/ViconData.h>
#include <khepera3_driver/UnicycleControl.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <math.h>
#include <list>
#include <sys/time.h>
#include <string>

class Gazelle {

private:

	ros::NodeHandle mNodeHandle;
	ros::Subscriber mPositionSubscriber;
    ros::ServiceClient mServiceClient;

	geometry_msgs::Pose mGazellePose;
	std::map<int, geometry_msgs::Point> mLionPose;

	struct timeval mPreviousTime;

	int mLionCount;

	int mPlatformID;

	double mMaxGazelleSpeed;

    double MAX_LINEAR;
    double MAX_ANGULAR;
    double mLinear, mAngular;


    double mAngularGain, mLinearGain;
    double mDelta;

	double mParameterA, mParameterB, mParameterC;

	std::string mPositioningSystem;
	void mOptiTrackCallback(const optitrack_driver::OptiTrackData &msg);
	void mViconCallback(const vicon_driver::ViconData &msg);

	bool mUpdatedPose, mCaptured;

	void computeControl(void );
    double enforceBound(double value, double bound);


public:

	Gazelle();
	//~Gazelle();

	void run();

};

#endif
