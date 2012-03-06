#ifndef MOVEPOSITION_H_
#define MOVEPOSITION_H_

#include <ros/ros.h>
#include <optitrack_driver/OptiTrackData.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

class MovePosition {

private:

	ros::NodeHandle mNodeHandle;
	ros::Subscriber mPositionSubscriber;
	ros::Publisher  mGoalPublisher;

	geometry_msgs::Point *mGoalList;
	int mIndex;
	double mDelta;

	geometry_msgs::Pose mPose;

	int mPlatformID;

	void mPositionCallback(const optitrack_driver::OptiTrackData &msg);

	void Positions();
	void iterate();

	bool mUpdatedPose;

	int wayPointCount;

public:

	MovePosition();
	~MovePosition();

	void run();

};

#endif
