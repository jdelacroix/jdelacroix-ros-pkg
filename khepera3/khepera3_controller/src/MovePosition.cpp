/*
 * MovePosition.cpp
 *
 *  Created on: February 25, 2012
 *      Author: Taeyup Kim
 *  
 *  Notes: I have erased all the spline related codes from FigureEight.cpp 
 */

using namespace std;

#include "MovePosition.h"

MovePosition::MovePosition(){
	if(!(ros::param::get("kheperaIII/PlatformID", mPlatformID)))
		mPlatformID = 30;

	wayPointCount = 4;
	mGoalList = (geometry_msgs::Point*) calloc(4, sizeof(geometry_msgs::Point));

	//SUBSCRIPTION
	mPositionSubscriber = mNodeHandle.subscribe("/optitrack/data", 1, &MovePosition::mPositionCallback, this);

	//PUBLICATIONS
	mGoalPublisher = mNodeHandle.advertise<geometry_msgs::Point>("khepera3/set_goal", 1);

	//INITIALIZE
	Positions();
	mUpdatedPose = false; //only call mUpdatedpose to be true when the robot recieves its new updated position. 
	mIndex = 0; mDelta = 10.0;
	mPose.position.x = 0; mPose.position.y = 0;

}

MovePosition::~MovePosition() {
	free(mGoalList);
}

void MovePosition::Positions() {
	//There will be four positions for a robot to perform a square dance
	mGoalList[0].x = 0;		mGoalList[0].y = 0;
	mGoalList[1].x = 1;		mGoalList[1].y = 0;
	mGoalList[2].x = 1;    		mGoalList[2].y = -1;
	mGoalList[3].x = 0;    		mGoalList[3].y = -1;
	wayPointCount=4;
}

void MovePosition::iterate() {
	// g = [g_x;g_y]
	double g_x = mGoalList[mIndex].x;
	double g_y = mGoalList[mIndex].y;

	// p = [x;y];
	double x = mPose.position.x;
	double y = mPose.position.y;

	// r = norm(p-g);
	double r = sqrt(pow(x-g_x,2)+pow(y-g_y,2));

	if(r < mDelta) {
		if(++mIndex > wayPointCount)
			mIndex = 0;
		//ROS_INFO("Moving onto target #%d.", mIndex+1);
	}

	mGoalPublisher.publish(mGoalList[mIndex]);
}


void MovePosition::mPositionCallback(const optitrack_driver::OptiTrackData &msg) {
	if(msg.id == mPlatformID) {
	//	ROS_INFO("Currently located at (%0.3g,%0.3g).", msg.position.x, msg.position.y);

		mPose.position.x = msg.position.x;
		mPose.position.y = msg.position.y;

		mUpdatedPose = true;

	} else {
	//	ROS_ERROR("Positional data not intended for this platform.");
	}
}

void MovePosition::run() {

	std::cout<<"start run function with mUpdatePose = "<<mUpdatedPose<<std::endl;
	ros::Rate mLoopRate(10); // 10Hz

	mGoalPublisher.publish(mGoalList[mIndex]);

	while(mNodeHandle.ok()) {
		if(mUpdatedPose) {
			iterate();
			mUpdatedPose = false;  
		}
		mLoopRate.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "k3_nav_move_position"); //make arrangement for the MovePosition
	MovePosition mController;

	mController.run();

	return 1;
}
