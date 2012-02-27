#include "UnicycleController.h"
#include <math.h>
using namespace std;

UnicycleController::UnicycleController() {

	if(!(ros::param::get("kheperaIII/gain/K1", mGainK1)))
		mGainK1 = 0.5;
	if(!(ros::param::get("kheperaIII/gain/K2", mGainK2)))
		mGainK2 = 0.025;
	if(!(ros::param::get("kheperaIII/PlatformID", mPlatformID)))
		mPlatformID = 30;

	MAX_LINEAR = 0.1; MAX_ANGULAR = 0.33;

	mPose.position.x = 0; mPose.position.y = 0;
	mGoal.x = 0; mGoal.y = 0;

	mUpdatedPose = false;

	mGoalSubscriber = mNodeHandle.subscribe("khepera3/set_goal", 1, &UnicycleController::mGoalCallback, this);
	mPositionSubscriber = mNodeHandle.subscribe("/vicon/batch_data", 2, &UnicycleController::mPositionCallback, this);

	mControlPublisher = mNodeHandle.advertise<khepera3_driver::KheperaIIIMovement>("khepera3/ctrl_vel", 1);
}

void UnicycleController::mPositionCallback(const vicon_driver::ViconBatchData &msg) {

	// assuming batch mode here
	int c=0;

	for(c=0; c<msg.count; c++) {
		if(msg.id[c] == mPlatformID) {

			//		ROS_INFO("Currently located at (%0.3g,%0.3g).", msg.position.x, msg.position.y);

			mPose.position.x = msg.position[c].x;
			mPose.position.y = msg.position[c].y;

			//	    ROS_INFO("Maintaining a heading of (%0.3g) radians.", msg.orientation.z);

			mPose.orientation.z = msg.orientation[c].z; // yaw

			mUpdatedPose = true;

		} else {
			//		ROS_ERROR("Positional data not intended for this platform.");
		}
	}
}

void UnicycleController::mGoalCallback(const geometry_msgs::Point &msg) {

	ROS_INFO("Received new target at (%0.3g,%0.3g).", msg.x, msg.y);

	mGoal.x = msg.x;
	mGoal.y = msg.y;
}

void UnicycleController::computeControl(void ) {

	//double r = sqrt(pow(mPose.position.x,2)+pow(mPose.position.y,2));

	// h = [h_x;h_y];
	double h_x = cos(mPose.orientation.z);
	double h_y = sin(mPose.orientation.z);

	// g = [g_x;g_y]
	double g_x = mGoal.x;
	double g_y = mGoal.y;

	// p = [x;y];
	double x = mPose.position.x;
	double y = mPose.position.y;

	// r = norm(p-g);
	double r = sqrt(pow(x-g_x,2)+pow(y-g_y,2));

	// w = -k1*p'*J*u;
	// (g_x*h_y - g_y*h_x - h_y*x + h_x*y)/r^2
	mAngular = -mGainK1*r*((g_x*h_y - g_y*h_x - h_y*x + h_x*y)/pow(r,2));

	// v = -k1*k2*p'*u;
	// -(h_x*(g_x - x) + h_y*(g_y - y))/r
	mGainK2 = .2 *mGainK2+.1*fabs(((g_x-x)*(h_x)+(g_y-y)*(h_y))/sqrt(pow(g_x-x,2)+pow(g_y-y,2)));
	//ROS_ERROR("K2 = %0.3g",(g_x-x)*(h_x));
	//ROS_ERROR("K2 = %0.3g",(g_y-y)*(h_y));
	//ROS_ERROR("sqrt = %0.3g",sqrt(pow(g_x-x,2)+pow(g_y-y,2)));
double temp123 = fabs(((g_x-x)*(h_x)+(g_y-y)*(h_y))/sqrt(pow(g_x-x,2)+pow(g_y-y,2)));
	//ROS_ERROR("everything = %0.3g",temp123);
	mLinear = -mGainK2*r*(-(h_x*(g_x - x) + h_y*(g_y - y))/r);

	//if (mLinear<0){ mLinear = 0; }

	//	ROS_INFO("Unbounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

	mLinear = enforceBound(mLinear, MAX_LINEAR);
	mAngular = enforceBound(mAngular, MAX_ANGULAR);

	//	ROS_INFO("Bounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

	//ROS_INFO("Distance to goal: %0.3g cm for node %i", r, mPlatformID);

	if(r < 5.0) // within 5 cm of the target.
	{
		//		ROS_INFO("Within range of the goal: %0.3g cm", r);
		mLinear = 0.0; mAngular = 0.0;
	}

	khepera3_driver::KheperaIIIMovement mControl;

	mControl.linearVelocity = mLinear;
	mControl.angularVelocity = mAngular;

	mControlPublisher.publish(mControl);

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
	ros::init(argc, argv, "k3_nav_goto");

	UnicycleController mController;

	mController.run();

	return 1;
}
