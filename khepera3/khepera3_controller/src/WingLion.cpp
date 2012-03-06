#include "WingLion.h"
#include <math.h>
using namespace std;

WingLion::WingLion() {

	k3_max_linear_velocity_ = 0.3148; k3_max_angular_velocity_ = 2.2763;

	lion_pose_.position.x = 0; lion_pose_.position.y = 0;
	gazelle_pose_.x = 0; gazelle_pose_.y = 0;

	updated_pose_info_ = false;
	captured_gazelle_ = false;

	node_handle_.param<std::string>("motion_capture_system", motion_capture_system_, "optitrack");

	node_handle_.param<int>("lion/id", lion_id_, 0);
	node_handle_.param<int>("lion/gazelle_id", gazelle_id_, 0);
	node_handle_.param<double>("lion/max_speed", lion_max_velocity_, 0.0);
	node_handle_.param<double>("lion/separation", lion_separation_, 0.0);
	node_handle_.param<std::string>("lion/position", lion_position_, "left");

	node_handle_.param<double>("linear_gain", linear_velocity_gain_, 0.0);
	node_handle_.param<double>("angular_gain", angular_velocity_gain_, 0.0);
	node_handle_.param<double>("delta", deadzone_radius_, 0.0);

	k3_max_linear_velocity_ = lion_max_velocity_;

	if(motion_capture_system_.compare("optitrack") == 0) {
	  position_subscriber_ = node_handle_.subscribe("/optitrack/data", 1, &WingLion::optitrack_callback, this);
	} else if(motion_capture_system_.compare("vicon") == 0) {
	  position_subscriber_ = node_handle_.subscribe("/vicon/data", 1, &WingLion::vicon_callback, this);
	} else if(motion_capture_system_.compare("odometry") == 0) {
	  position_subscriber_ = node_handle_.subscribe("khepera3/odometry", 1, &WingLion::odometry_callback, this);
	} else {
	  ROS_FATAL("Motion capture system %s not found!", motion_capture_system_.c_str());
	}

	service_client_ = node_handle_.serviceClient<khepera3_driver::UnicycleControl>("khepera3_send_control");
}

void WingLion::optitrack_callback(const optitrack_driver::OptiTrackData &msg) {

    if(msg.id == lion_id_) { // WingLion
      //ROS_INFO("[K3_%d] LION pose at (%0.3g,%0.3g,%0.3g).", lion_id_, msg.position.x, msg.position.y, msg.orientation.z);

      lion_pose_.position.x = msg.position.x;
      lion_pose_.position.y = msg.position.y;
      lion_pose_.orientation.z = msg.orientation.z;

      updated_pose_info_ = true;
    } else if(msg.id == gazelle_id_) {
      //ROS_INFO("[K3_%d] GAZELLE pose at (%0.3g,%0.3g,%0.3g).", lion_id_, msg.position.x, msg.position.y, msg.orientation.z);

      gazelle_pose_.x = msg.position.x;
      gazelle_pose_.y = msg.position.y;
    }
}

void WingLion::vicon_callback(const vicon_driver::ViconData &msg) {

  for(int i=0; i<msg.count; i++) {
    if(msg.id[i] == lion_id_) {
      ROS_INFO("[K3_%d] Received new pose at (%0.3g,%0.3g,%0.3g).", lion_id_, msg.position[i].x, msg.position[i].y, msg.orientation[i].z);

      lion_pose_.position.x = msg.position[i].x;
      lion_pose_.position.y = msg.position[i].y;
      lion_pose_.orientation.z = msg.orientation[i].z;

      updated_pose_info_ = true;
    }
  }
}

void WingLion::odometry_callback(const geometry_msgs::Pose &msg) {

    ROS_INFO("Received new pose at (%0.3g,%0.3g,%0.3g).", msg.position.x, msg.position.y, msg.orientation.z);

    lion_pose_.position.x = msg.position.x * 100; // m -> cm
    lion_pose_.position.y = msg.position.y * 100; // m -> cm
    lion_pose_.orientation.z = msg.orientation.z;

    updated_pose_info_ = true;
}

void WingLion::compute_control_signal(void ) {

	//double r = sqrt(pow(mPose.position.x,2)+pow(mPose.position.y,2));

	// h = [h_x;h_y];
	double h_x = cos(lion_pose_.orientation.z);
	double h_y = sin(lion_pose_.orientation.z);

	// g = [g_x;g_y]
	double g_x, g_y;

	if(lion_pose_.position.y > gazelle_pose_.y) {
      if(lion_position_.compare("left") == 0) {
        g_x = gazelle_pose_.x + lion_separation_;
      } else {
        g_x = gazelle_pose_.x - lion_separation_;
      }
	  g_y = gazelle_pose_.y;
	} else {
	  //g_x = gazelle_pose_.x;
	  //g_y = gazelle_pose_.y;
	  if(lion_position_.compare("left") == 0) {
        g_x = gazelle_pose_.x + 0.1;
      } else {
        g_x = gazelle_pose_.x - 0.1;
      }
	  g_y = gazelle_pose_.y;
	}

	// p = [x;y];
	double x = lion_pose_.position.x;
	double y = lion_pose_.position.y;

	// r = norm(p-g);
	double r = sqrt(pow(x-g_x,2)+pow(y-g_y,2));

	// w = -k1*p'*J*u;
	// (g_x*h_y - g_y*h_x - h_y*x + h_x*y)/r^2
	k3_angular_velocity_ = -angular_velocity_gain_*r*((g_x*h_y - g_y*h_x - h_y*x + h_x*y)/pow(r,2));

	// v = -k1*k2*p'*u;
	// -(h_x*(g_x - x) + h_y*(g_y - y))/r
	k3_linear_velocity_ = -angular_velocity_gain_*linear_velocity_gain_*r*(-(h_x*(g_x - x) + h_y*(g_y - y))/r);

	k3_linear_velocity_ = lion_max_velocity_;

	//ROS_INFO("Unbounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

	k3_linear_velocity_ = enforce_bounds(k3_linear_velocity_, k3_max_linear_velocity_);
	k3_angular_velocity_ = enforce_bounds(k3_angular_velocity_, k3_max_angular_velocity_);

	//ROS_INFO("Bounded control signal (v,w): (%0.3g,%0.3g).", mLinear, mAngular);

	//ROS_INFO("Distance to goal: %0.3g cm for node %i", r, mPlatformID);

	g_x = gazelle_pose_.x;
	g_y = gazelle_pose_.y;
	//r = sqrt(pow(x-g_x,2)+pow(y-g_y,2));
	r = sqrt(pow(x-g_x,2));

	if(r < deadzone_radius_) // within 5 cm of the target.
	{
		//ROS_INFO("Within range of the goal: %0.3g cm", r);
		k3_linear_velocity_ = 0.0; k3_angular_velocity_ = 0.0;
		captured_gazelle_ = true;
	}

	khepera3_driver::UnicycleControl m_data_service;
	m_data_service.request.linear_velocity = k3_linear_velocity_;
	m_data_service.request.angular_velocity = k3_angular_velocity_;

	if(service_client_.call(m_data_service)) {
	  ROS_INFO("[K3_%d] Sent control signal (v,w): (%0.3g,%0.3g).", lion_id_, k3_linear_velocity_, k3_angular_velocity_);
	} else {
	  ROS_ERROR("Failed to make service call: 'khepera3_send_control'");
	}

}

double WingLion::enforce_bounds(double value, double bound) {
	if(value > bound) {
		return bound;
	} else if(value < -bound) {
		return -bound;
	}
	return value;
}

void WingLion::run() {

	ros::Rate mLoopRate(20); // 10Hz

	while(node_handle_.ok()) {
		if(updated_pose_info_ && !captured_gazelle_) {
			compute_control_signal();
			//updated_pose_info_ = false;
		}

		ros::spinOnce();
		mLoopRate.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "k3_nav_wing_lion");

	WingLion mController;

	mController.run();

	return 1;
}
