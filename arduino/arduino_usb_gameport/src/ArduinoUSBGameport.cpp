/*
 * ArduinoUSBGameport.cpp
 *
 *  Created on: Jun 5, 2011
 *      Author: Jean-Pierre de la Croix
 */

#include "ArduinoUSBGameport.h"

ArduinoUSBGameport::ArduinoUSBGameport() {
	mClient = mNodeHandle.serviceClient<arduino_serial_port::SerialPacketReadEOL>("arduino_serial_port_read_eol");
	mJoystickPublisher = mNodeHandle.advertise<arduino_usb_gameport::GameportJoystick>("arduino/joystick", 1);
}

ArduinoUSBGameport::~ArduinoUSBGameport() {
	// TODO Auto-generated destructor stub
}

void ArduinoUSBGameport::parse(const char* buffer) {

  	/* parse the Vicon data */
	char *str,*token;
	char *saveptr;

	const char *delim = ",";

	GameportJoystick joy;

	str = strndup(buffer, strlen(buffer));

//	ROS_INFO("Starting to parse.");

	token = strtok_r(str, delim, &saveptr);
	if (token == NULL || strcmp(token, "$ARDJOY") != 0) {
		ROS_ERROR("Encountered unexpected format while parsing.");
		return;
	}

//	ROS_INFO("Parsed $ARDJOY header.");

	token = strtok_r(NULL, delim, &saveptr);
	if (token == NULL) {
		ROS_ERROR("Encountered unexpected format while parsing.");
		return;
	}
	joy.time = atoi(token);

//	ROS_INFO("Parsed time.");

	token = strtok_r(NULL, delim, &saveptr);
	if (token == NULL) {
		ROS_ERROR("Encountered unexpected format while parsing.");
		return;
	}
	joy.button_count = atoi(token);

//	ROS_INFO("Parsed button count.");

	token = strtok_r(NULL, delim, &saveptr);
	if (token == NULL) {
		ROS_ERROR("Encountered unexpected format while parsing.");
		return;
	}
	joy.axes_count = atoi(token);

//	ROS_INFO("Parsed axes count.");

	for(int i=0; i<joy.button_count; i++) {
		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL) {
			ROS_ERROR("Encountered unexpected format while parsing.");
			return;
		}
		joy.buttons[i] = atoi(token);
//		ROS_INFO("Parsed button %d.", i);
	}

	for(int i=0; i<joy.axes_count; i++) {
		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL) {
			ROS_ERROR("Encountered unexpected format while parsing.");
			return;
		}
		joy.axes[i] = atoi(token);
//		ROS_INFO("Parsed axes %d.", i);
	}

//	ROS_INFO("Finished parsing.");

	mJoystickPublisher.publish(joy);

}

void ArduinoUSBGameport::run() {

	mReaderService.request.eol = '\n';
	while(mNodeHandle.ok()) {
		if(mClient.call(mReaderService)) {
			ROS_INFO("Received %d bytes of data: %s", mReaderService.response.length, mReaderService.response.data.c_str());
			parse(mReaderService.response.data.c_str());
		} else {
			ROS_ERROR("Failed to make service call: 'arduino_serial_port_read_eol'");
		}
		ros::spinOnce();
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "arduino_usb_gameport");

	ArduinoUSBGameport mArduinoUSBGameport;

	mArduinoUSBGameport.run();

	return 0;
}
