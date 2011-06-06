/*
 * ArduinoUSBGameport.h
 *
 *  Created on: Jun 5, 2011
 *      Author: jdelacroix
 */

#ifndef ARDUINOUSBGAMEPORT_H_
#define ARDUINOUSBGAMEPORT_H_

#include <ros/ros.h>
//#include <arduino_serial_port/SerialPacketWriter.h>
#include <arduino_serial_port/SerialPacketReadEOL.h>
#include <arduino_usb_gameport/GameportJoystick.h>

using namespace arduino_usb_gameport;

class ArduinoUSBGameport {

private:
	ros::NodeHandle mNodeHandle;
	ros::ServiceClient mClient;

	ros::Publisher mJoystickPublisher;

	arduino_serial_port::SerialPacketReadEOL mReaderService;

	void parse(const char *buffer);


public:
	ArduinoUSBGameport();
	virtual ~ArduinoUSBGameport();

	void run();
};

#endif /* ARDUINOUSBGAMEPORT_H_ */
