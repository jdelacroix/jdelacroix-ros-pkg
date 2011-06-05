/*
 * ArduinoSerialPort.cpp
 *
 *  Created on: Jun 4, 2011
 *      Author: jdelacroix
 */

#include "ArduinoSerialPort.h"

ArduinoSerialPort::ArduinoSerialPort() {

}

ArduinoSerialPort::~ArduinoSerialPort() {
	// TODO Auto-generated destructor stub
}

void ArduinoSerialPort::setup() {
	mSerialPortWriter = mNodeHandle.advertiseService("arduino_serial_port_write", &ArduinoSerialPort::serial_port_write, this);
	mSerialPortReader = mNodeHandle.advertiseService("arduino_serial_port_read", &ArduinoSerialPort::serial_port_read, this);

	mNodeHandle.param<std::string>("serial_port_name", mSerialPortName, "/dev/ttyUSB0");
	mNodeHandle.param<int>("serial_port_baud_rate", mSerialPortBaudRate, 9600);

	mSerialPort = serial_port_init(mSerialPortName.c_str(), mSerialPortBaudRate);
}

void ArduinoSerialPort::run() {
	while(mNodeHandle.ok()) {
		ros::spin();
	}
}

bool ArduinoSerialPort::serial_port_write(arduino_serial_port::SerialPacketWrite::Request &req,
										  arduino_serial_port::SerialPacketWrite::Response &res) {

	int rc = write(mSerialPort, req.data.c_str(), req.length);
	if (rc != req.length) {
		res.status = false;
		return res.status;
	}
	res.length = rc;
	res.status = true;
	return res.status;
}

bool ArduinoSerialPort::serial_port_read(arduino_serial_port::SerialPacketRead::Request &req,
										 arduino_serial_port::SerialPacketRead::Response &res) {
	char buffer[req.length];

	int rc = read(mSerialPort, buffer, req.length);

	if(rc != req.length) {
		res.status = false;
		return res.status;
	}


	res.data = std::string(buffer);
	res.length = rc;
	res.status = true;
	return res.status;
}

int ArduinoSerialPort::serial_port_init(const char* name, int baud)
{
    struct termios toptions;
    int fd;

    //fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
    //        name,baud);

    fd = open(name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }

    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
		case 4800:   brate=B4800;   break;
		case 9600:   brate=B9600;   break;
		case 19200:  brate=B19200;  break;
		case 38400:  brate=B38400;  break;
		case 57600:  brate=B57600;  break;
		case 115200: brate=B115200; break;

    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;

    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "arduino_serial_port");

	ArduinoSerialPort mArduinoSerialPort;

	mArduinoSerialPort.setup();
	mArduinoSerialPort.run();

	return 0;
}
