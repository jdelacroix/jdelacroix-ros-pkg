// Copyright (C) 2014 Georgia Tech Research Corporation
// see the LICENSE file included with this software

#include "ArduinoSerialPort.h"

ArduinoSerialPort::ArduinoSerialPort() {

}

ArduinoSerialPort::~ArduinoSerialPort() {
	// TODO Auto-generated destructor stub
	close(mSerialPort);
}

void ArduinoSerialPort::setup() {
	mSerialPortWriter = mNodeHandle.advertiseService("arduino_serial_port_write", &ArduinoSerialPort::serial_port_write, this);
	mSerialPortReader = mNodeHandle.advertiseService("arduino_serial_port_read", &ArduinoSerialPort::serial_port_read, this);
	mSerialPortReaderEOL = mNodeHandle.advertiseService("arduino_serial_port_read_eol", &ArduinoSerialPort::serial_port_read_eol, this);

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
		ROS_ERROR("Error writing the requested number of bytes.");
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
	std::string data;
	int bytes_read = 0;

//	ROS_INFO("Requesting to read %d bytes.", req.length);

	while(bytes_read < req.length) {
		int rc = read(mSerialPort, buffer, req.length-bytes_read);

		if(rc == -1) { // != req.length) {
			ROS_ERROR("Error reading the requested number of bytes.");
			res.status = false;
			return res.status;
		} else {
			bytes_read += rc;
			data.append(buffer, rc);

		}
	}

	res.data = data;
	res.length = bytes_read;
	res.status = true;
	return res.status;
}

bool ArduinoSerialPort::serial_port_read_eol(arduino_serial_port::SerialPacketReadEOL::Request &req,
										     arduino_serial_port::SerialPacketReadEOL::Response &res) {
	char buffer[1];
	std::string data;
	bool eol_detected = false;
	int bytes_read = 0;

//	ROS_INFO("Requesting to read until EOL character %c appears.", req.eol);

	while(!eol_detected) {
		int rc = read(mSerialPort, buffer, 1);

		if(rc != 1) { // != req.length) {
			ROS_ERROR("Error reading until EOL character appears.");
			res.status = false;
			return res.status;
		} else {
			if(buffer[0] != req.eol) {
				data.append(buffer, 1);
				bytes_read++;
			} else {
				eol_detected = true;
			}
		}
	}

	res.data = data;
	res.length = bytes_read;
	res.status = true;
	return res.status;
}

int ArduinoSerialPort::serial_port_init(const char* name, int baud)
{
    // NOTE: This code was adapated from a Linux serial port tutorial, source currently unknown

    struct termios toptions;
    int fd;

    //fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
    //        name,baud);

    fd = open(name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
        ROS_ERROR("init_serialport: Unable to open port ");
        return -1;
    }

    fcntl(fd, F_SETFL, 0);

    if (tcgetattr(fd, &toptions) < 0) {
        ROS_ERROR("init_serialport: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud;
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
        ROS_ERROR("init_serialport: Couldn't set term attributes");
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
