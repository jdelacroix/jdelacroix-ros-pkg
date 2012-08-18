#include "ARDroneDriver.h"
#include <sstream>

void alarm_callback(int arg) { }

ARDroneDriver::ARDroneDriver() {

	//Check for port parameter
	if(!(ros::param::get("ardrone/port", m_port)))
		m_port = 5556; // default port

	//Check for IP address parameter
	if(!ros::param::get("ardrone/ip_addr", m_ip_address))
		m_ip_address = "192.168.1.1"; // default IP address

//	m_timeout = 1;

	ROS_INFO("Connecting to ARDrone at %s:%i.", m_ip_address.c_str(), m_port);

	m_command_sender = m_node_handle.advertiseService("ardrone_send_command", &ARDroneDriver::send_command, this);
	m_control_sender = m_node_handle.advertiseService("ardrone_send_control", &ARDroneDriver::send_control, this);

	m_is_flying = false;
	m_is_ready = true;

	m_sequence_number = 0;
}

ARDroneDriver::~ARDroneDriver() {

}

bool ARDroneDriver::send_command(ardrone_driver::ARDroneCommand::Request &req,
				 	 	 	 	 ardrone_driver::ARDroneCommand::Response &res) {

	const bool takeoff = req.takeoff;
	const bool land = req.land;
	const bool emergency = req.emergency;

	if (m_is_ready) {
		if (emergency) {
			// set state to emergency
			res.status = send_command_udp(SIG_EMERGENCY);
		} else if (takeoff || !m_is_flying) {
			// takeoff
			res.status = send_command_udp(SIG_TAKEOFF);
		} else if (land || m_is_flying) {
			// land
			res.status = send_command_udp(SIG_LAND);
		} else {
			res.status = false;
		}
	} else {
		if (!emergency) {
			// set state back to normal from emergency
			res.status = send_command_udp(SIG_NORMAL);
		} else {
			res.status = false;
		}
	}

	return res.status;
}

bool ARDroneDriver::send_command_udp(ARDroneSignal s) {

	std::ostringstream command_msg;

	int32_t command_flag = 0x11540000;	// 0b 0001 0001 0101 0100 0000 0000 0000 0000

	switch (s) {
		case SIG_EMERGENCY:
		case SIG_NORMAL:
			command_msg << "AT*REF=" << (++m_sequence_number) << "," << command_flag << "\r";
			command_msg << "AT*REF=" << (++m_sequence_number) << "," << (command_flag ^ 0x00000010) << "\r";
			command_msg << "AT*REF=" << (++m_sequence_number) << "," << command_flag << "\r";
			break;
		case SIG_TAKEOFF:
			command_msg << "AT*REF=" << (++m_sequence_number) << "," << (command_flag ^ 0x00000020) << "\r";
			break;
		case SIG_LAND:
		case SIG_NOOP:
			command_msg << "AT*REF=" << (++m_sequence_number) << "," << command_flag << "\r";
			break;
		default:
			return false;
	}

	std::string command = command_msg.str();

	char message[256];
	sprintf(message, "%s", command.c_str());

	/* Send received datagram back to the client */
	printf("Sending request: %s\n", message);
	if (sendto(m_socket, message, strlen(message), 0,
	  (struct sockaddr *) &m_server_address, sizeof(m_server_address)) != (ssize_t) strlen(message)) {
	  ROS_ERROR("sendto() sent a different number of bytes than expected");
	  return false;
	}
	
	return true;
}

bool ARDroneDriver::send_control(ardrone_driver::ARDroneControl::Request &req,
				 	 	 	 	 ardrone_driver::ARDroneControl::Response &res) {

	// limit all control parameters to float in range [-1.0,1.0] as per documentation

	int32_t roll = fmaxf(fminf(req.roll,1.0),-1.0);
	int32_t pitch = fmaxf(fminf(req.pitch,1.0),-1.0);
	int32_t yaw = fmaxf(fminf(req.yaw,1.0),-1.0);
	int32_t gaz = fmaxf(fminf(req.gaz,1.0),-1.0);

	res.status = send_control_udp(roll, pitch, yaw, gaz);

	return res.status;

}

bool ARDroneDriver::send_control_udp(int32_t roll, int32_t pitch, int32_t yaw, int32_t gaz) {
	// BOOKMARK

	std::ostringstream control_msg;

	control_msg << "AT*PCMD=" << (++m_sequence_number) << ",";

	int32_t control_flag = 0x00000001; // bit 0: enable progressive commands, bit 1: enable combined yaw, bit 2: enable absolute control (ARDrone 2.0 only).

	control_msg << control_flag << "," << roll << "," << pitch << "," << gaz << "," << yaw << "\r";

	std::string control = control_msg.str();

	char message[256];
	sprintf(message, "%s", control.c_str());

	/* Send received datagram back to the client */
	printf("Sending request: %s\n", message);
	if (sendto(m_socket, message, strlen(message), 0,
	  (struct sockaddr *) &m_server_address, sizeof(m_server_address)) != (ssize_t) strlen(message)) {
	  ROS_ERROR("sendto() sent a different number of bytes than expected");
	  return false;
	}

	return true;

}



bool ARDroneDriver::initialize() {
	return send_command_udp(SIG_NOOP);
}

void ARDroneDriver::connect() {

	/* Create socket for sending/receiving datagrams */
	if ((m_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		ROS_FATAL("socket() failed");
		exit(-1);
	}

	/* Construct local address structure */
	memset(&m_server_address, 0, sizeof(m_server_address));   /* Zero out structure */
	m_server_address.sin_family = AF_INET;                /* Internet address family */
	m_server_address.sin_addr.s_addr = inet_addr(m_ip_address.c_str()); /* Any incoming interface */
	m_server_address.sin_port = htons(m_port);      /* Local port */

	/* Bind to the local address */
	if (bind(m_socket, (struct sockaddr *) &m_server_address, sizeof(m_server_address)) < 0)
		ROS_FATAL("bind() failed");

	/* Set reading timeout */

//	m_signal_callback.sa_handler = alarm_callback;
//	if (sigfillset(&m_signal_callback.sa_mask) < 0) {
//		ROS_FATAL("sigfillset() failed");
//		exit(-1);
//	}
//
//	m_signal_callback.sa_flags = 0;
//
//	if (sigaction(SIGALRM, &m_signal_callback, 0) < 0) {
//		ROS_FATAL("sigaction() failed");
//		exit(-1);
//	}

	// connect
}

void ARDroneDriver::disconnect() {
	close(m_socket);
}

void ARDroneDriver::run() {

	while(m_node_handle.ok()) {
		ros::spin();
	}
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "ardrone_driver");

	ARDroneDriver driver;

	driver.connect();
	while(driver.initialize() != true) {}
	driver.run();
	driver.disconnect();

	return 1;
}
