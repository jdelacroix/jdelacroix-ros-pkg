#include "Khepera3Driver.h"

K3Driver::Khepera3Driver() {

	//Check for port parameter
	if(!(ros::param::get("khepera3/port", mPort)))
		mPort = 4555; // default port

	//Check for IP address parameter
	if(!ros::param::get("khepera3/ip_addr", mIPAddress))
		mIPAdress = "192.168.1.201"; // default IP address

	ROS_INFO("Connecting to K3 at %s:%i.", mIPAddress, mPort);
	this.mConnect();

	mDataPublisher = node_handle.advertise<khepera3_driver::Khepera3Data>("khepera3/sensor_data", 1);
	mControlSubscriber = node_handle.subscribe("khepera3/ctrl",1, &Khepera3Driver::mControlCallback,this);

}

K3Driver::~Khepera3Driver() {

}

void Khepera3Driver::mControlCallback(const khepera3_driver::UnicycleControl &msg) {

	// convert unicycle to differential driver
	// limit v \in [-0.3148,0.3148] and w \in [-2.2763,2.2763]

	const float v = fmaxf(fminf(msg.linear_velocity,0.3148),-0.3148);
	const float w = fmaxf(fminf(msg.angular_velocity,2.276),-2.2763);

	const float R = 0.021; // wheel radius
	const float L = 0.0885; // wheel base length

	// hardware conversion factor
	const float SF = 6.2953e-6;

	const float vel_r = v/R + (w*L)/(2*R);
	const float vel_l = v/R - (w*L)/(2*R);

	const int right_wheel_speed = floor(vel_r*R/SF);
	const int left_wheel_speed = floor(vel_l*R/SF);

	mSendControl(right_wheel_speed, left_wheel_speed);

}

void mSendControl() {

void Khepera3Driver::mAlarmCallback(int arg) { }

void Khepera3Driver::mConnect() {

	unsigned int cliAddrLen;         /* Length of incoming message */
	char echoBuffer[ECHOMAX];        /* Buffer for echo string */
	unsigned short echoServPort;     /* Server port */
	int recvMsgSize;                 /* Size of received message */

	/* Create socket for sending/receiving datagrams */
	if ((mSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		ROS_FATAL("socket() failed");
		exit(-1);
	}

	/* Construct local address structure */
	memset(&mServerAddress, 0, sizeof(mServerAddress));   /* Zero out structure */
	mServerAddress.sin_family = AF_INET;                /* Internet address family */
	mServerAddress.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
	mServerAddress.sin_port = htons(mPort);      /* Local port */

	/* Bind to the local address */
//	if (bind(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
//		DieWithError("bind() failed");

	/* Set reading timeout */

	mSignalCallback.sa_handler = Khepera3Driver::mAlarmCallback;
	if (sigfillset(&mSignalCallback.sa_mask) < 0) {
		ROS_FATAL("sigfillset() failed");
		exit(-1);
	}

	mSignalCallback.sa_flags = 0;

	if (sigaction(SIGALRM, &mSignalCallback, 0) < 0) {
		ROS_FATAL("sigaction() failed");
		exit(-1);
	}

	// connect
}



	    for (;;) /* Run forever */
	    {
	        /* Set the size of the in-out parameter */
	        cliAddrLen = sizeof(mClientAddress);

	        printf("Waiting to receive a message on port %d (timeout = %ds).\n", algorithm.configuration.udp_port, algorithm.configuration.timeout);

	        alarm(algorithm.configuration.timeout);
	        /* Block until receive message from a client */
	        if ((recvMsgSize = recvfrom(sock, echoBuffer, ECHOMAX, 0,
	            (struct sockaddr *) &mClientAddress, &cliAddrLen)) < 0) {
	        	if(errno == EINTR) {
	        		alarm(0);
	        		khepera3_drive_set_speed(0,0);
	        	} else {
	                DieWithError("recvfrom() failed");
	        	}
	        }
		/* UDP protocol:
		 *
		 * $K3DRV,REQ,INIT
		 * $K3DRV,RES,INIT
		 *
		 * $K3DRV,REQ,CTRL,R,L
		 * $K3DRV,RES,CTRL
		 *
		 * $K3DRV,REQ,DATA
		 * $K3DRV,RES,DATA,IRC,IR0,...,IR10,ENC,EN0,EN1
		 *
		 */

	        printf("Handling client %s\n", inet_ntoa(mClientAddress.sin_addr));

			char replyBuffer[ECHOMAX];
			parse_udp_packet(echoBuffer, replyBuffer);

			if(replyBuffer != NULL) {
			  /* Send received datagram back to the client */
			  printf("Sending reply: %s\n", replyBuffer);
			  if (sendto(sock, replyBuffer, strlen(replyBuffer), 0,
				  (struct sockaddr *) &mClientAddress, sizeof(mClientAddress)) != strlen(replyBuffer))
				  DieWithError("sendto() sent a different number of bytes than expected");
			} else {
			  //
			}
	    }
	    /* NOT REACHED */
	    }
}

int main(int argc, char **argv)
{
  Khepera3Driver *drv;
  ros::init(argc, argv, "khepera3_driver");


  (new Khepera3Driver())->run();

  return 1;
}
