/*
 * (c) 2006-2008 EPFL, Lausanne, Switzerland
 * Thomas Lochmatter
 *
 * Copyright (C) 2014 Georgia Tech Research Corporation
 * see the LICENSE file included with this software
 */

#include "khepera3.h"
#include "commandline.h"
#include "odometry_track.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket() and bind() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */

#include <errno.h>
#include <signal.h>

#define ECHOMAX 255     /* Longest string to echo */

void DieWithError(char *errorMessage) {
    perror(errorMessage);
    exit(1);
}  /* External error handling function */


// Algorithm parameters and results
struct sAlgorithm {
	struct {
        int udp_port;
	int verbosity;
	int timeout;
	} configuration;
	struct {
		void (*hook)();
		int turn_counter;
	} state;
};

// Algorithm instance
struct sAlgorithm algorithm;
struct sOdometryTrack ot;

// Prints the help text.
void help() {
	printf("Template program.\n");
	printf("\n");
	printf("Usage: k3driver [OPTIONS]\n");
	printf("\n");
	printf("Options:\n");
	printf("  -p --port PORT       Sets the UDP port to listen on (default: 4555)\n");
	printf("  -t --timeout MS	   Sets the timeout for waiting on a request (default: 5s)\n");
	printf("  -v --verbosity V     Sets the verbosity level (0=quiet, 1=default, 2=verbose, 3=very verbose, ...)\n");
	printf("\n");
}

// State functions
void state_forward();
void state_turn();
void state_udp();
void parse_udp_packet(char *msg, char *reply);

void alarm_callback(int ignored) { }

// Initializes the algorithm.
void algorithm_init() {
	// Initialize modules
	khepera3_init();
	odometry_track_init();

	// Read command line options
	algorithm.configuration.udp_port = commandline_option_value_int("-p","--port", 4555);
	algorithm.configuration.verbosity = commandline_option_value_int("-v", "--verbosity", 1);
	algorithm.configuration.timeout = commandline_option_value_int("-t", "--timeout", 2);

	// Set the initial state
	algorithm.state.hook = &state_udp;
}

// Runs the algorithm by calling the appropriate state function.
void algorithm_run() {
	// Put the wheels in normal (control) mode
	
	khepera3_motor_initialize(&khepera3.motor_left);
	khepera3_motor_initialize(&khepera3.motor_right);

	khepera3_drive_start();
	
	khepera3_motor_set_current_position(&khepera3.motor_left, 0);
	khepera3_motor_set_current_position(&khepera3.motor_right, 0);
	
	odometry_track_start(&ot);

	// Execute the current state
	while (1) {
		algorithm.state.hook();
	}
}

void state_udp() {
    
    int sock;                        /* Socket */
    struct sockaddr_in echoServAddr; /* Local address */
    struct sockaddr_in echoClntAddr; /* Client address */
    unsigned int cliAddrLen;         /* Length of incoming message */
    char echoBuffer[ECHOMAX];        /* Buffer for echo string */
    unsigned short echoServPort;     /* Server port */
    int recvMsgSize;                 /* Size of received message */

    echoServPort = algorithm.configuration.udp_port; /* local port */

    /* Create socket for sending/receiving datagrams */
    if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
        DieWithError("socket() failed");

    /* Construct local address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
    echoServAddr.sin_family = AF_INET;                /* Internet address family */
    echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
    echoServAddr.sin_port = htons(echoServPort);      /* Local port */

    /* Bind to the local address */
    if (bind(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
        DieWithError("bind() failed");

    /* Set reading timeout */

    struct sigaction callback;
    callback.sa_handler = alarm_callback;
    if (sigfillset(&callback.sa_mask) < 0)
    	DieWithError("sigfillset() failed");
    callback.sa_flags = 0;

    if (sigaction(SIGALRM, &callback, 0) < 0)
    	DieWithError("sigaction() failed");


  
    for (;;) /* Run forever */
    {
        /* Set the size of the in-out parameter */
        cliAddrLen = sizeof(echoClntAddr);

        printf("Waiting to receive a message on port %d (timeout = %ds).\n", algorithm.configuration.udp_port, algorithm.configuration.timeout);

        alarm(algorithm.configuration.timeout);
        /* Block until receive message from a client */
        while ((recvMsgSize = recvfrom(sock, echoBuffer, ECHOMAX, 0,
            (struct sockaddr *) &echoClntAddr, &cliAddrLen)) < 0) {
        	if(errno == EINTR) {
        		alarm(0);
        		printf("timeout(): stopping motors\n");
        		khepera3_drive_set_speed(0,0);
        		//alarm(algorithm.configuration.timeout);
                printf("Waiting to receive a message on port %d.\n", algorithm.configuration.udp_port, algorithm.configuration.timeout);
        	} else {
                DieWithError("recvfrom() failed");
        	}
        }
        alarm(0);
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
        echoBuffer[recvMsgSize] = '\0';



        printf("Handling client %s\n", inet_ntoa(echoClntAddr.sin_addr));

        alarm(0);

		char replyBuffer[ECHOMAX];
		char echo[strlen(echoBuffer)];
		sprintf(echo, "%s", echoBuffer);
		parse_udp_packet(echo, replyBuffer);

		if(replyBuffer != NULL) {
		  /* Send received datagram back to the client */
		  printf("Sending reply: %s\n", replyBuffer);
		  if (sendto(sock, replyBuffer, strlen(replyBuffer), 0,
			  (struct sockaddr *) &echoClntAddr, sizeof(echoClntAddr)) != strlen(replyBuffer))
			  DieWithError("sendto() sent a different number of bytes than expected");
		} else {
		  //
		}
    }
    /* NOT REACHED */
    }
    
void parse_udp_packet(char *msg, char *reply) {
  
	printf("Parsing received datagram: %s\n", msg);
  
	char *token;
	char *saveptr;
	const char *delim = ",";

	token = strtok_r(msg, delim, &saveptr);
	if (token == NULL || strcmp(token, "$K3DRV") != 0) {
		perror("Parsing failed: Expected $K3DRV token.");
		reply = NULL;
		return;
	}

	token = strtok_r(NULL, delim, &saveptr);
	if (token == NULL || strcmp(token, "REQ") != 0) {
		perror("Parsing failed: Expected REQ token.");
		reply = NULL;
		return;
	}

	token = strtok_r(NULL, delim, &saveptr);
	if (token == NULL) {
		perror("Parsing failed: Expected TYPE token.");
		reply = NULL;
		return;
	}

	if (strcmp(token, "CTRL") == 0) {

		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL) {
			perror("Parsing failed: CTRL expected VEL_R token.");
			reply = NULL;
			return;
		}

		const int vel_r_k3 = atoi(token);

		token = strtok_r(NULL, delim, &saveptr);
		if (token == NULL) {
			perror("Parsing failed: CTRL expected VEL_L token.");
			reply = NULL;
			return;
		}

		const int vel_l_k3 = atoi(token);

		printf("Sending motor control (right,left); (%d,%d)\n", vel_r_k3, vel_l_k3);
		khepera3_drive_set_speed(vel_l_k3, vel_r_k3);


		sprintf(reply, "$K3DRV,CTRL,RES");

	} else if (strcmp(token, "DATA") == 0) {


//		int ir[11];
//		for(i=0; i<11; i++) ir[i] = khepera3.infrared_proximity.sensor[i];

		khepera3_infrared_proximity();
		int *ir = khepera3.infrared_proximity.sensor;

	// 	int sonar[5];
	// 	for(i=0; i<5; i++) sonar[i] = khepera3.ultra_sound.sensor.distance[i];

		odometry_track_step(&ot);
		int enc_r = ot.state.pos_right_prev;
		int enc_l = ot.state.pos_left_prev;

		sprintf(reply, "$K3DRV,RES,DATA,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%d,%d,%d", "IR", 11,
												  ir[0], ir[1], ir[2], ir[3], ir[4], ir[5],
												  ir[6], ir[7], ir[8], ir[9], ir[10],
												  "ENC", 2, enc_r, enc_l);
	} else if (strcmp(token, "INIT") == 0) {
		khepera3_motor_set_current_position(&khepera3.motor_left, 0);
		khepera3_motor_set_current_position(&khepera3.motor_right, 0);

		sprintf(reply, "$K3DRV,RES,INIT");
	} else {
		printf("Got %s instead. %d vs. %d\n", token, strlen(token), strlen("INIT"));
		int i;
		for(i=0; i<strlen(token); i++) {
			printf("[%c]", token[i]);
		}
		printf("\n");
		perror("Parsing failed: Expected CTRL, DATA, or INIT token.");
		reply = NULL;
		return;
	}
  
  return;
}

// retrive sonar sensor data

// int sonarDistance(int i) {
// 	return khepera3.ultrasound.sensor[i].distance[0];
// }



// Main program.
int main(int argc, char *argv[]) {
	// Command line parsing
	commandline_init();
	commandline_parse(argc, argv);

	// Help
	if (commandline_option_provided("-h", "--help")) {
		help();
		exit(1);
	}

	// Initialization
	algorithm_init();

	// Run
	algorithm_run();
	return 0;
}
