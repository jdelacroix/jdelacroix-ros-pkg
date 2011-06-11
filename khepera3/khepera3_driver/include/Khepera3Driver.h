/*
 * K3Driver.h
 *
 *  Created on: Jun 8, 2011
 *      Author: jdelacroix
 */

#ifndef K3DRIVER_H_
#define K3DRIVER_H_

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket() and bind() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */

#include <errno.h>
#include <signal.h>

#include <math.h>

class Khepera3Driver {

private:

	std::string mIPAddress;
	int mPort;

	struct sigaction mSignalCallback;	/* Signal for timeouts */

	int mSocket;                        /* Socket */
	struct sockaddr_in mServerAddress; 	/* Local address */
	struct sockaddr_in mClientAddress; 	/* Client address */

	void mAlarmCallback(int arg);
	void mConnect();

public:

	Khepera3Driver();
	virtual ~Khepera3Driver();
	void run();
};

#endif /* K3DRIVER_H_ */
