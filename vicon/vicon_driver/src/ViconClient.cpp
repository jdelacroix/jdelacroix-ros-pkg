#include "ViconClient.h"

ViconClient::ViconClient()
{
  mViconDataPublisher = mNodeHandle.advertise<vicon_driver::ViconData>("/vicon/batch_data", 1);
  
  std::string param;
  
//   isChunkedMode = false;
//   if(mNodeHandle.getParam("chunked", param))
//     if(param.compare("true")==0)
//       isChunkedMode = true;
}

ViconClient::~ViconClient()
{
//   free(mRecvBuffer);
}

void ViconClient::run(int delay)
{ 
  try
  {
    int sockfd, rc, len;
    struct sockaddr_in broadcastAddr, clientAddr;
    struct hostent *broadcastServer = gethostbyname(BROADCAST_ADDRESS);
    char mRecvBuffer[MAXRCVLENGTH];
    
//     mRecvBuffer = calloc(MAXRCVLENGTH, sizeof(char));
    
    if(broadcastServer == NULL)
    {
	ROS_ERROR("Unknown broadcast address.");
	exit(1);
    }
    
    /* build the server's Internet address */
    memset(&broadcastAddr,0,sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    memcpy((char *) &broadcastAddr.sin_addr.s_addr, 
	 broadcastServer->h_addr_list[0], broadcastServer->h_length);
    //addr.sin_addr.s_addr = inet_addr(BROADCAST_ADDRESS);
    broadcastAddr.sin_port = htons(BROADCAST_PORT);
    
    /* check command line arguments */
    ROS_INFO("Listening to %s on UDP port %d.", BROADCAST_ADDRESS, BROADCAST_PORT);

    /* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) 
        ROS_ERROR("Error opening UDP socket.");

    /* socket: set socket options */
    struct timeval tv;
    tv.tv_sec = 10;  /* 30 Secs Timeout */
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,(char *)&tv,sizeof(struct timeval));
    
    /* socket: bind to socket */
    memset(&clientAddr, 0, sizeof(clientAddr));
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    clientAddr.sin_port = htons(BROADCAST_PORT);
    
    rc = bind(sockfd, (struct sockaddr *) &clientAddr, sizeof(clientAddr));
    if(rc<0)
    {
      ROS_ERROR("Cannot bind to port %d.", BROADCAST_PORT);
      exit(1);
    }
    
    ros::Rate mLoopRate(1000.0/((double) delay));
    
    while(ros::ok())
    {
    
      /* print the server's reply */
      len = sizeof(broadcastAddr);
      rc = recvfrom(sockfd, mRecvBuffer, MAXRCVLENGTH, 0, (struct sockaddr *) &broadcastAddr, (socklen_t *) &len);
      
      if (rc < 0)
      {
	ROS_ERROR("Error receiving data or timeout.");
	rc = 0;
      }
      else
      {
	mRecvBuffer[rc] = '\0';
	ROS_INFO("Data packet (%d bytes) from Vicon: %s", rc, mRecvBuffer);
	
	/* parse vicon data */
	parse(mRecvBuffer);
	
      }
      
      ros::spinOnce();
      //mLoopRate.sleep();
    }
  }
  catch(int ec)
  {
    ROS_ERROR("Unexpected error. Exiting.");
  }
}

void ViconClient::parse(char *buffer) {
	vicon_driver::ViconData mViconData;
  	/* parse the Vicon data */
	char *str1, *token;
        char *saveptr1, *saveptr2;
        int j; int c = 0;
	
	char *delim1 = "()"; char *delim2 = ",";
	
	for (j = 1, str1 = buffer; ; j++, str1 = NULL) {
	    token = strtok_r(str1, delim1, &saveptr1);
	    if (token == NULL)
	    	break;

	    //printf("%d: %s\n", j, token);

// 		subtoken = strtok_r(token, delim2, &saveptr2);
// 		if (subtoken == NULL)
// 		    break;
		//printf(" --> %s\n", subtoken);
	
	    c++;

	    mViconData.id[c-1] = atoi(strtok_r(token, delim2, &saveptr2));

	    mViconData.position[c-1].x = atof(strtok_r(NULL, delim2, &saveptr2));
	    mViconData.position[c-1].y = atof(strtok_r(NULL, delim2, &saveptr2));
	    mViconData.position[c-1].z = atof(strtok_r(NULL, delim2, &saveptr2));
	    
	    mViconData.orientation[c-1].x = atof(strtok_r(NULL, delim2, &saveptr2));
	    mViconData.orientation[c-1].y = atof(strtok_r(NULL, delim2, &saveptr2));
	    mViconData.orientation[c-1].z = atof(strtok_r(NULL, delim2, &saveptr2));
	      
	}

	mViconData.count = c;
	mViconDataPublisher.publish(mViconData);
}

int main(int argc, char **argv)
{
  
  int delay = 100;
  
  if(argc < 2) {
    ROS_INFO("Using default delay of 200ms.");
    ROS_INFO("Set delay with './viconux <delay ms>'.");
  } else {
    delay = atoi(argv[1]);
    ROS_INFO("Setting delay to %dms.", delay);
  }
  
  
  ros::init(argc, argv, "batch_client");
  
  
  
  ViconClient().run(delay);
  
  return 0;
  
}
