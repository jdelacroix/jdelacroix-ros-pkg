#ifndef _VICON_CLIENT_H_
#define _VICON_CLIENT_H_

#include <ros/ros.h>
#include <sstream>

#include <vicon_driver/ViconData.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h> 

#define BROADCAST_PORT 1026
#define BROADCAST_ADDRESS "192.168.1.255"

#define MAXRCVLENGTH 256

class ViconClient
{

  private:
    
    ros::NodeHandle mNodeHandle;
    ros::Publisher mViconDataPublisher;
    
    void parse(char *buffer);
    
  public:
    
    ViconClient();
    ~ViconClient();
    
    void run(int delay);

};

#endif
