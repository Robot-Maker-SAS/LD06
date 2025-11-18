#ifndef LD06_H
#define LD06_H

#include <termios.h>  // For terminal control
#include <unistd.h>   // For close, read, sleep
#include <fcntl.h>    // For open
#include <arpa/inet.h> // For inet_pton, sockaddr_in

// DATATYPES DEFINITION
// Hardware LiDAR configuration
typedef struct {
    int serialPort;
    unsigned int serialSpeed;
} lidar;

// 360-degree overview. Distances to the objects
typedef unsigned int observ[360];

// FUNCTIONS DECLARATION
int openSerialPort(lidar *lidarDevice, const char *portName, unsigned int speed);
void closeSerialPort(lidar *lidarDevice);
void read360(lidar *lidarDevice, observ data);
void udpDataSharing(observ data, const char *ip, int port);

#endif // LD06_H
