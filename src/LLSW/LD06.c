#include "LD06.h"
#include <stdio.h>  // For printf

int openSerialPort(lidar *lidarDevice, const char *portName, unsigned int speed) {
    lidarDevice->serialPort = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (lidarDevice->serialPort == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(lidarDevice->serialPort, &options);

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; // 8 data bits

    options.c_cflag &= ~CRTSCTS; // No hardware flow control
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, local mode

    options.c_lflag &= ~ICANON;
    options.c_lflag &= ~ECHO; // Disable echo
    options.c_lflag &= ~ECHOE; // Disable erasure
    options.c_lflag &= ~ECHONL;
    options.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // No XON/XOFF software flow control
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    options.c_oflag &= ~OPOST; // No output processing
    options.c_oflag &= ~ONLCR;

    options.c_cc[VTIME] = 10; // Read timeout
    options.c_cc[VMIN] = 0;

    tcsetattr(lidarDevice->serialPort, TCSANOW, &options);

    return 0;
}

void closeSerialPort(lidar *lidarDevice) {
    close(lidarDevice->serialPort);
    printf("Serial port closed.\n");
}

void read360(lidar *lidarDevice, observ data) {
    unsigned char buffer[1024];
    int bytesRead = read(lidarDevice->serialPort, buffer, sizeof(buffer));
    if (bytesRead > 0) {
        printf("Bytes read: %d\n", bytesRead);
        // Обробка даних з LiDAR
        for (int i = 0; i < bytesRead / 2 && i < 360; i++) {
            data[i] = buffer[2 * i] | (buffer[2 * i + 1] << 8);
        }
    } else {
        printf("No data read from LiDAR.\n");
    }
}

void udpDataSharing(observ data, const char *ip, int port) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Unable to create UDP socket");
        return;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serverAddr.sin_addr);

    int sentBytes = sendto(sockfd, data, sizeof(observ), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if (sentBytes < 0) {
        perror("Failed to send data over UDP");
    } else {
        printf("Sent %d bytes over UDP.\n", sentBytes);
    }
    
    close(sockfd);
}
