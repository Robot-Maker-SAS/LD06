#include "LD06.h"
#include <stdio.h>

int main(void) {
    lidar lidarDevice;

    // Відкриваємо серійний порт
    if (openSerialPort(&lidarDevice, "/dev/ttyUSB0", B230400) == -1) {
        return -1;
    }

    printf("Serial port opened successfully.\n");

    while (1) {
        // Зчитуємо дані з LiDAR
        observ data;
        read360(&lidarDevice, data);

        // Передаємо дані через UDP
        udpDataSharing(data, "127.0.0.1", 12348);  // Використання нового порту

        printf("Data sent over UDP.\n");

        sleep(1);
    }

    // Закриваємо серійний порт
    closeSerialPort(&lidarDevice);
    return 0;
}
