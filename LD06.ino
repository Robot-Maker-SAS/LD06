#include "ld06.h"

LD06 ld06;

void setup() {
  Serial.begin(115200);
  ld06.init(1);
}

void loop() {
  if (ld06.readFullScan()) {     // Read lidar packets and return true when a new full 360° scan is available
    ld06.teleplotPrintScan();    // Print full scan using teleplot format (check :https://teleplot.fr/)
  }
}
