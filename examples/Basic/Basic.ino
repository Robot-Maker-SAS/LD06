#include "ld06.h"

const int LD06PWM = 1;
LD06 ld06;

bool ledState = false; 
uint32_t ref = millis();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  ld06.init(LD06PWM);
  ld06.enableFiltering();              // If filtering is enable only data that are in range will be stored in lidar scan
  ld06.setIntensityThreshold(10);      // Value from 0 to 255, discard data if intensity is lower than threshold 
  ld06.setDistanceRange(100, 1000);    // Values are in mm
  ld06.setAngleRange(0, 360);          // Values are in ° . You can set "setAngleRange(-10, 10); " or "setAngleRange(350, 10);" to get +- 10° range around 0.
  //ld06.disableFullScan();            // If you use an AVR board and want to print each points without filtering you may need to print each chunks instead of full scan and reduce MAX_PTS_SCAN to 40 in ld06.h
}

void loop() {
  if(millis() - ref > 100){
    ref=millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
  if (ld06.readScan()) {   // Read lidar packets and return true when a new full 360° scan is available
    ld06.printScanTeleplot(Serial);   // Print full scan using teleplot format (check :https://teleplot.fr/)
    /*
    ld06.printScanCSV(Serial);        // Print scan in csv format
    if(ld06.isNewScan()) {            // Even if fullScan is disabled you can know that last chunk have a loop closure
      Serial.println("This is a new scan! "); 
    }
    if(ld06.getNbPointsInScan()) {    // Give the number of point in the scan
      Serial.println("There are obstacles in the defined range !");
    }
    */
  }
}
