// "Simple" sketch to show what you can do with the ld06 lidar https://www.robot-maker.com/shop/capteurs/468-lidar-ld06-468.html
// This sketch have been mainly tested with a teensy 4.0 board, but it can be compatible with other boards.
// If this sketch does not compile because of memory usage for the board you want, you can reduce MAX_PTS_SCAN to 40 or lower in ld06.h file
// If you have any issue feel free to ask for support : https://www.robot-maker.com/forum/topic/14388-test-ld06-library/

#include "ld06.h"

const uint8_t LD06PWMPIN = 1;    // Lidar PWM pin
LD06 ld06(Serial1, LD06PWMPIN);  // ld06 constructor requiring an hardware serial and a pwm pin use. ( You can use Serial instead of Serial1 on arduino uno )

// Toggle builtin led to show that the board is alive, you can just call this function in the main loop.
void toggleBuiltinLed() {
#ifdef LED_BUILTIN
  static bool ledState = false;
  static uint32_t ref = millis();
  if (millis() - ref > 100) {
    pinMode(LED_BUILTIN, OUTPUT);
    ref = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
#endif
}

void setup() {
  Serial.begin(115200);              // Start the Serial you want to display data
  ld06.init();                       // Init Serial Lidar to 2 and set lidar pwm pin
  ld06.enableFiltering();            // If filtering is enable only data that are in range will be stored in lidar scan
  ld06.setIntensityThreshold(10);    // Value from 0 to 255, discard data if intensity is lower than threshold
  ld06.setDistanceRange(100, 1000);  // Values are in mm
  ld06.setAngleRange(0, 360);        // Values are in ° . You can set "setAngleRange(-10, 10); " or "setAngleRange(350, 10);" to get +- 10° range around 0.
  //ld06.disableCRC();                // CRC usage is active by default but you can disable it
  //ld06.disableFullScan();           // If you use an AVR board and want to print each points without filtering you may need to print each chunks instead of full scan
}

void loop() {
  toggleBuiltinLed();                // Only to show that the board is alive you can delete this line
  if (ld06.readScan()) {             // Read lidar packets and return true when a new full 360° scan is available
    ld06.printScanTeleplot(Serial);  // Print full scan using teleplot format (check :https://teleplot.fr/)
    /*
    ld06.printScanCSV(Serial);       // Print scan in csv format
    if (ld06.isNewScan()) {          // Even if fullScan is disabled you can know when last data chunk have a loop closure
      Serial.println("This is a new scan! ");
    }
    if (ld06.getNbPointsInScan()) {   // Give the number of point in the scan, can be usefull with filtering to tell if there are abstacles around the lidar!
      Serial.println("There are obstacles in the defined range !");
    }
    */
  }
}