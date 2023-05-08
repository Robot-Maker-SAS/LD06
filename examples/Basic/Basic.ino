// "Simple" sketch to show what you can do with the ld06 lidar https://www.robot-maker.com/shop/capteurs/468-lidar-ld06-468.html
// This sketch have been mainly tested with a teensy 4.0 board, but it can be compatible with other boards.
// If this sketch does not compile because of memory usage for the board you want, you can reduce MAX_PTS_SCAN to 40 or lower in ld06.h file
// If you have any issue feel free to ask for support : https://www.robot-maker.com/forum/topic/14388-test-ld06-library/

#include "ld06.h"
LD06 ld06(Serial1);  // ld06 constructor, need to specify the hardware serial you want to use with the ld06 ( You can use Serial instead of Serial1 on arduino uno )

/*
// ld06 constructor with pwm pin
#define LD06PWMPIN 1             // Lidar PWM pin, by default it is not needed.
LD06 ld06(Serial1, LD06PWMPIN);  // ld06 constructor with hardware serial and pwm pin.
*/

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

  Serial.begin(115200);  // Start the Serial you want to display data
  ld06.init();           // Init Serial Lidar to 230400 Bauds and set lidar pwm pin mode if pwm pin is specified

  /*
  // PWM config
  #ifdef LD06PWMPIN
  analogWriteFrequency(LD06PWMPIN, 30000);  // if lidar pwm is connected on a pin, you can adjust the lidar speed using a pwm pin frequency set to 30kHz ( 20KHz to 50Khz)
  analogWrite(LD06PWMPIN, 40);              // 40 is one of the lowest working value around 1120pts / scan angle step 0.32° 4HZ Scan rate around 10HZ when PWM duty at 102 = 40% duty
  #endif
  */

  /*
  // Lidar position config
  ld06.setOffsetPosition(0, 0, 0);  // Set Lidar x y and angular offset positions, x and y in mm and angle in °
  ld06.setBasePosition(0, 0, 0);    // Set "moving base" initial position if lidar is on a "moving base"
  ld06.setUpsideDown(true);         // Set to true if you ar setting your lidar upside down ... By default the lidar is not cosiderer upside down.
  */

  /*
  //Points filtering config
  ld06.enableFiltering();            // If filtering is enable only data that are in range will be stored in lidar scan
  ld06.setIntensityThreshold(200);   // Value from 0 to 255, discard data if intensity is lower than threshold. 200 is a standard value specified in the datasheet to remove false positive detection.
  ld06.setDistanceRange(100, 1000);  // Values are in mm
  ld06.setAngleRange(0, 360);        // Values are in ° . You can set "setAngleRange(-10, 10); " or "setAngleRange(350, 10);" to get +- 10° range around 0.
  */

  /*
  // Other modes
  ld06.disableCRC();       // CRC usage is active by default but you can disable it
  ld06.disableFullScan();  // If you use an AVR board and want to print each points without filtering you may need to print each chunks instead of full scan
  */
}

void loop() {
  toggleBuiltinLed();                 // Only to show that the board is alive you can delete this line
  //ld06.setBasePosition(x, y, angle);  // update "moving base" position in real time if lidar is on a "moving base"

  if (ld06.readScan()) {             // Read lidar packets and return true when a new full 360° scan is available
    ld06.printScanTeleplot(Serial);  // Print full scan using teleplot format (check :https://teleplot.fr/)
    /*
    // Other displays examples and getters
    ld06.printScanCSV(Serial);  // Print scan in csv format
    if (ld06.isNewScan()) {     // Even if fullScan is disabled you can know when last data chunk have a loop closure
      Serial.println("This is a new scan! ");
    }
    Serial.println(ld06.getSpeed());        // Show the lidar speed in degrees ° / second
    Serial.println(ld06.getAngleStep());    // Show the angle step resolution in degree
    uint16_t n = ld06.getNbPointsInScan();  // Give the number of points in the scan, can be usefull with filtering to tell if there are abstacles around the lidar
    Serial.println(String() + "There are " + ld06.getNbPointsInScan() + " lidar points in the defined range !");
    for (uint16_t i = 0; i < n; i++) {
      Serial.println(String() + ld06.getPoints(i)->angle + "," + ld06.getPoints(i)->distance + ";");  // example to show how to extract data. ->x, ->y and ->intensity are also available.
    }
    */
  }
}