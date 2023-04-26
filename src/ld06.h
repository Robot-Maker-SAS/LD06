#ifndef ld06_h
#define ld06_h

#include "Arduino.h"
#include "ld06crc.h"

// Packets size
const uint8_t LD06_PACKET_SIZE = 47;      // Note: 1(Start)+1(Datalen)+2(Speed)+2(SAngle)+36(DataByte)+2(EAngle)+2(TimeStamp)+1(CRC)
const uint16_t LD06_MAX_PTS_SCAN = 1200;  // 480 is a correct value for typical 10 Hz rotation. 1200 is when using pwm pin to reduce lidar rotation frequency to 4Hz...
const uint8_t LD06_PTS_PER_PACKETS = 12;

// Headers
const uint8_t LD06_HEADER = 0x54;
const uint8_t LD06_VER_SIZE = 0x2c;

struct DataPoint {
  uint16_t distance;  // mm
  float angle;        // degrees
  int16_t x;          // mm
  int16_t y;          // mm
  uint8_t intensity;  // 0-255
};

class LD06 {
public:
  LD06(HardwareSerial &serial, uint8_t pin = 255);
  void init();
  bool readScan();

  // Print Data over Serial
  void printScanCSV(Stream &serialport);       // Print full scan using csv format
  void printScanTeleplot(Stream &serialport);  // Print full scan using teleplot format (check :https://teleplot.fr/)

  // Settings
  void enableCRC();         // Enable CRC checking
  void disableCRC();        // Disable CRC checking
  void enableFullScan();    // Enable full scan mode, readScan will return true only when a new 360° scan is available
  void disableFullScan();   // Disable full scan mode, readScan will return true for each data chunks
  void enableFiltering();   // Enable filtering, lidar points available are only the ones that pass the filter
  void disableFiltering();  // Disable filtering, all lidar points are available

  // Filtering parameters
  void setIntensityThreshold(uint8_t threshold);              // Set intensity threshold filter (0-255 value), when filtering enable points with intensity below threshold will be filtered
  void setMaxDistance(uint16_t maxDist);                      // Set max distance filter (mm value), when filtering enable points with distance value higher than max will be filtered
  void setMinDistance(uint16_t minDist);                      // Set min distance filter (mm value), when filtering enable points with distance value lower than min will be filtered
  void setDistanceRange(uint16_t minDist, uint16_t maxDist);  // Set both max and min distance filter (mm values)
  void setMaxAngle(int16_t maxAngle);                         // Set max angle filter (integer in °), when filtering enable points with angle value higher than max will be filtered
  void setMinAngle(int16_t minAngle);                         // Set min angle filter (integer in °), when filtering enable points with angle value lower than min will be filtered
  void setAngleRange(int16_t minAngle, int16_t maxAngle);     // Set both max and min angle filter (integers in °)

  // Lidar position parameters
  inline void setBasePosition(int16_t xPos, int16_t yPos, float anglePos) __attribute__((always_inline));  // Set "moving base" position if lidar is on a "moving base", x and y in mm and angle in °
  void setOffsetPosition(int16_t xPos, int16_t yPos, float anglePos);                                      // Set lidar offset positions from position of the "moving base", x and y in mm and angle in °
  void setUpsideDown(bool upsideDown = false);                                                             // Set this to true if you put the lidar upside down

  // Getters
  inline uint16_t getSpeed() __attribute__((always_inline));
  inline float getAngleStep() __attribute__((always_inline));
  inline float getTimeStamp() __attribute__((always_inline));
  inline uint16_t getNbPointsInScan() __attribute__((always_inline));
  inline bool isNewScan() __attribute__((always_inline));
  inline DataPoint getPoints(uint16_t n) __attribute__((always_inline));

  // Others
  int16_t rescaleAngle(int16_t angle);

private:
  bool readData();
  bool readDataCRC();
  bool readDataNoCRC();
  void computeData(uint8_t *values);
  inline bool filter(DataPoint point) __attribute__((always_inline));

  // Data
  bool _currentBuffer = 0;
  DataPoint _scan[LD06_MAX_PTS_SCAN][2];
  uint16_t _scanIndex[2] = { 0, 0 };
  bool _newScan = false;

  // Temporary variables
  uint16_t _speed;      // Degrees °/ second
  float _FSA;           // Degree °
  float _LSA;           // Degree °
  float _angleStep;     // Degree °
  uint16_t _timeStamp;  // ms (max value is 30000)

  // Reading buffers
  float _angles[LD06_PTS_PER_PACKETS];
  uint16_t _distances[LD06_PTS_PER_PACKETS];
  uint8_t _confidences[LD06_PTS_PER_PACKETS];

  // Settings
  HardwareSerial *_lidarSerial;
  uint8_t _pin;
  bool _useCRC = true;
  bool _fullScan = true;
  bool _useFiltering = false;
  bool _upsideDown = false;
  int16_t _xPosition = 0;
  int16_t _yPosition = 0;
  float _angularPosition = 0;
  int16_t _xOffset = 0;
  int16_t _yOffset = 0;
  float _angularOffset = 0;

  // Filtering Settings
  uint16_t _minDist = 0;      // Minimum Distance mm
  uint16_t _maxDist = 12000;  // Maximum Distance mm
  uint16_t _minAngle = 0;     // Minimum angle °
  uint16_t _maxAngle = 360;   // Maximum angle °
  uint16_t _threshold = 0;    // Minimum point intensity 0-255
};

// Inline setters

void LD06::setBasePosition(int16_t xPos = 0, int16_t yPos = 0, float anglePos = 0) {
  _xPosition = xPos;
  _yPosition = yPos;
  _angularPosition = anglePos;
}

// Inline getters

uint16_t LD06::getNbPointsInScan() {
  return _scanIndex[!_currentBuffer];
}

uint16_t LD06::getSpeed() {
  return _speed;
}

float LD06::getAngleStep() {
  return _angleStep;
}

float LD06::getTimeStamp() {
  return _timeStamp;
}

bool LD06::isNewScan() {
  return _newScan;
}

DataPoint LD06::getPoints(uint16_t n) {
  DataPoint result;
  if (n < _scanIndex[!_currentBuffer])
    result = _scan[n][!_currentBuffer];
  return result;
}

// Inline compute
/* Points filter.
   return : true if point pass the filter
*/
bool LD06::filter(DataPoint point) {
  bool distanceFilter = point.distance <= _maxDist && point.distance >= _minDist;
  bool intensityFilter = point.intensity >= _threshold;
  bool angularFilter;
  if (_minAngle <= _maxAngle) {
    angularFilter = point.angle <= _maxAngle && point.angle >= _minAngle;
  } else {
    angularFilter = point.angle <= _maxAngle || point.angle >= _minAngle;
  }
  return distanceFilter && intensityFilter && angularFilter;
}

#endif