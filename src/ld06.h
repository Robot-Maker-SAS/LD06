#ifndef ld06_h
#define ld06_h

#include "Arduino.h"
#include "ld06crc.h"

// Packets size
const uint8_t PACKET_SIZE = 47;     // note: 1(Start)+1(Datalen)+2(Speed)+2(SAngle)+36(DataByte)+2(EAngle)+2(TimeStamp)+1(CRC)
const uint16_t MAX_PTS_SCAN = 480;  // Correct value for typical 10 Hz rotation. May need to be set to 960 if rotation is set to 5Hz and no filtering.
const uint8_t PTS_PER_PACKETS = 12;

// Headers
const uint8_t HEADER = 0x54;
const uint8_t VER_SIZE = 0x2c;

struct DataPoint {
  uint16_t distance;  // mm
  float angle;        // degrees
  int16_t x;          // mm
  int16_t y;          // mm
  uint8_t intensity;  // 0-255
};

class LD06 {
public:
  LD06(HardwareSerial& serial, uint8_t pin);
  void init();
  bool readScan();

  // Print Data over Serial
  void printScanCSV(Stream &serialport);       // Print full scan using csv format
  void printScanTeleplot(Stream &serialport);  // Print full scan using teleplot format (check :https://teleplot.fr/)

  // Settings
  void enableCRC();        // Enable CRC checking
  void disableCRC();       // Disable CRC checking
  void enableFullScan();   // readScan will return true only when a new 360° scan is available
  void disableFullScan();  // readScan will return true for each data chunks
  void enableFiltering();
  void disableFiltering();
  void setIntensityThreshold(uint8_t threshold);
  void setMaxDistance(uint16_t maxDist);
  void setMinDistance(uint16_t minDist);
  void setDistanceRange(uint16_t minDist, uint16_t maxDist);
  void setMaxAngle(int16_t maxAngle);
  void setMinAngle(int16_t minAngle);
  void setAngleRange(int16_t minAngle, int16_t maxAngle);

  // Getters
  inline float getSpeed() __attribute__((always_inline));
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
  bool filter(DataPoint point);

  // Data
  DataPoint scan[MAX_PTS_SCAN];
  uint16_t _scanIndex = 0;
  bool _newScan = false;

  // Temporary variables
  float _speed;
  float _FSA;
  float _LSA;
  float _angleStep;
  int _timeStamp;

  // Reading buffers
  float angles[PTS_PER_PACKETS];
  uint16_t distances[PTS_PER_PACKETS];
  uint8_t confidences[PTS_PER_PACKETS];

  // Settings
  HardwareSerial* _lidarSerial;
  uint8_t _pin;
  bool _useCRC = true;
  bool _fullScan = true;
  bool _useFiltering = false;

  // Filtering Settings
  uint16_t _minDist = 0;     // Minimum Distance mm
  uint16_t _maxDist = 3000;  // Maximum Distance mm
  uint16_t _minAngle = 0;    // Minimum angle °
  uint16_t _maxAngle = 360;  // Maximum angle °
  uint16_t _threshold = 0;   // Minimum point intensity 0-255
};

// Getters

uint16_t LD06::getNbPointsInScan() {
  return _scanIndex;
}

float LD06::getSpeed() {
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
  if (n < _scanIndex)
    result = scan[n];
  return result;
}

#endif