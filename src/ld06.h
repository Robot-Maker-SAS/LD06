#ifndef ld06_h
#define ld06_h

#include "Arduino.h"
#include "ld06crc.h"


// Configuration 
#define LD06_COMPUTE_XY        // Comment this if you don't want to us x and y computation
#define LD06_MAX_PTS_SCAN 1200 // 480 is a correct value for typical 10 Hz rotation. 1200 is when using pwm pin to reduce lidar rotation frequency to 4Hz... You can reduce it to save memory space

// Packets size
const uint8_t LD06_PACKET_SIZE = 47;   // Note: 1(Start)+1(Datalen)+2(Speed)+2(SAngle)+36(DataByte)+2(EAngle)+2(TimeStamp)+1(CRC)
const uint8_t LD06_PTS_PER_PACKETS = 12;
const uint8_t LD06_MEASURE_SIZE = 3;

// Headers
const uint8_t LD06_HEADER = 0x54;
const uint8_t LD06_VER_SIZE = 0x2c;

// Parameters
const uint8_t LD06_ANGLE_STEP_MAX = 5;

// Data structures
struct LD06Measure {
  union {
    struct {
      uint16_t distance;  // Unit is mm
      uint8_t intensity;  // Unsigned value 0-255}
    } __attribute__((packed));
    uint8_t bytes[LD06_MEASURE_SIZE];
  };
} __attribute__((packed));

struct LD06Packet {
  union {
    struct {
      uint8_t header;                              // LD06_HEADER
      uint8_t version_size;                        // LD06_VER_SIZE
      uint16_t lidarSpeed;                         // Degrees ° / second
      uint16_t startAngle;                         // Unit is 0.01 Degree°
      LD06Measure measures[LD06_PTS_PER_PACKETS];  // 12 distances data distance + intensity
      uint16_t endAngle;                           // Unit is 0.01 Degree°
      uint16_t timeStamp;                          // Unit id ms (max value is 30000)
      uint8_t crc;                                 // Checksum
    } __attribute__((packed));
    uint8_t bytes[LD06_PACKET_SIZE];
  };
} __attribute__((packed));

struct LD06PacketHandler {
  LD06Packet packet;
  uint8_t computedCrc;
  uint8_t index;
};

struct DataPoint {
  uint16_t distance = 0;  // mm
  float angle = 0;        // degrees
  #ifdef LD06_COMPUTE_XY
  int16_t x = 0;          // mm
  int16_t y = 0;          // mm
  #endif
  uint8_t intensity = 0;  // 0-255
};

struct DataPointHandler {
  DataPoint points[LD06_MAX_PTS_SCAN];
  uint16_t index;
};

class LD06 {
public:
  LD06(HardwareSerial &serial, uint8_t pin = 255);
  void init();
  bool readScan();

  // Print Data over Serial
  void printScanCSV(Stream &serialport);       // Print full scan using csv format
  #ifdef LD06_COMPUTE_XY
  void printScanTeleplot(Stream &serialport);  // Print full scan using teleplot format (check :https://teleplot.fr/)
  #endif

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
  void setUpsideDown(bool upsideDown = false);  // Set this to true if you put the lidar upside down
  #ifdef LD06_COMPUTE_XY
  inline void setBasePosition(int16_t xPos, int16_t yPos, float anglePos) __attribute__((always_inline));  // Set "moving base" position if lidar is on a "moving base", x and y in mm and angle in °
  void setOffsetPosition(int16_t xPos, int16_t yPos, float anglePos);                                      // Set lidar offset positions from position of the "moving base", x and y in mm and angle in °
  #endif

  // Getters
  inline uint16_t getSpeed() __attribute__((always_inline));
  inline float getAngleStep() __attribute__((always_inline));
  inline float getTimeStamp() __attribute__((always_inline));
  inline uint16_t getNbPointsInScan() __attribute__((always_inline));
  inline bool isNewScan() __attribute__((always_inline));
  inline DataPoint *getPoints(uint16_t n) __attribute__((always_inline));
  inline uint16_t getChecksumFailCount() __attribute__((always_inline));

  // Others
  inline bool isChecksumOk() __attribute__((always_inline));
  int16_t rescaleAngle(int16_t angle);

private:
  bool readData();
  bool readDataCRC();
  bool readDataNoCRC();
  void computeData();
  inline bool filter(const DataPoint &point) __attribute__((always_inline));
  void swapBuffers();

  // Data
  DataPointHandler _scanA;
  DataPointHandler _scanB;
  DataPointHandler *_currentScan = &_scanA;
  DataPointHandler *_previousScan;
  bool _currentBuffer = 0;  //

  bool _newScan = false;
  uint16_t _checksumFailCount = 0;

  // Reading buffers
  LD06PacketHandler _receivedData;
  float _angles[LD06_PTS_PER_PACKETS];

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
#ifdef LD06_COMPUTE_XY
void LD06::setBasePosition(int16_t xPos = 0, int16_t yPos = 0, float anglePos = 0) {
  _xPosition = xPos;
  _yPosition = yPos;
  _angularPosition = anglePos;
}
#endif

// Inline getters

uint16_t LD06::getNbPointsInScan() {
  return _previousScan->index;
}

uint16_t LD06::getSpeed() {
  return _receivedData.packet.lidarSpeed;
}

float LD06::getAngleStep() {
  float fsa = (float)_receivedData.packet.startAngle / 100.0;
  float lsa = (float)_receivedData.packet.endAngle / 100.0;

  float range = lsa - fsa;
  if (range < 0)
    range += 360;

  float angleStep = range / (LD06_PTS_PER_PACKETS - 1);
  return angleStep;
}

float LD06::getTimeStamp() {
  return _receivedData.packet.timeStamp;
}

bool LD06::isNewScan() {
  return _newScan;
}

DataPoint *LD06::getPoints(uint16_t n) {
  static DataPoint empty;
  DataPoint *result = &empty;
  if (n < _previousScan->index)
    result = &_previousScan->points[n];
  return result;
}

uint16_t LD06::getChecksumFailCount() {
  return _checksumFailCount;
}

// Inline compute
/* Checksum failure check. 
   return true only if a failure occured since last time you checked
*/
bool LD06::isChecksumOk() {
	static uint16_t previousChecksumFailCount = 0;
	uint16_t checksumFailCount = getChecksumFailCount();
	if(checksumFailCount != previousChecksumFailCount) {
		checksumFailCount = previousChecksumFailCount;
		return false;
	}
	return true;
}

/* Points filter.
   return : true if point pass the filter
*/
bool LD06::filter(const DataPoint &point) {
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