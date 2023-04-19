#include "ld06.h"


#define LIDAR_SERIAL Serial1

void LD06::init(int pin) {
  _pin = pin;
  LIDAR_SERIAL.begin(230400);
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);
}

/* Read lidar packet data without checking CRC,
 * return : true if a valid package was received
 */
bool LD06::readData() {
  return _useCRC ? readDataCRC() : readDataNoCRC();
}

/* Read lidar packet data without checking CRC,
 * return : true if a valid packet is received
 */
bool LD06::readDataCRC() {
  bool result = 0;
  static uint8_t crc = 0;
  static uint8_t n = 0;
  static uint8_t lidarData[PACKET_SIZE];
  while (LIDAR_SERIAL.available()) {
    uint8_t current = LIDAR_SERIAL.read();
    if (n > 1 || (n == 0 && current == HEADER) || (n == 1 && current == VER_SIZE)) {
      lidarData[n] = current;
      if (n < PACKET_SIZE - 1) {
        crc = CrcTable[crc ^ current];
        n++;
      } else {
        if (crc == current) {
          computeData(lidarData);
          result = 1;
        } else {
          // TODO Handle CRC error
        }
        n = 0;
        crc = 0;
      }
    } else {
      n = 0;
      crc = 0;
    }
  }
  return result;
}

/* Read lidar packet data without checking CRC,
 * return : true if a packet is received
 */
bool LD06::readDataNoCRC() {
  static uint8_t n = 0;
  static uint8_t lidarData[PACKET_SIZE];
  while (LIDAR_SERIAL.available()) {
    uint8_t current = LIDAR_SERIAL.read();
    if (n > 1 || (n == 0 && current == HEADER) || (n == 1 && current == VER_SIZE)) {
      lidarData[n] = current;
      if (n == PACKET_SIZE - 1) {
        computeData(lidarData);
        n = 0;
        return true;
      }
    } else
      n = 0;
  }
  return false;
}

/* Read lidar packets and compute x y points coordinates.
 * return : true if a new full 360° scan is available when in _fullScan mode
 * or true each time partial data chunks are available if not in _fullScan mode.
 */
bool LD06::readScan() {
  static DataPoint currentScan[MAX_PTS_SCAN];
  static uint16_t currentScanIndex = 0;
  static bool isInit = false;
  static float lastAngle = 0;
  static float startAngle = 0;
  _newScan = false;
  bool result = false;
  DataPoint data;

  if (readData()) {
    for (int i = 0; i < PTS_PER_PACKETS; i++) {
      if (angles[i] < lastAngle) {
        if (!isInit) {
          isInit = true;
        } else {
          if (lastAngle - startAngle > 340) {  // TODO check if this condition is enough to prevent partial scan prints
            _newScan = true;
            result = true;
          }
          startAngle = angles[i];

          if (_fullScan) {
            _scanIndex = 0;
            for (uint16_t j = 0; j < currentScanIndex; j++) {
              if (!_useFiltering || filter(currentScan[j])) {
                scan[_scanIndex] = currentScan[j];
                _scanIndex++;
              }
            }
            currentScanIndex = 0;
          }
        }
      }
      lastAngle = angles[i];
      if (currentScanIndex < MAX_PTS_SCAN) {
        data.angle = angles[i];
        data.distance = distances[i];
        data.x = data.distance * cos(data.angle * PI / 180);
        data.y = -data.distance * sin(data.angle * PI / 180);
        data.intensity = confidences[i];
        currentScan[currentScanIndex] = data;
        currentScanIndex++;
      }
    }
  }
  return result;
}

void LD06::computeData(uint8_t *values) {
  _speed = float(values[3] << 8 | values[2]) / 100;
  _FSA = float(values[5] << 8 | values[4]) / 100;
  _LSA = float(values[PACKET_SIZE - 4] << 8 | values[PACKET_SIZE - 5]) / 100;
  _timeStamp = int(values[PACKET_SIZE - 2] << 8 | values[PACKET_SIZE - 3]);

  _angleStep = ((_LSA - _FSA > 0) ? (_LSA - _FSA) / (PTS_PER_PACKETS - 1) : (_LSA + (360 - _FSA)) / (PTS_PER_PACKETS - 1));

  if (_angleStep > 20)
    return;

  for (uint16_t i = 0; i < PTS_PER_PACKETS; i++) {
    float raw_deg = _FSA + i * _angleStep;
    angles[i] = (raw_deg <= 360 ? raw_deg : raw_deg - 360);
    confidences[i] = values[8 + i * 3];
    distances[i] = int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]);
  }
}

/* Points filter.
 * return : true if point pass the filter
 */
bool LD06::filter(DataPoint point) {
  if (_minAngle < _maxAngle) {
    return point.angle <= _maxAngle && point.angle >= _minAngle && point.distance <= _maxDist && point.distance >= _minDist && point.intensity >= _threshold;
  } else {
    return (point.angle <= _maxAngle || point.angle >= _minAngle) && point.distance <= _maxDist && point.distance >= _minDist && point.intensity >= _threshold;
  }
}

// Print full scan using csv format
void LD06::printScanCSV(Stream &serialport) {
  static bool init = false;
  if (!init) {
    serialport.println(F("Angle(°),Distance(mm),x(mm),y(mm)"));
    init = true;
  }
  if (_scanIndex) {
    for (uint16_t i = 0; i < _scanIndex; i++) {
      serialport.println(String() + scan[i].angle + "," + scan[i].distance + "," + scan[i].x + "," + scan[i].y);
    }
    serialport.println("");
  }
}

// Print full scan using teleplot format (check :https://teleplot.fr/)
void LD06::printScanTeleplot(Stream &serialport) {
  if (_scanIndex) {
    serialport.print(F(">lidar:"));
    for (uint16_t i = 0; i < _scanIndex; i++) {
      serialport.print(String() + scan[i].x + ":" + scan[i].y + ";");
    }
    serialport.println(F("|xy"));
  }
}

// Settings
void LD06::enableCRC() {
  _useCRC = true;
}

void LD06::disableCRC() {
  _useCRC = false;
}

void LD06::enableFullScan() {  // readScan will return true only when a new 360° scan is available
  _fullScan = true;
}
void LD06::disableFullScan() {  // readScan will return true for each data chunks
  _fullScan = false;
}

void LD06::enableFiltering() {
  _useFiltering = true;
}

void LD06::disableFiltering() {
  _useFiltering = false;
}

void LD06::setIntensityThreshold(uint8_t threshold) {
  _threshold = threshold;
}

void LD06::setMaxDistance(uint16_t maxDist) {
  _maxDist = maxDist;
}

void LD06::setMinDistance(uint16_t minDist) {
  _minDist = rescaleAngle(minDist);
}

void LD06::setDistanceRange(uint16_t minDist, uint16_t maxDist) {
  _minDist = minDist;
  _maxDist = maxDist;
}

int16_t LD06::rescaleAngle(int16_t angle) {
  while (angle < 0)
    angle += 360;
  if (angle > 360)
    angle %= 360;
  return angle;
}

void LD06::setMaxAngle(int16_t maxAngle) {
  _maxAngle = rescaleAngle(maxAngle);
}

void LD06::setMinAngle(int16_t minAngle) {
  _minAngle = minAngle;
}

void LD06::setAngleRange(int16_t minAngle, int16_t maxAngle) {
  _minAngle = rescaleAngle(minAngle);
  _maxAngle = rescaleAngle(maxAngle);
}
