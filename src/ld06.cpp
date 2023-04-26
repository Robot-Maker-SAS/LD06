#include "ld06.h"

LD06::LD06(HardwareSerial &serial, uint8_t pwmPin)
  : _lidarSerial(&serial),
    _pin(pwmPin) {}

void LD06::init() {
  _lidarSerial->begin(230400);
  if (_pin != 255) {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, HIGH);
  }
}

/* Read lidar packet data without checking CRC,
   return : true if a valid package was received
*/
bool LD06::readData() {
  return _useCRC ? readDataCRC() : readDataNoCRC();
}

/* Read lidar packet data without checking CRC,
   return : true if a valid packet is received
*/
bool LD06::readDataCRC() {
  bool result = 0;
  static uint8_t crc = 0;
  static uint8_t n = 0;
  static uint8_t lidarData[LD06_PACKET_SIZE];
  while (_lidarSerial->available()) {
    uint8_t current = _lidarSerial->read();
    if (n > 1 || (n == 0 && current == LD06_HEADER) || (n == 1 && current == LD06_VER_SIZE)) {
      lidarData[n] = current;
      if (n < LD06_PACKET_SIZE - 1) {
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
   return : true if a packet is received
*/
bool LD06::readDataNoCRC() {
  static uint8_t n = 0;
  static uint8_t lidarData[LD06_PACKET_SIZE];
  while (_lidarSerial->available()) {
    uint8_t current = _lidarSerial->read();
    if (n > 1 || (n == 0 && current == LD06_HEADER) || (n == 1 && current == LD06_VER_SIZE)) {
      lidarData[n] = current;
      n++;
      if (n == LD06_PACKET_SIZE - 1) {
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
   return : true if a new full 360° scan is available when in _fullScan mode
   or true each time partial data chunks are available if not in _fullScan mode.
*/
bool LD06::readScan() {
  static bool isInit = false;
  static float lastAngle = 0;
  static float startAngle = 0;
  _newScan = false;
  bool result = false;
  DataPoint data;

  if (readData()) {
    for (int i = 0; i < LD06_PTS_PER_PACKETS; i++) {
      if (_angles[i] < lastAngle) {
        if (!isInit) {
          isInit = true;
        } else {
          if (lastAngle - startAngle > 340) {  // TODO check if this condition is enough to prevent partial scan prints
            _newScan = true;
            result = true;
          }
          startAngle = _angles[i];

          if (_fullScan) {
            _currentBuffer = !_currentBuffer;
            _scanIndex[_currentBuffer] = 0;
          }
        }
      }
      lastAngle = _angles[i];
      if (_scanIndex[_currentBuffer] < LD06_MAX_PTS_SCAN) {
        data.angle = _angles[i];
        data.distance = _distances[i];
        data.intensity = _confidences[i];
        if (!_useFiltering || filter(data)) {
          data.x = _xPosition + _xOffset * cos(_angularPosition) - _yOffset * sin(_angularPosition) + data.distance * cos((data.angle + _angularPosition + _angularOffset) * PI / 180);
          data.y = _yPosition + _xOffset * sin(_angularPosition) + _yOffset * cos(_angularPosition) - data.distance * sin((data.angle + _angularPosition + _angularOffset) * PI / 180);
          _scan[_scanIndex[_currentBuffer]][_currentBuffer] = data;
          _scanIndex[_currentBuffer]++;
        }
      }
    }
    if (!_fullScan) {
      _currentBuffer = !_currentBuffer;
      _scanIndex[_currentBuffer] = 0;
      result = true;
    }
  }
  return result;
}

void LD06::computeData(uint8_t *values) {
  _speed = values[3] << 8 | values[2];
  _FSA = float(values[5] << 8 | values[4]) / 100;
  _LSA = float(values[LD06_PACKET_SIZE - 4] << 8 | values[LD06_PACKET_SIZE - 5]) / 100;
  _timeStamp = int(values[LD06_PACKET_SIZE - 2] << 8 | values[LD06_PACKET_SIZE - 3]);

  _angleStep = ((_LSA - _FSA > 0) ? (_LSA - _FSA) / (LD06_PTS_PER_PACKETS - 1) : (_LSA + (360 - _FSA)) / (LD06_PTS_PER_PACKETS - 1));

  if (_angleStep > 20)
    return;

  int8_t reverse = (_upsideDown ? -1 : 1);

  for (uint16_t i = 0; i < LD06_PTS_PER_PACKETS; i++) {
    float raw_deg = _FSA + i * _angleStep;
    _angles[i] = (raw_deg <= 360 ? raw_deg : raw_deg - 360) * reverse;
    _confidences[i] = values[8 + i * 3];
    _distances[i] = int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]);
  }
}

// Print full scan using csv format
void LD06::printScanCSV(Stream &serialport) {
  static bool init = false;
  if (!init) {
    serialport.println(F("Angle(°),Distance(mm),x(mm),y(mm)"));
    init = true;
  }
  if (_scanIndex[!_currentBuffer]) {
    for (uint16_t i = 0; i < _scanIndex[!_currentBuffer]; i++) {
      serialport.println(String() + _scan[i][!_currentBuffer].angle + "," + _scan[i][!_currentBuffer].distance + "," + _scan[i][!_currentBuffer].x + "," + _scan[i][!_currentBuffer].y);
    }
    serialport.println("");
  }
}

// Print full scan using teleplot format (check :https://teleplot.fr/)
void LD06::printScanTeleplot(Stream &serialport) {
  if (_scanIndex[!_currentBuffer]) {
    serialport.print(F(">lidar:"));
    for (uint16_t i = 0; i < _scanIndex[!_currentBuffer]; i++) {
      serialport.print(String() + _scan[i][!_currentBuffer].x + ":" + _scan[i][!_currentBuffer].y + ";");
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
  if (angle > 360)
    angle %= 360;
  else
    while (angle < 0)
      angle += 360;
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

void LD06::setUpsideDown(bool upsideDown) {
  _upsideDown = upsideDown;
}

void LD06::setOffsetPosition(int16_t xPos = 0, int16_t yPos = 0, float anglePos = 0) {
  _xOffset = xPos;
  _yOffset = yPos;
  _angularOffset = anglePos;
}