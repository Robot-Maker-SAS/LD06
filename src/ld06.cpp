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
  while (_lidarSerial->available()) {
    uint8_t current = _lidarSerial->read();
    if (_receivedData.index > 1 || (_receivedData.index == 0 && current == LD06_HEADER) || (_receivedData.index == 1 && current == LD06_VER_SIZE)) {
      _receivedData.packet.bytes[_receivedData.index] = current;
      if (_receivedData.index < LD06_PACKET_SIZE - 1) {
        _receivedData.computedCrc = CrcTable[_receivedData.computedCrc ^ current];
        _receivedData.index++;
      } else {
        if (_receivedData.computedCrc == current) {
          computeData();
          result = 1;
        } else {
          _checksumFailCount++; // CRC error counter
        }
        _receivedData.index = 0;
        _receivedData.computedCrc = 0;
      }
    } else {
      _receivedData.index = 0;
      _receivedData.computedCrc = 0;
    }
  }
  return result;
}

/* Read lidar packet data without checking CRC,
   return : true if a packet is received
*/
bool LD06::readDataNoCRC() {
  while (_lidarSerial->available()) {
    uint8_t current = _lidarSerial->read();
    if (_receivedData.index > 1 || (_receivedData.index == 0 && current == LD06_HEADER) || (_receivedData.index == 1 && current == LD06_VER_SIZE)) {
      _receivedData.packet.bytes[_receivedData.index] = current;
      _receivedData.index++;
      if (_receivedData.index == LD06_PACKET_SIZE - 1) {
        computeData();
        _receivedData.index = 0;
        return true;
      }
    } else
      _receivedData.index = 0;
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
            swapBuffers();
          }
        }
      }
      lastAngle = _angles[i];
      if (_currentScan->index < LD06_MAX_PTS_SCAN) {
        data.angle = _angles[i];
        data.distance = _receivedData.packet.measures[i].distance;
        data.intensity = _receivedData.packet.measures[i].intensity;
        if (!_useFiltering || filter(data)) {
		  #ifdef LD06_COMPUTE_XY
          data.x = _xPosition + _xOffset * cos(_angularPosition) - _yOffset * sin(_angularPosition) + data.distance * cos((data.angle + _angularPosition + _angularOffset) * PI / 180);
          data.y = _yPosition + _xOffset * sin(_angularPosition) + _yOffset * cos(_angularPosition) - data.distance * sin((data.angle + _angularPosition + _angularOffset) * PI / 180);
          #endif
		  _currentScan->points[_currentScan->index] = data;
          _currentScan->index++;
        }
      }
    }
    if (!_fullScan) {
      swapBuffers();
    }
  }
  return result;
}

void LD06::computeData() {
  float angleStep = getAngleStep();
  if (angleStep > LD06_ANGLE_STEP_MAX)  // Should not be possible
    return;

  int8_t reverse = (_upsideDown ? -1 : 1);
  float fsa = _receivedData.packet.startAngle / 100.0;
  for (uint16_t i = 0; i < LD06_PTS_PER_PACKETS; i++) {
    float raw_deg = fsa + i * angleStep;
    _angles[i] = (raw_deg <= 360 ? raw_deg : raw_deg - 360) * reverse;
  }
}

// Print full scan using csv format
void LD06::printScanCSV(Stream &serialport) {
  static bool init = false;
  if (!init) {
    serialport.println(F("Angle(°),Distance(mm),x(mm),y(mm)"));
    init = true;
  }
  if (_previousScan->index) {
    for (uint16_t i = 0; i < _previousScan->index; i++) {
      serialport.print(String() + _previousScan->points[i].angle + "," + _previousScan->points[i].distance + "," + _previousScan->points[i].intensity);
	  #ifdef LD06_COMPUTE_XY
      serialport.print(String() + "," + _previousScan->points[i].x + "," + _previousScan->points[i].y);
	  #endif
	  serialport.println("");
    }
    serialport.println("");
  }
}

// Print full scan using teleplot format (check :https://teleplot.fr/)
#ifdef LD06_COMPUTE_XY
void LD06::printScanTeleplot(Stream &serialport) {
  if (_previousScan->index) {
    serialport.print(F(">lidar:"));
    for (uint16_t i = 0; i < _previousScan->index; i++) {
      serialport.print(String() + _previousScan->points[i].x + ":" + _previousScan->points[i].y + ";");
    }
    serialport.println(F("|xy"));
  }
}
#endif

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

#ifdef LD06_COMPUTE_XY
void LD06::setOffsetPosition(int16_t xPos = 0, int16_t yPos = 0, float anglePos = 0) {
  _xOffset = xPos;
  _yOffset = yPos;
  _angularOffset = anglePos;
}
#endif

void LD06::swapBuffers() {
  if (_currentBuffer) {
    _currentScan = &_scanB;
    _previousScan = &_scanA;
  } else {
    _currentScan = &_scanA;
    _previousScan = &_scanB;
  }
  _currentBuffer = !_currentBuffer;
  _currentScan->index = 0;
}