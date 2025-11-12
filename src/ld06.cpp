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

/* Read lidar packet data with or without checking CRC,
   return : true if a valid package was received (with CRC : only if CRC ok)
*/
bool LD06::readData() {
  return _useCRC ? readDataCRC() : readDataNoCRC();
}

/* Read lidar packet data and check CRC,
   return : true if a valid packet is received
*/
bool LD06::readDataCRC() {
  bool result = false;
  while (_lidarSerial->available()) {
    uint8_t current = _lidarSerial->read();
    if (_receivedData.index > 1 || (_receivedData.index == 0 && current == LD06_HEADER) || (_receivedData.index == 1 && current == LD06_VER_SIZE)) {
      _receivedData.packet.bytes[_receivedData.index] = current;
      if (_receivedData.index < LD06_PACKET_SIZE - 1) {
        _receivedData.computedCrc = CrcTable[_receivedData.computedCrc ^ current];
        _receivedData.index++;
      } else {
        if (_receivedData.computedCrc == current) {
          _previousPacket = _receivedData.packet;
          computeData();
          result = true;
        } else {
          _checksumFailCount++;  // CRC error counter
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
  bool result = false;
  while (_lidarSerial->available()) {
    uint8_t current = _lidarSerial->read();
    if (_receivedData.index > 1 || (_receivedData.index == 0 && current == LD06_HEADER) || (_receivedData.index == 1 && current == LD06_VER_SIZE)) {
      _receivedData.packet.bytes[_receivedData.index] = current;
      _receivedData.index++;
      if (_receivedData.index == LD06_PACKET_SIZE - 1) {
        _previousPacket = _receivedData.packet;
        computeData();
        _receivedData.index = 0;
        result = true;
      }
    } else {
      _receivedData.index = 0;
    }
  }
  return result;
}

/* Read lidar packets and update scan buffers.
   return : true if:
     - in _fullScan mode : a new 360° scan is available
     - otherwise : partial chunk of data is available
*/
bool LD06::readScan() {
  _newScan = false;
  bool result = false;
  if (readData()) {
    if (_newScan) {
      result = true;
    }
  }
  return result;
}

void LD06::computeData() {
  static bool  isInit         = false;
  static float lastPhysAngle  = 0.0f;  // Previous point LD06 angle data CW
  static float startPhysAngle = 0.0f;  // LD06 starting angle data CW

  float angleStep = getAngleStep();
  if (angleStep > LD06_ANGLE_STEP_MAX || angleStep <= 0.0f) {
    // should not be possible
    isInit = false;
    return;
  }

  int8_t reverse = (_upsideDown ? -1 : 1);

  float fsa = _receivedData.packet.startAngle / 100.0f;

  DataPoint data;

  for (uint16_t i = 0; i < LD06_PTS_PER_PACKETS; i++) {

    float physAngle = fsa + (i + 0.5f) * angleStep;

    while (physAngle >= 360.0f) physAngle -= 360.0f;
    while (physAngle <   0.0f) physAngle += 360.0f;

    if (physAngle < lastPhysAngle) {
      if (!isInit) {
        isInit = true;
      } else {
        if (lastPhysAngle - startPhysAngle > 340.0f) {
          _newScan = true;
          if (_fullScan) {
            swapBuffers();
          }
        }
      }
      startPhysAngle = physAngle;
    }
    lastPhysAngle = physAngle;


    float angle;
    if (_upsideDown) {
      angle = physAngle;                 // Upside down turn CW to CCW => Nothing to change
    } else {
      angle = 360.0f - physAngle;        // Convert LD06 angle data CW to mathematical CCW convension
      if (angle >= 360.0f) angle -= 360.0f;
    }

    // Normalize [0 360°]
    if (angle < 0.0f)     angle += 360.0f;
    if (angle >= 360.0f)  angle -= 360.0f;

    _angles[i] = angle;

    if (isInit && _currentScan->index < LD06_MAX_PTS_SCAN) {
      data.angle     = angle;                                   // Mathematical angle CCW
      data.distance  = _receivedData.packet.measures[i].distance;
      data.intensity = _receivedData.packet.measures[i].intensity;

      if (!_useFiltering || filter(data)) {
#ifdef LD06_COMPUTE_XY
        float angRad = (data.angle + _angularPosition + _angularOffset) * PI / 180.0f;
        float cosPos = cos(_angularPosition * PI / 180.0f);
        float sinPos = sin(_angularPosition * PI / 180.0f);

        data.x = _xPosition
                 + _xOffset * cosPos - _yOffset * sinPos
                 + data.distance * cos(angRad);

        data.y = _yPosition
                 + _xOffset * sinPos + _yOffset * cosPos
                 - data.distance * sin(angRad);
#endif
        _currentScan->points[_currentScan->index++] = data;
      }
    }
  }

  // En mode "non full scan", chaque paquet déclenche un swap
  if (!_fullScan) {
    swapBuffers();
    _newScan = true;
  }
}

// Print full scan using csv format
void LD06::printScanCSV(Stream &serialport) {
  static bool init = false;
  if (!init) {
    serialport.println(F("N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)"));
    init = true;
  }
  if (_previousScan->index) {
    for (uint16_t i = 0; i < _previousScan->index; i++) {
      serialport.print(i);
      serialport.print(",");
      serialport.print(_previousScan->points[i].angle, 2);
      serialport.print(",");
      serialport.print(_previousScan->points[i].distance);
      serialport.print(",");
      serialport.print(_previousScan->points[i].intensity);
#ifdef LD06_COMPUTE_XY
      serialport.print(",");
      serialport.print(_previousScan->points[i].x);
      serialport.print(",");
      serialport.print(_previousScan->points[i].y);
#endif
      serialport.println();
    }
    serialport.println();
  }
}

// Print full scan using teleplot format (check :https://teleplot.fr/)
#ifdef LD06_COMPUTE_XY
void LD06::printScanTeleplot(Stream &serialport) {
  if (_previousScan->index) {
    serialport.print(F(">lidar:"));
    for (uint16_t i = 0; i < _previousScan->index; i++) {
      serialport.print(_previousScan->points[i].x);
      serialport.print(":");
      serialport.print(_previousScan->points[i].y);
      serialport.print(";");
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

void LD06::enableFullScan() {   // readScan will return true only when a new 360° scan is available
  _fullScan = true;
}
void LD06::disableFullScan() {  // readScan will return true for each data chunk
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
  _minDist = minDist;
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