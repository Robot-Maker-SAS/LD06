#include "ld06.h"

void LD06::init(const int pin)
{
  Serial1.begin(230400, SERIAL_8N1);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void LD06::calc_lidar_data(std::vector<char>& values) {
  start_byte  = values[0];
  data_length = 0x1F & values[1];
  Speed       = float(values[3] << 8 | values[2]) / 100;
  FSA         = float(values[5] << 8 | values[4]) / 100;
  LSA         = float(values[values.size() - 4] << 8 | values[values.size() - 5]) / 100;
  time_stamp  = int(values[values.size() - 2] << 8 | values[values.size() - 3]);
  CS          = int(values[values.size() - 1]);

  if (LSA - FSA > 0)
    angle_step = (LSA - FSA) / (data_length - 1);
  else
    angle_step = (LSA + (360 - FSA)) / (data_length - 1);

  if (angle_step > 20)
    return;

  angles.clear();
  confidences.clear();
  distances.clear();

  for (int i = 0; i < data_length; i++) {
    float raw_deg = FSA + i * angle_step;
    angles.push_back(raw_deg <= 360 ? raw_deg : raw_deg - 360);
    confidences.push_back(values[8 + i * 3]);
    distances.push_back(int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]));
  }
}

bool LD06::read_lidar_data() {
  return read_lidar_data_with_crc();
}

// Using vector with CRC

bool LD06::read_lidar_data_with_crc() {
  bool result = false;
  static uint8_t crc = 0;
  static std::vector<char> tmpChars;
  while (Serial1.available()) {
    char tmpInt = Serial1.read();
    uint8_t vectorSize = tmpChars.size();
    if (vectorSize > 1 || (!vectorSize && tmpInt == HEADER) || (vectorSize == 1 && tmpInt == VER_SIZE)) {
      tmpChars.push_back(tmpInt);
      if (tmpChars.size() < TOTAL_DATA_BYTE)
        crc = CrcTable[crc ^ tmpInt];
      else {
        if (crc == tmpInt) {
          calc_lidar_data(tmpChars);
          result = true;
        } else {
          // CRC Error
        }
        tmpChars.clear();
        crc = 0;
      }
    } else {
      tmpChars.clear();
      crc = 0;
    }
  }
  return result;
}


bool LD06::read_lidar_data_without_crc() {
  static std::vector<char> tmpChars;
  while (Serial1.available()) {
    char tmpInt = Serial1.read();
    uint8_t vectorSize = tmpChars.size();
    if (vectorSize > 1 || (!vectorSize && tmpInt == HEADER) || (vectorSize == 1 && tmpInt == VER_SIZE)) {
      tmpChars.push_back(tmpInt);
      if (tmpChars.size() == TOTAL_DATA_BYTE) {
        calc_lidar_data(tmpChars);
        tmpChars.clear();
        return true;
      }
    } else
      tmpChars.clear();
  }
  return false;
}

bool LD06::readFullScan() {
  static std::vector<DataPoint> fullScan;
  static bool isInit = false;
  static float lastAngle = 0;
  bool newScan = false;
  DataPoint data;

  if (read_lidar_data()) {
    for (int i = 0; i < data_length; i++) {
      if (angles[i] < lastAngle) {
        if (!isInit) {
          isInit = true;
        } else {
          newScan = true;
          storeFullScan(fullScan);
          fullScan.clear();
        }
      }
      lastAngle = angles[i];

      data.angle = angles[i];
      data.distance = distances[i];
      data.x = data.distance * cos(data.angle * PI / 180);
      data.y = -data.distance * sin(data.angle * PI / 180);
      data.intensity = confidences[i];
      fullScan.push_back(data);
    }
  }
  return newScan;
}

void LD06::storeFullScan(std::vector<DataPoint>& fullScan) {
  scan.clear();
  for (uint16_t i = 0; i < fullScan.size(); i ++) {
    scan.push_back(fullScan[i]);
  }
  //printScan();
  teleplotPrintScan();
}


void LD06::printScan() {
  for (uint16_t i = 0; i < scan.size(); i ++) {
    Serial.print(scan[i].angle);
    Serial.print(",");
    Serial.print(scan[i].distance);
    Serial.print(",");
    Serial.print(scan[i].x);
    Serial.print(",");
    Serial.print(scan[i].y);
    Serial.println("");
  }
  Serial.println("");
}

void LD06::teleplotPrintScan() {
  Serial.print(">lidar:");
  for (uint16_t i = 0; i < scan.size(); i ++) {
    Serial.print(scan[i].x);
    Serial.print(":");
    Serial.print(scan[i].y);
    Serial.print(";");
  }
  Serial.println("|xy");
}
