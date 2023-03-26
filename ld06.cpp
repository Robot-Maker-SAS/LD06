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

// Read lidar packet data
int LD06::read_lidar_data() {
  return read_lidar_data_with_crc();
}

// Read lidar packet data and check CRC,
// return -1 if new packet received but crc error,
// return 1 if new packet received and crc ok
// else return 0
int LD06::read_lidar_data_with_crc() {
  int result = 0;
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
          result = 1;
        } else {
          result = -1;
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

// Read lidar packet data without checking CRC,
// return 1 if new packet received
// else return 0
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

// Read lidar packets and return true when a new full 360° scan is available
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
          scan.clear();
          for (uint16_t j = 0; j < fullScan.size(); j ++) {
            scan.push_back(fullScan[j]);
          }
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

// Print full scan using csv format
void LD06::csvPrintScan() {
  Serial.println("Angle(°),Distance(mm),x(mm),y(mm)");
  for (uint16_t i = 0; i < scan.size(); i ++) {
    Serial.println(String() + scan[i].angle + "," + scan[i].distance + "," + scan[i].x + "," + scan[i].y);
  }
  Serial.println("");
}

// Print full scan using teleplot format (check :https://teleplot.fr/)
void LD06::teleplotPrintScan() {
  Serial.print(">lidar:");
  for (uint16_t i = 0; i < scan.size(); i++) {
    Serial.print(String() + scan[i].x + ":" + scan[i].y + ";");
  }
  Serial.println("|xy");
}
